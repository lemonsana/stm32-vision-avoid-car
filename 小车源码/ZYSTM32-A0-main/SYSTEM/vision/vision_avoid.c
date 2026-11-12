#include "vision_avoid.h"
#include "vision_link.h"
#include "sched.h"
#include "IRSEARCH.h"
#include "motor.h"
#include "cfg.h"
#include "app_mode.h"
#include <stdio.h>

/* ==========================================================================
 *  视觉避障状态机（模式 5）
 *
 *  设计原则：运动控制权在任意时刻只属于一个状态。
 *    - 只有 VA_LINE / VA_SLOW / VA_COOL 调用 SearchRun()（循迹 PID）
 *    - 其余状态一律用 drive()/turn() 自己写 PWM，绝不和 PID 抢
 *    - 所有避障输出都过 slew_step()，没有阶跃，所以不会"突然甩头"
 *
 *  流程（每 10ms 调用一次 VisionAvoid_Update）：
 *    WAIT  ──有视觉心跳──▶ LINE
 *    LINE  ──30cm 内连续2帧──▶ SLOW（收油到 60%，仍循迹）
 *    SLOW  ──18cm 内连续2帧──▶ TURN_OUT（提交绕行，锁定绕行侧）
 *    TURN_OUT ──转动到位──▶ SHIFT（弧线侧移，离开原线）
 *    SHIFT ──灰度确认离线/超时──▶ STRAIGHTEN（回正）
 *    STRAIGHTEN ──转动到位──▶ PASS（直行越障 + 视觉保持距离）
 *    PASS  ──车尾越障（时间下限+目标离开视野）──▶ SEEK（弧线回线）
 *    SEEK  ──灰度连续3拍压线──▶ ALIGN（对正）
 *    ALIGN ──重心误差进死区──▶ COOL（交回 PID，速度 55%→100% 爬升）
 *    COOL  ──冷静期结束──▶ LINE
 *    任意段超时/传感器异常 ──▶ FAULT（安全停车，需重发 '5' 才能恢复）
 * ========================================================================== */

enum { VA_WAIT = 0, VA_LINE, VA_SLOW, VA_TURN_OUT, VA_SHIFT, VA_STRAIGHTEN,
       VA_PASS, VA_SEEK, VA_ALIGN, VA_COOL, VA_FAULT };
/* 故障码，对应串口 [vision] 的 fault 字段 */
enum { VF_NONE = 0, VF_LINK, VF_GRAY, VF_SEEK, VF_BLOCKED, VF_OVERRUN };

static u8  state, fault;
static u8  confirmed;        /* 滑窗命中的帧数（LINE 用） */
static u8  hit_hist;         /* 最近 VA_CONFIRM_WINDOW 个新检测帧的命中位，bit0=最新 */
static u8  have_packet;
static u8  armed;            /* 1 = 允许触发避障（重触发抑制闩锁） */
static u8  side;             /* 本次绕行往哪边：1=左，0=右 */
static u8  near_cnt;         /* SLOW：连续落在触发距离内的帧数 */
static u8  off_cnt;          /* SHIFT：连续看不到线的拍数 */
static u8  clear_cnt;        /* PASS：连续看不到目标的帧数 */
static u8  line_cnt;         /* SEEK/ALIGN：连续压线的拍数 */
static u8  lost_cnt;         /* ALIGN：连续丢线的拍数 */
static u8  gray_fail;        /* 灰度连续读失败拍数 */
static int16_t out_l, out_r; /* 斜率限制后的上一拍输出 */
static int prev_yb;
static u32 prev_stamp;
static u8  have_prev;
static u32 entered;
static u32 maneuver_start;
static u32 seek_start;
static u32 clear_start;      /* 最近一次"视野里有近处目标"的时刻 */
static K210_MSG_TypeDef target;

/* ---------- 一帧灰度数据的解析结果 ---------- */
typedef struct
{
    u8  any;      /* 至少一路有黑（很轻的擦过也算） */
    u8  solid;    /* 压得足够实：加权和达到 IRSEARCH 的 on_line 门限 */
    u8  center;   /* 中段两路（通道4/5）压线 */
    int err;      /* 加权重心 -100~+100，负数=线偏左 */
} va_line_t;

/* ==========================================================================
 *  基础工具
 * ========================================================================== */

static void enter(u8 next)
{
    state    = next;
    entered  = g_tick_ms;
    confirmed = 0;
    hit_hist  = 0;
    have_prev = 0;
    near_cnt  = 0;
    off_cnt   = 0;
    clear_cnt = 0;
    line_cnt  = 0;
    lost_cnt  = 0;
}

static void fail(u8 reason)
{
    fault = reason;
    enter(VA_FAULT);
    Gray_SetSpeedScale(100);
    out_l = out_r = 0;
    Motor_Stop();
    Gray_ResetIntegral();
}

static int16_t slew_step(int16_t cur, int16_t tgt)
{
    int d = (int)tgt - (int)cur;
    if (d >  VA_SLEW_STEP) d =  VA_SLEW_STEP;
    if (d < -VA_SLEW_STEP) d = -VA_SLEW_STEP;
    return (int16_t)((int)cur + d);
}

/* 所有避障动作的唯一出口：限幅 + 斜率限制 + 下发电机 */
static void drive(int l, int r)
{
    if (l >  100) l =  100;
    if (l < -100) l = -100;
    if (r >  100) r =  100;
    if (r < -100) r = -100;
    out_l = slew_step(out_l, (int16_t)l);
    out_r = slew_step(out_r, (int16_t)r);
    Motor_SetSpeed(out_l, out_r);
}

/* dir=+1 向外（离开原轨迹那一侧），dir=-1 向内（回原轨迹方向）
 * side=1 时向外是左转：左轮后退/右轮前进 */
static void turn(int dir, int speed)
{
    int sign = side ? dir : -dir;
    drive(-sign * speed, sign * speed);
}

/* 提交绕行时决定绕行侧：障碍偏左就向右绕，反之向左绕 */
static u8 choose_side(const K210_MSG_TypeDef *m)
{
    int cx = (int)m->x + (int)m->w / 2;
    if (cx < 160 - (int)VA_SIDE_MARGIN_PX) return 0;   /* 障碍偏左 → 向右绕 */
    if (cx > 160 + (int)VA_SIDE_MARGIN_PX) return 1;   /* 障碍偏右 → 向左绕 */
    return (u8)(VA_LEFT ? 1 : 0);                      /* 居中：用默认侧 */
}

/* 读一帧灰度，同时算出"有没有线 / 压得实不实 / 加权重心"。
 * 重心算法与 IRSEARCH.c 的 gray_read_frame 保持一致（weight = cutoff - v，
 * pos = -7..+7，err = sum(pos*w)*100/(sum(w)*7)），这样"对正"用的误差和
 * 后面 PID 用的误差是同一个量纲，交回 PID 时不会因为口径不同而突变。
 * 返回 0 = I2C 读失败（调用方不能把它当成"没看到线"）。 */
static u8 read_line(va_line_t *l)
{
    static const int8_t pos_w[8] = { -7, -5, -3, -1, 1, 3, 5, 7 };
    u8  v[8];
    u8  i;
    int cutoff = Cfg_Get(CFG_CUTOFF);
    int sum_w = 0;
    int sum_pos = 0;

    l->any = 0;
    l->solid = 0;
    l->center = 0;
    l->err = 0;

    if (!Gray_IsReady()) return 0;
    if (!Gray_ReadRaw(v)) return 0;
    if (cutoff < 1) cutoff = 1;

    for (i = 0; i < 8; i++)
    {
        int w = cutoff - (int)v[i];
        if (w <= 0) continue;                 /* 这一路是白底 */
        if (w > cutoff) w = cutoff;           /* 反相/过曝时别把权重放大 */
        l->any = 1;
        sum_w  += w;
        sum_pos += w * pos_w[i];
    }
#if GRAY_CHANNEL_MIRROR
    sum_pos = -sum_pos;
#endif
    l->center = (u8)((v[3] < cutoff || v[4] < cutoff) ? 1 : 0);
    /* 加权和不到 100 视为"只是擦到一点"，与 IRSEARCH 的 on_line 判据一致 */
    l->solid = (u8)(sum_w >= 100 ? 1 : 0);
    if (sum_w > 0) l->err = sum_pos * 100 / (sum_w * 7);
    return 1;
}

/* ---------- 距离：框底边行号 -> 厘米 ----------
 * 贴地障碍物的框底边 yb=y+h 就是接地点，越近越靠下。1/d 对 yb 线性，
 * 用 10cm/20cm 两个标定点插值，标定点之外按同一直线外推。
 * 全部整数运算：把 1/d 放大 10000 倍，避免在 ARMCC 上引入浮点库。 */
#define VA_ROW_SPAN (((int)VA_ROW_AT_20CM) - ((int)VA_ROW_AT_10CM))

static int rows_to_inv10k(int yb)
{
    if (VA_ROW_SPAN == 0) return 1000;      /* 两个标定点写重合了：退化成固定 10cm */
    return 1000 - (int)((long)(yb - (int)VA_ROW_AT_10CM) * 500L / VA_ROW_SPAN);
}

/* 行号 -> 距离(cm)，只用于调试打印 */
static u16 rows_to_cm(int yb)
{
    int inv = rows_to_inv10k(yb);
    if (inv <= 0) return 9999u;
    return (u16)(10000 / inv);
}

/* 距离 cm 对应的行号：d <= cm 等价于 yb >= row_for_cm(cm) */
static int row_for_cm(u16 cm)
{
    if (cm == 0u) return 0x7FFF;            /* 0 = 关闭门限 */
    return (int)VA_ROW_AT_10CM +
           (int)((long)(1000 - 10000 / (int)cm) * VA_ROW_SPAN / 500L);
}

/* 框底边：贴地障碍物的接地点（K210 已把 y+h 钳在 239 以内） */
static int box_bottom(const K210_MSG_TypeDef *m)
{
    int yb = (int)m->y + (int)m->h;
    return yb > 239 ? 239 : yb;
}

/* 用最近两帧底边行号的变化率把 yb 往前推，补偿"一帧曝光+卡尔曼+串口"的时延；
 * 只在确实在靠近（step>0）时外推，别把远处的框拉近。 */
static int predict_row(int yb, u32 stamp)
{
    long dt, step, pred;
    if (VA_PREDICT_MS == 0u || !have_prev) return yb;
    dt   = (long)(u32)(stamp - prev_stamp);
    step = (long)yb - (long)prev_yb;
    if (dt < 10 || dt > 500 || step <= 0) return yb;
    pred = (long)yb + step * (long)VA_PREDICT_MS / dt;
    return pred > 239 ? 239 : (int)pred;
}

/* 一个检测帧是否算目标：类别对、found=1、框与前方通道有重叠 */
static u8 is_target(const K210_MSG_TypeDef *m)
{
    return (u8)(m->found && m->id == VA_CLASS_ID &&
                m->x < VA_ROI_RIGHT && (u32)m->x + m->w > VA_ROI_LEFT);
}

/* 是否已经进入"该收油"的距离（VA_SLOW_CM） */
static u8 in_slow_range(const K210_MSG_TypeDef *m)
{
    if (!is_target(m)) return 0;
    if (VA_SLOW_CM == 0u) return 1;
    return (u8)(predict_row(box_bottom(m), m->stamp) >= row_for_cm(VA_SLOW_CM));
}

/* 是否已经进入"该绕行"的距离（VA_TRIGGER_CM） */
static u8 in_trigger_range(const K210_MSG_TypeDef *m)
{
    if (!is_target(m)) return 0;
    if (VA_TRIGGER_CM == 0u) return 1;
    return (u8)(predict_row(box_bottom(m), m->stamp) >= row_for_cm(VA_TRIGGER_CM));
}

/* ==========================================================================
 *  对外接口
 * ========================================================================== */

void VisionAvoid_Start(void)
{
    K210_MSG_TypeDef discard;

    VisionLink_Read(&discard);   /* 丢弃进入本模式之前的那一帧，要求新模式有新心跳 */
    state       = VA_WAIT;
    fault       = VF_NONE;
    confirmed   = 0;
    hit_hist    = 0;
    have_packet = 0;
    armed       = 1;             /* 刚进模式允许第一次触发 */
    side        = (u8)(VA_LEFT ? 1 : 0);
    gray_fail   = 0;
    out_l       = out_r = 0;
    target.found = 0;
    clear_start = g_tick_ms;
    seek_start  = g_tick_ms;
    maneuver_start = g_tick_ms;
    enter(VA_WAIT);

    Gray_SetSpeedScale(100);
    Motor_Stop();
    Gray_ResetIntegral();
}

void VisionAvoid_Stop(void)
{
    enter(VA_WAIT);
    confirmed   = 0;
    have_packet = 0;
    hit_hist    = 0;
    gray_fail   = 0;
    out_l       = out_r = 0;
    Gray_SetSpeedScale(100);
    Motor_Stop();
    Gray_ResetIntegral();
}

void VisionAvoid_Update(void)
{
    u8 fresh;
    u8 line_ok = 0;
    u32 elapsed;
    va_line_t line;

    line.any = 0; line.solid = 0; line.center = 0; line.err = 0;

    fresh = VisionLink_Read(&target);
    if (fresh) have_packet = 1;

    /* 故障一旦进入就锁死：即使视觉恢复也不自己起步，必须重发 '5' */
    if (state == VA_FAULT) { Motor_Stop(); return; }

    if (!have_packet)
    {
        /* 视觉还没就绪：保留原有灰度循迹，K210 不影响基本行驶 */
        SearchRun();
        return;
    }

    if ((u32)(g_tick_ms - target.stamp) >= VA_LINK_TIMEOUT_MS)
    {
        if (state == VA_WAIT || state == VA_LINE || state == VA_SLOW || state == VA_COOL)
        {
            /* 还没离开轨道：退化成纯灰度循迹，等链路恢复 */
            have_packet  = 0;
            target.found = 0;
            Gray_SetSpeedScale(100);
            enter(VA_WAIT);
            SearchRun();
        }
        else
        {
            /* 已经在绕行途中失联：不能盲走，停车 */
            fail(VF_LINK);
        }
        return;
    }

    if (!Gray_IsReady()) { fail(VF_GRAY); return; }
    if (state == VA_WAIT) enter(VA_LINE);

    elapsed = (u32)(g_tick_ms - entered);

    /* 全程超时保护：绕行+找线+恢复的总时长封顶 */
    if (state >= VA_TURN_OUT && state <= VA_COOL)
    {
        if ((u32)(g_tick_ms - maneuver_start) >= VA_MANEUVER_MAX_MS)
        {
            fail(VF_OVERRUN);
            return;
        }
    }

    /* 绕行段每拍都看一眼灰度：侧移是否已经离开原线、找线有没有压上 */
    if (state >= VA_SHIFT && state <= VA_ALIGN)
    {
        if (read_line(&line)) { line_ok = 1; gray_fail = 0; }
        else if (++gray_fail >= VA_GRAY_FAIL_MAX)
        {
            fail(VF_GRAY);      /* 连续 150ms 读不到，才认为是传感器故障 */
            return;
        }
    }

    switch (state)
    {
        /* ------------------- 循迹 + 障碍确认 ------------------- */
        case VA_LINE:
            Gray_SetSpeedScale(100);    /* 正常循迹：原速，行为与旧版一致 */

            if (fresh)
            {
                u8 i, hits = 0, hit = in_slow_range(&target);
                hit_hist = (u8)(((hit_hist << 1) | hit) &
                                (u8)((1u << VA_CONFIRM_WINDOW) - 1u));
                for (i = 0; i < VA_CONFIRM_WINDOW; ++i)
                    hits = (u8)(hits + ((hit_hist >> i) & 1u));
                confirmed = hits;

                /* 记录本帧底边，供下一帧算靠近速率；非目标帧断开历史 */
                if (is_target(&target))
                {
                    prev_yb    = box_bottom(&target);
                    prev_stamp = target.stamp;
                    have_prev  = 1;
                }
                else have_prev = 0;
            }

            /* 重触发抑制：视野里持续没有近处目标 VA_REARM_MS，才重新允许触发 */
            if (in_trigger_range(&target)) clear_start = g_tick_ms;
            else if (!armed && (u32)(g_tick_ms - clear_start) >= VA_REARM_MS) armed = 1;

            if (armed && confirmed >= VA_CONFIRM_HITS)
            {
                if (target.w >= VA_BLOCK_WIDTH_PX)
                {
                    fail(VF_BLOCKED);   /* 障碍宽到占满前方通道：两侧都堵，别硬绕 */
                    break;
                }
                side = choose_side(&target);
                armed = 0;
                clear_start = g_tick_ms;
                Gray_SetSpeedScale(VA_SLOW_PCT);
                enter(VA_SLOW);         /* 先减速，不直接原地转 */
                SearchRun();
            }
            else SearchRun();
            break;

        /* ------------------- 减速接近 ------------------- */
        case VA_SLOW:
            Gray_SetSpeedScale(VA_SLOW_PCT);
            if (fresh)
            {
                if (in_trigger_range(&target))
                {
                    if (near_cnt < 255u) near_cnt++;
                }
                else near_cnt = 0;
            }

            if (near_cnt >= VA_CONFIRM_HITS)
            {
                /* 到达触发距离：此刻才交出控制权，开始绕行 */
                Gray_ResetIntegral();
                maneuver_start = g_tick_ms;
                enter(VA_TURN_OUT);
                break;
            }
            if (elapsed >= VA_SLOW_MAX_MS)
            {
                /* 减速半天也没靠近：判定是误检/别的物体，回原速循迹 */
                Gray_SetSpeedScale(100);
                enter(VA_LINE);
                break;
            }
            SearchRun();
            break;

        /* ------------------- ① 原地向外转 ------------------- */
        case VA_TURN_OUT:
            turn(1, VA_TURN_SPEED);
            if (elapsed >= VA_TURN_OUT_MS) enter(VA_SHIFT);
            break;

        /* ------------------- ② 弧线侧移，离开原线 ------------------- */
        case VA_SHIFT:
            /* 外侧轮快、内侧轮慢：一边前进一边把车身带向绕行侧。
             * 比"直行侧移"多一个横向分量，同样的时间能让出更大的距离。 */
            if (side) drive(VA_SHIFT_SLOW, VA_SHIFT_FAST);
            else      drive(VA_SHIFT_FAST, VA_SHIFT_SLOW);

            if (line_ok)
            {
                if (!line.any) { if (off_cnt < 255u) off_cnt++; }
                else off_cnt = 0;
            }
            /* 时间下限保证横向让距；"灰度连续 3 拍看不到线"是"确实离开原轨迹"的
             * 传感确认；MAX 兜底，防止一直看不到线时卡在这一段。 */
            if (elapsed >= VA_SHIFT_MIN_MS &&
                (off_cnt >= VA_LINE_CONFIRM || elapsed >= VA_SHIFT_MAX_MS))
            {
                enter(VA_STRAIGHTEN);
            }
            break;

        /* ------------------- ③ 回正 ------------------- */
        case VA_STRAIGHTEN:
            turn(-1, VA_TURN_SPEED);
            if (elapsed >= VA_STRAIGHTEN_MS) enter(VA_PASS);
            break;

        /* ------------------- ④ 越障 ------------------- */
        case VA_PASS:
        {
            int l = VA_SPEED, r = VA_SPEED;

            /* 看得见障碍就往外让一点：这比纯定时可靠，直接对应"别蹭上去"。
             * 期望把障碍保持在绕行侧的画面外侧（side=1 时希望框中心在右侧）。 */
            if (is_target(&target))
            {
                int off = (int)target.x + (int)target.w / 2 - 160;  /* >0：障碍在右 */
                int err, corr;
                if (side) err = (int)VA_PASS_KEEP_PX - off;
                else      err = off + (int)VA_PASS_KEEP_PX;
                if (err > 0)
                {
                    corr = err * VA_PASS_GAIN / 100;
                    if (corr > VA_PASS_CORR_MAX) corr = VA_PASS_CORR_MAX;
                    if (side) { l = VA_SPEED - corr; r = VA_SPEED + corr; }
                    else      { l = VA_SPEED + corr; r = VA_SPEED - corr; }
                }
            }
            drive(l, r);

            if (fresh)
            {
                if (!is_target(&target)) { if (clear_cnt < 255u) clear_cnt++; }
                else clear_cnt = 0;
            }
            if (elapsed >= VA_PASS_MS && clear_cnt >= VA_CLEAR_CONFIRM)
            {
                seek_start = g_tick_ms;     /* 找线预算从这里开始计 */
                enter(VA_SEEK);
            }
            else if (elapsed >= VA_PASS_MAX_MS)
            {
                /* 走了 2.2s 障碍还挡在正前方：侧移不够/被卡住，停车更安全 */
                fail(VF_BLOCKED);
            }
            break;
        }

        /* ------------------- ⑤ 弧线回线 ------------------- */
        case VA_SEEK:
            /* 反方向的弧线：车一边前进一边把车头扫向原轨迹，线一定会被扫到；
             * 旧的"直行 45 直到撞线"在线不在正前方时只能超时停车。 */
            if (side) drive(VA_SEEK_FAST, VA_SEEK_SLOW);
            else      drive(VA_SEEK_SLOW, VA_SEEK_FAST);

            if (line_ok && line.any)
            {
                if (line_cnt < 255u) line_cnt++;
                if (line_cnt >= VA_LINE_CONFIRM) { enter(VA_ALIGN); break; }
            }
            else if (line_ok) line_cnt = 0;

            if ((u32)(g_tick_ms - seek_start) >= VA_SEEK_BUDGET_MS)
            {
                fail(VF_SEEK);      /* 找线超时：安全停车，不死循环 */
            }
            break;

        /* ------------------- ⑥ 对正 ------------------- */
        case VA_ALIGN:
        {
            int corr;

            if (line_ok && line.solid)
            {
                if (line_cnt < 255u) line_cnt++;
                lost_cnt = 0;
                /* 线压稳了、而且基本在阵列中间：可以交回 PID */
                if (line_cnt >= VA_ALIGN_CONFIRM &&
                    line.err >= -VA_ALIGN_DEADBAND && line.err <= VA_ALIGN_DEADBAND)
                {
                    Gray_ResetIntegral();               /* 清 I/D/滤波/斜坡，防止接管瞬间甩尾 */
                    Gray_SetSpeedScale(VA_HANDOVER_PCT);/* 交回时先降速，再爬升 */
                    enter(VA_COOL);
                    break;
                }
            }
            else
            {
                line_cnt = 0;
                if (line_ok && !line.any)
                {
                    if (lost_cnt < 255u) lost_cnt++;
                    if (lost_cnt >= VA_LINE_CONFIRM)
                    {
                        enter(VA_SEEK);     /* 又滑出去了：继续找，预算仍然有效 */
                        break;
                    }
                }
            }

            /* mini-P 对正：重心偏左就往左修，和 PID 同一个方向感 */
            corr = line.err * VA_ALIGN_GAIN / 100;
            if (corr >  VA_ALIGN_MAX_CORR) corr =  VA_ALIGN_MAX_CORR;
            if (corr < -VA_ALIGN_MAX_CORR) corr = -VA_ALIGN_MAX_CORR;
            drive(VA_ALIGN_SPEED + corr, VA_ALIGN_SPEED - corr);

            if (elapsed >= VA_ALIGN_MS) enter(VA_SEEK);
            break;
        }

        /* ------------------- 恢复循迹：软接管 ------------------- */
        case VA_COOL:
        {
            int pct = (int)VA_HANDOVER_PCT +
                      (int)((100 - (int)VA_HANDOVER_PCT) * (u32)elapsed / VA_HANOVER_RAMP_MS);
            if (pct > 100) pct = 100;
            Gray_SetSpeedScale((u8)pct);
            SearchRun();                    /* PID 接管，限速从 55% 平滑爬回 100% */
            if (elapsed >= VA_COOLDOWN_MS)
            {
                /* 重新武装的计时从"回到 LINE"这一刻起算：绕行期间车就在障碍旁边，
                 * 那段时间不能算"视野里没有障碍"，否则一回到 LINE 就被判重新武装，
                 * 紧接着又触发一次绕行（旧版 COOL 500ms 后就会这样死循环）。 */
                clear_start = g_tick_ms;
                enter(VA_LINE);
            }
            break;
        }

        /* ------------------- 兜底 ------------------- */
        default:
            fail(VF_GRAY);
            break;
    }
}

void VisionAvoid_Debug(void)
{
#if DEBUG_UART
    int yb = box_bottom(&target);
    printf("[vision] st=%u fault=%u link=%u found=%u x=%u yb=%u d=%ucm tr=%u slow=%u side=%u arm=%u cf=%u near=%u off=%u rx=%lu ok=%lu bad=%lu\r\n",
           (unsigned)state, (unsigned)fault,
           (unsigned)(have_packet && (u32)(g_tick_ms - target.stamp) < VA_LINK_TIMEOUT_MS),
           (unsigned)target.found, (unsigned)target.x,
           (unsigned)(yb < 0 ? 0 : yb), (unsigned)rows_to_cm(yb),
           (unsigned)row_for_cm(VA_TRIGGER_CM), (unsigned)row_for_cm(VA_SLOW_CM),
           (unsigned)side, (unsigned)armed, (unsigned)confirmed,
           (unsigned)near_cnt, (unsigned)off_cnt,
           (unsigned long)VisionLink_GetRxBytes(),
           (unsigned long)VisionLink_GetValidFrames(),
           (unsigned long)VisionLink_GetBadFrames());
#endif
}
