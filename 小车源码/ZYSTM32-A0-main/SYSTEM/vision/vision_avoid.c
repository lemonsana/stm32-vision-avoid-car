#include "vision_avoid.h"
#include "vision_link.h"
#include "sched.h"
#include "IRSEARCH.h"
#include "motor.h"
#include "cfg.h"
#include "app_mode.h"
#include <stdio.h>

enum { VA_WAIT, VA_LINE, VA_OUT, VA_SHIFT, VA_STRAIGHTEN, VA_PASS,
       VA_IN, VA_SEEK, VA_ALIGN, VA_COOL, VA_FAULT };
enum { VF_NONE, VF_LINK, VF_GRAY, VF_NOT_LEFT, VF_SEEK, VF_ALIGN };
static u8 state, fault, confirmed, line_count, left_line, off_count, have_packet;
static u8 hit_hist;                 /* 最近 3 个新检测帧的命中位，bit0=最新 */
static int prev_yb;                 /* 上一帧目标框底边行号，用于算靠近速率 */
static u32 prev_stamp;
static u8 have_prev;
static u32 entered;
static K210_MSG_TypeDef target;

static void enter(u8 next)
{
    state = next;
    entered = g_tick_ms;
    line_count = 0;
    hit_hist = 0;
    have_prev = 0;
}

static void fail(u8 reason)
{
    fault = reason;
    enter(VA_FAULT);
    Motor_Stop();
    Gray_ResetIntegral();
}

/* dir=1 means outward, dir=-1 means toward the original line. */
static void turn(int dir, int speed)
{
    int sign = VA_LEFT ? dir : -dir;
    Motor_SetSpeed((int16_t)(-sign * speed), (int16_t)(sign * speed));
}

/* A failed I2C read must never count as having left the line. */
static u8 read_line(u8 *any, u8 *center)
{
    u8 v[8], i;
    int cutoff = Cfg_Get(CFG_CUTOFF);
    if (!Gray_IsReady() || !Gray_ReadRaw(v)) return 0;
    *any = 0;
    for (i = 0; i < 8; i++) if (v[i] < cutoff) *any = 1;
    *center = (v[3] < cutoff || v[4] < cutoff) ? 1u : 0u;
    return 1;
}

/* ---------- 距离：框底边行号 -> 厘米 ----------
 * 贴地障碍物的框底边 yb=y+h 就是接地点，越近越靠下。1/d 对 yb 线性，
 * 用 10cm/20cm 两个标定点插值，标定点之外按同一直线外推。
 * 全部整数运算：把 1/d 放大 10000 倍，避免在 ARMCC 上引入浮点库。 */
#define VA_ROW_SPAN (((int)VA_ROW_AT_20CM) - ((int)VA_ROW_AT_10CM))  /* 20cm 行号减 10cm 行号，正常为负数 */

static int rows_to_inv10k(int yb)
{
    if (VA_ROW_SPAN == 0) return 1000;      /* 两个标定点写重合了：退化成固定 10cm */
    return 1000 - (int)((long)(yb - (int)VA_ROW_AT_10CM) * 500L / VA_ROW_SPAN);
}

/* 行号 -> 距离(cm)，只用于调试打印。 */
static u16 rows_to_cm(int yb)
{
    int inv = rows_to_inv10k(yb);
    if (inv <= 0) return 9999u;
    return (u16)(10000 / inv);
}

/* 距离 cm 对应的触发行号：d <= cm 等价于 yb >= row_for_cm(cm)。 */
static int row_for_cm(u16 cm)
{
    if (cm == 0u) return 0x7FFF;            /* 0 = 关闭门限 */
    return (int)VA_ROW_AT_10CM +
           (int)((long)(1000 - 10000 / (int)cm) * VA_ROW_SPAN / 500L);
}

/* 框底边：贴地障碍物的接地点。K210 已把 y+h 钳在 239 以内。 */
static int box_bottom(const K210_MSG_TypeDef *m)
{
    int yb = (int)m->y + (int)m->h;
    return yb > 239 ? 239 : yb;
}

/* 用最近两帧底边行号的变化率，把 yb 往前推 VA_PREDICT_MS 毫秒再判定，
 * 补偿 一帧曝光 + 卡尔曼滞后 + 串口 的时延；只在确实在靠近时外推。 */
static int predict_row(int yb, u32 stamp)
{
    long dt, step, pred;
    if (VA_PREDICT_MS == 0u || !have_prev) return yb;
    dt = (long)(u32)(stamp - prev_stamp);
    step = (long)yb - (long)prev_yb;
    if (dt < 10 || dt > 500 || step <= 0) return yb;
    pred = (long)yb + step * (long)VA_PREDICT_MS / dt;
    return pred > 239 ? 239 : (int)pred;
}

/* 一个检测帧是否算命中：类别对、found=1、框与前方通道有重叠。 */
static u8 is_target(const K210_MSG_TypeDef *m)
{
    return (u8)(m->found && m->id == VA_CLASS_ID &&
                m->x < VA_ROI_RIGHT && (u32)m->x + m->w > VA_ROI_LEFT);
}

/* 是否已经进入触发距离：本次新增的距离门限，替代"看到框就绕"。 */
static u8 is_near(const K210_MSG_TypeDef *m)
{
    int yb;
    if (!is_target(m)) return 0;
    if (VA_TRIGGER_CM == 0u) return 1;      /* 门限关闭：恢复旧行为 */
    yb = box_bottom(m);
    return (u8)(predict_row(yb, m->stamp) >= row_for_cm(VA_TRIGGER_CM));
}


void VisionAvoid_Start(void)
{
    K210_MSG_TypeDef discard;
    VisionLink_Read(&discard); /* Require a new heartbeat after entering mode 5. */
    confirmed = line_count = left_line = off_count = have_packet = fault = 0;
    hit_hist = 0;
    target.found = 0;
    enter(VA_WAIT);
    Motor_Stop();
    Gray_ResetIntegral();
}

void VisionAvoid_Stop(void)
{
    enter(VA_WAIT);
    confirmed = have_packet = 0;
    hit_hist = 0;
    Motor_Stop();
    Gray_ResetIntegral();
}

void VisionAvoid_Update(void)
{
    u8 fresh, any = 0, center = 0;
    u32 elapsed;
    fresh = VisionLink_Read(&target);
    if (fresh) have_packet = 1;
    if (state == VA_FAULT) { Motor_Stop(); return; }

    if (!have_packet)
    {
        /* 视觉只负责触发避障；K210未就绪时保留原灰度循迹功能。 */
        SearchRun();
        return;
    }
    if ((u32)(g_tick_ms - target.stamp) >= VA_LINK_TIMEOUT_MS)
    {
        if (state == VA_WAIT || state == VA_LINE || state == VA_COOL)
        {
            /* 尚未离开轨道：退化成纯灰度循迹，等待视觉链路恢复。 */
            have_packet = 0;
            confirmed = 0;
            target.found = 0;
            enter(VA_WAIT);
            SearchRun();
        }
        else
        {
            /* 已经开始绕行：中途失联时停车，避免盲走。 */
            fail(VF_LINK);
        }
        return;
    }
    if (!Gray_IsReady()) { fail(VF_GRAY); return; }
    if (state == VA_WAIT) enter(VA_LINE);
    elapsed = (u32)(g_tick_ms - entered);

    if (state >= VA_OUT && state <= VA_ALIGN)
    {
        if (!read_line(&any, &center)) { fail(VF_GRAY); return; }
        if (!any) { if (off_count < VA_LINE_CONFIRM) off_count++; }
        else off_count = 0;
        if (off_count >= VA_LINE_CONFIRM) left_line = 1;
    }

    switch (state)
    {
        case VA_LINE:
            if (fresh)
            {
                /* 滑窗确认：命中=类别对+在前方通道内+估算距离已进入 VA_TRIGGER_CM。
                 * 单帧漏检只把命中位挤出窗口，不再一票否决清零。 */
                u8 i, hits = 0, hit = is_near(&target);
                hit_hist = (u8)(((hit_hist << 1) | hit) &
                                (u8)((1u << VA_CONFIRM_WINDOW) - 1u));
                for (i = 0; i < VA_CONFIRM_WINDOW; ++i)
                    hits = (u8)(hits + ((hit_hist >> i) & 1u));
                confirmed = hits;
                /* 记录本帧底边，供下一帧算靠近速率；非目标帧断开历史。 */
                if (is_target(&target))
                {
                    prev_yb = box_bottom(&target);
                    prev_stamp = target.stamp;
                    have_prev = 1;
                }
                else have_prev = 0;
            }
            if (confirmed >= VA_CONFIRM_HITS)
            {
                confirmed = 0;
                left_line = off_count = 0;
                Motor_Stop();
                Gray_ResetIntegral();
                enter(VA_OUT);
            }
            else SearchRun();
            break;
        case VA_OUT:
            turn(1, VA_TURN_SPEED);
            if (elapsed >= VA_TURN_OUT_MS) { Motor_Stop(); enter(VA_SHIFT); }
            break;
        case VA_SHIFT:
            Motor_SetSpeed(VA_SPEED, VA_SPEED);
            if (elapsed >= VA_SHIFT_MS) { Motor_Stop(); enter(VA_STRAIGHTEN); }
            break;
        case VA_STRAIGHTEN:
            turn(-1, VA_TURN_SPEED);
            if (elapsed >= VA_STRAIGHTEN_MS) { Motor_Stop(); enter(VA_PASS); }
            break;
        case VA_PASS:
            Motor_SetSpeed(VA_SPEED, VA_SPEED);
            if (elapsed >= VA_PASS_MS)
            {
                Motor_Stop();
                /* 离线距离由实车调整VA_TURN_OUT_MS/VA_SHIFT_MS；
                 * 不因灰度未出现全白而锁死整个定时绕行。 */
                enter(VA_IN);
            }
            break;
        case VA_IN:
            turn(-1, VA_TURN_SPEED);
            if (elapsed >= VA_TURN_IN_MS) { Motor_Stop(); enter(VA_SEEK); }
            break;
        case VA_SEEK:
            if (any) { if (line_count < VA_LINE_CONFIRM) line_count++; }
            else line_count = 0;
            if (line_count >= VA_LINE_CONFIRM)
            {
                Motor_Stop();
                enter(VA_ALIGN);
            }
            else if (elapsed >= VA_SEEK_TIMEOUT_MS) fail(VF_SEEK);
            else Motor_SetSpeed(VA_SEEK_SPEED, VA_SEEK_SPEED);
            break;
        case VA_ALIGN:
            if (elapsed >= VA_ALIGN_MIN_MS && center)
            { if (line_count < VA_LINE_CONFIRM) line_count++; }
            else line_count = 0;
            if (line_count >= VA_LINE_CONFIRM)
            {
                Motor_Stop();
                Gray_ResetIntegral();
                enter(VA_COOL);
            }
            else if (elapsed >= VA_ALIGN_TIMEOUT_MS) fail(VF_ALIGN);
            else turn(1, VA_ALIGN_SPEED);
            break;
        case VA_COOL:
            SearchRun();
            if (elapsed >= VA_COOLDOWN_MS) { confirmed = 0; enter(VA_LINE); }
            break;
        default:
            fail(VF_GRAY);
            break;
    }
}

void VisionAvoid_Debug(void)
{
#if DEBUG_UART
    int yb = box_bottom(&target);
    printf("[vision] st=%u fault=%u link=%u found=%u x=%u yb=%u d=%ucm tr=%u confirm=%u left=%u rx=%lu ok=%lu bad=%lu\r\n",
           (unsigned)state, (unsigned)fault,
           (unsigned)(have_packet && (u32)(g_tick_ms-target.stamp)<VA_LINK_TIMEOUT_MS),
           (unsigned)target.found, (unsigned)target.x,
           (unsigned)(yb < 0 ? 0 : yb), (unsigned)rows_to_cm(yb),
           (unsigned)row_for_cm(VA_TRIGGER_CM),
           (unsigned)confirmed, (unsigned)left_line,
           (unsigned long)VisionLink_GetRxBytes(),
           (unsigned long)VisionLink_GetValidFrames(),
           (unsigned long)VisionLink_GetBadFrames());
#endif
}
