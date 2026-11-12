"""Compile actual parser/controller with fake hardware. Run with Python and GCC.

这个测试不是"跑仿真看效果"，而是把真实的 vision_link.c 解析段和 vision_avoid.c
状态机原样编进来，只替换电机/灰度/串口这些硬件接口，然后用断言逐条钉住状态机的
进入条件、退出条件和保护动作。改状态机必须让这里的断言同步更新。
"""
import ast
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def no_includes(s):
    return '\n'.join(line for line in s.splitlines() if not line.startswith('#include'))


def packet_function(path):
    tree = ast.parse(path.read_text(encoding='utf-8-sig'))
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'make_packet']
    ns = {'CUSTOM_MODEL_COMMAND': 0x12}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), ns)
    return ns['make_packet']


make = packet_function(ROOT/'K210配套/main.py')
original_make = packet_function(ROOT/'K210配套/通信原版.py')
for args in [(), (44, 35, 256, 187, 0, 95, 1), (120, 50, 40, 100, 0, 90, 1)]:
    p = make(*args)
    assert p == original_make(*args) and len(p) == 29
    assert p[27] == sum(p[1:27]) & 255
ast.parse((ROOT/'K210配套/main.py').read_text(encoding='utf-8'))
link = (ROOT/'SYSTEM/vision/vision_link.c').read_text(encoding='utf-8')
# Only hardware init/IRQ are excluded; parser and atomic handoff stay unchanged.
link = (link[link.index('static volatile'):link.index('void VisionLink_Init')]
        + link[link.index('uint8_t VisionLink_Read'):link.index('void USART2_IRQHandler')])
source = '''#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32;
#define DEBUG_UART 0
#define CFG_CUTOFF 0
static u32 g_tick_ms;
static int ml, mr, searching, gray_ok=1, read_ok=1, sensor=0, scale=100;
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t x){(void)x;}
static int Gray_IsReady(void){return gray_ok;}
static int Cfg_Get(int x){(void)x; return 50;}
static int Gray_ReadRaw(u8 *v){
    int i;
    if(!read_ok)return 0;
    for(i=0;i<8;i++)v[i]=100;
    if(sensor==1)v[0]=0;
    if(sensor==2)v[3]=0;
    if(sensor==3){v[3]=0;v[4]=0;}
    return 1;
}
static void Motor_SetSpeed(int16_t l,int16_t r){ml=l;mr=r;}
static void Motor_Stop(void){ml=mr=0;}
static void Gray_ResetIntegral(void){}
static void Gray_SetSpeedScale(u8 p){scale=p;}
static void SearchRun(void){searching++;ml=mr=20;}
'''
source += no_includes((ROOT/'SYSTEM/vision/vision_link.h').read_text(encoding='utf-8'))
source += '\n' + link
source += '\n' + no_includes((ROOT/'SYSTEM/vision/vision_avoid.h').read_text(encoding='utf-8'))
source += '\n' + no_includes((ROOT/'SYSTEM/vision/vision_avoid.c').read_text(encoding='utf-8'))
source += r'''
static void send_full(int found,int x,int y,int w,int h) {
    u8 p[29]={0x24,27,0x12,0xbb,22},raw[11]={0},sum=0;
    int i;
    raw[0]=x&255;raw[1]=x>>8;raw[2]=y&255;raw[3]=y>>8;
    raw[4]=w&255;raw[5]=w>>8;raw[6]=h&255;raw[7]=h>>8;
    raw[9]=5;raw[10]=found;
    for(i=0;i<11;i++){p[5+2*i]=raw[i];p[6+2*i]=44;}
    for(i=1;i<27;i++){sum+=p[i];} p[27]=sum;p[28]=35;
    for(i=0;i<29;i++){ReceiveByte(p[i]);}
}
/* 默认框：y=40、w=40，底边 yb=40+h */
static void send_box(int found,int x,int h) { send_full(found,x,40,40,h); }
/* 指定框底边行号（决定 STM32 端算出的距离）*/
static void send_yb(int found,int x,int w,int h,int yb) {
    int y=yb-h;
    /* 框高不能盖过接地点，否则 y 被钳到 0，算出来的距离就不是想要的 yb 了 */
    assert(y>=0);
    send_full(found,x,y,w,h);
}
static int last_state=-1;
static void trace_state(void) {
    if(state!=last_state){
        fprintf(stderr,"  [t=%5lu] %2d -> %2d  fault=%d scale=%3d ml=%4d mr=%4d\n",
               (unsigned long)g_tick_ms, last_state, state, fault, scale, ml, mr);
        last_state=state;
    }
}
static void tick(int found,int x,int h) {
    g_tick_ms+=10;
    if(g_tick_ms%100==0)send_box(found,x,h);
    VisionAvoid_Update();
    trace_state();
}
static void tick_yb(int found,int x,int w,int h,int yb) {
    g_tick_ms+=10;
    if(g_tick_ms%100==0)send_yb(found,x,w,h,yb);
    VisionAvoid_Update();
    trace_state();
}
static void reset(void) {
    frame_count=message_ready=0;g_tick_ms=byte_stamp=0;
    sensor=2;gray_ok=1;read_ok=1;scale=100;searching=0;
    VisionAvoid_Start();
    last_state=-1;
}
/* 跑到 TURN_OUT（提交绕行的时刻）*/
static void to_turn_out(void) {
    int i;
    reset();
    for(i=0;i<60 && state!=VA_SLOW;i++){tick_yb(1,120,40,40,72);}   /* 30cm：进减速 */
    for(i=0;i<60 && state!=VA_TURN_OUT;i++){tick_yb(1,120,40,40,115);} /* 18cm：提交 */
    assert(state==VA_TURN_OUT);
}
int main(void) {
    K210_MSG_TypeDef msg;
    int i;
    u8 golden[29]=GOLDEN_PACKET;
    int s0;

    /* ---------------- 解析层 ---------------- */
    for(i=0;i<29;i++){ReceiveByte(golden[i]);}
    assert(VisionLink_Read(&msg) && msg.x==44 && msg.y==35 && msg.w==256 && msg.h==187);
    assert(!VisionLink_Read(&msg));
    golden[27]^=1;
    for(i=0;i<29;i++){ReceiveByte(golden[i]);}
    assert(!VisionLink_Read(&msg));
    send_box(1,120,100);assert(VisionLink_Read(&msg));
    send_box(1,310,100);assert(!VisionLink_Read(&msg));
    send_box(0,0,0);assert(VisionLink_Read(&msg) && !msg.found);
    ReceiveByte(0x24);ReceiveByte(27);g_tick_ms+=110;
    send_box(1,120,100);assert(VisionLink_Read(&msg));

    /* ---------------- 距离模型 ---------------- */
    assert(row_for_cm(10u)==VA_ROW_AT_10CM);
    assert(row_for_cm(20u)==VA_ROW_AT_20CM);
    assert(rows_to_cm(VA_ROW_AT_10CM)==10u && rows_to_cm(VA_ROW_AT_20CM)==20u);
    assert(rows_to_cm(239)<=10u && rows_to_cm(60)>25u);
    assert(row_for_cm(0u)>239);
    /* 30cm 的行号必须比 18cm 的行号小（越远越靠上） */
    assert(row_for_cm(VA_SLOW_CM) < row_for_cm(VA_TRIGGER_CM));

    /* ---------------- 无目标：正常循迹 ---------------- */
    reset();for(i=0;i<30;i++){tick(0,0,0);}
    assert(state==VA_LINE && searching>0 && scale==100);

    /* ---------------- 远处大框不得触发（旧版"看到框就绕"会在这里立刻开始） ---------------- */
    reset();for(i=0;i<60;i++){tick_yb(1,20,280,40,60);}
    assert(state==VA_LINE && confirmed==0);

    /* ---------------- 完整绕行链路 ---------------- */
    /* 30cm：进减速段，仍然由循迹 PID 驱动 */
    reset();
    for(i=0;i<40 && state!=VA_SLOW;i++){tick_yb(1,120,40,40,72);}
    assert(state==VA_SLOW && scale==VA_SLOW_PCT && searching>0);
    /* 18cm：连续2帧进触发距离 → 提交绕行（cx=140 居中，默认左绕） */
    for(i=0;i<40 && state!=VA_TURN_OUT;i++){tick_yb(1,120,40,40,115);}
    assert(state==VA_TURN_OUT && side==1);
    /* 转出：左转（左轮负/右轮正），且第一拍受斜率限制，不是阶跃 */
    tick(1,120,100);
    assert(ml<0 && mr>0 && (mr-ml)<=2*VA_SLEW_STEP+2);

    for(i=0;i<40 && state==VA_TURN_OUT;i++){tick(1,120,100);}
    assert(state==VA_SHIFT);
    /* 侧移段：弧线（左右不等速），且不再调用 SearchRun —— 控制权唯一 */
    s0=searching;
    for(i=0;i<20 && state==VA_SHIFT;i++){tick(1,120,100);}
    assert(state==VA_SHIFT && searching==s0 && ml!=mr);
    /* 灰度连续3拍看不到线 + 到了时间下限 → 回正 */
    sensor=0;
    for(i=0;i<200 && state==VA_SHIFT;i++){tick(0,0,0);}
    assert(state==VA_STRAIGHTEN);
    /* 回正 → 越障（时间下限 + 目标离开视野）→ 找线 */
    for(i=0;i<400 && state!=VA_SEEK;i++){tick(0,0,0);}
    assert(state==VA_SEEK);
    /* 找线是向内弧线：左绕之后左轮快、右轮慢 */
    for(i=0;i<15;i++){tick(0,0,0);}
    assert(ml>mr);
    /* 线出现在阵列下 → 对正 */
    sensor=3;
    for(i=0;i<10 && state==VA_SEEK;i++){tick(0,0,0);}
    assert(state==VA_ALIGN);
    /* 重心进死区 → 交回 PID，先降到 55% */
    for(i=0;i<10 && state==VA_ALIGN;i++){tick(0,0,0);}
    assert(state==VA_COOL && scale==VA_HANDOVER_PCT);
    for(i=0;i<80 && state==VA_COOL;i++){tick(0,0,0);}
    assert(state==VA_LINE && scale==100);

    /* ---------------- 重触发抑制：刚绕完不能立刻再绕 ---------------- */
    for(i=0;i<50;i++){tick_yb(1,120,40,40,115);}   /* 障碍还在旁边 500ms */
    assert(state==VA_LINE);
    for(i=0;i<130;i++){tick(0,0,0);}                /* 视野干净 1.3s 才重新武装 */
    assert(armed==1);
    for(i=0;i<40 && state!=VA_SLOW;i++){tick_yb(1,120,40,40,115);}
    assert(state==VA_SLOW);

    /* ---------------- 两侧都堵：安全停车 ---------------- */
    reset();
    for(i=0;i<40 && fault==VF_NONE;i++){tick_yb(1,20,280,40,115);}
    assert(fault==VF_BLOCKED && ml==0 && mr==0);

    /* ---------------- 绕行途中视觉失联：停车 ---------------- */
    to_turn_out();
    g_tick_ms+=VA_LINK_TIMEOUT_MS;VisionAvoid_Update();
    assert(state==VA_FAULT && fault==VF_LINK && ml==0 && mr==0);

    /* ---------------- 灰度单次读失败不能锁死，连续失败才停车 ---------------- */
    to_turn_out();
    for(i=0;i<40 && state==VA_TURN_OUT;i++){tick(0,0,0);}
    assert(state==VA_SHIFT);
    read_ok=0;
    for(i=0;i<(int)VA_GRAY_FAIL_MAX-1;i++){tick(0,0,0);}
    assert(fault==VF_NONE);
    read_ok=1;tick(0,0,0);
    assert(fault==VF_NONE);
    read_ok=0;
    for(i=0;i<(int)VA_GRAY_FAIL_MAX+2 && fault==VF_NONE;i++){tick(0,0,0);}
    assert(fault==VF_GRAY && ml==0 && mr==0);

    /* ---------------- 找不到线：超时安全停车，不死循环 ---------------- */
    to_turn_out();
    sensor=0;
    for(i=0;i<800 && fault==VF_NONE;i++){tick(0,0,0);}
    assert(fault==VF_SEEK && ml==0 && mr==0);

    /* ---------------- Stop ---------------- */
    VisionAvoid_Stop();
    assert(ml==0 && mr==0 && state==VA_WAIT && scale==100);

    puts("PASS: distance model, decel->turn->shift->pass->seek->align->recover, retrigger latch, block/link/gray/seek faults");
    return 0;
}
'''
source = source.replace('GOLDEN_PACKET', '{'+','.join(map(str,make(44,35,256,187,0,95,1)))+'}')
gcc = os.environ.get('GCC') or shutil.which('gcc') or 'D:/C语言/CodeBlocks/MinGW/bin/gcc.exe'
os.environ['PATH'] = str(Path(gcc).parent) + os.pathsep + os.environ.get('PATH', '')
with tempfile.TemporaryDirectory(prefix='vision_test_') as folder:
    src = Path(folder)/'test.c'
    exe = Path(folder)/'test.exe'
    src.write_text(source, encoding='utf-8')
    subprocess.run([gcc, '-std=c99', '-Wall', '-Wextra', str(src), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('PASS: Python syntax and original sender packet compatibility')
