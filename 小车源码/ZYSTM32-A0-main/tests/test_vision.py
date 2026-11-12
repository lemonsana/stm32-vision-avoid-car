"""Compile actual parser/controller with fake hardware. Run with Python and GCC."""
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
static int ml, mr, searching, gray_ok=1, sensor=0;
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t x){(void)x;}
static int Gray_IsReady(void){return gray_ok;}
static int Cfg_Get(int x){(void)x; return 50;}
static int Gray_ReadRaw(u8 *v){int i; if(!gray_ok)return 0; for(i=0;i<8;i++)v[i]=100; if(sensor==1)v[0]=0; if(sensor==2)v[3]=0; return 1;}
static void Motor_SetSpeed(int16_t l,int16_t r){ml=l;mr=r;}
static void Motor_Stop(void){ml=mr=0;}
static void Gray_ResetIntegral(void){}
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
/* 默认框：y=40、w=40，底边 yb=40+h，用于"看得见但还远"的用例 */
static void send_box(int found,int x,int h) { send_full(found,x,40,40,h); }
/* 指定框底边行号的发送，yb 决定 STM32 端算出的距离 */
static void send_row(int found,int x,int h,int yb) {
    int y=yb-h; if(y<0)y=0; send_full(found,x,y,40,h);
}
static void tick(int found,int x,int h) {
    g_tick_ms+=10;
    if(g_tick_ms%100==0)send_box(found,x,h);
    VisionAvoid_Update();
}
static void tick_row(int found,int x,int h,int yb) {
    g_tick_ms+=10;
    if(g_tick_ms%100==0)send_row(found,x,h,yb);
    VisionAvoid_Update();
}
static void reset(void) {
    frame_count=message_ready=0;g_tick_ms=byte_stamp=0;sensor=2;gray_ok=1;
    VisionAvoid_Start();
}
static void trigger(void) {
    int i; reset();
    for(i=0;i<50 && state!=VA_OUT;i++){tick_row(1,120,100,220);}
    assert(state==VA_OUT && ml==0 && mr==0);
}
int main(void) {
    K210_MSG_TypeDef msg;
    int i;
    u8 golden[29]=GOLDEN_PACKET;
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
    reset();for(i=0;i<30;i++){tick(1,0,100);}assert(state==VA_LINE);
    /* 距离模型自检：两点标定与触发行反算 */
    assert(row_for_cm(VA_TRIGGER_CM)==VA_ROW_AT_10CM);
    assert(row_for_cm(20u)==VA_ROW_AT_20CM);
    assert(rows_to_cm(VA_ROW_AT_10CM)==10u && rows_to_cm(VA_ROW_AT_20CM)==20u);
    assert(rows_to_cm(239)<=10u && rows_to_cm(60)>25u);
    assert(row_for_cm(0u)>239);
    /* 距离门限：远处的大框不再触发（旧版这里立刻开始绕行），这就是"避障开始过早"的根因 */
    reset();for(i=0;i<60;i++){g_tick_ms+=10;
        if(g_tick_ms%100==0){send_full(1,20,40,280,120);}VisionAvoid_Update();}
    assert(state==VA_LINE && confirmed==0);
    /* 框底边压到 10cm 触发行才允许触发 */
    reset();for(i=0;i<60 && state!=VA_OUT;i++){tick_row(1,120,120,VA_ROW_AT_10CM);}
    assert(state==VA_OUT);
    /* 时延补偿：两帧都还在触发行之外，按靠近速率外推后第二帧就算命中 */
    reset();
    send_row(1,120,140,180);VisionAvoid_Update();assert(confirmed==0&&state==VA_LINE);
    g_tick_ms+=100;send_row(1,120,140,196);VisionAvoid_Update();
    assert(confirmed==1&&state==VA_LINE);
    g_tick_ms+=100;send_row(1,120,140,205);VisionAvoid_Update();assert(state==VA_OUT);
    /* 滑窗确认：命中-漏检-命中 仍触发；单个孤立命中 3 帧后被挤出窗口 */
    reset();
    send_row(1,120,100,220);VisionAvoid_Update();assert(confirmed==1&&state==VA_LINE);
    send_box(0,0,0);VisionAvoid_Update();assert(confirmed==1&&state==VA_LINE);
    send_row(1,120,100,220);VisionAvoid_Update();assert(state==VA_OUT);
    reset();
    send_row(1,120,100,220);VisionAvoid_Update();
    send_box(0,0,0);VisionAvoid_Update();
    send_box(0,0,0);VisionAvoid_Update();assert(confirmed==1&&state==VA_LINE);
    send_box(0,0,0);VisionAvoid_Update();assert(confirmed==0&&state==VA_LINE);
    g_tick_ms+=VA_LINK_TIMEOUT_MS;VisionAvoid_Update();
    assert(state==VA_WAIT && fault==VF_NONE && ml==20 && mr==20);
    send_box(0,0,0);VisionAvoid_Update();assert(state==VA_LINE);
    reset();for(i=0;i<200;i++){g_tick_ms+=10;VisionAvoid_Update();}
    assert(state==VA_WAIT && ml==20 && mr==20);
    send_box(0,0,0);VisionAvoid_Update();assert(state==VA_LINE);
    trigger();g_tick_ms+=VA_LINK_TIMEOUT_MS;VisionAvoid_Update();
    assert(state==VA_FAULT && fault==VF_LINK && ml==0 && mr==0);
    trigger();sensor=0;
    for(i=0;i<400 && state!=VA_SEEK;i++){tick(0,0,0);}
    assert(state==VA_SEEK && left_line);
    sensor=1;for(i=0;i<3;i++){tick(0,0,0);}assert(state==VA_ALIGN);
    sensor=2;for(i=0;i<200 && state!=VA_COOL;i++){tick(0,0,0);}assert(state==VA_COOL);
    for(i=0;i<60;i++){tick(0,0,0);}assert(state==VA_LINE);
    VisionAvoid_Stop();assert(ml==0 && mr==0);
    trigger();
    for(i=0;i<500 && state!=VA_COOL && state!=VA_FAULT;i++){tick(0,0,0);}
    assert(fault!=VF_NOT_LEFT);
    trigger();sensor=0;
    for(i=0;i<1000 && state!=VA_FAULT;i++){tick(0,0,0);}
    assert(fault==VF_SEEK && ml==0 && mr==0);
    trigger();sensor=0;
    for(i=0;i<400 && state!=VA_SEEK;i++){tick(0,0,0);}
    sensor=1;for(i=0;i<3;i++){tick(0,0,0);}
    for(i=0;i<200 && state!=VA_FAULT;i++){tick(0,0,0);}assert(fault==VF_ALIGN);
    trigger();gray_ok=0;tick(0,0,0);assert(fault==VF_GRAY && ml==0 && mr==0);
    puts("PASS: checksum/resync/bounds, filtering, sliding-window confirm, full bypass, timeout/gray faults, stop");
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
