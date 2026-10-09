#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include <math.h>     // fmaxf/fabsf/sqrtf（go 指令与串口速度通道）
#include "serialplot.h"
#include "oled_ui/oled.h"
#include "pid.h"
#include "oled_api.h"
#include "uart.h"
#include "chassis.h"
#include "hwt101ct.h"
#include "tb6612.h"   // 单轮调参启动时要把其余三轮 PWM 清零
#include "robot.h"    // at 指令：直接走一遍 ROBOT_Angle（验证停止档退出流程）

/* ★圆周运动（绕柱）以本工程 Core/Src/main.c 的 LiZhu_Circle_Run() 为准
   （陀螺仪累积转角 + GY-53 实时测距 + 径向闭环 + w 前馈/KD_W 修正那套）。
   队友那版引用的 circle.c/circle.h（cstage/ckp/cki/ckd/cvy/cyawkp/calpha/cstep/cdriftper/cprint
   十个参数项，是旧的圆周方案）**本工程不采用、也没有这两个文件**，
   故相关 include 与 10 个参数项已整块删除，参数表只留本工程实际用得到的 9 项。 */
#define PARAM_Number 9              //参数个数（航向环 + 整车速度 + 规划：kp/ki/kd/target/vx/vy/w/mv/mvacc）
#define YAW_Loop  chassis.yaw_pid   //要调参的pid环：整车航向环
/* 死区无需在线调：控制循环按是否平移自动切换 —— 静止旋转 YAW_DEAD_ZONE_TURN 防来回飘，
   走直线(有平移) YAW_DEAD_ZONE_MOVE 让1°内偏航也被纠正（平移档低/高速共用这一套）。
   ★ 值以 chassis.h 的宏为准（实测反复微调过），不在这里写死数字。 */

typedef struct{
  void *p;        //某一个参数变量的地址
  char *name;     //该参数变量的名字
  uint8_t type;   //数据类型：0=float（默认），1=int（写入 4 字节，仅限 int 型字段！）
}Param;

Param param[PARAM_Number] = { //可以修改的变量列表（名字匹配后按名字分发）
  /* ★ 下面 kp/ki/kd 三项全部失效（2026-09-19 起 kp/ki/kd/bias 都分三档了）：控制循环每 20ms
     都会按工况把该档整套参数写进 yaw_pid 的 kp/ki/kd，发这三条下一周期就被冲掉，调不动。
     请改用 ykp1/2/3、yki1/2/3、ykd1/2/3、ybias1/2/3（档号 1旋转 2低速 3高速，单独分发）。*/
  {&YAW_Loop.kp, "kp", 0},        // ★ 已失效 → ykp1/ykp2/ykp3
  {&YAW_Loop.ki, "ki", 0},        // ★ 已失效 → yki1/yki2/yki3
  {&YAW_Loop.kd, "kd", 0},        // ★ 已失效 → ykd1/ykd2/ykd3
  {&chassis.target_yaw, "target", 0}, // 航向环目标角度（0~360，遥控转向）
  {&chassis.v_x, "vx", 0},        // 整车x速度 cm/s（右移为正；手动设速模式，见 ChangeParam）
  {&chassis.v_y, "vy", 0},        // 整车y速度 cm/s（前进为正；手动设速模式）
  {&chassis.w,   "w", 0},         // 整车角速度 rad/s（航向环开启时被角度环接管，需 flag.angle=0 才直接生效）
  {&chassis.move_speed, "mv", 0},   // 梯形规划目标速度 cm/s（mx/my 走固定距离用，默认20）
  {&chassis.move_acc,   "mvacc", 0},// 梯形规划加减速 cm/s²（默认30）
};

/* ==================== 高速平移档调参用的长距直行动作（2026-09-19 加） ====================
   串口发 "go f <速度>" / "gb f <速度>" 就跑一次（速度传 0 就用默认 120）：
     go → CHASSIS_Start_Move(HIMOVE_X_DIST,      HIMOVE_Y_DIST,      spd, spd, spd, spd)
          ≡ ROBOT_Move(-60, 410, 120, 120, 120, 120)
     gb → CHASSIS_Start_Move(HIMOVE_BACK_X_DIST, HIMOVE_BACK_Y_DIST, spd, spd, spd, spd)
          ≡ ROBOT_Move(0, -410, 120, 120, 120, 120)
   两条都**不阻塞**（原因见 go 指令处的注释）。*/
#define HIMOVE_X_DIST   (-60.0f)   // x 距离 cm（负 = 左移）
#define HIMOVE_Y_DIST   410.0f     // y 距离 cm（正 = 前进）
#define HIMOVE_SPEED    120.0f     // 默认速度/加速度 cm/s（120 > YAW_KP_LOW_SPEED_BOUND(80) ⇒ 高速档）
/* 第二组：同速度、纯后退（2026-09-19 加，串口 gb 触发） */
#define HIMOVE_BACK_X_DIST   0.0f      // x 距离 cm（0 = 不横移）
#define HIMOVE_BACK_Y_DIST   (-410.0f) // y 距离 cm（负 = 后退）

/**
  * @brief 接收串口绘图软件发送的参数修改指令，为快速调参PID而生
  * @param string 接收到的字符串指令 格式:"name type num "，其中name为变量名字，type为数据类型：f/i，num为需要修改的数值
  * @note   指令示例：
  *         "kp f -0.028"  调航向环参数；"target f 90" 遥控转向；"vx f 30" 手动x横移（持续）
  *         "mx f 30"      x方向平移30cm自动停；"my f -50" y方向后退50cm自动停（梯形速度规划）
  * @attention 禁止传入字符串，本函数会对传入字符数组进行修改！！！
  */
void SERIALPLOT_ChangeParam(char *string){
  char *str_name, *str_type, *str_num;
  char *str_temp;

  /*先数空格：指令必须 2 个空格分隔 3 段（name type num），不足直接忽略。
    ！！！原实现找第3个空格时 strchr 返回 NULL 后 *str_temp='\0' 写地址0，会崩溃，已修*/
  str_temp = string;
  int sp = 0;
  while((str_temp = strchr(str_temp, ' ')) != NULL){ sp++; str_temp++; }
  if(sp < 2) return;

  str_name = string;
  str_temp = string;
  for(int i=0; i <= 1; i++){
    str_temp = strchr(str_temp, ' ');
    *str_temp = '\0';
    str_temp++;
    if(i == 0) str_type  = str_temp;
    else if(i == 1) str_num = str_temp;
  }

  char *end_ptr;
  float val = 0.0f;
  switch(*str_type){//根据指定的数据类型转换数值（PID参数均为float）
    case 'f': //数值为浮点数
      val = strtof(str_num, &end_ptr);
      break;
    case 'i': //数值为整数
      val = (float)strtol(str_num, &end_ptr, 0);
      break;
    default:
      break;
  }

  /* 梯形规划指令（mx/my 不在 param 表，单独分发）：mx f 30 → x方向平移30cm自动停；my 同理
     速度/加速度用 param 表默认值 mv/mvacc（距离0的轴速度/加速度传0即可，不参与规划） */
  if(strcmp(str_name, "mx") == 0){ CHASSIS_Start_Move(val, 0.0f, chassis.move_speed, 0.0f, chassis.move_acc, 0.0f); return; }
  if(strcmp(str_name, "my") == 0){ CHASSIS_Start_Move(0.0f, val, 0.0f, chassis.move_speed, 0.0f, chassis.move_acc); return; }

  /* ★ 高速平移档调参专用（2026-09-19 加）：发一条就跑一次长距直行，参数见文件上方 HIMOVE_*
     go f 0    → 用默认 120 跑
     go f 100  → 速度/加速度改用 100（val > 0 时生效）—— 顺便能对比"高速档 vs 低速档"的纠偏差别
     ★ 和 mx/my 一样只调 CHASSIS_Start_Move、**非阻塞**（不等规划结束、不等停稳）。
       这不是偷懒：ROBOT_Move 会阻塞到四轮停稳才返回，那期间主循环一个数都发不出来 ——
       而调角度环参数恰恰要看**跑动全程**的曲线，阻塞了就什么都看不见。
       （ROBOT_Move 里除 CHASSIS_Start_Move 外还有"等规划结束 + 等停稳"两段，这里都不要；
         所以触发前请自己确认车是停着的。）
     ★ 角度环参数在 chassis.yaw_param[YAW_STAGE_MOVE_HI]，串口 ykp3/yki3/ykd3/ybias3 在线调 */
  if(strcmp(str_name, "go") == 0){
    float spd = (val > 0.0f) ? val : HIMOVE_SPEED;
    CHASSIS_Start_Move(HIMOVE_X_DIST, HIMOVE_Y_DIST, spd, spd, spd, spd);
    return;
  }

  /* ★ 同上的后退版（2026-09-19 加）：纯后退长距直行，参数见文件上方 HIMOVE_BACK_*
     gb f 0    → CHASSIS_Start_Move(0, -410, 120, 120, 120, 120)  即 ROBOT_Move(0, -410, 120,...)
     gb f 100  → 速度/加速度改用 100
     同样非阻塞（不等规划结束、不等停稳），触发前请确认车是停着的。
     ★ 后退工况值得单独测：减速段末尾 v 归零后落旋转档，车头若没对正会自己转，方向与前进相反。 */
  if(strcmp(str_name, "gb") == 0){
    float spd = (val > 0.0f) ? val : HIMOVE_SPEED;
    CHASSIS_Start_Move(HIMOVE_BACK_X_DIST, HIMOVE_BACK_Y_DIST, spd, spd, spd, spd);
    return;
  }

  /* 单轮速度环临时调参指令（不在 param 表，单独分发；用法见 SERIALPLOT_SpeedTuneLoop）：
     只改 chassis.tune_* 这几个临时量，绝不触碰 SPEED_PID_DFT 表 —— 试出合适的值后自己填表 */
  if(strcmp(str_name, "tspd") == 0){ chassis.speed_pid[chassis.tune_wheel].target = val; return; }  // 被调轮目标速度 cm/s
  if(strcmp(str_name, "tkp")  == 0){ chassis.tune_kp = val; return; }                               // 临时 kp
  if(strcmp(str_name, "tki")  == 0){ chassis.tune_ki = val; return; }                               // 临时 ki
  if(strcmp(str_name, "tkd")  == 0){ chassis.tune_kd = val; return; }                               // 临时 kd

  /* 航向环开关（不在 param 表，单独分发；手动给 vx/vy 看四轮一致性时用）：
     ang i 0 → 关角度环（w 恒为0，四轮 target 严格相等）   ang i 1 → 开回来（角度环接管 w 纠偏）。
     flag 定义在 chassis.h，此处已 include */
  if(strcmp(str_name, "ang") == 0){ flag.angle = (uint8_t)val; return; }

  /* ★ 短距精细档（编码器位置闭环判停）在线开关 / 标定 — 2026-10-07 加（字段说明见 chassis.h）
     背景：chassis.fine_max_dist 与 chassis.brake_decel 原先全工程无人赋值（恒为 0）⇒
           精细档判停从未生效、ROBOT_Move 一直是纯时间开环。默认值已在 CHASSIS_Init 里补上，
           这两条指令用来**现场试验和标定**：
     sdist f 0     → 关掉精细档（回到"纯时间开环"，所有 Move 走常规档）—— **上电默认就是这个**
     sdist f 10    → 两轴距离都 ≤10cm 的 Move 走精细档（★先在整图上验证过的档位再上，见下）
     brkd  f 100   → 断电自然滑停等效减速度 cm/s²（默认 = BRAKE_DECEL_DFT）
     ★改完**下一次调用 CHASSIS_Start_Move / ROBOT_Move 时生效**（fine_move 是在 Start_Move 里现算的），
       不需要复位，也不需要重发正在跑的那一段。
     ★两者必须都是"合理非 0 值"：sdist>0 而 brkd=0 时提前量 = v²/(2·0) = ∞ ⇒ 判停条件恒成立
       ⇒ 规划一启动当周期就被清标志（车几乎不动）。
     ★2026-10-07 现场教训：sdist 给 10 时"前后想动动不起来" —— 梯形斜坡在 1cm 小步上峰值只有
       √(a·d)=12cm/s（PWM≈120 压在起转死区）且只撑一两个周期 ⇒ 本档只适合"≥3cm 且速度能跑到
       20cm/s"的短距走位，几 cm 的贴靠微调请用下面 nspd/nbrk 那套速度阶跃法。 */
  if(strcmp(str_name, "sdist") == 0){ chassis.fine_max_dist = val; return; }
  if(strcmp(str_name, "brkd")  == 0){ chassis.brake_decel  = val; return; }

  /* ★ 校准微调（JieTi_Nudge_Y：起转阶跃 + 收尾降速 + 编码器判停）的三个现场旋钮 — 2026-10-07 加
     nspd f 20   起转/上限速度 cm/s：推不动就往上加(25~30)，甩太猛就往回收（要 ≥18~20 才越得过
                  四轮起转死区：位置式速度环 P 项 = kp×v）
     ncrl f 8    收尾爬行下限 cm/s：越接近断速线速度越往它降。降速后推不动（日志 "stall"）就加；
                  觉得收尾太磨蹭就减（★别低于 6，怕四轮推不动）
     nbrk f 0    断速余量**附加量** cm：真正的余量按当前速度现算 v²/(2×220) + v×20ms，这里只加在它上面
                  落点调法：总是冲过头 → 加大(0.1/0.2)；总是差一点没到 → 减小(-0.1/-0.2)
     改完下一次挪车立即生效（每次现读这三个字段，不用复位）。 */
  if(strcmp(str_name, "nspd") == 0){ chassis.nudge_speed    = val; return; }
  if(strcmp(str_name, "ncrl") == 0){ chassis.nudge_crawl    = val; return; }
  if(strcmp(str_name, "nbrk") == 0){ chassis.nudge_brake_cm = val; return; }

  /* ★ 四轮统一速度环参数（在线批量改，不在 param 表）：一条指令同时改 4 轮 × 4 个速度段。
     当前表里四轮全速段本就是同一组值，所以这等效于"改整个速度环的参数"。
     改的是 chassis.speed_seg 表，而控制循环每周期都从表里取参数 ⇒ 发完下一周期立即生效，无需复位。
     用法：skp f 2.4    ski f 0.034    skd f 0.0
     ★ 只改参数、不动 PID 状态：改 ki 后各轮的 i_out 累加器还是旧值。若改完觉得过冲/发顿，
       先发 vy f 0 让车停稳（静止零漂保护会自动清零各轮状态）再重新给速度即可。 */
#define SET_ALL_SEG(field, v) do{                                                  \
    for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++)                  \
      for(uint8_t s = 0; s < SPEED_SEG_NUM; s++) chassis.speed_seg[i][s].field = (v); \
  }while(0)
  if(strcmp(str_name, "skp") == 0){ SET_ALL_SEG(kp, val); return; }
  if(strcmp(str_name, "ski") == 0){ SET_ALL_SEG(ki, val); return; }
  if(strcmp(str_name, "skd") == 0){ SET_ALL_SEG(kd, val); return; }
#undef SET_ALL_SEG

  /* ★ 单轮速度环参数（2026-09-18 加；落地实测发现 1/4 号轮响应比 2/3 号慢，要单独给参数）：
     指令名 = 参数名 + 轮号：kp1/ki1/kd1 … kp4/ki4/kd4。轮号 1左前 2左后 3右后 4右前。
     一条指令改该轮的全部 4 个速度段（延续"不分速度段"的约定）。
     典型用法（先用 skp/ski 打底，再单独修个别轮子）：
         skp f 4.4     四轮统一 kp
         ski f 0.04    四轮统一 ki
         kp1 f 5.2     只改左前轮 kp，其余三轮原值不动
         kp4 f 5.2     只改右前轮 kp
     和 skp 一样改的是 chassis.speed_seg 表（控制循环每周期从表里取值）⇒ 下一周期立即生效，无需复位。
     ★ 但它是内存值：复位/重新烧录会回到 CHASSIS_Init 的 SPEED_PID_DFT 表 —— 调好后要把最终值填回那张表。
     ★ 判断顺序有讲究：必须靠 && 短路先判 [1] 再判 [2]/[3]，否则发单字符 "k" 会越界读。
       同时保证 "kp"/"ki"/"kd" 不被这里截胡 —— 它们是航向环参数，[2] 是 '\0' 不落 '1'~'4' 区间，
       自动短路放行给下面的 param 表（ckp/tkp/skp 首字母不是 'k'，同样不受影响）。 */
  if(str_name[0] == 'k'
     && (str_name[1] == 'p' || str_name[1] == 'i' || str_name[1] == 'd')
     && str_name[2] >= '1' && str_name[2] <= '4'
     && str_name[3] == '\0'){
    uint8_t wheel = (uint8_t)(str_name[2] - '0');                              // 1~4
    uint8_t field = (str_name[1] == 'p') ? 0 : (str_name[1] == 'i') ? 1 : 2;  // 0=kp 1=ki 2=kd
    for(uint8_t s = 0; s < SPEED_SEG_NUM; s++){
      float *dst = (field == 0) ? &chassis.speed_seg[wheel][s].kp
                 : (field == 1) ? &chassis.speed_seg[wheel][s].ki
                                : &chassis.speed_seg[wheel][s].kd;
      *dst = val;
    }
    return;
  }

  /* ★ 位置式速度环的在线调参（2026-09-25 队友加，2026-09-27 接入本工程；字段说明见 chassis.h）
     ★ 先说清楚：**位置式和增量式的参数是两套、存在两处，别搞混**
        增量式 → chassis.speed_seg[轮][速度段] 表（按速度段分 4 档；改它的是 skp/ski/skd 与 kp1~4/ki1~4/kd1~4）
        位置式 → chassis.speed_pid_pos[轮] 自己身上（四轮各一份、**不分速度段**；改它的是下面这组 p* 指令）
       位置式压根不查 speed_seg 表 ⇒ 发完 pkp1 再发 pq，看到 W1 还是旧值是正常的（那是增量式的值），
       要看位置式的值请认 pq 输出里的 P1~P4 / POS 那几行。
     指令一览（格式仍是 "名字 类型 数值"）：
       pkp1 f 5.0 / pki1 f 0.1 / pkd1 f 0.0     单轮 kp/ki/kd（轮号 1左前 2左后 3右后 4右前）
       pomax f 900 / pomin f -900               输出上下限（PWM 量纲）
       pimax f 3000                             误差积分限幅（★限的是 Σerror，不是 i_out）
       psep  f 30                               积分分离阈值：|误差|>它 → 不积分（0=关闭）
       pidz  f 2                                积分死区阈值：|误差|<它 → 不积分（0=关闭）
       pkip  f 0.001                            |ki| 低于它 → 视为无积分并清空 errorint
       pmode i 1                                1=位置式(上电默认) 0=增量式 —— 出问题最快的回退开关
     ★ 前缀辨析：pkp1 是"位置式1号轮 kp"，kp1 是"增量式1号轮 kp"，别混。
     ★ 判断顺序同样是靠 && 短路先判 [1]/[2] 再判 [3]/[4]，否则发单字符 "p" 会越界读；
       "pkip" 因 [3]='p' 不落 '1'~'4' 区间，自动短路放行给下面的 strcmp。 */
  if(str_name[0] == 'p' && str_name[1] == 'k'
     && (str_name[2] == 'p' || str_name[2] == 'i' || str_name[2] == 'd')
     && str_name[3] >= '1' && str_name[3] <= '4' && str_name[4] == '\0'){
    uint8_t wheel = (uint8_t)(str_name[3] - '0');                              // 1~4
    uint8_t field = (str_name[2] == 'p') ? 0 : (str_name[2] == 'i') ? 1 : 2;  // 0=kp 1=ki 2=kd
    float *dst = (field == 0) ? &chassis.speed_pid_pos[wheel].kp
               : (field == 1) ? &chassis.speed_pid_pos[wheel].ki
                              : &chassis.speed_pid_pos[wheel].kd;
    *dst = val;
    return;
  }
  /* 位置式的限制/积分条件（四轮统一一套，与 skp/ski/skd 那组同一个思路） */
#define SET_ALL_POS(field, v) do{                                              \
    for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++)              \
      chassis.speed_pid_pos[i].field = (v);                                    \
  }while(0)
  if(strcmp(str_name, "pomax") == 0){ SET_ALL_POS(out_max,      val); return; }
  if(strcmp(str_name, "pomin") == 0){ SET_ALL_POS(out_min,      val); return; }
  if(strcmp(str_name, "pimax") == 0){ SET_ALL_POS(integral_max, val); return; }
  if(strcmp(str_name, "psep")  == 0){ SET_ALL_POS(int_sep_th,   val); return; }
  if(strcmp(str_name, "pidz")  == 0){ SET_ALL_POS(int_dz,       val); return; }
  if(strcmp(str_name, "pkip")  == 0){ SET_ALL_POS(ki_eps,       val); return; }
#undef SET_ALL_POS
  /* 位置式/增量式切换（★队友强调的指令）：pmode i 1 位置式（上电默认） / pmode i 0 增量式。
     立即生效、不用复位 —— 两套状态各存一份，切过去只是换掉这一周期读谁 */
  if(strcmp(str_name, "pmode") == 0){ chassis.speed_pos_mode = (uint8_t)val; return; }

  /* ★ 手动设定速度环目标值（2026-09-25 加，专为"目标钉住不动"地调速度环）：
       ptgt f 40   四轮 target 全部钉在 40cm/s（正=前进，负=后退，0=目标0），旁路麦轮解算
       pauto i 0   退出定速 → 车停，回到"静止等指令"（发 mx/my/vx/vy 也会自动交给解算）
     ★为什么需要它：自动流程里 target 由梯形规划给 —— 峰值只有 20cm/s 且一直在变，
       想验证 40/80/120cm/s 下 kp/ki 还够不够用，就必须能把目标钉住。
     ★ptgt 顺手把规划标志清掉：定速期间不该再让梯形规划在后台跑（规划每周期都会改写
       chassis.v_x/v_y、累加 dist_acc_x/y、把 ti 一直往前推）。速度环的 target 已被
       speed_dbg_manual 钉住、不受它影响，清掉后行为更可预期。 */
  if(strcmp(str_name, "ptgt")  == 0){
    chassis.speed_dbg_tgt     = val;
    chassis.speed_dbg_manual  = 1;
    chassis.x_speed_plan_flag = 0;
    chassis.y_speed_plan_flag = 0;
    chassis.x_set_speed_flag  = 0;   // 一并清掉，免得 vx/vy 的手动值又来插一脚
    chassis.y_set_speed_flag  = 0;
    return;
  }
  /* pauto i 0：退出四轮同速定速，速度环 target 交回麦轮解算（v_x/v_y 为 0 ⇒ 车停） */
  if(strcmp(str_name, "pauto") == 0){ chassis.speed_dbg_manual = 0; return; }

  /* ★ 航向环三档参数在线调（2026-09-19 改：kp/ki/kd/bias 四者**每档一套、彼此独立**）
     指令名 = 参数名 + 档号：ykp1/2/3、yki1/2/3、ykd1/2/3、ybias1/2/3
     档号：1 = 旋转档   2 = 低速平移档   3 = 高速平移档。用法：ykp2 f -0.02    ybias2 f 0.05
     ★ 为什么要单独开指令：控制循环每周期都按工况把该档整套参数写进 yaw_pid 的 kp/ki/kd，
       param 表里那三项（kp/ki/kd）会被下一周期立刻冲掉、根本调不动 —— 只有改 yaw_param[] 才有效。
     三档由控制循环自动选（无平移→旋转档；有平移按合速度 40 分低/高速），不需要手动切档。
     调完请把值填回 chassis.h 的 YAW_TURN_* / YAW_MOVE_LO_* / YAW_MOVE_HI_* 那十二个宏。
     参考：调档时用 SERIALPLOT_PIDAdjustParam() 那个入口看 目标角/实际角/输出w 更直观。 */
  if(str_name[0] == 'y'){
    uint8_t pi = 0, pidx = 0;    // pi: 0=kp 1=ki 2=kd 3=bias；pidx: 档号字符在名字里的位置
    if     (strncmp(str_name, "ykp",   3) == 0){ pi = 0; pidx = 3; }
    else if(strncmp(str_name, "yki",   3) == 0){ pi = 1; pidx = 3; }
    else if(strncmp(str_name, "ykd",   3) == 0){ pi = 2; pidx = 3; }
    else if(strncmp(str_name, "ybias", 5) == 0){ pi = 3; pidx = 5; }
    /* pidx != 0 表示前缀匹配上了；末尾必须是单个 '1'~'3'（所以裸的 "ybias" 会走到 param 表，无害）*/
    if(pidx != 0 && str_name[pidx] >= '1' && str_name[pidx] <= '3' && str_name[pidx+1] == '\0'){
      uint8_t st = (uint8_t)(str_name[pidx] - '1');            // 0=旋转 1=低速 2=高速
      float  *dst = (pi == 0) ? &chassis.yaw_param[st].kp
                  : (pi == 1) ? &chassis.yaw_param[st].ki
                  : (pi == 2) ? &chassis.yaw_param[st].kd
                              : &chassis.yaw_param[st].bias;
      *dst = val;
      return;
    }
  }

  /* 参数查询（2026-09-18 加）：发 "pq i 0" → 串口打印当前参数。
     内容（★2026-09-27 起"两套速度环参数分开打"，认字母前缀即可）：
       W1~W4  = **增量式**速度环（speed_seg 表，改它的是 skp/ski/skd 与 kp1~4/ki1~4/kd1~4）
       P1~P4  = **位置式**速度环（speed_pid_pos[]，改它的是 pkp1~4/pki1~4/pkd1~4）★当前生效的是这段
       POS    = 位置式的限制与积分条件（out_max/out_min/integral_max/sep/dz/ki_eps）
       YAW1~3 = 航向环三档（改它的是 ykp1~3 / yki1~3 / ykd1~3 / ybias1~3）
     ★ 为什么要分开打：上面 W1~W4 打的是 speed_seg 表，而位置式压根不查那张表 ⇒
       发完 pkp1 再发 pq，看到 W1 纹丝不动会误判成"指令没生效"。位置式的参数在
       speed_pid_pos[] 自己身上，只有 P1~P4 / POS 那几行能反映 pkp1~4 的结果。
     用途：这些指令改完看不到当前值（OLED 上显示不全），用它确认。
     注意打印是文本、会插进浮点数据流里。 */
  if(strcmp(str_name, "pq") == 0){
    for(uint8_t w = CHASSIS_MOTOR_LF; w <= CHASSIS_MOTOR_RF; w++){
      UART1_Printf("W%d kp%f ki%f kd%f\r\n", (int)w,
                   chassis.speed_seg[w][0].kp, chassis.speed_seg[w][0].ki, chassis.speed_seg[w][0].kd);
    }
    /* 位置式速度环参数（2026-09-27 加）：P1~P4 四轮 + POS 限制/积分条件 */
    for(uint8_t w = CHASSIS_MOTOR_LF; w <= CHASSIS_MOTOR_RF; w++){
      UART1_Printf("P%d kp%f ki%f kd%f\r\n", (int)w,
                   chassis.speed_pid_pos[w].kp, chassis.speed_pid_pos[w].ki,
                   chassis.speed_pid_pos[w].kd);
    }
    UART1_Printf("POS out%f/%f imax%f sep%f dz%f kieps%f mode%d\r\n",
                 chassis.speed_pid_pos[CHASSIS_MOTOR_LF].out_max,
                 chassis.speed_pid_pos[CHASSIS_MOTOR_LF].out_min,
                 chassis.speed_pid_pos[CHASSIS_MOTOR_LF].integral_max,
                 chassis.speed_pid_pos[CHASSIS_MOTOR_LF].int_sep_th,
                 chassis.speed_pid_pos[CHASSIS_MOTOR_LF].int_dz,
                 chassis.speed_pid_pos[CHASSIS_MOTOR_LF].ki_eps,
                 (int)chassis.speed_pos_mode);
    for(uint8_t st = 0; st < 3; st++){   // 1=旋转 2=低速 3=高速
      UART1_Printf("YAW%d kp%f ki%f kd%f bias%f\r\n", (int)(st + 1),
                   chassis.yaw_param[st].kp, chassis.yaw_param[st].ki,
                   chassis.yaw_param[st].kd, chassis.yaw_param[st].bias);
    }
    /* 校准微调三个旋钮（2026-10-07 加）：nspd/ncrl/nbrk 改完用它确认
       （★都是内存值，复位/重烧回默认宏 NUDGE_*_DFT，调好记得填回 chassis.h） */
    UART1_Printf("NUD spd%f crl%f brk%f\r\n",
                 chassis.nudge_speed, chassis.nudge_crawl, chassis.nudge_brake_cm);
    return;
  }

  /* ★ 直接走一遍 ROBOT_Angle（2026-09-19 加，专为验证"停止档"退出流程）：
     ★ 和 param 表的 target 不是一回事 —— target 只改 chassis.target_yaw 就完事，不等待、
       也不经过 ROBOT_Angle，所以拿它测不出"进入停止档才返回"这套逻辑。
     用法：at f 90    （角度 0~360，类型 f/i 都写同一个 val）
     ★ 必须先挡负数：ROBOT_Angle 形参是 uint32_t，发 -90 会变成 42.9 亿，
       车永远转不停、函数永不返回（把整条比赛流程卡死）。这里拦一道。
     调用期间本函数阻塞（车在转，OLED 停更），转完打印耗时再回到 while(1) 继续刷屏。
     耗时用来判断 YAW_STOP_CONFIRM 那 200ms 确认延迟在整套动作里占多少（见 chassis.h）。 */
  if(strcmp(str_name, "at") == 0){
    if(val < 0.0f || val >= 360.0f){
      UART1_Printf("AT range 0~360, got %f\r\n", val);
    }else{
      uint32_t at_t0 = HAL_GetTick();
      ROBOT_Angle((uint32_t)val);
      UART1_Printf("AT %d ok, %dms\r\n", (int)val, (int)(HAL_GetTick() - at_t0));
    }
    return;
  }

  for(int i=0; i<= PARAM_Number-1; i++){  //确定接收到的子串名字与哪个变量名字相对应
    if(strcmp(str_name, param[i].name) == 0){
      if(param[i].type == 1){                    // 整数型参数（如 cstage）：按 int 写入，避免破坏相邻字节
        *(int*)param[i].p = (int)val;
      }else{                                     // 浮点型参数（默认）
        *(float*)param[i].p = val;
      }
      /* vx/vy 为手动设速模式：置对应轴 set_speed_flag（中断不清零该轴 v_），并取消该轴正在执行的规划 */
      if(strcmp(str_name, "vx") == 0){
        chassis.x_set_speed_flag  = 1;
        chassis.x_speed_plan_flag = 0;
      }else if(strcmp(str_name, "vy") == 0){
        chassis.y_set_speed_flag  = 1;
        chassis.y_speed_plan_flag = 0;
      }
      break;
    }
  }
}

/**
  * @brief 利用serialplot画图软件进行PID调参
  * @note  串口发6通道数据（空格分隔+\r\n）：yaw目标角/实际角/输出w、目标合速度/实际合速度、
  *        当前档位编码 0旋转/2低速/3高速（1不介入自 2026-09-27 起不再出现；各通道含义详见 while(1) 里 UART1_Printf 处的注释）。
  *        调参指令格式：名字 类型 数值，如 "kp f 15" / "target f 30"
  *        ★ 调高速平移档：发 "go f 0"（前进长距）或 "gb f 0"（后退长距）跑一次直行，
  *          见 SERIALPLOT_ChangeParam 的 HIMOVE_* 宏与 go/gb 指令；用 ykp3/yki3/ykd3/ybias3 在线调，
  *          OLED 后两行实时显示高速档参数
  */
void SERIALPLOT_PIDAdjustParam(void){
  /*整车速度清零：车静止只做转向，w 由航向环接管（角度环自动算 w）*/
  chassis.v_x = 0.0f;
  chassis.v_y = 0.0f;
  while(1){
    /* OLED 四行（2026-09-19 改：后两行固定显示**高速平移档**的 kp/ki/kd/bias）
         行1 tar/act = 目标角 / 实际角（act 是归一化后、控制真正在跟的那个）
         行2 e/w     = 角度环误差 / 输出 w(rad/s)
         行3/4       = 高速平移档 kp / ki / kd / bias（直接读 chassis.yaw_param[YAW_STAGE_MOVE_HI]）
       ★ 为什么不显示 yaw_pid.kp：它每 20ms 被控制循环按**当前工况档**覆盖 —— 车静止时它是旋转档
         的值，只有车真的高速平移时才变成高速档的值。调高速档时读它只会误导（改 ykp3 却纹丝不动）。
         这里读 yaw_param[] 是参数源头，跟当前落在哪一档无关，车停着也能看到高速档被改成什么了。
       （原来这四行里的 陀螺仪原始角 raw 和 里程计 x/y 已让位，需要时换回来：
         OLED_Printf(0, 16, OLED_8X16_HALF, "raw:%05.2f", HWT101CT_Data.yaw);
         OLED_Printf(0, 48, OLED_8X16_HALF, "x:%05.2f y:%05.2f", chassis.pos_x, chassis.pos_y);
         raw 的用途：正常时 raw 与 act 应相等（或差 360 的整数倍），不等说明 PID_Angle 归一化有问题。） */
    OLED_Printf(0, 0,  OLED_8X16_HALF, "tar:%05.2f act:%05.2f", chassis.yaw_pid.target, chassis.yaw_pid.actual);
    OLED_Printf(0, 16, OLED_8X16_HALF, "e:%+05.2f  w:%+04.1f", chassis.yaw_pid.error0, chassis.yaw_pid.out);
    OLED_Printf(0, 32, OLED_8X16_HALF, "kp:%+.3f ki:%+.3f", chassis.yaw_param[YAW_STAGE_MOVE_HI].kp,
                                                             chassis.yaw_param[YAW_STAGE_MOVE_HI].ki);
    OLED_Printf(0, 48, OLED_8X16_HALF, "kd:%+.3f b:%+.3f",  chassis.yaw_param[YAW_STAGE_MOVE_HI].kd,
                                                             chassis.yaw_param[YAW_STAGE_MOVE_HI].bias);
    OLED_Update();
    /*6通道（2026-09-19 改，为高速档调参）：
       1 yaw目标角 / 2 yaw实际角(归一化，控制真正跟的) / 3 角度环输出w /
       4 目标合速度 / 5 实际合速度 / 6 分档依据 max(|v_x|,|v_y|)
      ★ 第4通道 = 整车目标合速度 sqrt(v_x²+v_y²)；第5通道 = 实际合速度
        sqrt(now_v_x²+now_v_y²)（now_v_* 是里程计转到全局系的速度，但旋转不改变模长
        ⇒ 数值上等于车体合速度）。4/5 同口径，可直接对比着看速度环跟随得怎么样。
      ★ 第6通道 = 当前档位编码（chassis.yaw_gear，见 chassis.h）：
          0 = 旋转档（无平移）   2 = 低速平移档（0~80）
          3 = 高速平移档（> 80）
        ★ 1 = 不介入**2026-09-27 起不再出现**（YAW_MOVE_MIN_SPEED 定为 0，0~20 归低速档）；
          把该宏改回 20.0f 就会重新看到 1。
        ★ 用 %d 发整数、不是 %f —— 图上直接看 2/3（旧版是 1/2/3）的台阶，比看连续数值直观。
        ★ 旋转档给 0 是**额外的**（上面三档才是分档结果）：车停下、原地转向时落这一档，
          它和"不介入"正好相反（角度环介入且 bias 最猛），混进 1 会看错。 */
    float tgt_spd = sqrtf(chassis.v_x * chassis.v_x + chassis.v_y * chassis.v_y);
    float act_spd = sqrtf(chassis.now_v_x * chassis.now_v_x + chassis.now_v_y * chassis.now_v_y);
    UART1_Printf("%f %f %f %f %f %d\r\n",
                 chassis.yaw_pid.target,
                 chassis.yaw_pid.actual,
                 chassis.yaw_pid.out,
                 tgt_spd,
                 act_spd,
                 chassis.yaw_gear);
    if(UART1_RxFlag){
      UART1_RxFlag = 0;
      SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
    }
    HAL_Delay(10);
  }
}

/* ==================== 单轮速度环临时调参（调试用，字段说明见 chassis.h 的 speed_tune） ====================
   用途：一次只让一个轮子转，用一组"临时 kp/ki/kd"覆盖全轮、全速段（不查 SPEED_PID_DFT 表、不分段），
        在串口上把这组值试出来之后，自己填进 CHASSIS_Init 的 SPEED_PID_DFT 表。
   为什么必须旁路麦轮解算：4 方程 3 未知数，数学上不存在只让一轮非零的 v_x/v_y/w，
        所以控制循环里 speed_tune=1 时直接跳过 梯形规划/航向环/麦轮解算/Mecanum，只跑速度环。
   串口指令（SerialPlot 用户指令区发送，格式 "名字 类型 数值"）：
     tspd f 30    被调轮目标速度 cm/s（正=前进，负=后退；先给个能起转的值如 30）
     tkp  f 3.0   临时 kp        tki f 0.05   临时 ki        tkd f 0.0   临时 kd
   串口输出 4 通道：目标速度 / 实测速度 / PWM输出 / 本周期脉冲
     第4通道脉冲用来盯低速量化台阶：1 个脉冲 ≈ 9.09 cm/s，目标 15cm/s 时一周期只有 1.65 脉冲
     ⇒ actual 只会在 0 / 9.1 / 18.2 … 之间跳，这是编码器分辨率的上限，调 kp/ki 突破不了。
   退出：本函数不返回（调试期间 main 就停在这里），调完把 main.c 里的这次调用注释掉即可 */
static const char *SPEED_TUNE_WHEEL_NAME[5] = {"--", "LF", "LB", "RB", "RF"};  // 索引=轮号，与 CHASSIS_MOTOR_* 一致

/* ★取"当前生效那套"速度环的 PWM 输出（2026-09-27 位置式速度环接入后加；与 main.c 的
   CHASSIS_PwmOut 同一个道理）：速度环有位置式/增量式两套状态，同一时刻只有一套在算、在驱动
   电机，另一套的 out 是残值。调试打印直接用 speed_pid[w].out 的话，在默认的位置式下会一直打出
   恒 0，看着像"PID 没输出"。行为：位置式→speed_pid_pos[w].out；增量式→speed_pid[w].out */
static float SERIALPLOT_PwmOut(uint8_t wheel){
  return chassis.speed_pos_mode ? chassis.speed_pid_pos[wheel].out
                                : chassis.speed_pid[wheel].out;
}

void SERIALPLOT_SpeedTuneLoop(uint8_t wheel){
  if(wheel < CHASSIS_MOTOR_LF || wheel > CHASSIS_MOTOR_RF) wheel = CHASSIS_MOTOR_LF;  // 越界回落左前
  chassis.tune_wheel = wheel;
  chassis.speed_tune = 1;            // 让控制循环只跑速度环（旁路 梯形规划/航向环/麦轮解算）
  /* 清干净 4 轮 PID 与 PWM：只留被调轮转，其余三轮彻底停住（避免残存的 out 把别的轮子拖起来）
     ★2026-09-27 位置式接入：位置式的状态（speed_pid_pos[]）同样要清 —— 它的 out 是**绝对 PWM 值**
       （由 errorint 累加而来），残留下来的话会把被调轮直接拖起来；errorint 不清则起步多冲一下 */
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    PID_INC *pid = &chassis.speed_pid[i];
    pid->target   = 0.0f;
    pid->actual   = 0.0f;
    pid->out      = 0.0f;
    pid->err      = 0.0f;
    pid->last_err = 0.0f;
    pid->prev_err = 0.0f;
    pid->p_out    = 0.0f;
    pid->i_out    = 0.0f;
    pid->d_out    = 0.0f;
    PID_POS *pp = &chassis.speed_pid_pos[i];
    pp->target   = 0.0f;
    pp->actual   = 0.0f;
    pp->out      = 0.0f;
    pp->p_out    = 0.0f;
    pp->i_out    = 0.0f;
    pp->d_out    = 0.0f;
    pp->error0   = 0.0f;
    pp->error1   = 0.0f;
    pp->errorint = 0.0f;
    TB6612_Control(i, 0);
  }
  chassis.tune_pulse = 0;
  while(1){
    PID_INC *pid = &chassis.speed_pid[wheel];
    OLED_Printf(0,  0, OLED_8X16_HALF, "tune:%s kp:%04.2f", SPEED_TUNE_WHEEL_NAME[wheel], chassis.tune_kp);
    OLED_Printf(0, 16, OLED_8X16_HALF, "ki:%04.2f kd:%04.2f", chassis.tune_ki, chassis.tune_kd);
    OLED_Printf(0, 32, OLED_8X16_HALF, "tar:%+04.1f p:%+03d", pid->target, chassis.tune_pulse);
    OLED_Printf(0, 48, OLED_8X16_HALF, "act:%+04.1f o:%+05.0f", pid->actual, SERIALPLOT_PwmOut(wheel));
    OLED_Update();
    /* 4通道：SerialPlot 画前3条看收敛；第4条脉冲看量化台阶（整数，画出来是阶梯）
       ★第3通道是"当前生效那套速度环"的 PWM（位置式/增量式自动选，见上面 SERIALPLOT_PwmOut） */
    UART1_Printf("%f %f %f %d\r\n", pid->target, pid->actual, SERIALPLOT_PwmOut(wheel), chassis.tune_pulse);
    if(UART1_RxFlag){
      UART1_RxFlag = 0;
      SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
    }
    HAL_Delay(10);
  }
}

/* ==================== 四轮联动测试（2026-09-18 调试用） ====================
   目的：给"车身整体"一个目标速度，看四个轮子一起转动时 实际速度 vs 目标速度 的关系。
        四轮目标由麦轮解算给出：纯 vy / 纯 vx 工况下四个 target 应当恒相等，
        所以"4 条 actual 曲线散开的程度" = 四轮速度环 + 机械阻力的一致性。
  ★ 与单轮调参模式(SERIALPLOT_SpeedTuneLoop)的关键区别：本模式**不旁路麦轮解算**，
    走完整控制流程（麦轮解算 → 4轮速度环），即"正常的四轮一起跑"。
    （单轮调参模式必须旁路：麦轮 4 方程 3 未知数，数学上不存在只让一轮非零的 v_x/v_y/w）
  串口指令（SerialPlot 用户指令区，格式 "名字 类型 数值"）：
    vy f 30    ★ 车身整体速度 cm/s（前进为正 / 负为后退）—— 这就是"改 target"的那个值，随时可改
    vx f 20      车身整体横移速度 cm/s（右移为正）
    ang i 1      临时开/关航向环（默认 0 关闭，见下）
    ★2026-09-27 起另有更直接的入口（推荐，比本函数还灵活）：ptgt f 40 四轮同速定速 / pauto i 0 退出。
  航向环默认关闭（ang=0）：关掉后 w 恒为 0，四个轮子 target 严格相等，最适合看四轮一致性。
    想测"带纠偏的真实工况"就发 ang i 1 打开 —— 此时角度环会输出 w，四轮 target 会各差一点。
  串口 8 通道输出（SerialPlot 通道号从 1 数起）：
    1~4: 左前 / 左后 / 右后 / 右前  目标速度 cm/s（正常应完全重合，画出来就是一条线）
    5~8: 左前 / 左后 / 右后 / 右前  实际速度 cm/s（四轮一致性看这里）
  退出：本函数不返回（调试期间 main 就停在这里），测完把 main.c 里的这次调用注释掉即可。
  （想看每轮 PWM：OLED 第 4 行已显示；要进串口就加进 printf 变 12 通道，UART1_TxLengthMax=200 够用） */
void SERIALPLOT_ChassisTestLoop(void){
  /* 关航向环 + 清 w：让四轮 target 严格相等，纯粹观察四个速度环的一致性 */
  flag.angle = 0;
  chassis.w  = 0.0f;
  /* 清干净 4 轮 PID 与 PWM，从零起步（避免上一次运行残留的 out 让轮子先窜一下）
     ★2026-09-27 位置式接入：位置式那套状态（speed_pid_pos[]）一起清 —— 见 SERIALPLOT_SpeedTuneLoop 的同一段注释 */
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    PID_INC *pid = &chassis.speed_pid[i];
    pid->target   = 0.0f;
    pid->actual   = 0.0f;
    pid->out      = 0.0f;
    pid->err      = 0.0f;
    pid->last_err = 0.0f;
    pid->prev_err = 0.0f;
    pid->p_out    = 0.0f;
    pid->i_out    = 0.0f;
    pid->d_out    = 0.0f;
    PID_POS *pp = &chassis.speed_pid_pos[i];
    pp->target   = 0.0f;
    pp->actual   = 0.0f;
    pp->out      = 0.0f;
    pp->p_out    = 0.0f;
    pp->i_out    = 0.0f;
    pp->d_out    = 0.0f;
    pp->error0   = 0.0f;
    pp->error1   = 0.0f;
    pp->errorint = 0.0f;
    TB6612_Control(i, 0);
  }
  /* 手动设速标志置 1：否则控制循环每周期都会把 v_x/v_y 归零（见 CHASSIS_Control_Loop 规划执行段）
     ★ 串口发 vy/vx 时 ChangeParam 也会自动置位，这里只是把初始状态先准备好 */
  chassis.x_set_speed_flag = 1;
  chassis.y_set_speed_flag = 1;
  chassis.v_x = 0.0f;
  chassis.v_y = 0.0f;
  /* 里程计清零：规划验收时 OLED 上的 pos_y 就直接读作"从进入本模式起走了多远"（cm）。
     注意它来自编码器积分，本身有标定误差 —— 判断"走准没走准"仍以尺子量为准，
     这个读数主要用来看 里程计读数 与 尺子量值 差多少（顺带复核 ENCODER_ACCURACY 标定）。 */
  chassis.pos_x = 0.0f;
  chassis.pos_y = 0.0f;
  while(1){
    PID_INC *p1 = &chassis.speed_pid[CHASSIS_MOTOR_LF];
    PID_INC *p2 = &chassis.speed_pid[CHASSIS_MOTOR_LB];
    PID_INC *p3 = &chassis.speed_pid[CHASSIS_MOTOR_RB];
    PID_INC *p4 = &chassis.speed_pid[CHASSIS_MOTOR_RF];
    /* OLED：整车目标速度 + 航向环开关 + 四轮的 目标/实际/PWM（每行 4 个值，顺序 LF/LB/RB/RF）
       ★2026-09-27 位置式接入：PWM 那行必须取"当前生效那套速度环"的输出（SERIALPLOT_PwmOut），
         直接打 p1->out 的话，在默认的位置式下会恒为残值 0，看着像"没输出" */
    OLED_Printf(0,  0, OLED_8X16_HALF, "%+4d y%+4d k%.1f", (int)chassis.v_y, (int)chassis.pos_y, p1->kp);
    OLED_Printf(0, 16, OLED_8X16_HALF, "tar%+04.0f%+04.0f%+04.0f%+04.0f", p1->target, p2->target, p3->target, p4->target);
    OLED_Printf(0, 32, OLED_8X16_HALF, "act%+04.0f%+04.0f%+04.0f%+04.0f", p1->actual, p2->actual, p3->actual, p4->actual);
    OLED_Printf(0, 48, OLED_8X16_HALF, "out%+04.0f%+04.0f%+04.0f%+04.0f",
                SERIALPLOT_PwmOut(CHASSIS_MOTOR_LF), SERIALPLOT_PwmOut(CHASSIS_MOTOR_LB),
                SERIALPLOT_PwmOut(CHASSIS_MOTOR_RB), SERIALPLOT_PwmOut(CHASSIS_MOTOR_RF));
    OLED_Update();
    /* 8 通道：前 4 条 = 四轮目标速度（应重合为一条），后 4 条 = 四轮实际速度（分散度 = 一致性） */
    UART1_Printf("%f %f %f %f %f %f %f %f\r\n",
                 p1->target, p2->target, p3->target, p4->target,
                 p1->actual, p2->actual, p3->actual, p4->actual);
    if(UART1_RxFlag){
      UART1_RxFlag = 0;
      SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
    }
    HAL_Delay(10);
  }
}

/* ==================== 四轮实际值实时输出（2026-09-25 加） ====================
   目的：串口1(115200) 实时打四个轮子的**实际速度**，SerialPlot/串口助手直接看 4 条曲线。
         看什么：四轮一致性(曲线散开程度) / 起转迟滞(目标给了但 actual 迟迟不走) /
                 停车反接(目标 0 后 actual 迅速归零) / 速度环跟踪(actual 能不能追上 target)。
   通道顺序（与 chassis.h 的 ChassisMotorIndex 一致）：
     1=左前LF  2=左后LB  3=右后RB  4=右前RF    单位 cm/s（20ms 下 1 个编码器脉冲 = 2.27cm/s）
   发送格式："%.1f %.1f %.1f %.1f\r\n" —— **纯数字、没有前缀文字**，SerialPlot 按空格分通道即可；
     用串口助手看就是一行 4 个数。小数只留 1 位：分辨率 2.27cm/s，留 6 位纯属刷屏
     （想和别的打印一样用 %f，就把下面那条 UART1_Printf 的格式改回 "%f %f %f %f\r\n"）。
   ★为什么是"泵"函数（非阻塞 + 内部按 WHEEL_ACT_SEND_MS 节流）而不是 while 里直接发：
      调用点只管频繁地调，发不发、隔多久发由本函数按 HAL_GetTick 判断 ——
      ① 主循环转一圈的时间不固定（OLED/按键/视觉解析都占时间），节流后周期才稳定；
      ② 车在跑的时候主循环往往停在上层的阻塞等待里（见 robot.c），所以那几个等待循环里也要调本函数。
   ★调用点（开关/周期都在 serialplot.h 的 WHEEL_ACT_SEND_EN / WHEEL_ACT_SEND_MS）：
      Core/Src/main.c  主循环开头 + "红蓝方选择"菜单循环里
      Mycode/robot.c   ROBOT_Angle 等停止档的循环、ROBOT_Move 的两段等待循环
   ★绝不放在 TIM7 1ms 中断里（别搬过去）：UART1_Printf 是阻塞发送，一行≈3ms@115200，
     塞进中断会把 20ms 控制周期拖长 → 速度环/航向环时序全乱。
   ★和别的串口1打印混在一起怎么办：现场用单键 'v' 关掉（见 main.c 的 UART1_DebugCmd），
     或把 serialplot.h 的 WHEEL_ACT_SEND_EN 改 0 重新编译。 */
uint8_t serialplot_wheel_on = WHEEL_ACT_SEND_EN;   // 运行期开关：1=发 0=停（串口1单键 'v' 切换）

void SERIALPLOT_WheelActualPump(void){
  if(!serialplot_wheel_on) return;                 // 开关关掉：一个字都不发
  static uint32_t t_last = 0;                      // 上次发送时刻(ms)：节流用
  uint32_t now = HAL_GetTick();
  if(now - t_last < WHEEL_ACT_SEND_MS) return;     // 还没到发送间隔：本次不打扰调用方
  t_last = now;
  UART1_Printf("%.1f %.1f %.1f %.1f\r\n",
               (double)chassis.speed_pid[CHASSIS_MOTOR_LF].actual,   // 1 左前
               (double)chassis.speed_pid[CHASSIS_MOTOR_LB].actual,   // 2 左后
               (double)chassis.speed_pid[CHASSIS_MOTOR_RB].actual,   // 3 右后
               (double)chassis.speed_pid[CHASSIS_MOTOR_RF].actual);  // 4 右前
}
