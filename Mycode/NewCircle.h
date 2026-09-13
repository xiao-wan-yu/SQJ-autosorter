#ifndef __NEWCIRCLE_H
#define __NEWCIRCLE_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ============================================================================
   NewCircle —— 新圆周运动模块（激光矫正左右 + 测距矫正前后 + 名义圆周运动）

   【现场几何】（2026-09 实测口径）
     · 前方测距(镜头)装在车中心正前方 NC_SENSOR_OFFSET_CM = 14cm 处；
     · 测距读的是"镜头→水管表面"的距离，目标值 NC_D_TARGET_MM_DEF = 200mm(20cm)；
     · 水管半径 NC_PIPE_RADIUS_CM = 4cm；
     · 于是"车中心→水管中心"的目标半径 D = 14 + 20 + 4 = 38cm（车就绕这个半径转圈）。

   【三个传感器怎么分工】
     LASER3(左前) / LASER4(右前) —— 主要矫正【左右】(切向，v_x)：
       · 两个都有障碍物         → 车头正对水管、左右已对准，不用左右补；
       · 只有 LASER3 没障碍物    → 车偏右 → 往左补（v_x 为负）；
       · 只有 LASER4 没障碍物    → 车偏左 → 往右补（v_x 为正）；
       · 两个都没有障碍物        → 偏太多（或车头歪太多），按上次方向继续补
                                   （NC_LR_BOTH_MISS_KEEP 置 0 则改为不补）。
     GY53_2(前方测距) —— 主要矫正【前后】(径向，v_y)：
       ★只有"两个激光都有障碍物"时，测距读到才是正对水管的垂直距离(斜着测会偏大)，
         所以默认 NC_FB_DIST_TRUST = 1：不满足条件时测距不参与前后微调。

   【微调方式】参考阶梯阶段的对准做法，全部"脉冲式"：
     每 NC_*_CHK_MS 采一次传感器 → 决定接下来最多 NC_*_MS 内给 ±5cm/s 的修正速度
     → 到点(或对准了/方向变了)立刻撤掉修正，只留名义圆周运动（车始终在转圈，不趴窝）。
     调参口诀：偏哪边补哪边；补过头(来回晃)就把 NC_*_MS 调小、NC_*_CHK_MS 调大
     （补一段停一段，更保守）；补不回来就把 NC_*_MS / NC_*_CHK_MS 调大。

   【名义圆周运动】（与 circle.c 同一套整车速度解耦）
     v_x = dir·ω·D_target（切向）+ 左右修正
     v_y = 前后修正
     w   = dir·ω（角速度前馈）+ 可选航向同步环(stage>=3)
   坐标系：v_x 车身右移为正、v_y 车头前进为正、w 逆时针为正(rad/s)；
          HWT101CT yaw 0~360° 顺时针为正，所以逆时针绕圈时 yaw 递减。

   【整圈判停】陀螺仪累积转角 |yaw_acc| ≥ arc_deg 自动停；另有 NC_TIMEOUT_MS 总超时
     保护、NC_LOST_MS 丢目标(两个激光连续都看不到水管)保护，都会停车并串口打印原因。

   【分步调试】串口指令 "nstage i N" 在线切换（N = 0~3）：
     stage 0 = 纯开环：只走切向+角速度前馈，不读传感器（先验证绕圈方向/快慢）
     stage 1 = + 激光左右微调
     stage 2 = + 测距前后微调（默认；测距要两个激光都有障碍物才可信）
     stage 3 = + 航向同步环（陀螺仪，让车头始终对准水管；需 flag.hwt101ct=1）
   其余在线调参名：nlrmv/nlrchk/nlrms/nfbmv/nfbtol/nfbtrig/nfbchk/nfbms/nfbmm/
                   nalpha/nyawkp/nwmax/nprint/nloss（含义见下面参数集中表）

   【在 main.c 里怎么调用】（当前 main.c 里已按这段接好：串口指令 7 触发，见 UART1_Data[0]==7）
     if(UART1_Data[0]==7){
       UART1_Data[0]=0;                     // 立即清指令，防止循环重复触发
       NCIRCLE_Run(200, 0.35f, 360, 1);     // 目标测距200mm(20cm)、公转0.35rad/s、整圈、逆时针
     }
   ============================================================================ */

/* ============================================================================
   ★★★ 参数集中表（改这里 → 重新编译即生效；带串口名的可在线调）★★★
   ============================================================================ */

/* ---------------- 0. 现场接线（传感器端口/引脚，在 NewCircle.c 里落地） ----------------
   引脚宏来自 Core/Inc/main.h：测距 GY53_2、激光 LASER3 / LASER4。
   只有换板子/换接口时才需要动 NewCircle.c 顶部那 3 行接线宏。 */

/* ---------------- 1. 几何常量（拿尺量准，一次性） ---------------- */
#define NC_SENSOR_OFFSET_CM  14.0f  /* 测距镜头到车中心的纵向距离(cm)
                                       调大→算出来的轨迹半径偏大、车会靠管外侧跑
                                       调小→半径算小、车往管上贴；量准即可 */

#define NC_PIPE_RADIUS_CM     4.0f  /* 水管外径半径(cm)，比赛场地柱径 8cm→4cm
                                       调大→半径算大、车离管远；量准即可 */

#define NC_D_TARGET_MM_DEF     200  /* 目标测距(mm)：镜头→水管表面，实测 20cm
                                       调大→离管远（半径 D 变大，转的圈更大）
                                       调小→离管近（圈更小，太近会撞管）
                                       NCIRCLE_Run() 的入参会覆盖它 */

/* ---------------- 2. 调试阶段（串口 nstage i N） ---------------- */
#define NC_STAGE_DEF             2  /* 当前阶段 0~3，见文件头"分步调试"
                                       调大→闭环越多、效果越接近最终
                                       调小→越简单、方便定位是哪一环的问题 */

/* ---------------- 3. 激光·左右微调（stage>=1 生效） ----------------
   逻辑：激光3没障碍物→车偏右→往左补；激光4没障碍物→车偏左→往右补；两个都有→不用补 */
#define NC_LR_SPEED_CM         5.0f /* 左右微调速度(cm/s)——现场要求 5cm
                                       调大→补得快，但容易一下冲过再次来回晃
                                       调小→更柔和，但偏太多时补不回来（会一直有个偏） */

#define NC_LR_CHK_MS            200 /* 多少时间采一次激光(=多久重新决定一次左右补)
                                       调小→反应快、采得勤（激光读一次最多几 ms）
                                       调大→更省、更保守（配 NC_LR_MS 形成"补一段停一段"） */

#define NC_LR_MS                400 /* 单次左右微调最长持续(ms)：超过就先停，等下次采样再决定
                                       调大→一次补得久、补得动大偏差
                                       调小→一次只补一点点，最保守、不会补过头 */

#define NC_LR_FIX_SIGN         (+1) /* ★左右补整体反向开关：现场若发现"补反了"就改成 (-1)
                                       正常情况保持 (+1) */

#define NC_LR_BOTH_MISS_KEEP      1 /* 两个激光都没有障碍物时：1=按上次方向继续补 / 0=不补
                                       1 → 大偏差也能一点点纠回来（推荐）
                                       0 → 传感器都看不到时干脆不乱动，更"怂" */

/* ---------------- 4. 测距·前后微调（stage>=2 生效；需要两个激光都有障碍物） ---------------- */
#define NC_FB_SPEED_CM         5.0f /* 前后微调速度(cm/s)——现场要求 5cm
                                       调大→离太远/太近补得快，但容易过冲
                                       调小→更稳，但偏差大时回来得慢 */

#define NC_FB_TOL_MM              5 /* 前后容差(mm)：|实测-目标| ≤ 它 视为到位、不再补
                                       调大→更早收手、少动
                                       调小→贴得更准，但临界值附近会反复微动 */

#define NC_FB_TRIG_MM            10 /* 前后触发阈值(mm)：偏差 ≥ 它 才开始补
                                       （容差~触发之间保持上次决定，形成滞回，避免临界抖动）
                                       调大→更懒得补、少动
                                       调小→更积极、贴得紧 */

#define NC_FB_CHK_MS            100 /* 前后微调决策周期(ms)：多久重判一次距离
                                       调小→判得勤、纠得快（测距读一次可能几十 ms）
                                       调大→更保守 */

#define NC_FB_MS                600 /* 单次前后微调最长持续(ms)：超过先停，下一周期再判
                                       调大→一次给得久（大偏差补得动）
                                       调小→一次只挪一点，最保守 */

#define NC_FB_DIST_TRUST          1 /* 1=必须"两个激光都有障碍物"才相信测距（现场要求，默认）
                                       0=不看激光、测距一直可用（调试用：先确认测距/前后补方向） */

/* ---------------- 5. 测距读取与滤波（stage>=1 生效） ---------------- */
#define NC_DIST_READ_MS          50 /* 测距读取节流(ms)：GY53 读一次是阻塞的(最坏 50ms)
                                       调小→数据新，但主循环被占得多
                                       调大→省时间，但前后补的反馈变慢 */

#define NC_ALPHA_DEF           0.7f /* 测距一阶低通系数(0~1)，越接近 1 越平滑
                                       调大→更稳、抗噪，但响应慢
                                       调小→跟手，但噪声会让前后补抖 */

#define NC_D_VALID_MIN_MM        50 /* 测距有效下限(mm)：小于它视为杂散，丢弃(保持上次值)
                                       调大→防近距离杂散 / 调小→允许更近读数 */

#define NC_D_VALID_MAX_MM       400 /* 测距有效上限(mm)：大于它视为测空/丢目标，丢弃
                                       调大→更抗丢目标 / 调小→更快识别测空（超量程返回 2000） */

/* ---------------- 6. 名义圆周运动 / 航向（stage>=3 生效） ---------------- */
#define NC_YAW_KP_DEF        (-0.03f)/* 航向同步环比例：车头与"正对水管"的偏差 1° → 附加角速度 rad/s
                                       必须为负（yaw 顺时针正、w 逆时针正，镜像）
                                       更负→车头跟得紧、激光更常两边都有障碍物
                                       太大→车头来回摆；接近 0→车头慢慢歪掉 */

#define NC_W_MAX_DEF           2.8f /* w 总输出限幅(rad/s)，与整车航向环限幅一致
                                       调大→大圈高速时转向够 / 调小→转向温和 */

/* ---------------- 7. 保护与调试打印 ---------------- */
#define NC_LOST_MS            3000U /* 丢目标保护(ms)：两个激光连续这么久都看不到水管 → 判丢目标停车
                                       0=关闭该保护（现场调试两个激光还没调好时先置 0，否则会误停） */

#define NC_TIMEOUT_MS        60000U /* 整圈总超时(ms)：到点强制停车（防陀螺仪异常/卡死转不停）
                                       调大→容错大 / 调小→更安全 */

#define NC_PRINT_MS            200  /* 串口打印周期(ms)，0=关闭（SerialPlot 观察收敛）
                                       打印 10 通道：d D Dt vx vy w lr fb l3 l4 */

#define NC_DT_MAX_S           0.05f /* 单步 dt 上限(s)：防止首次/卡顿后步进过大 */

/* ============================================================================
   运行时可调参数结构体（默认值 / 含义 / 在线调参名见上面参数集中表）
   ============================================================================ */
typedef struct{
  int      stage;         /* 调试阶段 0~3（串口 nstage，见文件头） */
  /* 激光·左右微调 */
  float    lr_speed;      /* 左右微调速度 cm/s（nlrmv） */
  int      lr_chk_ms;     /* 左右微调采样周期 ms（nlrchk） */
  int      lr_ms;         /* 单次左右微调最长持续 ms（nlrms） */
  /* 测距·前后微调 */
  float    fb_speed;      /* 前后微调速度 cm/s（nfbmv） */
  int      fb_target_mm;  /* 目标测距 mm（nfbmm；NCIRCLE_Run 入参也会覆盖它） */
  int      fb_tol_mm;     /* 前后容差 mm（nfbtol） */
  int      fb_trig_mm;    /* 前后触发阈值 mm（nfbtrig） */
  int      fb_chk_ms;     /* 前后微调决策周期 ms（nfbchk） */
  int      fb_ms;         /* 单次前后微调最长持续 ms（nfbms） */
  /* 测距读取/滤波 */
  int      dist_read_ms;  /* 测距读取节流 ms（nrd） */
  float    alpha;         /* 测距低通系数 0~1（nalpha） */
  uint16_t d_min;         /* 测距有效下限 mm（改 NewCircle.h） */
  uint16_t d_max;         /* 测距有效上限 mm（改 NewCircle.h） */
  /* 航向同步环（stage>=3） */
  float    yaw_kp;        /* 航向环比例，负值（nyawkp） */
  float    w_max;         /* w 总输出限幅 rad/s（nwmax） */
  /* 保护 / 打印 */
  int      lost_ms;       /* 丢目标停车阈值 ms，0=关闭（nloss） */
  int      timeout_ms;    /* 整圈总超时 ms（改 NewCircle.h） */
  int      print_ms;      /* 串口打印周期 ms，0=关闭（nprint） */
} NCIRCLE_Param;

/* ============================================================================
   运行状态（调试/串口打印看这些）
   ============================================================================ */
typedef struct{
  uint8_t  running;       /* 1=正在绕圈 */
  int8_t   dir;           /* 方向：+1 逆时针 / -1 顺时针 */
  float    omega;         /* 公转角速度 rad/s */
  float    D_target;      /* 目标轨迹半径 cm（车中心→水管中心） */
  float    D_actual;      /* 实测轨迹半径 cm = d_filt + 偏置 + 管半径 */
  float    d_filt;        /* 滤波后测距 cm */
  uint16_t d_raw;         /* 最近一次原始测距 mm（2000=测空/超量程） */
  uint8_t  d_valid;       /* 最近一次测距是否有效(在 d_min~d_max 之间) */
  /* 激光（左右） */
  uint8_t  l3_hit;        /* LASER3 是否有障碍物（1=有） */
  uint8_t  l4_hit;        /* LASER4 是否有障碍物（1=有） */
  int8_t   lr_dir;        /* 最近一次决策的左右补方向：-1 往左 / +1 往右 / 0 不补 */
  int8_t   lr_out_dir;    /* 当前实际生效的左右补方向（脉冲计时内） */
  uint32_t lr_chk_tick;   /* 上次采激光时刻 ms */
  uint32_t lr_t0;         /* 本次左右补开始时刻 ms */
  uint32_t lr_cnt;        /* 左右补触发次数（调试：看它涨不涨就知道激光判没判到） */
  /* 测距（前后） */
  int16_t  fb_err_mm;     /* 距离偏差 mm：>0 离太远(前进) / <0 离太近(后退) */
  int8_t   fb_dir;        /* 最近一次决策的前后补方向：+1 前进 / -1 后退 / 0 不补 */
  int8_t   fb_out_dir;    /* 当前实际生效的前后补方向 */
  uint32_t fb_chk_tick;   /* 上次前后决策时刻 ms */
  uint32_t fb_t0;         /* 本次前后补开始时刻 ms */
  uint32_t fb_cnt;        /* 前后补触发次数 */
  uint32_t dist_read_tick;/* 上次读测距时刻 ms */
  uint32_t lost_t0;       /* 最近一次"至少一个激光看到水管"的时刻 ms（丢目标保护用） */
  /* 名义圆周 / 航向 */
  float    alpha_orbit;   /* 公转累计角 rad（前馈用的理论值） */
  float    arc_target;    /* 目标弧角 °（360=整圈；到角判停） */
  float    yaw_start;     /* 起始 yaw ° */
  float    yaw_acc;       /* 陀螺仪累积转角 °（判整圈） */
  float    yaw_last;      /* 上次 yaw ° */
  float    yaw_tgt;       /* 航向环目标角 °（stage>=3 时有意义） */
  /* 节拍 */
  uint32_t run_t0;        /* 本次绕圈开始时刻 ms（总超时用） */
  uint32_t tick_last;     /* 上次控制时刻 ms */
  uint32_t print_tick;    /* 上次打印时刻 ms */
} NCIRCLE_State;

extern NCIRCLE_Param nc_param;
extern NCIRCLE_State nc;

/* NCIRCLE_Step() 返回值（0 继续 / 非 0 结束原因，NCIRCLE_Run 会串口打印原因） */
#define NC_RUN_CONTINUE   0   /* 继续绕 */
#define NC_DONE_ARC       1   /* 绕满 arc_deg（正常结束） */
#define NC_DONE_TIMEOUT   2   /* 总超时保护停车 */
#define NC_DONE_LOST      3   /* 丢目标保护停车 */

/**
  * @brief 新方案绕圈（阻塞式）：名义圆周运动 + 激光左右微调 + 测距前后微调
  * @param d_target_mm 目标测距 mm（镜头→水管表面；现场 200mm）。同时会写进
  *                    nc_param.fb_target_mm（前后微调的目标值）
  * @param omega       公转角速度 rad/s（>0；切向速度自动 = omega·D_target）
  * @param arc_deg     绕行弧角 °（360 = 整圈；到角自动停）
  * @param dir         方向：1 逆时针 / -1 顺时针
  * @note  前置条件：flag.chassis = 1（底盘控制循环在跑）；stage >= 3 还需 flag.hwt101ct = 1。
  *        绕圈期间本模块临时把 flag.angle 置 0（w 由本模块接管），结束后恢复并重新锁向。
  *        传感器接线/几何常量见 NewCircle.h 顶部；调用前请让车头正对水管（测距读到的是
  *        垂直距离）——函数内部会先静止采 8 次测距给滤波器定初值。
  *        串口可选调参指令见 NewCircle.h 参数集中表（如 "nstage i 1"）。
  */
void NCIRCLE_Run(uint16_t d_target_mm, float omega, uint32_t arc_deg, int8_t dir);

/**
  * @brief 单步控制（10ms 级；内部按 tick 差值算 dt，无需外部延时）
  * @retval NC_RUN_CONTINUE / NC_DONE_ARC / NC_DONE_TIMEOUT / NC_DONE_LOST
  * @note  测距读取为阻塞式（最坏 50ms，已用 dist_read_ms 节流），本函数不可重入。
  */
uint8_t NCIRCLE_Step(void);

/**
  * @brief 串口打印一次状态（单行多通道，SerialPlot 可直接画图）
  *        通道顺序：d(滤波测距 mm) D(实际半径 mm) Dt(目标半径 mm) vx vy w(×100)
  *                 lr(左右补方向) fb(前后补方向) l3 l4(激光是否有障碍物)
  */
void NCIRCLE_PrintState(void);

#endif /* __NEWCIRCLE_H */
