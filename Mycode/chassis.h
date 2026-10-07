#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "pid.h"
#include "stm32f4xx_hal.h"
#include "velocityProfile.h"   // 梯形速度规划（go_to_xy 移植：mx/my 走固定距离）

/* ==================== 系统开关标志 ==================== */
/* 各模块由对应标志位开启/关闭（main.c 定义变量，chassis.c 等模块引用） */
typedef struct {
  uint8_t hwt101ct;   // 陀螺仪数据更新标志
  uint8_t chassis;    // 底盘控制循环标志
  uint8_t angle;      // 航向环（角度环）开关：1 开启（默认），0 关闭回手动 w
  uint8_t oled_ui;    // OLED UI 显示开关：1 开启，0 关闭（默认）——队友新增：上电 KEY0 调参菜单用
} FLAG;

extern FLAG flag;

/* 轮子编号：与 TB6612 电机编号、编码器编号、TIM1 PWM 通道一一对应 */
typedef enum {
    CHASSIS_MOTOR_LF = 1,  // 左前  TIM1_CH1
    CHASSIS_MOTOR_LB = 2,  // 左后  TIM1_CH2
    CHASSIS_MOTOR_RB = 3,  // 右后  TIM1_CH3
    CHASSIS_MOTOR_RF = 4,  // 右前  TIM1_CH4
} ChassisMotorIndex;

/* ==================== 车体可配置参数（待实测标定） ==================== */
#define WHEEL_DIAMETER   12.7f    // 实测：轮径 127mm
#define MECANUM_LENGTH   22.8f    // 实测：前后轮中心距离 cm（第二阶段用）
#define MECANUM_WIDTH    33.5f    // 实测：左右轮中心距离 cm（第二阶段用）
#define ENCODER_ACCURACY 878.0f   // 实测标定：手转轮10圈=8780脉冲 → 一圈878（对应 4 倍频 TI12）
                                  // 2026-09-18 编码器由 TIM_ENCODERMODE_TI1(2倍频) 改为 TI12(4倍频)，
                                  // 每转脉冲数翻倍 ⇒ 439→878；改完务必手转整圈复核一次
#define ENCODER_TIME_S   0.020f   // 控制周期 20ms（2026-09-18：10ms → 5ms → 20ms；main.c 的 TIM7 计数阈值必须同步）
                                  // ★ 全工程只有 chassis.c 的 4 处用它，语义都是"每周期秒数"：
                                  //   速度环 actual、里程计 now_v_x/now_v_y、梯形规划 ti 累加 —— 改这一个宏就全部同步
                                  // ★ 周期越长，同一个脉冲数代表的速度越小 ⇒ 低速量化台阶越细（分辨率越高）：
                                  //   1 个脉冲 = PERIMETER/ACCURACY/TIME_S，878 编码器下 10ms=4.55、5ms=9.09、**20ms=2.27cm/s**
                                  //   这就是选 20ms 的原因：低速段（如 5cm/s）才可能被测准
                                  // ★ 代价：控制频率只有 50Hz —— 角度环纠偏、梯形规划、速度环的更新都变粗，
                                  //   高速工况下的响应速度与速度曲线平滑度会下降（20ms 内车速最多变化 ~3cm/s）
#define PERIMETER        (3.14159265f * WHEEL_DIAMETER)  // 轮子周长 cm
/* ==================== 速度环 PID 参数（增量式，分段：不同目标速度区间用不同参数） ==================== */
#define SPEED_PID_KP       4.4f    // ★ 这三个宏目前"不被任何代码读取"——控制循环只从 speed_seg 表取值。
#define SPEED_PID_KI       0.04f   //   这里只是"不分段方案的基准兜底值"（2/3 号轮用的就是这组）。
#define SPEED_PID_KD       0.0f    //   ★ 四轮现已分两组（1/4 号轮 5.5/0.05），宏表达不了"分轮"，以 SPEED_PID_DFT 表为准
#define SPEED_PID_OUT_MAX  900.0f   // PWM输出对称限幅（已加大到900，实测最大车速~160cm/s）
#define SPEED_SEG_NUM      4        // 分段数
#define SPEED_SEG_BOUND_1  40.0f    // 段1上边界（0-40）
#define SPEED_SEG_BOUND_2  80.0f    // 段2上边界（40-80）
#define SPEED_SEG_BOUND_3  120.0f   // 段3上边界（80-120）
#define SPEED_TARGET_MAX   160.0f   // 4轮目标速度限幅（实测最大车速~160cm/s），防止麦轮解算出超限目标
/* ==================== 短距精细档（编码器位置闭环判停）默认值 ——— ★2026-10-07 加 / 同日又回退 ====================
   【为什么加】这两个字段原先**全工程没有一行代码赋值**（chassis 全局变量 ⇒ 恒 0）⇒ CHASSIS_Start_Move 里
     fine_move 恒 0 ⇒ chassis.c 判停块的精细档分支从未执行过 ⇒ ROBOT_Move 一直是纯时间开环梯形（∫v dt）。
     填上即启用：两轴目标都 ≤ 本阈值(cm) 的 Move 改由 dist_acc（1 脉冲≈0.455mm）判停 + 按 v²/(2·brkd) 提前断电。
   【为什么又默认关掉】填 10cm 后现场"前后一直想动但动不起来"：梯形是**斜坡**，1cm 小步峰值只有 √(a·d)=12cm/s
     （PWM≈120，正压在左前轮起转死区 110~130 上）且只撑一两个 20ms 周期；而提前量 v²/(2·brkd)+v·0.02 在这种
     小步上 ≈0.99cm ≈ 整个目标 ⇒ 一动就判停。⇒ 本档只适合"距离 ≥3cm 且速度能跑到 20cm/s"的短距走位；几 cm
     的贴靠微调请用 JieTi_Nudge_Y 那套（速度阶跃 + 编码器判停，见 main.c）。要用：先标 brkd（标法见
     串口指令速查.md）再发 sdist f 6 起试。
   ★提前量随 v² 涨（20cm/s→2cm、40cm/s→8.8cm），阈值给大反而半路断电长滑 ⇒ 更容易走歪。 */
#define FINE_MOVE_MAX_DIST_DFT    0.0f  // 精细档距离阈值 cm（★默认 0 = 关；串口 sdist 在线打开，建议 6~10）
#define BRAKE_DECEL_DFT         100.0f  // 断电自然滑停等效减速度 cm/s²（旧注释实测≈100；串口 brkd 在线微调）
                                        // ★只在 sdist>0 时被用到；填 0 会让提前量 = ∞
/* ==================== 校准微调（起转阶跃 + 收尾降速 + 编码器判停）默认值 ——— ★2026-10-07 加 ====================
   给 main.c 的 JieTi_Nudge_Y() 用（逐坑/圆环前后校准的挪车），字段说明见下面 Chassis 结构。
   一趟挪车分两段：① 起转段全速 nudge_speed —— 位置式速度环 P 项 = kp×v ≈ 10×20 = 200 PWM，
   稳稳越过四轮起转死区（实测 60~130；给 10cm/s 只有 ~100 PWM，正好卡在死区里 ⇒ "速度小反而推不动"）；
   ② 收尾段按剩余距离把速度一路降到 nudge_crawl 再断速 ⇒ 断速那一刻速度小，刹车滑行只剩 1~2mm。
   ★为什么必须收尾降速（2026-10-07 现场数据）：一路 20cm/s 冲到断速线再刹，实测要多冲 9~15mm
     —— 一个坑才 16cm、容差 ±5mm，这就是"前后一下子走太多"的来源。
   ★断速余量不再是个固定值：main.c 按**当前速度**现算 v²/(2×220cm/s²) + v×20ms，
     这里的 nudge_brake_cm 只是叠在它上面的**附加量**（冲过头加大 / 差一点没到减小，可为负）。 */
#define NUDGE_SPEED_DFT         20.0f   // 校准微调起转/上限速度 cm/s（serialplot: nspd）
#define NUDGE_CRAWL_DFT          8.0f   // 校准微调收尾爬行下限 cm/s（serialplot: ncrl）—— 再低怕推不动
#define NUDGE_BRAKE_CM_DFT       0.0f   // 校准微调断速余量**附加量** cm（serialplot: nbrk，可为负）
/* ==================== 航向环（角度环）三档参数 ====================
   ★ 2026-09-19 改：kp / ki / kd / bias 四者**每档一套、彼此独立**。
     改之前只有 kp 分档，ki/kd/bias 是三档共用的 —— 调平移档会把旋转档一起带坏。
   档位由控制循环按"有没有平移"+"合速度"自动选，调用侧无需指定：
     旋转档     no_move（v_x≈0 且 v_y≈0）：原地转向 / 被推动后纠偏 —— 要快，kp 该猛
     低速平移档 该值在 [YAW_MOVE_MIN_SPEED, YAW_KP_LOW_SPEED_BOUND] —— 走直线纠偏要柔
                ★ 2026-09-27 用户要求：0~20cm/s 的平移也走本档（用下面 YAW_MOVE_LO_* 那套
                  参数）⇒ YAW_MOVE_MIN_SPEED 定为 0，只要在平移就必落本档，"不介入"不再出现
     高速平移档 该值 >  YAW_KP_LOW_SPEED_BOUND
     （原"不介入"档：max(|vx|,|vy|) < YAW_MOVE_MIN_SPEED 的平移，角度环 w 恒为 0、完全不管
       航向 —— 2026-09-27 起该区间并入低速平移档；想恢复就把 YAW_MOVE_MIN_SPEED 改回 20.0f）
   下面这些宏只是**上电初值**：实际取的是 chassis.yaw_param[档].{kp,ki,kd,bias}，
   可串口在线调（ykp1/2/3、yki1/2/3、ykd1/2/3、ybias1/2/3，档号 1旋转 2低速 3高速），
   调好请把值填回对应宏。
   kp 符号为负：HWT101CT yaw 顺时针为正，与 w 的定义（逆时针为正）镜像 */
#define YAW_MOVE_MIN_SPEED      0.0f  // 平移介入下界（cm/s，判据见下面 max 那一段）：低于它 → 不介入
                                      // ★ 2026-09-27 用户定 0（原 20）：要求 0~20cm/s 的平移也介入，
                                      //   且**采用 20~80cm/s（低速档）那套参数**。判速是
                                      //   max(|vx|,|vy|) 的绝对值（恒 ≥ 0），取 0 后判据 < 恒不成立
                                      //   （两轴都 <0.001 时先被 no_move 判成旋转档，走不到这里）
                                      //   ⇒ 有平移必介入、必落低速档；"不介入"(yaw_gear=1) 不再出现。
                                      // ★ 想回退不用改代码：把本宏填回 20.0f 即恢复
                                      //   "0~20 平移期间不纠偏"的旧行为。
                                      // ★ 旧值 20 的理由（2026-09-19，留档给回退时参考）：20cm 以下的
                                      //   短距平移，平移期间纠偏反而添乱 —— 四轮静摩擦差很大，纠偏那点 w
                                      //   让某个轮先动、别的还没动，距离忽多忽少甚至干脆不动；
                                      //   等平移跑完落回旋转档再纠，效果反而更稳。
                                      //   ★ 这次按用户要求低速也纠偏，上面那条代价跟着回来：极低速段
                                      //   （起步/收尾 v 只有几 cm/s）可能因静摩擦差出现"某个轮先动"，
                                      //   属已知、已被用户接受。
                                      // ★ 覆盖范围：main.c 里大量 ROBOT_MoveSpeed(10~20cm/s) 原来
                                      //   落在"不介入"区间，现在整个行程（含起步/收尾的极低速段）
                                      //   都在向 target_yaw 纠偏。
                                      // ★ 判据是 <（不含等号）：该值 == 20 仍落低速档（回退时有效）
#define YAW_KP_LOW_SPEED_BOUND  80.0f // 低速/高速平移分界（cm/s，同一判据）
                                      // 2026-09-19 用户定 80（原 40）：40~80 这段从高速档改判低速档
/* ★ 分档判速为什么用 max(|vx|,|vy|) 而不是合速度 sqrt(vx²+vy²)（2026-09-19 用户定）：
   麦轮斜移时四轮的平移分量是 vy±vx —— 越接近 45°，必有一对轮子的分量越接近 0，
   与车速快慢无关。合速度在斜移时是虚高的（反映车位移动速度，不是轮子被推着转多快）：
   (15,15) 合速度 21.2 会判"介入"，可慢轮根本没转；(60,60) 合速度 84.9 会判"高速档
   不介入"，可正拖着一个卡住的轮子跑、必然走歪。取 max 分量两个方向都更准。
   （纯轴向平移时两者完全等价 —— main.c 里绝大多数调用都是纯轴，改判据对它们无影响。）*/

/* 旋转档：2026-09-19 落地实测定稿（= 串口 ykp1 f -0.013 / yki1 0 / ykd1 f 0.025 / ybias1 f 0.083）
   ★ kp 由 -0.014 微调到 -0.013（用户实测）
   ★ kd 非 0 是这版的特征 —— "加 D 反而抖"是早期无偏置时的经验，有了 bias 后 D 才压得住过冲 */
#define YAW_TURN_KP    (-0.0136f)
#define YAW_TURN_KI    0.0f
#define YAW_TURN_KD    0.025f
#define YAW_TURN_BIAS  0.083f

/* 低速平移档：2026-09-19 用户实测落地
   （= 串口 ykp2 f -0.1 / yki2 f 0.0 / ykd2 f 0.04 / ybias2 f 0.03）
   ★ 从"全 0（角度环完全不参与）"变成非 0 之后，凡落这一档的平移都会带上航向纠偏 ——
     包括串口 mx/my（默认 20cm/s）和 main.c 里那十几处 ROBOT_MoveSpeed(10~20cm/s)。
     以前这些平移跑偏了是没人管的，现在会被揪回来。 */
#define YAW_MOVE_LO_KP    (-0.13f)
#define YAW_MOVE_LO_KI    0.0f
#define YAW_MOVE_LO_KD    0.04f
#define YAW_MOVE_LO_BIAS  0.03f

/* 高速平移档：2026-09-19 用户实测定稿
   （= 串口 ykp3 f -0.035 / yki3 f 0.0 / ykd3 f 0.025 / ybias3 f 0.0）
   ★ bias 为 0（= 关闭偏置）：高速平移时轮子本来就在转（车速 > 80cm/s），不需要偏置去
     克服静摩擦 —— 偏置是给旋转档/低速档那种"w 太小推不动轮子"的场合用的。 */
#define YAW_MOVE_HI_KP    (-0.035f)
#define YAW_MOVE_HI_KI    0.0f
#define YAW_MOVE_HI_KD    0.025f
#define YAW_MOVE_HI_BIAS  0.0f

#define YAW_PID_OUT_MAX  2.8f    // w 输出上限 rad/s
#define YAW_PID_OUT_MIN  (-2.8f) // w 输出下限 rad/s
#define YAW_DEAD_ZONE_TURN  1.20f   // 静止旋转死区（°）：误差进入该范围 → w 归零（只归零 w，不停速度环）
                                    // 防小w输出累加到增量式速度环PWM、克服静摩擦猛动造成来回飘动（实测有效）
                                    // 2026-09-19 用户定 0.92（1.0 → 0.65 → 0.75 → 0.92，实测微调）
                                    // ★ 这是本值唯一的定义处，其他文件一律引用本宏、不写死数字
                                    // ★ 死区同时是"到位"判据（ROBOT_Angle 靠它返回）：改小就是要求车多纠
                                    //   那点角度，而小误差下 w 只等效几 cm/s、靠增量式速度环慢慢累积分
                                    //   才推得动，可能推不进去。若 ROBOT_Angle 开始频繁等满
                                    //   YAW_STOP_TIMEOUT_MS(3000ms) 才返回，就是这个原因。
#define YAW_DEAD_ZONE_MOVE  0.05f    // 走直线(有平移)死区（°）：更小，让1°内偏航也被角度环纠正，直线更直
                                    // 两套死区由控制循环按 v_x/v_y 是否≈0 自动切换，无需在线调
/* ==================== 航向环输出偏置 bias（静摩擦补偿，每档一个） ====================
   角度环是 P 控制：w = kp·error。误差只剩零点几度时 w 很小，经麦轮解算
   (half_sum=28.15) 成四轮目标速度只剩几 cm/s —— 低于起转 PWM(悬空实测 60~110)，
   轮子推不动、误差不减小，就卡在死区外，ROBOT_Angle 要等很久才返回。
   叠一个与误差同向的固定偏置后，任何非零误差都有"推得动轮子"的最小输出。
   （pid.c 底部注释掉的旧"输出偏移"改进措施就是这个思路。）
   ★ 单位 rad/s（与 w 一致）。换算关系：w × 28.15 = 四轮目标速度 cm/s，
     即 0.178rad/s ≈ 5cm/s、0.7rad/s ≈ 20cm/s。
   ★ 值必须实测扫：太小 → 推不动，老问题照旧；太大 → 进死区刹不住、来回摆。
   ★ 0 = 关闭本档的偏置（平移档默认就是 0）。串口 ybias1/2/3 在线调。 */
#define YAW_TARGET_NONE  (-999.0f)  // target_yaw 哨兵：首次进入控制循环时锁定当前陀螺仪朝向
/* ==================== 航向环档位状态（含"停止档"，2026-09-19 加） ====================
   上面三个 yaw_kp_* 回答的是"用哪套参数"，这里的档位回答的是"当前处于什么状态"——
   供 ROBOT_Angle 判断"真的转到位、而且车真的停稳了"再返回（ROBOT_Move 也靠它表示跑完了）。
   ★ 由控制循环每周期刷新，不锁存：车又动了就自动退回运动档。
   ★ 别拿它当"用哪个 kp"的依据 —— kp 是按 no_move + 合速度现算的（chassis.c 控制循环），
     两者独立，只是恰好都能用 no_move 表达"无平移"。 */
#define YAW_STAGE_TURN     0   // 旋转档：无平移（原地转向 / 被推动后纠偏）
#define YAW_STAGE_MOVE_LO  1   // 低速平移档：YAW_MOVE_MIN_SPEED ≤ max(|vx|,|vy|) ≤ YAW_KP_LOW_SPEED_BOUND
                               //   ★ 2026-09-27 起 YAW_MOVE_MIN_SPEED=0 ⇒ 只要在平移就落本档
                               //     （0~20cm/s 也用本档那套 kp/ki/kd/bias，不再"不介入"）
                               //   ★ 旧行为（留档）：低于 YAW_MOVE_MIN_SPEED(20) 的平移**角度环不介入**
                               //     （w 恒 0），但档位仍报本档 —— 档位只有三个，"介不介入"是另一个开关
                               //     （控制循环里的 yaw_on），不为它新增枚举值
#define YAW_STAGE_MOVE_HI  2   // 高速平移档：合速度 >  该阈值
#define YAW_STAGE_STOP     3   // ★ 停止档：误差已在死区内 且 四轮都停稳（连续确认过）

/* ---- 停止档判据 ----
   "到位"和"停稳"是两回事：误差进死区只说明某一瞬间的采样点在 1° 内，车可能还在惯性滑行。
   所以停止档要求 误差在死区内 **且** 四轮速度都低于阈值，连续保持 YAW_STOP_CONFIRM 个周期。*/
#define YAW_STOP_SPEED_TH   3.0f   // 单轮算"停住"的速度阈值 cm/s（≈1 个编码器量化台阶，同 MOVE_STOP_SPEED_TH）
                                   // ★ 不能判严格 ==0：20ms 下 1 脉冲就是 2.27cm/s，车基本停住时
                                   //    编码器偶尔抖出 1 个脉冲，actual 会在 0 和 ±2.27 之间跳
#define YAW_STOP_CONFIRM    10     // 判据要连续满足多少个控制周期才算停稳（10 × 20ms = 200ms）
#define YAW_STOP_TIMEOUT_MS 8000   // 误差进死区后最多再等多久让它停稳，到点强制置停止档。
                                   // ★ 必须有这个兜底：死区边缘来回蹭时"四轮都停"永远不成立
                                   //    （航向环微动 + 惯性反复越过死区），没有它 ROBOT_Angle 会死等。
                                   // ★ 0 = 关闭超时（死等真停稳），蹭起来会导致比赛流程卡死，慎用
                                   // 2026-09-21 用户定 8000ms（原 3000）：实际极少等满，正常都在
                                   // YAW_STOP_CONFIRM 那 200ms 就确认出去了，这里只是保险丝
/* ==================== 梯形速度规划（go_to_xy 移植：串口 mx/my 走固定距离自动停） ==================== */
#define MOVE_SPEED_DEFAULT  20.0f    // 规划目标速度 cm/s（serialplot param 表 mv 可在线调）
                                     // 2026-09-19 用户定 20（原 60）：恰好卡在 YAW_MOVE_MIN_SPEED
                                     // 下界上（判据是 <，20 不算"低于20"）⇒ mx/my 默认仍落
                                     // **低速平移档**、平移期间会纠偏
                                     // ★ 2026-09-27：YAW_MOVE_MIN_SPEED 改为 0 后，再往下调
                                     //   （10 / 5cm/s）也仍是低速平移档、照常纠偏，不再有"不介入"
#define MOVE_ACC_DEFAULT    30.0f    // 规划加减速 cm/s²（serialplot param 表 mvacc 可在线调）
                                     // 2026-09-19 用户定 30（原 100）：跟着 20cm/s 的速度降下来

/* ==================== 速度环 PID 参数（每轮一套 × 每速度段一组 kp/ki/kd） ==================== */
typedef struct{
  float kp;  // 比例增益
  float ki;  // 积分增益
  float kd;  // 微分增益（速度环一般置0）
}SpeedSegParam;
typedef struct{ int16_t fwd; int16_t rev; }StartPwm_t;  // 单轮正/反转启动 PWM 绝对值（静止→持续转动所需）

/* ==================== 航向环单档参数（每档一套 kp/ki/kd/bias，彼此独立） ====================
   数组下标直接用档位常量：YAW_STAGE_TURN(0) / YAW_STAGE_MOVE_LO(1) / YAW_STAGE_MOVE_HI(2)。
   ★ 数组只有 3 项，不含 YAW_STAGE_STOP(3) —— 停止档是"状态"，不参与取参数 */
typedef struct{
  float kp;    // 比例增益（符号为负，见上面的说明）
  float ki;    // 积分增益
  float kd;    // 微分增益
  float bias;  // 输出偏置 rad/s（静摩擦补偿，0=关闭本档偏置，换算见上面的 bias 说明）
}YawStageParam;

/* ==================== 车体结构体 ==================== */
/* 目标速度 = speed_pid[i].target，实测速度 = speed_pid[i].actual，PWM 输出 = speed_pid[i].out */
typedef struct {
    PID_INC  speed_pid[5];       // 4轮速度环（增量式；索引0不用；1~4 对应 TIM1_CH1~4）
    SpeedSegParam speed_seg[5][SPEED_SEG_NUM]; // [轮号1..4][速度段0..3] 每轮各自一套（索引0占位不用，与 speed_pid 编号一致）
                                               // 参数初值表在 CHASSIS_Init：改哪一轮就改那一行，其余代码不用动
                                               // 控制循环按 |target| 选段，再按轮号取该轮的 kp/ki/kd
    /* ==================== 位置式速度环（2026-09-25 加，2026-09-27 接入本工程） ====================
       与上面的增量式 speed_pid[] **并存**，由 speed_pos_mode 二选一，两套状态互不干扰：
         speed_pos_mode = 0 → 走 speed_pid[]     + PID_IncUpdate     （原逻辑，一行没动）
         speed_pos_mode = 1 → 走 speed_pid_pos[] + PID_PosSpeedUpdate（新逻辑，★上电默认）
       ★ 两者**共用上游**：麦轮解算/梯形规划把目标速度写在 speed_pid[i].target，编码器换算的实测
         速度写在 speed_pid[i].actual。位置式每周期把这两个值拷进自己再运算 ⇒ 上游一行都不用改，
         里程计/零漂保护/机器人层判停（robot.c 读 speed_pid[i].target/actual）同样一行都不用改。
       ★ 位置式的参数、限制、积分条件**全在 speed_pid_pos[i] 自己身上**（不查 speed_seg 表、不分速度段），
         初值在 CHASSIS_Init，串口在线调（见 serialplot.c 的位置式指令块）：
             pkp1~pkp4 / pki1~pki4 / pkd1~pkd4  f 值   单轮 kp/ki/kd
             pomax / pomin f 值                        输出上下限（PWM，默认 ±900）
             pimax  f 值                               误差积分限幅（限的是累加量 Σerror，不是 i_out）
             psep   f 值                               积分分离阈值：|误差|>它就不积分（0=关闭）
             pidz   f 值                               积分死区阈值：|误差|<它就不积分（0=关闭）
             pkip   f 值                               |ki| 低于它 → 视为无积分并清 errorint（默认 0.001）
             pmode  i 0/1                              增量式/位置式切换
       ★ 队友强调的两点，别搞混：
         ① 位置式的参数**不在 speed_seg 那张表里**（增量式按速度段分 4 档、1/4 号轮一组 2/3 号轮一组），
            位置式是四轮各一份、不分速度段，存在 chassis.speed_pid_pos[1..4]；
         ② 所以发 pq 看参数时，**W1~W4 那段是增量式的、P1~P4 那段才是现在生效的**。
       ★ 上电默认**位置式**。要切回增量式：串口发 pmode i 0，立即生效、无需复位
         （也是出问题时最快的回退手段）。 */
    PID_POS  speed_pid_pos[5];   // 4轮位置式速度环（索引0不用；1~4 对应 TIM1_CH1~4）
    uint8_t  speed_pos_mode;     // 0=增量式 1=位置式（CHASSIS_Init 置 1；串口 pmode i 0/1 在线切）
    /* ---- 调试手动定速（2026-09-25 加；默认 0，为 0 时整车逻辑一行都不参与）----
       speed_dbg_manual=1 时，CHASSIS_SpeedLoop 里把四轮 target 全钉成 speed_dbg_tgt、
       旁路麦轮解算 ⇒ 目标速度不再是往复的梯形（峰值只有 20cm/s 且一直在变），
       而是一直钉在你给的值上，便于考察 40/80/120cm/s 下 kp/ki 还够不够用。
       串口：ptgt f 40 → 定速 40cm/s（负=后退，0=目标0）   pauto i 0 → 回到自动（车停） */
    uint8_t  speed_dbg_manual;
    float    speed_dbg_tgt;
    /* ==================== 单轮速度环临时调参（2026-09-17 调试用；speed_tune=0 时以下字段不参与任何逻辑） ====================
       用途：一次只让一个轮子转，用一组"临时 kp/ki/kd"覆盖全轮、全速段（不查上面的表、不分段），
       在串口上把这组值试出来之后，自己填进 CHASSIS_Init 的 SPEED_PID_DFT 表。 */
    uint8_t  speed_tune;         // 1=单轮调参模式：只跑速度环，旁路 梯形规划/航向环/麦轮解算
    uint8_t  tune_wheel;         // 被调轮号（1左前 2左后 3右后 4右前），由 SERIALPLOT_SpeedTuneLoop(轮号) 设定
    float    tune_kp;            // 临时 kp（覆盖 SPEED_PID_DFT 表，调好后再填表）
    float    tune_ki;            // 临时 ki
    float    tune_kd;            // 临时 kd
    int16_t  tune_pulse;         // 被调轮本周期编码器脉冲（串口观察低速量化台阶：1脉冲≈9.09cm/s）
    float v_x;                   // 整车x方向速度 cm/s（右移为正）
    float v_y;                   // 整车y方向速度 cm/s（前进为正）
    float w;                     // 整车角速度 rad/s（逆时针为正）
    uint8_t  stop_flag;          // 停车标志：1 时停止输出
    /* 航向环（角度环）：flag.angle=1 时 w 由角度环输出接管（默认开启） */
    PID_POS  yaw_pid;            // 航向环：target=目标角度、actual=实测角度、out=输出w(rad/s)
    float    target_yaw;         // 航向环目标角度（HWT101CT 0~360°，哨兵锁定当前朝向）
    float    yaw;                // 航向环实际角度（镜像 HWT101CT_Data.yaw）
    /* 航向环 kp 三档：控制循环每周期按工况选一个赋给 yaw_pid.kp；串口 ykp1/ykp2/ykp3 在线调 */
    /* 航向环三档参数：控制循环每周期按工况选一档，把它整套写进 yaw_pid.kp/ki/kd 并取 bias。
       串口 ykp1/2/3、yki1/2/3、ykd1/2/3、ybias1/2/3 在线调（档号 1旋转 2低速 3高速） */
    YawStageParam yaw_param[3];  // [YAW_STAGE_TURN]=旋转 [YAW_STAGE_MOVE_LO]=低速 [YAW_STAGE_MOVE_HI]=高速
    /* 航向环档位状态（见 YAW_STAGE_*）：控制循环每周期刷新，ROBOT_Angle 靠它判断到位且停稳 */
    uint8_t  yaw_stage;          // 当前档位（YAW_STAGE_TURN/MOVE_LO/MOVE_HI/STOP）
    uint8_t  yaw_gear;           // ★档位编码（仅供串口观察，不参与控制）：0旋转 1不介入 2低速 3高速
                                 //   ★ 2026-09-27：YAW_MOVE_MIN_SPEED=0 ⇒ 1(不介入) 不再出现，
                                 //     0~20cm/s 的平移报 2(低速)；把该宏改回 20.0f 就会重新出现
                                 //   与 yaw_stage 的区别：yaw_stage 判停稳后会变成 STOP(3)，与本编码
                                 //   的"3=高速"撞车；且它表达"停没停稳"，不表达"介不介入"
    uint8_t  yaw_stage_cnt;      // 停止档判据已连续满足的周期数（控制循环内部用）
    uint16_t yaw_stage_dz_ms;    // 误差进死区后累计 ms（停止档超时计时；出死区清零）
    /* 里程计（④移植自旧代码 RobotCalculate：本车麦轮正解 + 标准旋转矩阵，位置全局坐标） */
    float    pos_x;              // 全局x坐标 cm（右移为正）
    float    pos_y;              // 全局y坐标 cm（前进为正）
    float    now_v_x;            // 当前整车x速度 cm/s
    float    now_v_y;            // 当前整车y速度 cm/s
    float    now_the;            // 当前朝向角 rad（车头相对全局x轴，逆时针为正）
    /* 梯形速度规划（go_to_xy 移植：串口 mx/my 走固定距离自动停，时间开环照搬旧代码） */
    TrapeVelprofile_t tp_x;          // x方向梯形规划曲线（起点静止、末速0、目标 move_speed）
    TrapeVelprofile_t tp_y;          // y方向梯形规划曲线
    uint8_t  x_speed_plan_flag;      // x轴规划进行中（1规划中，0结束）
    uint8_t  y_speed_plan_flag;      // y轴规划进行中
    uint8_t  x_set_speed_flag;       // x轴手动设速标志（串口 vx 置1：中断不清零 v_x）
    uint8_t  y_set_speed_flag;       // y轴手动设速标志（串口 vy 置1）
    float    speed_dir_x;            // x方向符号（+1右移 / -1左移）
    float    speed_dir_y;            // y方向符号（+1前进 / -1后退）
    float    ti;                     // 规划计时 s（每周期累加 ENCODER_TIME_S，20ms）
    float    move_speed;             // 规划目标速度 cm/s（param 表 mv 可调）
    float    move_acc;               // 规划加减速 cm/s²（param 表 mvacc 可调）
    /* 启动阈值整形 + 到位判停（2026-09-04 新增）：接口不变，见设计文档 */
    float    start_margin;        // 起转裕量（落地现场整定，serialplot param 名 stm）
    float    dist_acc_x;          // 本段规划已走距离 cm（车体系，fabs 累计，CHASSIS_Start_Move 清零）
    float    dist_acc_y;
    float    move_target_x;       // 本段目标距离 cm = |x_dist|（CHASSIS_Start_Move 记录，判停用）
    float    move_target_y;       // = |y_dist|
    float    brake_decel;        // 断电自然滑停等效减速度 cm/s²（判停提前量 = 当前速度²/(2·brake_decel)；serialplot brkd 在线调）
    uint8_t  fine_move;          // 精细档标志：1=本次 Start_Move 属短距(两轴≤sdist)或低速恒速(lspd)，启 整形+提前判停；0=常规档(大距/高速)纯时间开环
    float    fine_max_dist;      // 短距精细档判定阈值 cm（serialplot: sdist，默认 FINE_MOVE_MAX_DIST_DFT=0 即关）
    float    fine_max_spd;       // 低速精细档判定阈值 cm/s（serialplot: lspd，默认 FINE_MOVE_MAX_SPD_DFT）
    /* ---- 校准微调（JieTi_Nudge_Y：起转阶跃 + 收尾降速 + 编码器判停）的三个现场旋钮 ——— ★2026-10-07 加 ----
       和上面 fine_* 一样放在 chassis 里（而不是 main.c 的宏），因为这三个值**必须现场标**：
         nudge_speed    起转/上限速度 cm/s：要够大才压得过四轮起转死区（serialplot: nspd，默认 NUDGE_SPEED_DFT）
         nudge_crawl    收尾爬行下限 cm/s：越接近断速线速度越往这个值降（serialplot: ncrl，默认 NUDGE_CRAWL_DFT）
                        ★降速后推不动（日志会出现 "stall"）就往上加；总感觉收尾太磨蹭就减到 6
         nudge_brake_cm 断速余量**附加量** cm：真正的余量按当前速度现算 v²/(2a)+v×20ms，这里只是加在它上面
                        （serialplot: nbrk，默认 NUDGE_BRAKE_CM_DFT=0；总是冲过头就加大、总是差一点没到就减小） */
    float    nudge_speed;        // 校准微调起转/上限速度 cm/s（默认 NUDGE_SPEED_DFT）
    float    nudge_crawl;        // 校准微调收尾爬行下限 cm/s（默认 NUDGE_CRAWL_DFT）
    float    nudge_brake_cm;     // 校准微调断速余量的附加量 cm（默认 NUDGE_BRAKE_CM_DFT，可为负）
} Chassis;

extern Chassis chassis;

void CHASSIS_Init(void);
void CHASSIS_Control_Loop(void);
void CHASSIS_Mecanum(void);      // 整车速度(v_x,v_y,w) → 4轮目标速度
void CHASSIS_Odom_Calculate(const int16_t pulse[5]);  // 里程计：4轮脉冲 → 车体位移 → 全局坐标积分
void CHASSIS_Start_Move(float x_dist, float y_dist, float x_speed, float y_speed, float x_acc, float y_acc);
  // 梯形规划启动（非阻塞）：x/y方向走固定距离 cm，速度/加减速分别指定；走完自动停

#endif
