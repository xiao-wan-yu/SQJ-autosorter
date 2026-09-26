/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stm32f4xx_hal.h>
#include "./../../Mycode/led.h"
#include "./../../Mycode/oled_api.h"
#include "./../../Mycode/uart.h"
#include "./../../Mycode/delay.h"
#include "./../../Mycode/key.h"
#include "./../../Mycode/buzzer.h"
#include "./../../Mycode/laser.h"
#include "./../../Mycode/gy53.h"
#include "./../../Mycode/tb6612.h"
#include "./../../Mycode/chassis.h"
#include "./../../Mycode/encoder.h"
#include "./../../Mycode/serialplot.h"
#include "./../../Mycode/pid.h"
#include "./../../Mycode/myflash.h"
#include "./../../Mycode/storage.h"
#include "./../../Mycode/gw_grayscale.h"
#include "./../../Mycode/tcs34725.h"
#include "./../../Mycode/hwt101ct.h"
#include "./../../Mycode/robot.h"
#include "./../../Mycode/vision.h"
#include "./../../Mycode/lobot_servo.h"

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include "stm32f407xx.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
#include "stm32f4xx_hal_uart.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* FLAG 结构体定义已移至 Mycode/chassis.h（main.c 定义变量、chassis.c 等模块引用） */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* UART1 接收模板：帧格式 S;A;B;C;D;E;F;G（8个32位整数，分号分隔） */
#define UART1_DATA_NUM 8                     // 每帧数据个数，按需修改

/* ==================== ★★ 调试用"起始阶段"选择(按键0) ★★ ====================
   开机进"红蓝方选择"界面后：按键0 = 循环切换起始阶段，按键3 = 按选好的阶段开始执行。
   ★默认值(0=完整流程)时，主流程一条语句都不变，和不加这套调试入口时完全一样。
   新增一个起点只需4步：① 这里加一个 DBG_START_xxx；② 在 DBG_START_NAME/DESC 表里加一项；
                        ③ 在 main() 的"调试起点跳转"处加一个 goto；④ 在对应阶段开头打个标签(xxx_START:) */
#define DBG_START_ALL        0      //完整流程：圆盘机→仓库倒球→正面识别→阶梯→立柱→仓库倒方块→回家(默认)
#define DBG_START_ZHENGMIAN  1      //从"正面识别前"开始：跳过圆盘机+仓库倒球，车自己走到阶梯识别位
#define DBG_START_LIZHU      2      //★从"立柱"开始：直接跑 立柱前校准 → 绕柱转圈 → 仓库倒方块 → 回家
#define DBG_START_HUIJIA     3      //从"回家"开始：跳过前面全部，直接跑回家那段(红蓝区找色→停进红蓝区)
#define DBG_START_COLORCAL   4      //★颜色传感器单独校准：黑→红→蓝→白 四色重标定判色阈值(与比赛流程无关，跑完回菜单)
#define DBG_START_MAX        DBG_START_COLORCAL  //按键0 循环切换的上限(=最后一个起点)

/* ==================== ★★ 移动速度三档标准（2026-09-20 换新底盘后统一）★★ ====================
   队友重调的新底盘取消了"破静摩擦整形"：速度环第一拍输出 ≈ kp×目标速度（每轮 kp≈5.5），
   实测起转 PWM 60~110 ⇒ 目标速度 ≥20cm/s 才会立刻走；≤10cm/s 要等积分项爬升 ≈0.2~0.3s。
   所以速度按"用途"分三档，**主流程所有移动指令统一用这三个宏，不再各处手写数字**：
     ① SPD_SHORT_V/A = 20/30   —— 定点走位：有明确终点、走固定距离的 ROBOT_Move
        （两轴里较大距离 < 50cm；≥50cm 用长距档）
     ② SPD_AVG_V     = 10      —— 匀速靠近 / 边判边走：等激光、等测距、等灰度/颜色、视觉对准、
        前后脉冲校准这一类。**速度只给 V（恒速），没有终点、由传感器/视觉决定什么时候停**
     ③ SPD_LONG_V/A  = 120/120 —— 长距高速跑图（两轴里较大距离 ≥ 50cm；上限 160 = SPEED_TARGET_MAX）
   ★为什么"判断类"用 10 而不是 20（越慢落点越准）：
     GY53 测距的 PWM 档更新只有 ≈5Hz(200ms)、颜色判色带 150ms 去抖 ⇒ 从"条件成立"到"停稳"
     车还要多走 速度×0.2~0.35s：命令 10cm/s ≈ 2~4cm、命令 20cm/s ≈ 4~7cm。
     所以判断类宁可慢：多花一点时间，换落点稳定。要整体调速只改 SPD_AVG_V 一个数。
   ★距离为 0 的那个轴，速度/加速度填什么都一样（规划长度0、判停也跳过它），一律填同一组。
   ★两个特例（故意不按标准）：
     ① 极短位移（<13cm，如 3cm / 11.5cm 那几条）：20/30 下三角波峰值只有 ≈9~19cm/s < 20，
        第一拍可能推不动（先迟滞、再靠积分窜出去）。现场若发现"走不到位"，把那一行的 max_a
        单独加大到 100~200（只影响那一条）。
     ② "前后抖"两条（ROBOT_Move(0,±2,0,100,0,100)）是故意快抖、不是走位，保留 100。
   ★判断类里的"一步"（脉冲式：给速度+限时）原本用于阶梯的前后/左右校准，2026-09-21 已随校准一起删掉；
     原因是 ROBOT_Move 单次 ≤3cm 走不动（实测 ≥4cm 才起得来），而校准要修的正是几厘米的偏差。 */
#define SPD_SHORT_V   20     // ① 定点走位 目标速度 cm/s
#define SPD_SHORT_A   30     // ① 定点走位 加减速 cm/s^2
#define SPD_AVG_V     10     // ② 匀速靠近/边判边走 恒速 cm/s（5好像太慢了）
#define SPD_LONG_V   120     // ③ 长距高速 目标速度 cm/s（别超 160）
#define SPD_LONG_A   120     // ③ 长距高速 加减速 cm/s^2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FLAG flag;

int16_t data_encoder = 0;

int32_t UART1_Data[UART1_DATA_NUM] = {0};    // 存放解析后的8个32位整数
uint8_t CAM_Data[7] = {0};    // 存放视觉发送来的坐标数据（一帧：包头0xA1|类型|x两字节|y两字节|包尾0x0B）

bool mode_red = true;  // 红蓝模式标志：true=红方，false=蓝方

//阶梯(8物块)夹取工作模式：1=按顺序计数(第二字节 0x00不夹/0x01夹)；2=视觉反馈编号(0xMN: M=第几个物块1~8, N=1夹/0不夹, 如0x11=第1个夹、0x60=第6个不夹)
uint8_t JieTi_Grab_Mode = 1;   // 现场切换时改这里（置1/置2）

/* ★调试起始阶段的名字/说明(顺序必须和 DBG_START_xxx 一致)：OLED 第3行"k3:GO xxx"+第4行说明用，
   8x16半高字体一行最多16个字符，名字/说明都别写太长 */
static const char * const DBG_START_NAME[] = { "ALL", "ZM", "LZ", "HOME", "CAL" };
static const char * const DBG_START_DESC[] = { "full flow", "front recog", "pillar run", "go home", "color calib" };


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  if(htim == &htim7){ //1ms产生一次中断

    // static uint8_t count1 = 0;
    // if(++count1 >= 5){
    //   count1 = 0;
    //   ICM42688Mahony_Update();
    // }

    // static uint16_t ms_cnt = 0;
    // if(++ms_cnt >= 10){          // 每10ms执行一次车体控制周期
    //   ms_cnt = 0;
    //   CAR_Control_Loop();        // PID速度闭环 + 麦轮运动学 + 里程计
    // }


    //hwt101ct陀螺仪数据更新
    static uint8_t count1 = 0;
    if(flag.hwt101ct && ++count1 >= 5){
      count1 = 0;
      if(HWT101CT_RxFlag){
        HWT101CT_RxFlag = 0;
        HWT101CT_Update();
      }
    }


    //车体20ms控制周期：速度闭环（第一阶段）—— ★计数阈值必须和 chassis.h 的 ENCODER_TIME_S(0.020f) 同步
    static uint16_t count2 = 0;
    if(flag.chassis && ++count2 >= 20){
      count2 = 0;
      CHASSIS_Control_Loop();      // 4轮速度环（PID + 编码器 + PWM）
    }

  }
  /* 清零法无需编码器溢出中断（TIM3/TIM4），故无 htim3/htim4 分支 */

}





/* USER CODE END 0 */

/* ================= 正面识别通信显示(整个"正面识别"阶段只有这一个函数在动屏幕) =================
   一屏四行，位置固定，一眼看出"走到第几步 + 两个串口各自收到了什么"：
     y= 0  8x16  进度0="ZM 0/3 TO POS"(还没发0xA2) → 1="ZM 1/3 SEND OK" → 2="ZM 2/3 LETTER OK" → 3="ZM DONE 0xA7 OK"
     y=16  8x16  "V4 A,B 0xAB OK"      副视觉(备用串口UART4)：WAIT / 字母(A~D)+原始字节 / BAD
     y=32  8x16  "V2 0xA7 OK"          主视觉(串口2)：WAIT / 0xA7 OK / 收到的其它字节(BAD)
     y=48  6x8   "r4:1 r2:2 U4:RX 12s" 收帧数 + 串口4在听(RX正常/ERR停过) + 进入本阶段秒数(心跳)
   ★状态一变(或100ms心跳)就调用一次：函数内部把四行一次性画完再刷新，
     不会出现"只改半屏 / 字符串变短留旧字"的花屏。
   rx4/rx2 传主循环里的收帧数 comm_rx4/comm_rx2；字母直接取 ZhengMian_Letter(0xA~0xD → 'A'~'D') */
#define ZM_ST_PREP    0      //进度0：还没发0xA2(正在移动到识别位)
#define ZM_ST_SEND    1      //进度1：0xA2已发给主视觉+副视觉
#define ZM_ST_LETTER  2      //进度2：副视觉字母已收到→已转发主视觉+已回执副视觉
#define ZM_ST_RECV    3      //进度3：主视觉的0xA7已收到，正面识别结束
#define ZM_RES_WAIT   0      //结果：还没收到
#define ZM_RES_OK     1      //结果：收到有效内容
#define ZM_RES_BAD    2      //结果：收到无效内容

static uint8_t  zm_step     = ZM_ST_PREP;   //通信进度(0=还没发0xA2)
static uint8_t  zm_v4_res   = ZM_RES_WAIT;  //副视觉(串口4)结果状态
static uint8_t  zm_v4_byte  = 0;            //副视觉原始字节
static uint8_t  zm_v4_len   = 0;            //副视觉这一帧的帧长(帧长不是1时显示长度)
static uint8_t  zm_v2_res   = ZM_RES_WAIT;  //主视觉(串口2)结果状态
static uint8_t  zm_v2_byte  = 0;            //主视觉原始字节
static uint32_t zm_t0       = 0;            //进入本阶段的时刻(第4行"秒数"用)

/* 副视觉正面识别结果(0xAB~0xCD)：高4位=第一个字母、低4位=第二个字母(存原始nibble，0xA~0xD)
   ★原来声明在 main() 里面，移到文件顶部是为了让显示函数 ZM_ShowComm() 也能读到，不重复存一份 */
uint8_t ZhengMian_Letter[2] = {0, 0};

/* nibble(0xA~0xD) → 字母('A'~'D')；不是这几个值时原样变成字符，方便一眼看出收到了什么 */
static char ZM_Nibble2Char(uint8_t nib){
  return (nib >= 0x0A && nib <= 0x0D) ? (char)('A' + nib - 0x0A) : (char)('0' + (nib & 0x0F));
}

static void ZM_ShowComm(int rx4, int rx2){
  /* 整屏清掉(0~63行)：①新字符串比旧的短时不会留旧字；
     ②清掉上一屏"在 y=48 用 8x16 大字体"残留的下半截(y=56~63)——
       不清的话它会顶在第4行的6x8小字下面，看起来像两行小字互相覆盖 */
  OLED_Clear();

  /* 第1行：通信进度 */
  if(zm_step == ZM_ST_PREP)        OLED_Printf(0,  0, OLED_8X16_HALF, "ZM 0/3 TO POS");   //还没发0xA2(移动中)
  else if(zm_step == ZM_ST_SEND)   OLED_Printf(0,  0, OLED_8X16_HALF, "ZM 1/3 SEND OK");
  else if(zm_step == ZM_ST_LETTER) OLED_Printf(0,  0, OLED_8X16_HALF, "ZM 2/3 LETTER OK");
  else                             OLED_Printf(0,  0, OLED_8X16_HALF, "ZM DONE 0xA7 OK");

  /* 第2行：副视觉(备用串口UART4)的字母结果 */
  if(zm_v4_res == ZM_RES_OK){
    OLED_Printf(0, 16, OLED_8X16_HALF, "V4 %c,%c 0x%02X OK",
                ZM_Nibble2Char(ZhengMian_Letter[0]), ZM_Nibble2Char(ZhengMian_Letter[1]), zm_v4_byte);
  }else if(zm_v4_res == ZM_RES_BAD){
    if(zm_v4_len == 1) OLED_Printf(0, 16, OLED_8X16_HALF, "V4 0x%02X BAD", zm_v4_byte);
    else               OLED_Printf(0, 16, OLED_8X16_HALF, "V4 len%d BAD", zm_v4_len);
  }else{
    OLED_Printf(0, 16, OLED_8X16_HALF, "V4 WAIT letter");
  }

  /* 第3行：主视觉(串口2)的确认 */
  if(zm_v2_res == ZM_RES_OK)       OLED_Printf(0, 32, OLED_8X16_HALF, "V2 0xA7 OK");
  else if(zm_v2_res == ZM_RES_BAD) OLED_Printf(0, 32, OLED_8X16_HALF, "V2 0x%02X BAD", zm_v2_byte);
  else                             OLED_Printf(0, 32, OLED_8X16_HALF, "V2 WAIT 0xA7");

  /* 第4行(6x8)：收帧数 + 串口4接收状态 + 进入本阶段秒数(一直在涨=程序在跑，不动=死等在某一步) */
  OLED_Printf(0, 48, OLED_6X8_HALF, "r4:%d r2:%d U4:%s %02ds",
              rx4, rx2, (huart4.RxState == HAL_UART_STATE_BUSY_RX) ? "RX" : "ERR",
              (int)((HAL_GetTick() - zm_t0) / 1000U % 100U));

  OLED_Update();
}

/* ================= 阶梯阶段参数（2026-09-21 再精简：只走固定距离）=================
   阶梯阶段只做三件事：**读每坑"夹不夹" + 动作组 + 每个坑走固定距离并计数**。
   ★原来的"左右对准(cam_x 挪车)"和"前后测距校准(走一步)"整块删掉了（连同下面这些参数）：
     实测那套校准效果不好 —— 一步只有 1~2cm、落点靠测距/像素反推，来回摆还拖时间；
     不如老老实实按固定距离走，位置由"进阶梯时的到位 + 坑间距"决定。
   ★朝向不用额外管：底盘一直锁向（target_yaw = 进阶梯时校好的那个朝向），
     走固定位移 20cm/s ≥ 介入阈值，走的过程中角度环一直在纠偏。
   ★视觉协议：画面里没目标 → 视觉一个字节都不发；有目标但这一坑不用夹 → 照样发帧、坐标有效，只是 cmd 说不夹。
     所以"等不到帧" = 视野里没目标 = 这一坑不夹；cmd **只用来决定夹不夹**，不再拿坐标挪车。 */
#define JIETI_IMG_CX          160     //视觉x是0~320像素，减它转成相对画面中心的偏移（只用于屏幕显示）
#define JIETI_VIS_MS         200U     //等主视觉一帧的超时(ms)：站在坑前读这一坑的 cmd 用（帧间隔约20~50ms，够）
/* ---------------- 走近阶梯的前测距目标(GY53_2) ----------------
   全阶段只剩"进阶梯时走近"用这一次：走到 90mm 附近就停（提前 20mm 停，补读数滞后）；
   之后每个坑不再做任何测距校准（前后校准 2026-09-21 已删）。 */
#define JIETI_FWD_TARGET_MM   90      //★目标前后距离(mm)：也是"走近阶梯"的停止距离

/* ---------------- 8个坑固定位移步进(计数就靠它) ----------------
   ★两个"走多远"都是现场拿尺量出来的：
     JIETI_STEP_CM       = 同一个阶梯里相邻两个坑的距离
     JIETI_STEP_CROSS_CM = 换阶梯那一步的距离(第2→3个坑、第6→7个坑，就是动作组57/60/63切换的地方)
   每走完一段固定位移 = 到了下一个坑；站定后等一帧读这个坑的 cmd(夹不夹)，处理完计数 +1 */
#define JIETI_STEP_CM         15      //★阶梯内坑间距(cm)：实际是8,但是要给15
#define JIETI_STEP_CROSS_CM  15.5      //★换阶梯那一步走多远(cm)：第2→3个、第6→7个坑(矮/中/高阶梯之间)实际是10,要给16
#define JIETI_STEP_SPEED      (float)SPD_SHORT_V   //走固定位移速度 = 短距标准(20cm/s)：要≥20 才压得过起转PWM
#define JIETI_STEP_ACC        30     //★加减速(cm/s^2)：8cm 按 20/30 走是三角波，峰值只有 √(30×8)=15.5cm/s(<20)，
                                      //  第一拍 kp×v 压不过起转PWM → 每步会先迟滞一下再窜出去(能走，就是慢半拍)；
                                      //  想"一拍就起转"把它提到 100~150（8cm 就变成正常梯形，2cm 内到 20cm/s）
/* ---------------- ★★ 阶梯跑完去哪儿：**就一个开关**，改这一个数即可切换 ----------------
   1 = 阶梯跑完先去【立柱】：绕柱一圈(LiZhu_Circle_Run) → 走到仓库中间倒方块 → 再回家（★正常流程用这个）
   0 = 阶梯跑完【直接回家】：跳过立柱段（临时简化流程/单独测后面几段时用）
   ★立柱段和回家段里的绝对角都过了 Yaw_Abs()：跳转测试(yaw_shift_deg≠0)时自动换算，
     正常流程(yaw_shift_deg=0)原样不变，所以这个开关两种情况下都能用。
   ★跳转测试的红蓝摆车姿态相差 180°（场地镜像）：SetYawShift(红方姿态角) 会自动给蓝方 +180°。
   ★注意"回家"段的头几步位移是按"立柱那边跑完"的位置写的；若置 0 从阶梯旁直接回家、
     落点不对，改回家段最前面那几条 ROBOT_Move 即可(见 HUIJIA_START 标签)。 */
#define JIETI_GO_LIZHU          1     //★1=阶梯跑完先去立柱   0=阶梯跑完直接回家
/* ---------------- ★"等到条件满足"类死等的超时保护(修"车停着不动=卡死") ----------------
   阶梯/仓库/回家几个定位点都是 `while(激光或测距还没到位);` 这种原地空转等待：
   设计上"走十几厘米就该满足"，但传感器没接好 / 读数卡住 / 车被顶住时永远出不来——
   现场"车停在那儿不动、屏幕和串口也不动了"多半就是卡在这几行。
   WAIT_WHILE(条件, 标识)：原地等到条件不再成立，最长等 WAIT_TIMEOUT_MS；超时就打一行 TIMEOUT 退出，
   后面紧跟的"停车/刹车"语句照常执行 → 不会再永久卡死，而且串口能看出是哪一处超的。
   ★6000ms 是按 10cm/s 走十几厘米给的余量；正常这些等待只要 1~4s，别把这个值改得太小。 */
#define WAIT_TIMEOUT_MS     6000U     //"等条件"类循环的最长等待(ms)
#define WAIT_WHILE(cond, tag)                                            \
  do{                                                                    \
    uint32_t _wait_t0 = HAL_GetTick();                                   \
    while(cond){                                                         \
      SERIALPLOT_WheelActualPump();      /* 串口1实时发四轮实际值：这种原地死等也要有数据（见 serialplot.c） */ \
      if(HAL_GetTick() - _wait_t0 > WAIT_TIMEOUT_MS){                    \
        UART1_Printf("TIMEOUT: %s\r\n", tag);                            \
        break;                                                           \
      }                                                                  \
    }                                                                    \
  }while(0)

/* ================== 阶梯阶段：保持锁向目标（否则越走越斜） ==================
   ★ROBOT_MoveSpeed() 内部会把 chassis.target_yaw 置成哨兵(YAW_TARGET_NONE)，
     让底盘控制循环下一拍"锁定当前朝向"。角度环本身一直是开的(flag.angle=1)，
     但阶梯阶段车一直在左右/前后动、视觉每来一帧就设一次速度，
     于是"已经被走歪的朝向"被反复当成新目标锁住 → 角度环只保持歪掉的朝向、
     不再往原目标纠偏 → 现象就是"越来越斜、像没开角度环"。
   ★所以阶梯阶段统一用下面的 JieTi_MoveSpeed() 设速：设完速度立刻把目标朝向恢复回
     进入阶梯时校好的那个值。
   ★2026-09-20 新底盘下正好衔接得很自然（见 chassis.h 的 YAW_MOVE_MIN_SPEED）：
     ① 阶梯阶段的速度都 <20cm/s → 落在"角度环不介入"区，平移期间 w 恒 0（不瞎纠偏、不会拧头）；
     ② 每次停车/设 0 速后，控制循环落回"旋转档"，这时才按 target_yaw 纠偏 —— 也就是
        每个动作之间都自动把朝向掰回 jieti_keep_yaw。
     所以这个函数现在只干一件事：把"要锁的朝向"固定成进阶梯时校好的那个，
     而不是让哨兵把"当前已经走歪的朝向"当新目标锁住。 */
static float jieti_keep_yaw = -1.0f;   //阶梯阶段目标朝向(°)，-1=还没锁(没锁就不动它)

/* ================== ★调试跳转的"角度基准换算"（2026-09-21，每个起点一套 + 红蓝差180°）==================
   为什么需要它（"单独测某一段时角度乱套"的根因）：
     · 正常发车：上电时车头朝前 → HWT101CT 的 0° 就是"前"，所以程序里的绝对角
       0=前 / 90=右 / 180=后 / 270=左 都是按"场地"说的。
     · 单独测某一段：车是**按那一段起点的姿态摆好再上电**的 → HWT 的 0° 变成"那一段的车头方向"，
       整套绝对角相对场地都转过了一个角度，再直接拿去 ROBOT_Angle 就会白转、越走越乱。
   做法：跳转入口调用 SetYawShift(红方姿态角)，它会按红蓝自动补 180°（场地镜像），
        之后**每一处绝对角都过一遍 Yaw_Abs()**：
          · ZM / HOME 起点：红方摆"倒完球姿态"(车头朝右 → 90)  → 蓝方自动 270(车头朝左)
          · LZ      起点：红方摆"立柱起点姿态"(车头朝左 → 270) → 蓝方自动  90(车头朝右)
        例：shift=270 时 Yaw_Abs(270)→0（=摆车朝向，不转）、Yaw_Abs(90)→180、Yaw_Abs(0)→90。
   正常流程(yaw_shift_deg=0)时 Yaw_Abs() 原样返回，行为一个字都不变。 */
static uint16_t yaw_shift_deg = 0;   //0=不换算；否则=本次摆车姿态相对"车头朝前"转过的角度(°)
static uint32_t Yaw_Abs(uint32_t normal_angle){   //把"正常基准角"换算成本次实际该转的角度
  if(yaw_shift_deg == 0) return normal_angle;
  return (normal_angle + (360U - yaw_shift_deg)) % 360U;
}
/* 跳转测试的角度基准统一入口：红方传"红方摆车姿态角"，蓝方自动 +180°（场地镜像） */
static void SetYawShift(uint16_t red_shift_deg){
  yaw_shift_deg = (uint16_t)((red_shift_deg + (mode_red ? 0U : 180U)) % 360U);
  UART1_Printf("DEBUG: yaw base shift = %u deg (mode %s)\r\n",
               (unsigned)yaw_shift_deg, mode_red ? "RED" : "BLUE");
}

//在原有基础上加了锁定目标朝向的代码：设完速度立刻把目标朝向恢复回进入阶梯时校好的那个值
static void JieTi_MoveSpeed(float x_speed, float y_speed){
  ROBOT_MoveSpeed(x_speed, y_speed);
  if(jieti_keep_yaw >= 0.0f) chassis.target_yaw = jieti_keep_yaw;  //恢复锁向目标(停车后角度环会按它纠偏)
}

/* ---------------- 阶梯阶段运行时状态(视觉/显示共用) ---------------- */
static int16_t  jieti_cam_x     = 0;       //最近一帧：目标距画面中心偏移(负=偏左 / 正=偏右)
static uint8_t  jieti_cmd       = 0;       //最近一帧：cmd(要不要夹)
static uint8_t  jieti_blk_now   = 0;       //当前是第几个坑(1~8，0=还没对准第一个)

/* 实时显示"当前是第几个坑(物块)"+最近一帧视觉x：只占第4行(y=48, 6x8)，
   不动第1/2行(JIETI letter x y / scan & grab...)和第3行(实时测距) */
static void JieTi_ShowBlockNo(uint8_t no, uint8_t total, int16_t cam_x){
  OLED_ClearArea(0, 48, 128, 8);                                  //只清第4行
  if(no >= 1 && no <= total){
    /* X+12/X-30 = 最近一帧视觉给的"目标离画面中心多远"，现场一眼看出对准情况 */
    OLED_Printf(0, 48, OLED_6X8_HALF, "BLK %u/%u X%+4d", (unsigned)no, (unsigned)total, (int)cam_x);
  }else{
    OLED_Printf(0, 48, OLED_6X8_HALF, "BLK -/%u", (unsigned)total);  //还没对准第一个坑
  }
  OLED_Update();
}

/* ================= 阶梯阶段：视觉(主视觉/串口2)发来的信息，转发到串口1 =================
   现场调试用：阶梯阶段每收到视觉发来的一帧，就把内容原样(十六进制) + 解析结果打到串口1
   (电脑，115200)，一眼看出"视觉发的到底是要夹还是不夹(cmd)、坐标是多少(x/y)"。
   一条日志一行(与正面识别阶段的 RX2/RX4 日志同一格式，好对着看)：
     RX2 #12 len=7: A3 01 20 00 40 00 0B | A3 cmd=0x01 x=32 y=64 cx=-128
     帧长不够一个A3包(7字节)、或帧里没有 0xA3...0x0B → 前半段照样打，后面跟 " | no A3"
   ★打印点只有一个：对准阶段等帧(JieTi_GetVision)时收到一帧就打一帧——那正是视觉给
     "要夹/不要夹 + 坐标"的时刻。机械臂动作、走固定位移这些整段阻塞期间不打印
     (那段时间收到的帧本来就被清掉，没必要看)。
   ★JIETI_VIS_LOG=1 开(默认)；=0 关：只对准、一条都不打(怕刷屏/嫌占串口就置0) */
#define JIETI_VIS_LOG         1        //1=开启阶梯阶段视觉信息转发；0=关闭
#define JIETI_VIS_LOG_BYTES   8        //每条日志最多原样打几个字节(A3包7字节，8够看)

static uint32_t jieti_vis_cnt = 0;     //阶梯阶段累计收到主视觉多少帧(看帧号就知道视觉在不在发)

/* 处理主视觉刚发来的一帧：先原样转发到串口1，再在缓冲里找 A3...0x0B 完整包，
   把 cmd(要不要夹) / x / y 解析出来存进 jieti_cam_x / jieti_cmd 给对准逻辑用
   返回 1 = 这一帧里有合法A3包；0 = 没有新帧 或 帧里没有A3包 */
static uint8_t JieTi_VisionPoll(void){
  if(!VISION1_RxFlag) return 0;                        //没有新帧：直接走(几乎不占时间)
  VISION1_RxFlag = 0;                                  //必须立即清零
  uint8_t len = VISION1_RxRealLength;                  //这一帧的实际字节数
  jieti_vis_cnt++;                                     //累计收帧数(转发开关关掉也照样在数)

  if(JIETI_VIS_LOG){
    UART1_Printf("RX2 #%u len=%u:", (unsigned)jieti_vis_cnt, (unsigned)len);
    for(uint8_t k = 0; k < len && k < JIETI_VIS_LOG_BYTES; k++)
      UART1_Printf(" %02X", VISION1_RxBuf[k]);         //原样：帧里到底是什么，一眼看出来
  }

  for(uint8_t i = 0; i + 7 <= len; i++){               //在缓冲里找 A3...0x0B 完整包
    if(VISION1_RxBuf[i] == 0xA3 && VISION1_RxBuf[i + 6] == 0x0B){
      uint16_t px = (uint16_t)VISION1_RxBuf[i + 2]
                  | ((uint16_t)VISION1_RxBuf[i + 3] << 8);          //x像素(低字节在前,0~320)
      uint16_t py = (uint16_t)VISION1_RxBuf[i + 4]
                  | ((uint16_t)VISION1_RxBuf[i + 5] << 8);          //y像素(本阶段不用)
      /* ★不再做 x 范围过滤（2026-09-21 删）：协议里没有"未检测到"的填充帧(没目标根本不发帧)，
         视觉给的坐标都是有效的，加个范围闸门反而可能把真坐标挡掉。 */
      jieti_cam_x = (int16_t)px - JIETI_IMG_CX;         //减160 → 目标距画面中心偏移
      jieti_cmd   = VISION1_RxBuf[i + 1];               //cmd(要不要夹，含义看 JieTi_Grab_Mode)
      if(JIETI_VIS_LOG)
        UART1_Printf(" | A3 cmd=0x%02X x=%u y=%u cx=%d\r\n",
                     (unsigned)VISION1_RxBuf[i + 1], (unsigned)px, (unsigned)py,
                     (int)jieti_cam_x);
      return 1;
    }
  }
  if(JIETI_VIS_LOG) UART1_Printf(" | no A3\r\n");      //不是A3包/帧不完整：也打出来，免得"没反应"查不出来
  return 0;
}

/* 清掉主视觉残留帧：只认之后的实时坐标(走完固定位移后 / 夹取动作后调用) */
static void JieTi_FlushVision(void){
  VISION1_RxFlag = 0;
  VISION1_RxRealLength = 0;
  memset(VISION1_RxBuf, 0, VISION1_RxLength);
}

/* 取一帧主视觉A3包(等不到就返回0)；结果存进 jieti_cam_x / jieti_cmd
   ★收到的每一帧都会先被 JieTi_VisionPoll() 转发到串口1(见上方)，这里只管等 + 取 */
static uint8_t JieTi_GetVision(uint32_t wait_ms){
  uint32_t t0 = HAL_GetTick();
  while((HAL_GetTick() - t0) < wait_ms){
    if(JieTi_VisionPoll()) return 1;     //这一帧里有A3包：已转发串口1 + jieti_cam_x/jieti_cmd 已更新
  }
  return 0;
}

/* ★2026-09-21 变更记录（阶梯阶段）：
   · 删掉：左右视觉对准（JieTi_GoAlign，连同 JIETI_ALIGN_BAND / JIETI_ALIGN_MS / JIETI_NOVIS_MS）、
     "走一步"的脉冲式校准（JieTi_Step / JieTi_WaitCoord / JIETI_STEP_MS / JIETI_STEP_SETTLE_MS）、
     第3行测距显示（JieTi_ShowFwdMm）—— 实测那几套校准效果不好。
   · 改成：位置靠"进阶梯到位 + 每个坑走固定距离"，朝向靠底盘锁向；
     **前后距离校准也已删**（2026-09-21）—— 阶梯只走固定距离，位置全靠"到位 + 坑间距"。 */

/* ★2026-09-21 删除：JieTi_FwdFix（阶梯的前后距离校准）—— 实测这套校准效果不好。
   阶梯现在只走固定距离：前后位置由"进阶梯时走近到 90mm 附近（带 20mm 提前量）+ 坑间距"决定，
   不再逐坑用测距精修；要恢复就把上面这段函数和坑循环里那一次调用加回来。 */
/* ==================== 立柱转圈：绕柱（2026-09-26：开环三旋钮 + 两路可选反馈）====================
   立柱阶段(LiZhu_Flag==1) 和串口调试指令 7 都调它，同一套代码。

   【原理】车头一直指着柱子、车身横着走 ⇒ 轨迹天然是以柱子为圆心的圆
       圆半径 r = V_TAN / (W_TURN × π/180)     （V_TAN cm/s = 横向走多快；W_TURN °/s = 车头摆多快）
       本车 r 由几何定死：测距 17 + 传感器到车心 14 + 柱半径 4 = 35cm

   【W_TURN 怎么算 —— 改 V_TAN 就照这条重算，没有别的东西要跟着改】
       W_TURN = V_TAN / r × 57.3   ⇒  r=35cm 时：W_TURN = 1.64 × V_TAN
         V_TAN = 5→8.2 ｜ 10→16.4 ｜ 14→22.9 ｜ 20→32.7      （整圈时间 = 360/W_TURN 秒）
       反过来改半径：r = 57.3 × V_TAN / W_TURN （V_TAN=10 时：W=12→48cm、16.4→35cm、20→29cm）
     ★16.4 是"几何值"（假设四轮精确跑出命令速度）。实测纯开环平均半径会小 13%（见文末【实测】），
       但**闭环就该填几何值**：测距反馈把半径压到目标后，平衡点要求 w = V_TAN/r，正是这个数。

   【要调的只有这几行（都在函数开头，改完重新烧）】
       ① 三个旋钮：V_TAN=10 / W_TURN=16.4 / V_RAD=3
       ② 两个开关：FB_DIST=0 / FB_LASER=0（都置 0 = 纯开环）
       ③ 激光参数：LAS_YAW=4（纠偏摆速）/ LAS_TAN=0（切向纠偏，默认不用）
       其余 DEFAULT 段是定死的几何常数，不用动。

   【两路反馈（可单独开关，互不影响）】
       FB_DIST ：车头正对柱子 ⇒ 测距值就是半径。偏大往前靠、偏小往后退（调径向速度 v_y：
                 满幅 3cm 误差 → V_RAD cm/s；v_y=0 时半径只由 V_TAN/W_TURN 决定）
       FB_LASER：左4右2 两个激光平行打柱（间距≈柱半径），一个有一个没有 = 横向偏了 → 调车头摆速 w
     ★为什么全写成"速度"而不是"误差×增益"：增益一顶限幅就变成开关环（9.25 实测 1.96s 周期呼吸、
       27.5% 时间顶限幅）。现在斜率写死 3cm，手里只有几个速度，怎么调都不会退化成开关环。

   【怎么调（一次只动一个值）】
     ① e 绕一圈一直在同一符号上变大 → 改 W_TURN（e 为正 = 越来越远 = 圆太大 → 调大；e 为负 → 调小）。
        e 绕一圈正好摆一次（最低点在 yaw≈180）→ 开环固有摆动，改 W_TURN 没用，交给 FB_DIST
     ② 前轮顶着走/一顿一顿：靠柱那对前轮 = V_TAN×(1−28.15/r) = 0.18×V_TAN（10 → 1.9cm/s），
        同一时刻后轮 1.8×V_TAN = 18cm/s —— 绕圈几何决定的、不是故障（V_TAN<8 时前轮才真推不动；
        想让它有劲：V_TAN 提到 15，或半径放到 48cm → 前轮 0.41×V_TAN）
     ③ 开 FB_DIST 后：V_RAD = 半径环的阻尼，ζ = V_RAD×r/(6×V_TAN) ≈ V_RAD/1.7（V_TAN=10 时）
          · ζ≈1（V_RAD≈1.7）临界阻尼、最快不振荡
          · V_RAD=3（ζ≈1.75）过阻尼、更稳对噪声不敏感  ← 起步先用这个
          · 半径慢慢上下"呼吸"→ 加到 4~5；被噪声推得一抽一抽 → 降到 1.5~2
        ★别超 V_TAN 的一半；V_RAD 只决定"追上目标的速度"，不改目标半径
     ④ 开 FB_LASER 后：车头来回摆 → LAS_YAW 调小；偏了不回来 → 调大（≈1/4 基准摆速起步，方向反取负）
        ★LAS_TAN 想试"用切向速度纠偏"再改成 1~3（同样别超 V_TAN 的一半）
     ⑤ 两路都开：这台车没有独立转向，激光摆头会顺带改半径（r=V_TAN/w）→ 打架就先关一路

   【实测（2026-09-26 一趟：V_TAN=10 / W_TURN=16.4 / 纯开环）—— 开环只能这样，摆动改不掉】
      e 从 −4 滑到 −79（d 从 160 掉到 80mm、还卡住约 100°），末尾回到 +14 ⇒
      半径"绕一圈正好摆一个完整正弦"：最低≈26cm、最高≈36cm、整圈平均比目标小约 4.6cm。
      · 运动学：dψ/dt = −(v_x/r²)Δr、dΔr/dt = v_x·ψ（ψ = 车头偏离"指向柱子"的角度）
        ⇒ 无阻尼简谐振动，周期 = 整圈时间（实测最低点恰在 yaw≈180°，与理论吻合）
        ⇒ 摆动是开环的结构性质，旋钮消不掉，只能靠反馈压。
      · 摆幅只由起步那一刻决定：A ≈ √(Δr₀² + (r·ψ₀)²)，实测 A≈4.6cm ⇒ 起步车头就差约 8°。
      · 摆动中心 = 真实 v_x/w 比：实测 29.4cm（比几何小 13% ⇒ 靠柱前轮跑不满：v_x 偏小、w 偏大）
        ⇒ 纯开环想跑准，只能按这个比例微调 W_TURN；开 FB_DIST 后它自己收敛（稳态误差=0）。
      ★"d 卡在 80mm 不动" = 读数出窗口被丢了、打印的是上次值（真实半径更小），不是半径稳住了。

   串口每 200ms 一行：d=测距mm e=半径误差mm(正=远) l4/r2=左右激光(1=看到柱) vx/vy=切向/径向(0.1cm/s)
                      w=角速度×100 yaw=已绕角度° ｜ yaw 不涨=卡住；355° 该停
   ========================================================================== */

static void LiZhu_Circle_Run(void)
{
  if(!flag.chassis){                          // 前置条件：底盘控制循环在跑，否则车不会动、while 会一直空转
    UART1_Printf("no chassis!\r\n");
    return;
  }
  UART1_Printf("circle start\r\n");

  /* ===== ① 三个可调参数（开环基本圆 + 反馈强度都在这三行）===== */
  const float V_TAN  = 10.0f;               // 切向速度 cm/s（>0 逆时针 / <0 顺时针）—— 只管快慢（★本轮定 10）
  const float W_TURN = 16.4f;               // 车头摆速 °/s（只填正的；大小由公式算出来，见下）
  const float V_RAD  = 3.0f;                // 径向速度 cm/s（只有 FB_DIST=1 才起作用：路线像椭圆 → 加；车身一冲一停（抖）→ 减。）
  /* ★W_TURN 怎么算：W_TURN = V_TAN / r × 57.3（r 由几何定死，见下一行）
       ⇒ V_TAN=10 时：10/35 × 57.3 = 16.4°/s（整圈 360/16.4 ≈ 22s）
       ⇒ 通用表：W_TURN = 1.64 × V_TAN（5→8.2、10→16.4、14→22.9、20→32.7）；反算 r = 57.3×V_TAN/W_TURN
     ★闭环必须填几何值：半径被测距反馈压到目标后，平衡点要求 w = V_TAN/r，正是它。
       纯开环实测平均半径小 13%（前轮跑不满），按实测比例凑 W_TURN 只是临时手段，别当基准。 */
  /* r 由几何定死：车心到柱轴 = 前测距(立柱校准停在 170mm) + 传感器到车心 14 + 柱半径 4 = 35cm */

  /* ===== ② 两个反馈开关（0=关 1=开；都置0 = 只有三个旋钮的纯开环圆）===== */
  const uint8_t FB_DIST  = 1;               // 测距反馈：测距偏大→往前靠、偏小→往后退（调 v_y）
  const uint8_t FB_LASER = 1;               // 双激光反馈：一个有一个没有→横向偏了（调 w，可选同时调 v_x）

  /* ===== ③ 激光反馈参数（只有 FB_LASER=1 才用到；LAS_TAN=0 表示切向那一路不用）===== */
  const float LAS_YAW = 4.0f;               // 偏了时额外加的车头摆速 °/s（纠偏主力；方向反了取负）
  const float LAS_TAN = -2.0f;               // 偏了时额外加的切向速度 cm/s（想试"调切向速度"就改成 1~3，别超 V_TAN 一半）

  /* ===== DEFAULT：定死常数（不用调，理由都写在这）===== */
  const float RAD_FULL_CM = 3.0f;           // 测距反馈满幅误差：≥3cm 都按满幅算（斜率=V_RAD/3cm，写死）
  const float RAD_DEAD_CM = 0.3f;           // 测距反馈死区 ±3mm（读数残余抖动别变成轮子一直抖）
  const float D_ALPHA     = 0.5f;           // 测距一阶低通系数（压掉 GY-53 的 ±5~10mm 抖动）
  const float GY53_2_OFFSET_CM = 14.0f;     // 前测距(GY53_2)到车心的纵向距离 cm（实测；算半径的几何常数）
  const float PIPE_RADIUS_CM   = 4.0f;      // 柱子(水管)半径 4cm（外径 8cm；算半径的几何常数）

  /* ★靠柱那对前轮天生很慢（绕圈几何决定：r=35cm 时 0.18×V_TAN=1.9cm/s，同时后轮 1.8×V_TAN=18cm/s）——
     不是故障；V_TAN<8 时它才真的推不动，想让它有劲就加大 V_TAN 或把半径调大。 */
  float v_tan  = V_TAN;                                         // 切向速度 cm/s（= 旋钮值，不做任何隐藏修改）
  float dir    = (v_tan < 0.0f) ? -1.0f : 1.0f;                 // 绕向：+1 逆时针 / -1 顺时针（由 V_TAN 符号定）
  float w_base = dir * W_TURN * 0.0174532925f;                  // 开环基准角速度 rad/s（w>0 逆时针）
  float r_knob = (W_TURN > 0.05f) ? fabsf(V_TAN) / (W_TURN * 0.0174532925f) : 999.0f;  // 旋钮隐含半径cm（串口对照用）

  /* ===== 测距定参考：开圈前静止采 12 次，只收有效值(8~22cm)、取中值，就用它当目标半径 =====
     不写死 180mm：起步时半径误差≈0，一开圈不会先往柱里冲（9.25 写死 180 而实测 194 → 起振源）。
     ★FB_DIST=0 时这组数只用于串口显示（e 就是给你调 W_TURN 看的），不参与控制。 */
  uint16_t d_ok[10];                            // 有效采样缓存
  uint8_t  n = 0;                               // 有效采样个数
  for(uint8_t i = 0; i < 12; i++){              // 最多采12次
    uint16_t dd = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
    if(dd >= 80 && dd <= 220){                  // 只收8~22cm：丢目标返回2000/杂散直接丢弃
      d_ok[n++] = dd;
      if(n >= 10) break;
    }
    HAL_Delay(30);                              // 采样间隔，避开电机/震动噪声
  }
  UART1_Printf("valid=%d\r\n", n);
  uint8_t have_ref = (n >= 3) ? 1 : 0;          // 有没有真实的半径参考（FB_DIST=0 时只影响打印）
  if(n < 3){                                    // 有效采样太少
    if(FB_DIST){                                // 测距反馈开着 → 没有半径参考，绝不乱转
      UART1_Printf("no pipe! (dist fb needs it)\r\n");
      return;
    }
    UART1_Printf("no pipe; open-loop anyway\r\n");   // 纯开环不需要测距，照跑（e 只作显示）
    d_ok[0] = 180; n = 1;                       // 显示用的名义值 180mm（此时 e 的零点随便，看趋势就行）
  }
  /* 冒泡排序取中值：比平均更抗单次大值/小值 */
  for(uint8_t i = 0; i < n-1; i++)
    for(uint8_t j = i+1; j < n; j++)
      if(d_ok[j] < d_ok[i]){ uint16_t t = d_ok[i]; d_ok[i] = d_ok[j]; d_ok[j] = t; }
  uint16_t d_ref    = d_ok[n/2];                // 目标测距 mm（= 开圈那一刻的半径）
  float    d_ref_cm = (float)d_ref / 10.0f;     // 目标测距 cm
  float    d_cm     = d_ref_cm;                 // 当前测距 cm（低通后的；无效读数保持上次值）

  /* ===== 开圈前把配置打到串口，一眼确认"这次到底开了什么" ===== */
  UART1_Printf("cfg: V_TAN=%d W_TURN=%d -> r~%dcm | V_RAD=%d | fb dist=%d laser=%d\r\n",
               (int)V_TAN, (int)W_TURN, (int)r_knob, (int)V_RAD, FB_DIST, FB_LASER);
  /* ★半径几何对照：车心到柱轴 = 测距 + 14 + 4 = 35cm —— 这一行直接告诉你本圈的 W_TURN 该填多少 */
  if(have_ref){
    float r_tgt = d_ref_cm + GY53_2_OFFSET_CM + PIPE_RADIUS_CM;
    UART1_Printf("ref=%dmm -> r_tgt=%dcm -> W_TURN_ideal=%d (x0.1deg/s; now=%d)\r\n",
                 d_ref, (int)r_tgt, (int)(v_tan / r_tgt * 572.9578f), (int)(W_TURN * 10.0f));
  }else{
    UART1_Printf("ref=? (no valid reading) -> e 与 W_TURN 的绝对值没意义，只看趋势\r\n");
  }
  if(FB_LASER) UART1_Printf("laser: LAS_YAW=%d deg/s LAS_TAN=%d cm/s\r\n",
                            (int)LAS_YAW, (int)LAS_TAN);

  /* ===== 接管底盘：角度环让位（w 由本闭环接管）+ 手动设速标志（防控制循环把速度归零）===== */
  flag.angle = 0;
  chassis.v_x = 0.0f;  chassis.v_y = 0.0f;  chassis.w = 0.0f;
  chassis.x_speed_plan_flag = 0;
  chassis.y_speed_plan_flag = 0;
  chassis.x_set_speed_flag  = 1;
  chassis.y_set_speed_flag  = 1;

  float yaw_last = HWT101CT_Data.yaw;           // 起点朝向（此时车头正对柱子）
  float yaw_acc  = 0.0f;                        // 陀螺仪累积转角(°)（车头一直跟着柱子转，所以它就等于已绕角度）
  uint8_t  lost      = 0;                       // 连续无效测距计数
  uint8_t  lost_stop = 0;                       // 丢目标保护停车标志
  uint32_t t_prt     = HAL_GetTick();           // 打印节拍

  while(fabsf(yaw_acc) < 355.0f){           // 绕满一整圈
    /* ===== 读一次测距（每拍都读：显示 + 测距反馈都用它）=====
       有效窗口 80~220mm；丢目标/杂散 → 保持上次值（误差不跳）；
       ★只有"测距反馈开着"时才做连续20次的保护停车（纯开环不需要测距，丢目标照转）
       ★杂散交给"固定斜率 + V_RAD 封顶"消化（一次杂散最多把车推 0.6cm），不用冻结读数来治：
         9.25 把窗口收到 150~215 后，d 一越界就整段冻结、相位滞后变大，反而更难收。 */
    uint16_t dd = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
    if(dd >= 80 && dd <= 220){
      d_cm = d_cm + D_ALPHA * ((float)dd / 10.0f - d_cm);   // 一阶低通（几十 ms 的滞后，可忽略）
      lost = 0;
    }else if(FB_DIST && ++lost >= 20){
      lost_stop = 1; break;
    }
    /* ★读日志提醒：读数出窗口(贴太近 dd<80 或没打到柱子)时 d_cm 会一直保持上次值不更新，
       所以串口看到 "d=80 卡住不动、l4/r2 在闪" 不是"半径稳住了"，而是读数被丢了
       （真实半径只会比 80mm 更小）。纯开环(FB_DIST=0)时这不算问题，照转。 */
    float e = d_cm - d_ref_cm;                              // 半径误差 cm（正 = 离柱子比目标远）

    /* ===== 读两个激光（每拍都读，只为串口显示；★纠不纠由 FB_LASER 决定）=====
       ★纯开环调试时先看这行的 l4/r2：正对柱子应该一直是 1 1，能直接看出偏的方向和程度 */
    uint8_t l4 = LASER_Barrier(LASER4_GPIO_Port, LASER4_Pin);   // 左激光（有障碍=1）
    uint8_t r2 = LASER_Barrier(LASER2_GPIO_Port, LASER2_Pin);   // 右激光

    /* ===== ① 基准：开环圆（三个旋钮）===== */
    float vx = v_tan;                                       // 切向：横向沿圈走（车头正对柱子，圆心就在车头正前方）
    float vy = 0.0f;                                        // 径向：默认不动
    float w  = w_base;                                      // 车头摆速：开环基准（r = V_TAN/W_TURN 就靠它）

    /* ===== ② 测距反馈（FB_DIST=1）：测距值偏大偏小 → 调径向速度 =====
       远（正误差）→ 往车头方向前进（车头正对柱子，所以就是朝柱子靠）→ 半径收回来
       近（负误差）→ 后退（背离柱子）→ 半径退出去；±3mm 死区内不动 */
    if(FB_DIST){
      float u = e / RAD_FULL_CM;                            // 归一化：±1 封顶 = 满幅
      if(u >  1.0f)      u =  1.0f;
      else if(u < -1.0f) u = -1.0f;
      if(fabsf(e) < RAD_DEAD_CM) u = 0.0f;
      vy = V_RAD * u;                                       // 满幅 V_RAD cm/s（斜率 = V_RAD/3cm，写死）
    }

    /* ===== ③ 双激光反馈（FB_LASER=1）：一个有一个没有 = 横向偏了 =====
       左4右2 两个激光平行打柱子（间距≈柱半径 4cm，柱宽 8cm）：
         · 1 1 → 柱轴在车头轴线 ±2cm 内 = 正对，不纠
         · 左4有 右2无 → 柱子偏在车头轴线**左边** → 车头往左摆（w 加正 = 逆时针）
         · 右2有 左4无 → 柱子偏在**右边** → 车头往右摆（w 减）
         · 0 0 → 偏了 6cm 以上 / 丢失 → 这一拍不纠（看串口 l4/r2 就知道）
       ★纠偏方向只跟"柱子在左还是右"有关，和绕向(V_TAN正负)无关。
       ★切向那一路（LAS_TAN≠0 才用）：柱子偏右 → vx 加正、偏左 → vx 加负
         （由 dp/dt = d·w − vx 得出：w 那一路和 vx 那一路对横向偏移是同向效果）。 */
    if(FB_LASER){
      int8_t s = 0;                                         // +1 = 柱子偏左（要往左摆/往负方向偏）
      if(l4 && !r2)      s = +1;
      else if(r2 && !l4) s = -1;
      w  += (float)s * LAS_YAW * 0.0174532925f;             // 车头纠偏（默认走这一路）
      vx += (float)s * LAS_TAN;                             // 切向纠偏（默认 LAS_TAN=0 = 不用）
    }

    /* ===== 下发：三个速度解耦，互不干涉 ===== */
    if(w >  YAW_PID_OUT_MAX) w =  YAW_PID_OUT_MAX;          // 摆速别超角度环的限幅（打印的就是真下发的）
    else if(w < -YAW_PID_OUT_MAX) w = -YAW_PID_OUT_MAX;
    chassis.v_x = vx;
    chassis.v_y = vy;
    chassis.w   = w;

    /* 实时打印(每200ms)：d=测距mm(低通后) e=半径误差mm(正=远) l4/r2=左/右激光(1=看到柱子)
       vx=切向速度(0.1cm/s) vy=径向速度(0.1cm/s) w=角速度×100 yaw=已绕角度(°)
       355° 就该停；yaw 不涨=卡住了；d 一直涨/跌=半径没调对（先调 W_TURN） */
    if(HAL_GetTick() - t_prt >= 200){
      UART1_Printf("d=%d e=%d l4=%d r2=%d vx=%d vy=%d w=%d yaw=%d\r\n",
                   (int)(d_cm * 10.0f), (int)(e * 10.0f), l4, r2,
                   (int)(vx * 10.0f), (int)(vy * 10.0f),
                   (int)(w * 100.0f), (int)fabsf(yaw_acc));
      t_prt = HAL_GetTick();
    }

    /* 识别钩子：要边绕边等视觉就在这里查一次，命中就 break（停车/恢复角度环照常在下面执行） */

    /* if(视觉命中){ break; } */

    /* 陀螺仪累积转角判断已绕角度 */
    float ddg = HWT101CT_Data.yaw - yaw_last;
    yaw_last = HWT101CT_Data.yaw;
    if(ddg > 180.0f)       ddg -= 360.0f;
    else if(ddg < -180.0f) ddg += 360.0f;
    yaw_acc += ddg;

    HAL_Delay(10);                          // 本循环节拍（测距/激光都是阻塞读，这里 10ms 只是节拍）
  }
  /* 停车 + 恢复角度环（重新锁向当前朝向） */
  chassis.v_x = 0.0f;  chassis.v_y = 0.0f;  chassis.w = 0.0f;
  chassis.x_set_speed_flag = 0;
  chassis.y_set_speed_flag = 0;
  flag.angle = 1;
  chassis.target_yaw = YAW_TARGET_NONE;
  if(lost_stop) UART1_Printf("LOST! stop\r\n");
  else          UART1_Printf("circle done (yaw=%d)\r\n", (int)fabsf(yaw_acc));
}



/* ==================== 串口1调试指令：解析 + 就地执行（菜单界面也能用） ====================
   ★为什么要有这个函数（"发 7,0,0,0,0,0,0,0 没反应"就是它的原因）：
     ① 车平时停在下面"红蓝方选择"那个 while(1) 菜单里，主循环体根本没往下走 ——
        解析串口的代码在主循环里，菜单里没人解析，所以发什么指令都毫无反应(连回显都没有)。
     ② 原来"单独测试转圈"那几个 if 之前还有一句无条件的 UART1_Data[0]=0;，
        它把刚解析出来的指令(6/7)提前清成 0 → 那几个 if 永远进不去。
     现在把"解析 + 回显 + 圆周调试指令"提成本函数：主循环里调一次，菜单循环里也调一次，
     不管车停在哪个界面，发指令都立刻生效。
   帧格式："S,A,B,C,D,E,F,G"（逗号分隔、8 个整数，串口1发出去记得带换行）
     S = UART1_Data[0] 是命令号：
       3 = 车体走固定距离 + 转到指定角度（在本函数里不管，交给主循环下面的原逻辑）
       7 = 立柱绕圈 LiZhu_Circle_Run()（2026-09-26 重写：切向/径向/车头摆速三个“速度”解耦，实现在本函数上方）
   返回 1 = 这一帧已被本函数处理掉（7 绕圈已在里面阻塞跑完）；0 = 只是解析/回显，交给主循环原逻辑
   要改绕圈参数（V_TAN 切向速度 / W_TURN 车头摆速 / V_RAD 径向速度）和两个反馈开关（FB_DIST / FB_LASER）就去改上面 LiZhu_Circle_Run() 函数开头那几行 const，别改散落的其它地方
   ------------------------------------------------------------------------------------------
   ★2026-09-15 新增【单键手动测试指令】（专门用来单独测 前/后/左/右 / 原地转）：
     帧里没有逗号就按单键解释（外面那套 "S,A,B,..." 数值指令完全不受影响）：
       w/s/a/d = 前进/后退/左移/右移（持续走，发 x 停）   W/S/A/D = 同上但只走 1 秒自动停
       x 或空格或回车 = 停车     +/- = 测试速度 ±10cm/s（默认 30，10~100）
       1/2/3/4 = 原地转到 0°/90°/180°/270°      p = 打印状态   o = 里程清零   h = 帮助
       v = 开关"串口1实时发四轮实际值"（默认开：每 WHEEL_ACT_SEND_MS=20ms 一行 4 个数字，
           通道1~4 = 左前/左后/右后/右前 实际速度 cm/s，可直接接 SerialPlot；见 serialplot.c）
     ★用法：上电后在"红蓝方选择"菜单界面直接发（这个函数菜单循环里也在调），不用进比赛流程。
     ★这套指令只做"设恒速/转向"，不做任何位置闭环；测"走歪"就看 ST 行里的 yaw 和 now 实测速度。
   ------------------------------------------------------------------------------------------ */
static uint8_t UART1_DebugCmd(void){
  /* ★单键手动测试用的状态（对应下面 ① 单字节快捷指令；静态变量在两次调用之间保持） */
  static uint32_t cmd_stop_at = 0;      /* 定时自动停车的时刻（大写 W/S/A/D = 走 1 秒自动停） */
  static float    cmd_spd     = 30.0f;  /* 单键测试速度 cm/s（+/- 每档 10，范围 10~100） */
  if(cmd_stop_at != 0 && (int32_t)(HAL_GetTick() - cmd_stop_at) >= 0){
    cmd_stop_at = 0;
    ROBOT_MoveSpeed(0.0f, 0.0f);
    UART1_Printf("AUTO STOP (1s)\r\n");
  }
  if(!UART1_RxFlag) return 0;                       // DMA空闲中断没收到帧：什么都不做
  UART1_RxFlag = 0;                                 // 必须立即清零
  char line[UART1_RxLength + 1];                    // 拷贝一份并补'\0'（DMA缓冲末尾没有结束符）
  uint16_t len = UART1_RxRealLength;
  if(len > UART1_RxLength) len = UART1_RxLength;    // 防越界
  memcpy(line, UART1_RxBuf, len);
  line[len] = '\0';

  /* ==================== ① 单字节快捷指令：手动测"前后左右单独动 / 原地转 / 调速" ====================
     触发条件：这一帧里**没有逗号**（即不是 "S,A,B,..." 数值指令）→ 按"单键"解释。
       w / s / a / d = 前进 / 后退 / 左移 / 右移（持续走，直到发 x 停）
       W / S / A / D = 同上，但只走 1 秒自动停（配合 o 清零、p 看状态，能量出"走了多少、歪了多少"）
       x 或 空格 或 回车 = 停车
       + / -         = 测试速度 ±10cm/s（默认 30，范围 10~100）
       1 2 3 4       = 原地转到 0° / 90° / 180° / 270°
       p = 打印状态   o = 里程清零   h / ? = 帮助
     ★注意：不带逗号的单键数字 1~4 现在是"原地转"；原流程的 3 指令请照旧带逗号发（3,distx,disty,...）。
     ★所有移动都用 ROBOT_MoveSpeed()：它会开启航向环并**锁定发指令那一刻的朝向**（走直线），
       所以"歪不歪"看状态行里的 yaw 有没有被拉住。
       ★2026-09-20 新底盘的分档（chassis.h）：速度 <20cm/s 角度环**不介入**（w 恒 0，平移期间不纠偏），
         20~80 走低速平移档、>80 走高速平移档；原地不动（发 x 停车后）自动落"旋转档"按 target_yaw 纠偏。
         所以单键测试想验"纠偏灵不灵"，就用 ≥20cm/s 走（命令 10 在新底盘下就是真的 10cm/s，不再被整形顶高）。 */
  if(strchr(line, ',') == NULL){
    char    c     = line[0];
    float   vx    = 0.0f, vy = 0.0f;
    uint8_t timed = 0;
    switch(c){
      case 'w': vy =  cmd_spd; break;
      case 'W': vy =  cmd_spd; timed = 1; break;
      case 's': vy = -cmd_spd; break;
      case 'S': vy = -cmd_spd; timed = 1; break;
      case 'a': vx = -cmd_spd; break;
      case 'A': vx = -cmd_spd; timed = 1; break;
      case 'd': vx =  cmd_spd; break;
      case 'D': vx =  cmd_spd; timed = 1; break;
      case 'x': case 'X': case ' ': case '\r': case '\n':
        cmd_stop_at = 0;
        ROBOT_MoveSpeed(0.0f, 0.0f);
        UART1_Printf("STOP (v=0)\r\n");
        return 1;
      case '+': cmd_spd += 10.0f; if(cmd_spd > 100.0f) cmd_spd = 100.0f;
                UART1_Printf("test speed = %.0f cm/s\r\n", (double)cmd_spd); return 1;
      case '-': cmd_spd -= 10.0f; if(cmd_spd <  10.0f) cmd_spd =  10.0f;
                UART1_Printf("test speed = %.0f cm/s\r\n", (double)cmd_spd); return 1;
      case '1': ROBOT_Angle(0);   UART1_Printf("angle -> 0\r\n");   return 1;
      case '2': ROBOT_Angle(90);  UART1_Printf("angle -> 90\r\n");  return 1;
      case '3': ROBOT_Angle(180); UART1_Printf("angle -> 180\r\n"); return 1;
      case '4': ROBOT_Angle(270); UART1_Printf("angle -> 270\r\n"); return 1;
      case 'o': chassis.pos_x = 0.0f; chassis.pos_y = 0.0f;
                chassis.dist_acc_x = 0.0f; chassis.dist_acc_y = 0.0f;
                UART1_Printf("odometry cleared (pos=0,0)\r\n"); return 1;
      case 'p': break;                          /* 只打印状态，见下面统一打印 */
      case 'v':                                 /* 串口1实时发四轮实际值 开/关（实现在 serialplot.c）
                                                   ★单键分支只看第一个字符（这一帧没有逗号就走这里），
                                                     所以 'v' 也照旧：想发别的以 v 开头的内容请用上面的数值指令 */
        serialplot_wheel_on = !serialplot_wheel_on;
        UART1_Printf("wheel actual log %s (%u ms/line)\r\n",
                     serialplot_wheel_on ? "ON" : "OFF", (unsigned)WHEEL_ACT_SEND_MS);
        return 1;
      case 'h': case '?':
        UART1_Printf("single-key: w/s/a/d=前进/后退/左移/右移  W/S/A/D=走1秒  x=停  +/-=调速\r\n");
        UART1_Printf("            1/2/3/4=原地转0/90/180/270  o=里程清零  p=状态  v=四轮实际值日志\r\n");
        return 1;
      default:
        UART1_Printf("? unknown key '%c' (send h for help)\r\n", c);
        return 1;
    }
    if(vx != 0.0f || vy != 0.0f){
      ROBOT_MoveSpeed(vx, vy);                  /* 内部锁向当前朝向 → 走直线 */
      cmd_stop_at = timed ? (HAL_GetTick() + 1000u) : 0u;
      UART1_Printf("KEY '%c' -> v=(%+.0f,%+.0f) cm/s%s\r\n", c, (double)vx, (double)vy,
                   timed ? "  [1s auto-stop]" : "");
    }
    /* 状态行：命令速度 / 实测速度 / 陀螺仪朝向(实测/目标) / 角速度输出 / 里程位置 */
    UART1_Printf("ST tgt=(%+.0f,%+.0f) now=(%+.1f,%+.1f) yaw=%.1f/%.1f w=%+.2f pos=(%+.1f,%+.1f)\r\n",
                 (double)chassis.v_x, (double)chassis.v_y,
                 (double)chassis.now_v_x, (double)chassis.now_v_y,
                 (double)HWT101CT_Data.yaw, (double)chassis.target_yaw,
                 (double)chassis.w,
                 (double)chassis.pos_x, (double)chassis.pos_y);
    return 1;
  }

  uint8_t i = 0;
  char *p = strtok(line, ",");                      // 按逗号切段
  while(p && i < UART1_DATA_NUM){                   // 逐段转成32位整数，支持负数
    UART1_Data[i++] = (int32_t)strtol(p, NULL, 10);
    p = strtok(NULL, ",");
  }
  /* 回显：串口上能看到这条，就说明指令收到了、也解析出来了（排查"没反应"先看有没有它） */
  UART1_Printf("S=%d A=%d B=%d C=%d D=%d E=%d F=%d G=%d\r\n",
               UART1_Data[0], UART1_Data[1], UART1_Data[2], UART1_Data[3],
               UART1_Data[4], UART1_Data[5], UART1_Data[6], UART1_Data[7]);

  if(UART1_Data[0]==7){                             // 立柱绕圈（2026-09-26 重写：开环三旋钮 + 测距/激光两路可选反馈，见上方 LiZhu_Circle_Run）
    UART1_Data[0] = 0;                              // 立即清指令，防止循环重复触发
    UART1_Printf("lizhu circle start\r\n");
    /* 直接调立柱阶段用的那个函数（两边同一套代码，方便先单独测）：
       车头先对着柱子 → 三个旋钮 V_TAN/W_TURN/V_RAD 跑开环圆，FB_DIST / FB_LASER 两个开关再单独叠加
       测距反馈（V_RAD 调径向速度）+ 双激光反馈（LAS_YAW 调摆速 / LAS_TAN 调切向速度），
       陀螺仪累计转角满 355° 停；测距连续 20 次无效 → 保护停车（只在测距反馈开着时生效） */
    LiZhu_Circle_Run();
    UART1_Printf("lizhu circle done\r\n");
    return 1;
  }
  return 0;                                         // 其它命令：交给主循环下面的原逻辑
}

/* ==================== 传感器触发后的停车（2026-09-20 新底盘） ====================
   原来的 SENSOR_BRAKE()（给一小段反向速度 + 临时改 chassis.start_margin 造"柔性反向力矩"）
   是给老底盘打的反冲补丁，已整段删除。新底盘停车只要一句 ROBOT_MoveSpeed(0, 0)：
   速度环是闭环的，target=0 而四轮还在转时当拍就反向修正（≈主动反接刹车），
   比"断电自由滑行"刹得快得多（老底盘要滑 6~20cm），也不用再补固定位移；
   真停稳后由底盘的"静止零漂保护"清零断电。
   下面 4 处（去仓库 / 回家 各 2 处）原来调它，现在统一换成 SENSOR_STOP()：
   紧跟着的 ROBOT_Angle 会阻塞到"航向到位且四轮停稳"才返回，所以"延时候补位移"都不需要了。
   ★现场核实：老的 SBRAKE 日志没有了；若某个停车点位置不对，直接改那一条固定位移即可。 */
static void SENSOR_STOP(void)
{
  ROBOT_MoveSpeed(0, 0);      /* 停车：速度环 target=0 → 主动反接刹车；停稳后底盘自己清零断电 */
}


/* ==================== ★★ 颜色传感器(TCS34725)校准模式（KEY0 起点选 CAL + KEY3 进入）★★ ====================
   只做一件事：把 黑/红/蓝/白 四种颜色各采 5 次，原始值+H/S/V 打到串口1；
   拿回来按实测数据重写 tcs34725.h 的三个判色阈值（★不写 Flash、不改运行时逻辑，
   阈值永远是头文件里的编译期宏 TCS_BLUE_S_MAX / TCS_BLUE_V_MAX / TCS_BLACK_V_THRESH）。

   按键： K1 = 切换颜色（黑→红→蓝→白→黑…，换颜色时采样计数清零）
          K2 = 采样一次：串口打一行(C/R/G/B 原始值 + H/S/V + 当前判色结果)，屏幕同步显示
          K3 = 返回（回"红蓝方选择"菜单）
   用法： 每种颜色放好 → 按 K2 采 5 次（串口 5 行）→ K1 换下一个颜色 → 四色采完按 K3 退出；
          串口那 20 行 `CAL[...]` 直接复制发回来即可。
   ★要求：采的时候和比赛时保持同样的距离/灯光，不然量出来的值没意义。 */
static const char * const COLORCAL_NAME[4] = { "BLACK", "RED", "BLUE", "WHITE" };  /* K1 的切换顺序 */

static void COLOR_Calib_Run(void)
{
  TCS34725_RGBC rgbc;
  uint8_t  idx    = 0;               /* 当前在采的颜色：0=黑 1=红 2=蓝 3=白 */
  uint8_t  n      = 0;               /* 当前颜色已经采了几次 */
  uint8_t  got    = 0;               /* 上一次采样是否有效（无效就不显示旧数据） */
  uint32_t oled_t = 0;

  UART1_Printf("\r\n--- TCS34725 CAL: K1=switch color, K2=sample+print, K3=exit ---\r\n");
  UART1_Printf("order BLACK->RED->BLUE->WHITE, 5 samples each, then paste the CAL lines back\r\n");
  UART1_Printf("TH in code (tcs34725.h): TCS_BLUE_S_MAX=%.3f TCS_BLUE_V_MAX=%.3f TCS_BLACK_V_THRESH=%.3f\r\n",
               (double)TCS_BLUE_S_MAX, (double)TCS_BLUE_V_MAX, (double)TCS_BLACK_V_THRESH);
  UART1_Printf("now target = %s (press K2)\r\n", COLORCAL_NAME[idx]);

  while(1){
    /* ---- K1：切换颜色（黑→红→蓝→白→黑…） ---- */
    if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){
      idx = (uint8_t)((idx + 1) & 0x03);
      n   = 0;                                     /* 换颜色 → 计数清零（每种颜色都从 #1 开始） */
      got = 0;
      UART1_Printf("now target = %s (press K2)\r\n", COLORCAL_NAME[idx]);
    }

    /* ---- K2：采样一次并打串口 ---- */
    if(KEY_ONE(KEY2_GPIO_Port, KEY2_Pin)){
      if(TCS34725_GetRawData(&rgbc) && rgbc.c > 0){
        got = 1;
        n++;
        UART1_Printf("CAL[%s] #%u C=%5u R=%5u G=%5u B=%5u H=%6.1f S=%.3f V=%.3f -> %s\r\n",
                     COLORCAL_NAME[idx], (unsigned)n,
                     (unsigned)rgbc.c, (unsigned)rgbc.r, (unsigned)rgbc.g, (unsigned)rgbc.b,
                     (double)rgbc.h, (double)rgbc.s, (double)rgbc.v,
                     TCS34725_ColorName(TCS34725_ClassifyColor(&rgbc)));
      }else{
        got = 0;
        UART1_Printf("CAL[%s] READ FAIL (C=0): check VCC=3.3V / SCL=PB9 / SDA=PB4\r\n", COLORCAL_NAME[idx]);
      }
    }

    /* ---- K3：返回菜单 ---- */
    if(KEY_ONE(KEY3_GPIO_Port, KEY3_Pin)){
      UART1_Printf("CAL exit\r\n");
      break;
    }

    /* ---- 屏幕(100ms)：当前颜色 + 上次采样结果 + 按键说明 ---- */
    if(HAL_GetTick() - oled_t >= 100U){
      oled_t = HAL_GetTick();
      OLED_Clear();
      OLED_Printf(0,  0, OLED_8X16_HALF, "CAL %s", COLORCAL_NAME[idx]);
      OLED_Printf(0, 16, OLED_6X8_HALF, "K1:col K2:sam K3:ex");
      if(got){
        OLED_Printf(0, 24, OLED_6X8_HALF, "S:%.2f V:%.2f H:%.0f",
                    (double)rgbc.s, (double)rgbc.v, (double)rgbc.h);
        OLED_Printf(0, 32, OLED_6X8_HALF, "C:%5u R:%5u", (unsigned)rgbc.c, (unsigned)rgbc.r);
        OLED_Printf(0, 40, OLED_6X8_HALF, "G:%5u B:%5u", (unsigned)rgbc.g, (unsigned)rgbc.b);
      }else{
        OLED_Printf(0, 24, OLED_6X8_HALF, "press K2 to sample");
      }
      OLED_Printf(0, 48, OLED_6X8_HALF, "n=%u (5 each)", (unsigned)n);
      OLED_Update();
    }
    HAL_Delay(10);
  }
}

/* ================= 回家阶段：读一次颜色 + 把"当前判成什么色"显示到 OLED 第3行 =================
   回家段全靠颜色传感器判断"进没进红/蓝区"，现场盯屏幕就能看出它现在判成什么色。
   显示格式：第3行(y=32) "COL:BLACK / RED / BLUE / ..."（判色规则见 tcs34725.h 的编译期阈值）。
   ★只有在"等颜色"的循环里调用：每次采样读一次、顺手刷一行屏，不额外占时间；
     返回值 = TCS34725_ClassifyColor() 的结果（调用方的判黑/判非黑逻辑一模一样，没变）。 */
static uint8_t TCS_PollColor(TCS34725_RGBC *rgbc){
  TCS34725_GetRawData(rgbc);                     //和原来一样：读一次（不看返回值）
  uint8_t col = TCS34725_ClassifyColor(rgbc);    //按 HSV 判色
  OLED_ClearArea(0, 32, 128, 16);                //只清第3行
  OLED_Printf(0, 32, OLED_8X16_HALF, "COL:%s", TCS34725_ColorName(col));
  OLED_Update();
  return col;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_UART5_Init();
  MX_TIM7_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */


  OLED_Init();


  /*外设启动区域*/
  HAL_TIM_Base_Start_IT(&htim7);//开启1ms中断

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//开启车轮的PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);//开启车轮的encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
  /* 编码器采用清零法：每10ms读CNT后清零，计数器不会溢出，无需开启溢出中断（之前开中断但缺处理函数会导致卡死） */
  HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin, GPIO_PIN_SET);//TB6612使能

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RxBuf, UART1_RxLength);//调试串口
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RxBuf, UART2_RxLength);//主视觉串口
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_RxBuf, UART5_RxLength);//步进电机串口
  // __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);    //使用DMA+UART5时，会开启传输过半中断，需手动关闭

  UART4_RxInit();//串口4(副视觉)接收初始化（DMA1_Stream2+空闲中断收帧，实现见 Mycode/uart.c）




  HAL_Delay(300);

  HWT101CT_Init();//陀螺仪初始化
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_RxBuf, UART3_RxLength);//陀螺仪串口
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
  flag.hwt101ct = 1;

  CHASSIS_Init();//底盘初始化：配置4轮速度环PID参数 + 航向环(角度环)参数
  flag.chassis = 1;
  flag.angle   = 1;   // 航向环（角度环）默认开启：上电锁定当前朝向，串口 tyaw 可遥控转向


  




  // /*接收副视觉目标字母*/
  // while(1){
  //   VISION_ReceiveLetter();
  //   UART4_Printf("%X %X\r\n", vision_target_letter[0], vision_target_letter[1]);//打印副视觉发过来的两个目标字母
  // }
  // /*主视觉测试（临时注释：先验证灰度，测完恢复）*/
  // while(1){
  //   if(VISION1_RxFlag){
  //     VISION1_RxFlag = 0;
  //     VISION_ReceiveData(VISION1_RxBuf, VISION1_RxRealLength);
  //     OLED_Printf(0, 0, OLED_8X16_HALF, "suc:%1d peri:%1d", VISION_Data.success, VISION_Data.period);
  //     OLED_Printf(0, 16, OLED_8X16_HALF, "tar:%1d", VISION_Data.target);
  //     OLED_Printf(0, 32, OLED_8X16_HALF, "x:%3d y:%3d", VISION_Data.x, VISION_Data.y);
  //     OLED_Printf(0, 48, OLED_8X16_HALF, "dis%4d", VISION_Data.distance);
  //     OLED_Update();
  //   }
  // }
//
//
  //// while(1){
  //   if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
  //     ROBOT_Move(-50, 390, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);
//
//
  ////     ROBOT_Angle(270);
  //   }
//
  ////   /* 串口打印角度环数据（目标/实际/输出w + 里程计位置x/y），SerialPlot 观察走直线纠偏/转向收敛
  //      并解析串口调参指令：kp/ki/kd/target 角度环、vx/vy 手动、mx/my 走距、mv/mvacc 规划速度 */
  //   UART1_Printf("%f %f %f %f %f\r\n",
  //                chassis.target_yaw,
  //                HWT101CT_Data.yaw,
  //                chassis.yaw_pid.out,
  //                chassis.pos_x,
  //                chassis.pos_y);
  //   if(UART1_RxFlag){
  //     UART1_RxFlag = 0;
  //     SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
  //   }
  //   HAL_Delay(10);
  // }
//
//
  /*//速度环调参测试（临时注释：先跑下方编码器裸测标定 ACCURACY，测完恢复）*/
  //// while(1){
  //   OLED_Printf(0, 0, OLED_8X16_HALF, "kp:%06.2f", chassis.speed_pid[1].kp);
  //   OLED_Printf(0, 16, OLED_8X16_HALF, "ki:%06.2f", chassis.speed_pid[1].ki);
  //   OLED_Printf(0, 32, OLED_8X16_HALF, "kd:%06.2f", chassis.speed_pid[1].kd);
  //   OLED_Printf(0, 48, OLED_8X16_HALF, "tar:%06.2f", chassis.speed_pid[1].target);
  //   OLED_Update();
  //   UART1_Printf("%f %f %f %f\r\n", chassis.speed_pid[1].target, chassis.speed_pid[1].actual, chassis.speed_pid[1].out, chassis.speed_pid[1].errorint);
  //   if(UART1_RxFlag){
  //     UART1_RxFlag = 0;
  //     SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
  //   }
  // }
  // /*激光传感器测试--通过*/
  // while(1){
  //   OLED_Printf(0, 0, OLED_8X16, "barrier:%1d", LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin));
  //   OLED_Update();
  // }
  // /*测距传感器--通过*/
  // while(1){
  //   OLED_Printf(0, 0, OLED_8X16, "distance:%4d", GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin));
  //   OLED_Update();
  // }
  // /*灰度传感器--通过*/
  /*// 灰度传感器读取：GRAY1/GRAY3 都走串行 IO 接口（GPIO 模拟时钟），读 8 路数字量（0=深、1=浅）
    // 引脚（CubeMX 配置）：GRAY1=PB4(DAT)/PB9(CLK)、GRAY3=PB6(DAT)/PB7(CLK)
    // OLED 第一行(y=0)显示 GRAY1 八通道，第三行(y=32)显示 GRAY3 八通道 */
  //
  //// /*陀螺仪测试--通过*/
  // HWT101CT_Init();
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_RxBuf, UART3_RxLength);//陀螺仪串口
  // __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
  // flag.hwt101ct = 1;
  // while(1){
  //   OLED_Printf(0, 0, OLED_8X16_HALF, "yaw:%6.2f", HWT101CT_Data.yaw);
  //   OLED_Update();
  // }
  // /*电机PWM、encoder测试--通过*/
  // int i = 0;
  // while(1){
  //   if(KEY_ONE(KEY3_GPIO_Port, KEY3_Pin)){
  //     i++;
  //     if(i > 10) i = -10;
  //     TB6612_Control(MOTOR_Left_Front, i*100);
  //     TB6612_Control(MOTOR_Left_Back, i*100);
  //     TB6612_Control(MOTOR_Right_Back, i*100);
  //     TB6612_Control(MOTOR_Right_Front, i*100);
  //     OLED_Printf(0, 0, OLED_8X16_HALF, "PWM: %+4d", i * 100);
  //     OLED_Update();
  //   }
  //   OLED_Printf(0, 16, OLED_8X16_HALF, "L_F:%+3d L_B:%+3d", ENCODER_GetPulse(ENCODER_LeftFront), ENCODER_GetPulse(ENCODER_LeftBack));
  //   OLED_Update();
  //   HAL_Delay(10);
  // }
  // /* 临时调参：SerialPlot 串口画图 + 在线改PID（调好即删，改回正常逻辑） */
  // SERIALPLOT_PIDAdjustParam();
//
//
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //
//
  ////   HAL_Delay(2000);
  //   if (GW_Gray_Init() == 0) {
  //   UART1_Printf("GW Gray online\r\n");
  // } else {
  //   UART1_Printf("GW Gray offline\r\n");
  // }
  // ROBOT_MoveSpeed(0,30);
  // HAL_Delay(2000);
  // ROBOT_MoveSpeed(0,0);
  // ROBOT_Move(0, 200, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);
//
  /*// ==================== TCS34725 颜色识别（替换原 1 号灰度传感器 GRAY1） ====================
    // SCL=PB9、SDA=PB4（原 GRAY1 的 CLK/DAT 线位），软件 I2C 100kHz，无需改 CubeMX；
    // 模块供电 3.3V，LED/INT 悬空。GRAY3 仍走串行接口用于循线（GRAY_Data[GRAY3] 依旧可用）。
    // 判色由 TCS34725_ClassifyColor() 完成：只分 黑/非黑 两类（红蓝统称非黑），
    // 串口打印原始 C/R/G/B + HSV，可接 SerialPlot 观察并按实际物体标定阈值（见 tcs34725.h）。 */
  //uint8_t tcs_online = TCS34725_Init();
  //UART1_Printf("TCS34725 %s, ID=0x%02X\r\n",
    //           tcs_online ? "ONLINE" : "OFFLINE", TCS34725_GetID());

  /* ==================== TCS34725 颜色传感器上电初始化（★必须有，别跟着调试代码一起注释掉） ====================
     ★2026-09-14 修复："颜色传感器又坏了"的根因就在这里 ——
       cf20b8f(8.31 圆盘机视觉通信) 那次整理调试代码时，把上面那两行
       (//uint8_t tcs_online = TCS34725_Init(); / //UART1_Printf(...)) 连同整个调试循环
       一起注释掉了，驱动里 tcs_i2c_gpio_init() 从此从没跑过：
         · PB9(SCL) 还是 CubeMX 配的推挽输出 —— 能翻转，看着"像在工作"；
         · PB4(SDA) 还是 CubeMX 配的"输入上拉"(见 gpio.c) —— 主机根本拉不低 SDA，
       起始条件发不出去、从机地址收不到 ACK → GetRawData() 里 I2C 全失败 → C/R/G/B 全 0
       → S=0、V=0 → ClassifyColor() 恒判 BLUE（不是黑），回家阶段
       "读到红/蓝才停""等到黑才停"全部判错 → 看起来就是传感器坏了。
     这里调用 TCS34725_Init() 会一次性做完：PB9/PB4 配成开漏+内部上拉、
     设 50ms 积分时间 + 1x 增益、PON+AEN 上电并使能 ADC。
     上电后串口1(115200) 会打一行 "TCS34725 ONLINE, ID=0x44"；
     若打成 OFFLINE，先查模块供电/接线（VCC=3.3V、GND、SCL=PB9、SDA=PB4），再看这里有没有被注释。
     另：GRAY1 的串行驱动 GRAY1_Serial_Update() 也不能恢复使用 —— 它会拿串口时序去驱动 PB9/PB4，
        一样会把 I2C 打坏（gw_grayscale.c 的 GRAY_Update() 里已不再调它，别加回去）。 */
  {
    uint8_t tcs_online = 0;
    for(uint8_t tcs_retry = 0; tcs_retry < 3 && !tcs_online; tcs_retry++){
      tcs_online = TCS34725_Init();       /* 上电初期模块可能还没就绪：最多重试 3 次 */
      if(!tcs_online) delay_ms(50);
    }
    UART1_Printf("TCS34725 %s, ID=0x%02X\r\n",
                 tcs_online ? "ONLINE" : "OFFLINE", TCS34725_GetID());
  }

  /* 判色阈值全部来自 tcs34725.h 的编译期宏（不读/不写 Flash）：
     标定 = 进 KEY0 起点里的 CAL 模式把四色各采 5 次，按实测值重写那三个宏后重新编译烧录。 */

  /* ★★ 固件版本标记（开机就打，用来确认"烧进去的到底是不是最新代码"）★★
     ★2026-09-15 踩过的坑：改了 main.c 但没重新 build+flash（或编辑器把旧内容覆盖回去），
       串口看到的还是旧行为，白折腾半天。以后看这一行的 build 时间就知道固件新旧。
     ★2026-09-20 换队友重调的新底盘（每轮独立速度环PID + 4倍频878编码器 + 20ms控制周期 +
       航向环三档/静摩擦bias + 停车由速度环闭环反接），主流程里的"反冲刹车(SENSOR_BRAKE)、
       白线反向急刹(LINE_BRAKE)、调 start_margin/brake_decel、move/angle 后的稳定延时"全部删除。
       看到 "NEW-CHASSIS 09-20" 才是这一版。 */
  UART1_Printf("FW: NEW-CHASSIS 09-20 (per-wheel PID, 878enc, 20ms, no brake-patch) built %s %s\r\n", __DATE__, __TIME__);

  while (1)
  {
    /* ===== 串口1实时发四轮实际值（2026-09-25 加，实现在 serialplot.c）=====
       每 WHEEL_ACT_SEND_MS(默认20ms) 打一行 4 个数字：左前/左后/右后/右前 实际速度 cm/s，
       直接接 SerialPlot 看四条曲线；串口1发单键 'v' 可随时开关（见 UART1_DebugCmd）。
       ★本函数非阻塞、自己按时间节流，放循环里不占时间；车在跑时的那几个阻塞等待
         循环里也各调了一次（见 robot.c），所以整趟动作都有数据。 */
    SERIALPLOT_WheelActualPump();

    ///* GRAY3 仍走串行更新（循线数据 GRAY_Data[GRAY3] 保持有效） */
    //GRAY3_Serial_Update();

    ///* TCS34725 读颜色：OLED 显示颜色名 + R/G/B + 亮度（串口不自动刷新，只在按键采样时打印） */
    static TCS34725_RGBC tcs_rgbc = {0};
    //if(TCS34725_GetRawData(&tcs_rgbc)){
    //  uint8_t tcs_col = TCS34725_ClassifyColor(&tcs_rgbc);
    //  OLED_Printf(0, 0,  OLED_8X16_HALF, "col:%s", TCS34725_ColorName(tcs_col));
    //  OLED_Printf(0, 16, OLED_8X16_HALF, "R:%3d G:%3d B:%3d", tcs_rgbc.r, tcs_rgbc.g, tcs_rgbc.b);
    //  OLED_Printf(0, 48, OLED_8X16_HALF, "C:%4d V:%3d%%", tcs_rgbc.c, (uint8_t)(tcs_rgbc.v * 100));
    //  delay_ms(200);
    //} else {
    //  OLED_Printf(0, 0, OLED_8X16_HALF, "TCS OFFLINE");
    //}
//
    ///* ==================== 颜色采样：按一次按钮 = 串口发一条当前值 ====================
    //   把黑/红/蓝物体放到传感器下，按对应按钮一次，串口立即打一条 H/S/V：
    //     按钮2(KEY1)=黑 -> BLK H=.. S=.. V=..
    //     按钮3(KEY2)=红 -> RED H=.. S=.. V=..
    //     按钮4(KEY3)=蓝 -> BLU H=.. S=.. V=..
    //   每色按 8 次共 24 条，直接复制发回来定阈值。 */
    //if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){                 /* 按钮2：黑 */
    //  TCS34725_GetRawData(&tcs_rgbc);                     /* 按下瞬间重新采一次 */
    //  UART1_Printf("BLK H=%5.1f S=%0.2f V=%0.2f\r\n", tcs_rgbc.h, tcs_rgbc.s, tcs_rgbc.v);
    //}
    //if(KEY_ONE(KEY2_GPIO_Port, KEY2_Pin)){                 /* 按钮3：红 */
    //  TCS34725_GetRawData(&tcs_rgbc);
    //  UART1_Printf("RED H=%5.1f S=%0.2f V=%0.2f\r\n", tcs_rgbc.h, tcs_rgbc.s, tcs_rgbc.v);
    //}
    //if(KEY_ONE(KEY3_GPIO_Port, KEY3_Pin)){                 /* 按钮4：蓝 */
    //  TCS34725_GetRawData(&tcs_rgbc);
    //  UART1_Printf("BLU H=%5.1f S=%0.2f V=%0.2f\r\n", tcs_rgbc.h, tcs_rgbc.s, tcs_rgbc.v);
    //}
//
    //HAL_Delay(50);
    //测距，2是前面的，1是后面的
    //OLED_Printf(0,16 , OLED_8X16_HALF, "dis_1:%4d", GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin));
    OLED_Printf(0,32 , OLED_8X16_HALF, "dis_2:%4d", GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin));
    //激光
    //OLED_Printf(0, 48 ,OLED_8X16_HALF, "ba_3:%1d", LASER_Barrier(LASER3_GPIO_Port, LASER3_Pin));
    //OLED_Printf(64, 48, OLED_8X16_HALF, "ba_2:%1d", LASER_Barrier(LASER2_GPIO_Port, LASER2_Pin));
    // OLED_Printf(32, 32, OLED_8X16_HALF, "bar_3:%1d", LASER_Barrier(LASER3_GPIO_Port, LASER3_Pin));
    OLED_Update();


    /* ===== UART1 指令：解析 + 回显 + 立柱绕圈调试指令(7)就地执行（实现在文件上方 UART1_DebugCmd） =====
       注意：7 已经在上面被处理掉了，所以下面只留注释说明，不再重复写代码 */
    UART1_DebugCmd();

    //1;-50;390;100;100;100;100;270
    if(UART1_Data[0]==3){
    ROBOT_Move(UART1_Data[1], UART1_Data[2], UART1_Data[3], UART1_Data[4], UART1_Data[5], UART1_Data[6]);
    ROBOT_Angle(UART1_Data[7]);
    UART1_Data[0]=0;
    }
    

    /* ---------- 正面识别通信的屏幕显示(实现在文件上方的 ZM_ShowComm()，这里只说"屏幕上该看到什么") ----------
       一屏四行、位置固定，状态一变就整屏重画一次(不会花屏、不会留旧字)：
         y=  0  8x16  "ZM 0/3 TO POS"        进度0：还没发0xA2(正在收倒球槽+移动到识别位)
                      "ZM 1/3 SEND OK"      进度1：0xA2已发给主视觉+副视觉
                      "ZM 2/3 LETTER OK"    第2步：副视觉字母已收到、已转发主视觉、已回执副视觉
                      "ZM DONE 0xA7 OK"     第3步：主视觉的0xA7已收到，正面识别结束
         y= 16  8x16  "V4 A,B 0xAB OK"      副视觉(备用串口UART4)：字母(A~D)+原始字节，成功一次就锁死不被覆盖
                      "V4 WAIT letter"      还没收到
                      "V4 0x12 BAD"         收到了别的单字节          "V4 len3 BAD" 帧长不是1
         y= 32  8x16  "V2 0xA7 OK"          主视觉(串口2)：确认成功
                      "V2 WAIT 0xA7"        还没收到                  "V2 0x12 BAD" 收到别的字节
         y= 48  6x8   "r4:1 r2:2 U4:RX 12s" 收帧数 + 串口4在听(RX正常/ERR停过，程序会自动重启它) + 进入本阶段秒数
                      ★秒数一直在涨=程序在跑；不动=死等在某一步(配合电脑串口1的日志看是哪一步)
       对应流程：0走到识别位 → 1发0xA2 → 2收副视觉字母(6种之一) → 3转发主视觉+回执副视觉 → 4等主视觉0xA7 → 5结束
       ★排查用(电脑串口1，115200)：完整收发时序都会打印
         "TX 0xA2 -> V1(UART2) + V4(UART4)" / "RX4 len=1: 0xAB" / "RX2 len=1 got=0xA7"
         "TX relay 0xAB -> V1(UART2), TX 0xA7 -> V4(UART4)"                              */
    int comm_rx4 = 0, comm_rx2 = 0;                //副视觉(串口4)收帧数 / 主视觉(串口2)收帧数
    uint32_t comm_t_oled = 0;                      //OLED刷新节流用时刻(ms)

    //进入各部分的标志位，红蓝可共用
    uint8_t YuanPanJi_Flag = 0;
    uint8_t ZhengMian_Flag = 0;
    uint8_t JieTi_Flag = 0;
    uint8_t LiZhu_Flag = 0;
    uint8_t HuiJia_Flag = 0; 

    uint8_t dbg_start = DBG_START_ALL;   //★按键0选好的"调试起始阶段"(0=完整流程；取值见文件上方 DBG_START_xxx)
    //完整走
    //if(UART1_Data[0]==4)

    //红蓝方选择
    while(1){
      /* 菜单界面也能用串口调试指令：发 "7,0,0,0,0,0,0,0" 就地绕圈，
         不用先按 KEY3 进流程。解析/回显/执行都在 UART1_DebugCmd() 里（见文件上方） */
      UART1_DebugCmd();
      SERIALPLOT_WheelActualPump();   // 串口1实时发四轮实际值（菜单里也发：单键 w/s/a/d 手动测车时就能看曲线）
      OLED_Printf(0, 0, OLED_8X16_HALF, "mode:%s", mode_red?"red ":"blue");
      OLED_Printf(0, 16, OLED_8X16_HALF, "k2:mode k0:sel");  //按键2=切红/蓝方，按键0=选起始阶段
      OLED_Printf(0, 32, OLED_8X16_HALF, "k3:GO %s", DBG_START_NAME[dbg_start]);   //按键3=从选好的起点开始跑
      OLED_Printf(0, 48, OLED_8X16_HALF, "%s", DBG_START_DESC[dbg_start]);   //当前起点说明
      OLED_Update();
      if(KEY_ONE(KEY2_GPIO_Port, KEY2_Pin)){//按键2--更改模式
        mode_red = !mode_red;
      }
      if(KEY_ONE(KEY3_GPIO_Port, KEY3_Pin)){//按键3--按选好的起始阶段开始(默认=完整流程，与以前完全一样)
        break;
      }

      /* ============== ★★ 按键0：选择"调试起始阶段"（想单独调哪一段，就让车从哪一段开始跑）★★ ==============
         每按一次 KEY0 往后切一个：0=ALL → 1=ZM → 2=HOME → 回到 0=ALL
           0=ALL  完整流程：圆盘机→仓库倒球→正面识别→阶梯→回家（默认，主流程一字不变）
           1=ZM   从"正面识别前"开始：跳过圆盘机+仓库倒球，车自己先走到阶梯识别位再开始正面识别；
                  车放在圆盘机/仓库方向随便一点的位置即可（会先补发红/蓝方给视觉）
           2=HOME 从"回家"开始：跳过前面全部阶段，直接跑回家那段；
                  车放在"回家"的起始位置附近（红蓝区旁边）即可
         ★当前选择显示在第3/4行("k3:GO ZM" + "front recog")；选好后按 KEY3 才开始跑。
           真正的跳转代码在菜单while外面（见下面"调试起点跳转"），所以起始阶段=ALL 时它什么都不做。 */
      if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
        dbg_start++;
        if(dbg_start > DBG_START_MAX) dbg_start = DBG_START_ALL;    //切到最后一个再按 → 回到"完整流程"
        UART1_Printf("DEBUG: KEY0 start stage = %u (%s) %s\r\n",
                     (unsigned)dbg_start, DBG_START_NAME[dbg_start], DBG_START_DESC[dbg_start]);
      }
      //按键0/按键1专用于调试（按键0=选起始阶段；按键1=显示左后激光，见下面那段）

        /* ---- 原来的按键0调试(手动把车开到阶梯对准位)已由上面跳转代替，保留备用 ----
        //右+前，移动到阶梯附近,要往右速度快点，不然撞到了
        ROBOT_Move(100, 160, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);

        //向左慢走，直到前面的两个光电都感应到障碍物
        ROBOT_MoveSpeed(-SPD_AVG_V, 0);
        //while(LASER_Barrier(LASER2_GPIO_Port, LASER2_Pin)==0);
        while(LASER_Barrier(LASER3_GPIO_Port, LASER3_Pin)==0);//左后是1，右边为2，左前是3


        //往左走一定距离，视觉中能完整看到两个字母（可以省去测距前后校准）
        ROBOT_Move(-50, 0, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);
        //停车准备识别
        ROBOT_MoveSpeed(0,0);
        HAL_Delay(100);//延时一下提高稳定性
        -------------------------------------------------------------------------- */

      if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){

        OLED_Printf(0, 0, OLED_8X16_HALF, "barrier:%1d", LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin));
        OLED_Update();

        /* ================= 旧代码开始（原来的按键1"倒球调试"，先整段注释保留）=================
           要恢复原来的倒球调试：把本行与下面"旧代码结束"那一行的注释符去掉，并把上面的等待接收测试删掉即可。

        //往后慢退，直到测距测得合适距离（适合倒球的距离）
        UART1_Printf("3");
        ROBOT_MoveSpeed(0, -SPD_AVG_V);
        while(GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin)>90);//100有点远，距离小于90就推出此循环
        ROBOT_MoveSpeed(0,0);
        
        //定位操作：向左慢平移到左后光电感应到无障碍物
        UART1_Printf("4");
        ROBOT_MoveSpeed(-SPD_AVG_V, 0);
        while (LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin)==1);
        ROBOT_MoveSpeed(0,0);

        //往右走固定距离（刚到对上仓库的距离）
        ROBOT_Move(10, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);//20太大，速度100会飘

        runActionGroup(16, 1); 	//这里是倒球动作组
	      delay_ms(2000);
        ================= 旧代码结束 ================= */

      }
    }
    /* ============== ★★ 调试起点跳转（在"红蓝方选择"界面按 KEY0 选、再按 KEY3 开始后才会走到这里）★★ ==============
       dbg_start==DBG_START_ALL(0)：一个字都不执行，原样往下跑完整流程（圆盘机→仓库倒球→正面识别→阶梯→回家）
                                     → 和"没有这套调试入口"时完全一致；
       ★跳转代码放在菜单while外面，就是为了让"完整流程"连一条语句都不变。 */
    /* ★每次运行开始先把"角度基准换算"复位：它是静态变量，跑过一次跳转起点后若不清，
       切回完整流程(dbg_start=ALL)时还会带着上一段的换算 → 所有绝对角都被换算，那就错了。 */
    yaw_shift_deg = 0;
    if(dbg_start == DBG_START_ZHENGMIAN){
      /* 从"正面识别前"开始：跳过圆盘机+仓库倒球，直接跳到 ZHENGMIAN_START 标签往下执行
         （往下依次是：收倒球槽 → 走到阶梯识别位 → 左前光电对准 → 清串口残留 → 发0xA2通信流程 → 阶梯 → 回家）
         ★车会先自己走到阶梯识别位，所以先把车放在圆盘机/仓库方向随便一点的位置即可 */
      UART1_Printf("DEBUG: start from ZHENGMIAN (before front recognize)\r\n");
      /* ★★ 角度基准：本次车是"按倒完球那一步的姿态"摆好再上电的
         （红方车头朝右=基准90°，蓝方自动 +180°=270°即车头朝左，两边差 180°）
         → SetYawShift() 会按红蓝自动设好 yaw_shift_deg，之后每一处 ROBOT_Angle 都走 Yaw_Abs()。 */
      SetYawShift(90);
      //跳过了圆盘机，就没走"告诉视觉红(0xAA)蓝(0xBB)方"那一步，这里补上（视觉也得知道红/蓝方）
      UART2_Printf("%c", mode_red ? 0xAA : 0xBB);//告诉视觉红(0xAA)蓝(0xBB)方
      goto ZHENGMIAN_START;
    }
    else if(dbg_start == DBG_START_LIZHU){
      /* 从"立柱"开始：直接跳到 LIZHU_START 标签往下执行
         （立柱前校准：往左 5cm/s 直到两个激光都有障碍物 → 向前 5cm/s 到前测距 200mm
           → LiZhu_Circle_Run() 绕柱转圈 → 走到仓库中间倒方块 → 回家）
         ★角度基准：立柱起点的车头是"朝左"(红方基准 270°) → 蓝方自动 +180° = 90°(车头朝右)，
           两边差 180°；换算后 Yaw_Abs(270)=0，即"按这个姿态摆好就不转"。 */
      UART1_Printf("DEBUG: start from LIZHU (pillar)\r\n");
      SetYawShift(270);
      LiZhu_Flag = 1;     //"立柱"段的进入条件
      goto LIZHU_START;
    }
    else if(dbg_start == DBG_START_HUIJIA){
      /* 从"回家"开始：跳过前面全部阶段，直接跳到 HUIJIA_START 标签往下执行（红蓝区找色→停进红蓝区）
         ★角度基准和"单独测阶梯(ZM)"完全一样：红方按"倒完球姿态"(车头朝右)摆、蓝方自动 +180°(朝左)，
           本段所有绝对角都走 Yaw_Abs() 换算。 */
      UART1_Printf("DEBUG: start from HUIJIA (go home)\r\n");
      SetYawShift(90);   //和 ZM 起点同一套（红90 / 蓝270）
      HuiJia_Flag = 1;   //"回家"那段的进入条件是 HuiJia_Flag==1：正常流程由前面阶段置位，这里跳转就先自己立好
                         //(其余阶段标志 YuanPanJi_Flag/JieTi_Flag/LiZhu_Flag 出菜单时本来就都是0，不用管)
      goto HUIJIA_START;
    }
    else if(dbg_start == DBG_START_COLORCAL){
      /* 从"颜色校准"开始：跟比赛流程完全无关（只读颜色传感器 + 按键 + 打串口），
         跑完 continue 回"红蓝方选择"菜单，不会往下走圆盘机/仓库等任何比赛阶段。
         用法：KEY0 切到 CAL → KEY3 进入 → K1 切颜色、K2 采一次并打串口、K3 返回。 */
      UART1_Printf("DEBUG: start COLOR CALIBRATION\r\n");
      COLOR_Calib_Run();
      continue;                 /* 回菜单：想继续跑比赛流程，把起点切回 ALL 再按 KEY3 */
    }

    //？后面，左蓝右红
    
      
    /**************圆盘机****************/
  
//while(1){
   // ROBOT_Move(15,0,20,0,30,0);
    //HAL_Delay(1000);
    //ROBOT_Move(-15,0,20,0,30,0);
//}
    YuanPanJi_Flag = 1;//圆盘机开始
    UART2_Printf("%c", mode_red ? 0xAA : 0xBB);//告诉视觉红(0xAA)蓝(0xBB)方

    //路上就先把机械臂举起来
    runActionGroup(1, 1);//不需要延时，因为和出发一起
    
    //先盲走到圆盘机中心+面向（★长距 418cm：两档标准里的 120/120）
    ROBOT_Move(mode_red?-88:88,433,SPD_LONG_V,SPD_LONG_V,SPD_LONG_A,SPD_LONG_A);//58太靠右
    /* ★2026-09-20 新底盘：ROBOT_Move / ROBOT_Angle 都是阻塞式，且 ROBOT_Angle 会等到
       "航向到位 且 四轮真正停稳"才返回 —— 后面不用再补 HAL_Delay(100)"提高稳定性"了 */
    mode_red ? ROBOT_Angle(270) : ROBOT_Angle(90);

    
    /* ==================== 向前慢走，直到灰度(探头1)感应到白线 ====================
       ★★★ 2026-09-15 修复"停不下来"的根因：运算符优先级 ★★★
         上一版写成了 while(GRAY_Data[GRAY3][0] | GRAY_Data[GRAY3][1] == 0)，而 C 里
         == 的优先级 高于 |，它其实等价于：
             GRAY_Data[GRAY3][0] | (GRAY_Data[GRAY3][1] == 0)
         于是"退出循环＝停车"的充要条件被悄悄改成【探头1是黑 且 探头2是白】：
           · 探头1变白（探头2任意）→ 1 | …  = 1 → 继续走，不停；
           · 两个探头都变白        → 1 | 0  = 1 → 还是继续走，不停！（＝你看到的现象）
         更坑的是：只要探头1这一位恒为 1（探头没压到线/通道坏/DAT线松或模块没电——串行
         DAT 上拉到 3.3V，脱线时全读 1），这行表达式恒为真 → 车永远不停、一直往前冲。
         编译日志里本来就有这条警告，说的就是它：
             warning: suggest parentheses around comparison in operand of '|' [-Wparentheses]
         现在按你的要求只用一个探头（序号0＝探头1）：读到黑(0)继续走，读到白(1)立刻停。 */
    /* ==================== 恒速逼近白线 + 检测到白线就停车 ====================
       ★2026-09-20 换成队友重调的新底盘（4倍频编码器878 + 20ms控制周期 + 每轮独立速度环PID +
         航向环三档/静摩擦bias + 停车由速度环闭环"反接"完成）后，本节原来的"反冲急刹"补丁已整段删除：
         · 逼近速度回到命令值本身（命令 10cm/s 就是 10cm/s）：老底盘的"破静摩擦整形"会把低速命令
           顶到 ≈16~29cm/s，那才需要"给反向速度冲一下"来抵消动能；新底盘不需要。
         · 停车直接用 ROBOT_MoveSpeed(0, 0)：速度环 target=0 而四轮还在转 → 当拍就反向修正
           （≈主动反接刹车），比"断电自由滑行"刹得快、落点重复；真停稳后底盘自己清零断电。
       ★现场核对：老注释里"滑 6~20cm 才停住"的现象应该消失；若落点整体偏前/偏后，
         直接改下面几条 ROBOT_Move 的固定位移量即可（不要再加反向冲/改裕量那类补丁）。 */
    const uint8_t  LINE_DBG_LOG  = 1;    /* ★诊断日志：1=开 0=关（把停车调好后改0） */
    /* 逼近速度直接用 SPD_AVG_V（见文件上方速度档）；原来这里有个 LINE_CRUISE_V，已被取代、删掉 */
    const uint32_t LINE_MAX_MS   = 6000; /* ★兜底：逼近最多 6s（正常 ≤2s），到点还没白线就停下报错 */
    uint8_t  line_g3_last = 0xFF;        /* 上一次打印过的 GRAY3 八位数字量 */
    uint32_t line_dbg_t   = 0;           /* 速度日志节流（每150ms一行） */
    uint32_t line_t0      = HAL_GetTick();
    float    line_v_hit   = 0.0f;        /* 触发瞬间前向速度(正=还在往前) */
    float    line_c_th    = 0.0f;        /* 触发时车头朝向的 cos/sin：把全局量投影到"车前方" */
    float    line_s_th    = 0.0f;
    float    line_x0      = 0.0f;        /* 触发瞬间的全局位置：用来算落点位移(正=还往前) */
    float    line_y0      = 0.0f;

    GRAY_Update();
    ROBOT_MoveSpeed(0, SPD_AVG_V);  /* ★匀速靠近：SPD_AVG_V */
    while(GRAY_Data[GRAY3][0] == 0)//探头1为0(黑)继续走，读到1(白)即停
    {
      GRAY_Update();
      if((HAL_GetTick() - line_t0) > LINE_MAX_MS){    /* ★兜底：一直没白线也别无限走 */
        if(LINE_DBG_LOG)
          UART1_Printf("LINE timeout %lums, g3=0x%02X -> check sensor!\r\n",
                       (unsigned long)(HAL_GetTick() - line_t0), (unsigned)GRAY_Data[GRAY3][0]);
        break;
      }
      if(LINE_DBG_LOG){
        uint32_t now = HAL_GetTick();
        uint8_t  g3  = 0;
        for(uint8_t i = 0; i < 8; i++) g3 |= (uint8_t)((GRAY_Data[GRAY3][i] & 0x01) << i);
        /* ① 每150ms一行：逼近时的实测速度 + 4轮PWM。正常应看到 tgt=10、实测 v≈10（新底盘下命令多少就是多少），
              PWM 稳定在能维持 10cm/s 的小值区间；若 tgt=0/pwm 全 0 → 车没动，先查底盘。
              ★若一开始 0.3s 左右不动：那是速度环积分在爬升（10cm/s 的第一拍输出只有 kp×10≈55，低于起转PWM），
                想让它立刻就走就把 LINE_CRUISE_V 提到 20（队友给的短距参考 20/30）。 */
        if((now - line_dbg_t) >= 150){
          line_dbg_t = now;
          float v_now = sqrtf(chassis.now_v_x*chassis.now_v_x + chassis.now_v_y*chassis.now_v_y);
          UART1_Printf("CRUISE v=%.1f tgt=%.0f pwm=%d/%d/%d/%d g3=0x%02X\r\n",
                       (double)v_now, (double)chassis.speed_pid[CHASSIS_MOTOR_LF].target,
                       (int)chassis.speed_pid[CHASSIS_MOTOR_LF].out,
                       (int)chassis.speed_pid[CHASSIS_MOTOR_LB].out,
                       (int)chassis.speed_pid[CHASSIS_MOTOR_RB].out,
                       (int)chassis.speed_pid[CHASSIS_MOTOR_RF].out,
                       (unsigned)g3);
        }
        /* ② 灰度 8 位数字量变化（压过白线时该看到某一位变1；p1/bit0 就是 while 里用的探头1）：
             · 一路不打变化行 / g3 恒 0x00 → 模块没识别出白线（竖着装后高度/角度、阈值、环境光）
             · g3 恒 0xFF → DAT 线松/模块没供电（输入上拉全读1）
             · g3 有 1 但不落在 p1 → 探头编号与竖装后的朝向不符，换 while 里的下标即可 */
        if(g3 != line_g3_last){
          line_g3_last = g3;
          UART1_Printf("LINE g3=0x%02X p1=%u\r\n", (unsigned)g3, (unsigned)GRAY_Data[GRAY3][0]);
        }
      }
    }


    /* ★★ 停车 = ROBOT_MoveSpeed(0, 0)（2026-09-20 新底盘；原"柔性反向刹车"补丁已删）★★
       为什么可以这么简单：新底盘的速度环是闭环的 —— target=0 而四轮还在转时，增量式速度环
       当拍就把 PWM 反向修正（≈主动反接刹车），比"断电自由滑行"刹得快、且每次条件一致 → 落点重复；
       真停稳后由底盘的"静止零漂保护"(target==0 且本周期没脉冲) 清零断电，不会残留通电发热。
       所以原来那套"LINE_BRAKE_V/LINE_BRAKE_MARGIN/LINE_BRAKE_MS ＋ 临时改 chassis.start_margin
       ＋ LINE_TRIM_CM 落点微调 ＋ 临时改 chassis.brake_decel"的补丁全部删除（它们是给老底盘
       "低速必须靠破静摩擦整形才动 + 停车=断电自由滑行"准备的）。 */
    {
      /* 本段车头朝向固定(锁向保持)，把"全局速度/位移"投影到车头前方才是真正的前向量：
         body_forward = -global_x*sin(θ) + global_y*cos(θ)，θ = chassis.now_the
         （直接打全局 pos_y 是错的：本段车头朝 270/90°，前进方向对应全局 x） */
      line_c_th  = cosf(chassis.now_the);
      line_s_th  = sinf(chassis.now_the);
      line_v_hit = -chassis.now_v_x*line_s_th + chassis.now_v_y*line_c_th;  /* 触发瞬间前向速度 */
      line_x0    = chassis.pos_x;
      line_y0    = chassis.pos_y;
      ROBOT_MoveSpeed(0, 0);                     /* ★停车：速度环 target=0 → 主动反接刹车 */
    }

    //再次校准（★ROBOT_Angle 现在阻塞到"航向到位 且 四轮停稳"才返回，后面不用再补延时）
    mode_red ? ROBOT_Angle(270) : ROBOT_Angle(90);

    /* ★落点测量（STOP）：把"触发 → 停车 → 原地转正校准"整段的车头前向位移打出来，
       用来核对拍球落点（原地转正那一下麦轮会蹭掉一点位移，属正常）。
       ★落点若整体偏前/偏后：直接改下面几条 ROBOT_Move 的固定位移量补回来（别再加反向冲补丁）。 */
    if(LINE_DBG_LOG)
      UART1_Printf("STOP v_hit=%.1f dfwd=%+.2fcm dxy=%+.2f/%+.2f total=%lums\r\n",
                   (double)line_v_hit,
                   (double)(-((chassis.pos_x - line_x0)*line_s_th) + ((chassis.pos_y - line_y0)*line_c_th)),
                   (double)(chassis.pos_x - line_x0), (double)(chassis.pos_y - line_y0),
                   (unsigned long)(HAL_GetTick() - line_t0));

    //往后走一点点（好像不用往后退）（往前会拍不到球）（★短距 2cm：20/30）
    ROBOT_Move(0, mode_red ? -6 : -6, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
    //HAL_Delay(100);

    //往后退到可以拍球，要快（不需要后退了，直接灰度校准更准确）
    //ROBOT_Move(0, mode_red ? -11 : -14, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);//20太多，12擦球

    //走到位后，放到识别状态
    runActionGroup(4, 1);
    delay_ms(1400);
    UART2_Printf("%c", 0xA1);//发0xA1告诉主视觉进入圆盘机识别

    /******************** 圆盘机视觉处理代码 ********************/
    /* 与视觉约定的数据包格式：包头0xA1 | 第一个数据(0x00/0x01/0x02) | 第二个数据x坐标(2字节) | 第三个数据y坐标(2字节) | 包尾0x0B
        触发条件：第二个字节==0x01或0x02，且第三个数(x,2字节)==100~200，且第四个数(y,2字节)==80~160 → 触发动作组1
        结束条件：收到单字节指令0xA6，则退出本while循环 
        ★新增：本机自己给"拍球"计数，每真拍一个球 +1，累计到 YPJ_HIT_MAX(10) 个后，
          不论视觉是否发来"需要夹"的信息(0xA6)，都强制退出本while循环（见循环末尾强制退出段）
        ★★再新增：连续 YPJ_NO_HIT_TIMEOUT_MS(9s) 一次都没触发过拍球（视觉不发/球流断了/没对位），
          也走同一段强制退出，防止一台球都拍不到时一直卡在圆盘机（细节见该常量处的注释）
        视觉屏幕320*240*/
    /* ==================== 圆盘机拍球防连拍（重点！） ====================
       ★现象解释（先看这里再调参）：
       1) "一个球拍两次" = 视觉按 0.02s 连续上报，同一个球在画面里占好几帧，
          旧代码每帧都 runActionGroup，所以一球被重复拍；
       2) "上一球要拍拍了下一个" = runActionGroup 只是给舵机控制板发一条启动指令
          就立刻返回(约7ms)，动作组本身要在控制板上跑几百ms；上一拍没跑完，
          下一帧(已是下一个球)又触发一次，动作就落到了下一个球上。
       所以别再去纠结视觉发帧间隔是0.02还是0.03：核心是在MCU侧
       "同一球只认第一帧 + 一次动作没执行完不再触发"。下面用冷却时间实现。
       ★YPJ_HIT_COOLDOWN_MS 调法：设成 ≥ 动作组7/10从开始到复位可再拍的实测耗时。
         调大→更不易连拍，但若相邻两球间隔比它还短，后一个球会漏拍；
         调小→能跟上连着的球，但小于动作实际耗时又会连拍。
         用串口1日志里相邻两次"HIT"行的时间戳就能标定真实动作耗时。
       ★YPJ_HIT_LOG：1=串口1打印 HIT/SKIP 时序（先在电脑上分析帧间隔和连拍），调好后改0。 

       ★8秒一圈，12个球，两个球之间间隔2/3秒，7和10动作组都是400ms

       ★★新增"拍够10个强制退出"（防视觉一直不发/漏发0xA6导致卡在圆盘机）：
         ypj_hit_cnt 在每次真正 runActionGroup(7/10) 之后 +1，累计到 YPJ_HIT_MAX(10) 时，
         不再等视觉"需要夹"的信息，直接抬臂(151)+延时+收臂(0)并 break 出本while
         （见while末尾"强制退出"段，收尾动作与收到0xA6时完全一致）。
         · 想改拍几个球，只改 YPJ_HIT_MAX 一个常量即可；
         · ★冷却值决定"计数准不准"：计数是按"拍"来的，而"拍"的节拍由 YPJ_HIT_COOLDOWN_MS 决定。
           冷却 < 一个球在画面里的停留时长 → 同一个球被拍2次 → 数到10时其实只过了5个球
           （比赛实测踩过这个坑），所以必须 动作耗时400ms < 冷却 < 球间隔667ms，当前取650。
         · 验证方法（串口1日志，YPJ_HIT_LOG=1）：
           HIT 行的 c= 是两次"拍"的间隔，应≈667ms（若≈650说明只是节拍在拍、球还没换）；
           RUN 行的 pres= 是一个球在画面里停留多久、gap= 两球之间空多久，可用来反推冷却该怎么取；
           EXIT 行的 t= 是从开始拍球到退出的总耗时，10个球应≈6.7s（若只有3.4s说明又是同一球拍两次）。
         · 若希望第一拍就计数：把下面 ypj_first_skipped"第一个球只跳过不拍"那段去掉即可
           （当前第一个球不拍也不计数，所以 ypj_hit_cnt=10 就是机械臂实打实拍了10次）。
       */

    /* 实测：圆盘8s/圈、12球 → 相邻两球间隔≈667ms；动作组7/10执行400ms。
       冷却窗口须满足 400ms < 冷却 < 球间隔667ms（>动作耗时保证上一拍跑完不连拍，
       <球间隔保证不漏下一个球）。
       ★★ 冷却窗口＝一次"拍"到下一次"拍"的最短间隔，同时也是本机"拍球计数"的节拍 ★★
       ★为什么从 300 改成 650（比赛实测：记够10个却只拍了5个球就走了）：
         300ms 比"一个球在画面里停留的时间"还短 → 同一个球被拍了2次，
         10次计数其实只过了约5个球（667/300≈2.2拍/球）。
         650≈球间隔667：同一个球不会被拍两次，球流不断时基本"一球一拍"，
         于是"拍球计数10"≈"拍了10个球"。 */
    const uint32_t YPJ_HIT_COOLDOWN_MS = 650;   /* 冷却窗口：400 < 值 < 球间隔667 */
    const uint16_t YPJ_RUN_GAP_MS = 100;       /* ★标定用：相邻两帧"有球"的间隔≥100ms 就认为上一个球已离开画面 */
    const uint8_t  YPJ_HIT_LOG = 1;            /* 1=开日志 0=关 */
    const uint8_t  YPJ_HIT_MAX = 10;           /* ★拍球次数上限：实拍够10个球就不再等视觉，强制退出圆盘机阶段 */
    const uint32_t YPJ_HIT_EXIT_WAIT_MS = 450; /* ★第10拍之后等动作组跑完(约400ms)再抬臂退出，避免把最后一拍打断 */
    /* ★★ 新增"9秒没有触发拍球就强行结束圆盘机" ★★
       为什么要它：视觉掉线/球流断了/车没对好位时，一台球都没得拍，以前会一直卡在
       本while里，后面去仓库倒球、正面识别、阶梯全都跑不到，等于整场比赛报废。
       为什么取9秒：圆盘 8s 一圈、12个球 → 只要视觉正常，最多8s 转一圈之内必然有球
       经过可拍位置（球间隔667ms），所以"连续9s一次都触发不了拍球"就一定不是球还没转
       过来，而是异常了，此时强行结束圆盘机、把比赛流程往下走。
       计时基准 ypj_no_hit_ref_t：进入拍球阶段的时刻，之后每次触发"拍"（包括识别到的
       第一个只跳过不拍的球）都刷新一次，所以判的是"连续9s无触发"，不是"总耗时9s"。
       想改时长只改这一个常量。 */
    const uint32_t YPJ_NO_HIT_TIMEOUT_MS = 9000;/* ★连续9s没有触发过拍球 → 强行结束圆盘机 */
    uint32_t ypj_t_start      = HAL_GetTick(); /* ★进入拍球阶段的时刻，退出时打印总耗时(10个球约6.7s) */
    uint32_t ypj_last_hit_t   = 0;             /* 上一次真正触发"拍"的时刻(ms) */
    uint32_t ypj_last_frame_t = 0;             /* 上一次收到"有球"帧的时刻(ms)，日志看发帧间隔 */
    uint32_t ypj_run_t0       = 0;             /* ★本轮"有球"帧的第一个帧时刻（算球在画面停留时长） */
    uint32_t ypj_run_end_t    = 0;             /* ★本轮"有球"帧的最后一个帧时刻 */
    uint32_t ypj_no_hit_ref_t = ypj_t_start;   /* ★9s无触发超时的基准时刻：进入阶段时=ypj_t_start，之后每触发一拍就刷新 */
    uint8_t  ypj_first_skipped = 0;            /* 0=还没跳过第一个球；1=第一个球已跳过，之后正常拍 */
    uint8_t  ypj_hit_cnt    = 0;               /* ★实拍计数：每执行一次动作组7/10就+1（冷却期忽略的帧、被跳过的第一个球都不计） */


    while(YuanPanJi_Flag == 1){
      if(VISION1_RxFlag){                              // 2号串口（主视觉）DMA收到一帧
        VISION1_RxFlag = 0;                            // 必须立即清零

        /* 退出条件1：收到单字节指令0xA6（视觉确认），则退出本while循环
           退出条件2：★本机拍球计数 ypj_hit_cnt 达到 YPJ_HIT_MAX(10) 个，见while末尾"强制退出"段 */
        if(VISION1_RxRealLength == 1 && VISION1_RxBuf[0] == 0xA6){
          YuanPanJi_Flag = 0;//只是圆盘机结束，不代表阶梯开始，还要倒球
          //单独的机械臂抬起
          runActionGroup(151, 1);
          HAL_Delay(1000);
      
          runActionGroup(0, 1);  // 收起机械臂
          //不需要延时，和跑图一起
          //HAL_Delay(3000);
          break;
        }

        memset(CAM_Data, 0, sizeof(CAM_Data));         // 清空上一帧数据
        if(VISION1_RxRealLength <= sizeof(CAM_Data)){
          memcpy(CAM_Data, VISION1_RxBuf, VISION1_RxRealLength);
        }
        if(CAM_Data[0] == 0xA1                                              // 包头
            && (CAM_Data[1] == 0x01 || CAM_Data[1] == 0x02)                  // 有球：0x01本色 / 0x02黄
            && CAM_Data[6] == 0x0B){                                         // 包尾
          uint16_t cam_x = (uint16_t)CAM_Data[2] | ((uint16_t)CAM_Data[3] << 8); // x坐标（第三个数，2字节）
          uint16_t cam_y = (uint16_t)CAM_Data[4] | ((uint16_t)CAM_Data[5] << 8); // y坐标（第四个数，2字节）
          uint32_t now    = HAL_GetTick();          // 当前时刻(ms)
          uint32_t d_last = now - ypj_last_frame_t; // 距上一帧"有球"帧的间隔(ms)，日志用于看视觉真实发帧间隔
          /* ★标定日志（用来确认 YPJ_HIT_COOLDOWN_MS 取值是否合适）：
             d_last ≥ 100ms → 上一个球已离开画面，这时打印上一轮球的"停留时长(pres)"和"空窗(gap)"。
             · 若日志里 pres 明显大于冷却值 → 就是本次"10次只拍5个球"的原因，需要把冷却调大（现在已取650）；
             · 若日志里几乎没有 RUN 行 → 说明视觉一直连着报"有球"（画面里同时有2个以上球），
               那就只能按球间隔(667ms)的节拍来计球，此时日志里的 c=（两次拍之间的间隔）应≈667ms。 */
          if(d_last >= YPJ_RUN_GAP_MS){
            if(YPJ_HIT_LOG && ypj_run_end_t != 0)
              UART1_Printf("RUN  gap=%lu pres=%lu\r\n",
                           (unsigned long)d_last,
                           (unsigned long)(ypj_run_end_t - ypj_run_t0));
            ypj_run_t0 = now;
          }
          ypj_run_end_t    = now;
          ypj_last_frame_t = now;

          //右半边的球才拍，如果在左边那边触发拍球容易拍到下一个
          if((cam_x >= 100 && cam_x <= 320)      // x为10~300，直接不限制，只要在屏幕内就拍
              && (cam_y >= 0 && cam_y <= 240)){  // y为10~240
            if((now - ypj_last_hit_t) < YPJ_HIT_COOLDOWN_MS){
              /* ===== 冷却期：还是同一个球 / 上一拍动作还没执行完 → 忽略（防连拍关键）===== */
              if(YPJ_HIT_LOG)
                UART1_Printf("SKIP b=%u x=%u y=%u d=%lu\r\n",
                             (unsigned)CAM_Data[1], (unsigned)cam_x, (unsigned)cam_y,
                             (unsigned long)d_last);
            }else{
              /* ===== 冷却已过：这一帧当作"新球"，真正拍一次 ===== */
              uint32_t c_last = now - ypj_last_hit_t;   // ★距上一次"拍"的间隔：正常应≈球间隔667ms；若≈冷却值说明只是节拍在拍、不是新球
              ypj_last_hit_t = now;
              /* ★9s无触发超时的基准时刻在这里刷新：
                 不管是下面"第一个球只跳过不拍"还是真发了动作组，都算"触发过拍球"，
                 所以只要球流不断就永远不会走到9s强制退出；球流一断，9s后自动收摊。 */
              ypj_no_hit_ref_t = now;
              if(ypj_first_skipped == 0){      /* 识别到的第一个球：只跳过、不拍，从第二个球开始正常拍 */
                ypj_first_skipped = 1;
                if(YPJ_HIT_LOG)
                  UART1_Printf("SKIP1 b=%u x=%u y=%u\r\n",
                               (unsigned)CAM_Data[1], (unsigned)cam_x, (unsigned)cam_y);
              }else{
              if(CAM_Data[1] == 0x01){
                runActionGroup(7, 1);    // 拍本色球，包括分流板
              }else{
                runActionGroup(10, 1);   // 拍黄球，包括分流板
              }
              /* ★★ 每拍一次球就记1 ★★
                 计数放在 runActionGroup 之后：只有真正把这拍发给舵机控制板才 +1，
                 冷却期被忽略的帧、以及第一个只跳过不拍的球都不会记进来 */
              ypj_hit_cnt++;
              if(YPJ_HIT_LOG)
                UART1_Printf("HIT  b=%u x=%u y=%u d=%lu c=%lu n=%u/%u\r\n",
                             (unsigned)CAM_Data[1], (unsigned)cam_x, (unsigned)cam_y,
                             (unsigned long)d_last, (unsigned long)c_last,
                             (unsigned)ypj_hit_cnt, (unsigned)YPJ_HIT_MAX);
              }
            }
          }
        }
      }

      /* ★★ 强制退出圆盘机（两个条件，收尾动作完全一致）★★
         视觉那边可能一直不发（或晚发、漏发）"需要夹"的0xA6，这里由本机自己兜底：
         条件1：拍够 YPJ_HIT_MAX(10) 个球 —— ypj_hit_cnt 每真拍一球 +1，数到10就不再等0xA6；
         条件2：★连续 YPJ_NO_HIT_TIMEOUT_MS(9s) 一次都没触发过拍球（视觉掉线/球流断了/
                车没对上，一台球都没得拍）—— 由 ypj_no_hit_ref_t 计时，见上面常量说明。
         两个条件走同一套收尾：等最后一拍动作组跑完 → 抬臂(151) → 延时 → 收臂(0) → break 出 while。
         退出后 YuanPanJi_Flag=0，下面"去仓库倒球"那一段照常执行，流程不断。 */
      uint8_t ypj_force_exit = 0;        // 1=本次循环要强制退出圆盘机
      if(ypj_hit_cnt >= YPJ_HIT_MAX){
        if(YPJ_HIT_LOG)
          UART1_Printf("EXIT by hit count %u/%u t=%lu\r\n",
                       (unsigned)ypj_hit_cnt, (unsigned)YPJ_HIT_MAX,
                       (unsigned long)(HAL_GetTick() - ypj_t_start));

        HAL_Delay(YPJ_HIT_EXIT_WAIT_MS);  // 等第10次拍球动作组跑完(约400ms)，否则会被下面的抬臂指令打断
        ypj_force_exit = 1;
      }else if((HAL_GetTick() - ypj_no_hit_ref_t) >= YPJ_NO_HIT_TIMEOUT_MS){
        /* ★连续9s没触发过拍球 → 强行结束圆盘机
           （这里没有拍球动作组在跑，不用等 YPJ_HIT_EXIT_WAIT_MS，直接收摊） */
        if(YPJ_HIT_LOG)
          UART1_Printf("EXIT by no-hit timeout %lums n=%u/%u t=%lu\r\n",
                       (unsigned long)(HAL_GetTick() - ypj_no_hit_ref_t),
                       (unsigned)ypj_hit_cnt, (unsigned)YPJ_HIT_MAX,
                       (unsigned long)(HAL_GetTick() - ypj_t_start));
        ypj_force_exit = 1;
      }

      if(ypj_force_exit){
        YuanPanJi_Flag = 0;//只是圆盘机结束，不代表阶梯开始，还要倒球
        //单独的机械臂抬起
        runActionGroup(151, 1);
        HAL_Delay(1000);

        runActionGroup(0, 1);  // 收起机械臂
        break;                 // 跳出 while(YuanPanJi_Flag == 1)
      }
    }

    /***************去仓库倒球*************/

    if(YuanPanJi_Flag == 0)//圆盘机结束时
    {

      //退后固定距离（★短距 25cm：20/30）
      ROBOT_Move(0,-20,SPD_SHORT_V,SPD_SHORT_V,SPD_SHORT_A,SPD_SHORT_A);
      //收起机械臂
      //重复了runActionGroup(0, 1);
      //向左平行到仓库（★长距 185/195cm：120/120）
      ROBOT_Move(mode_red ? -190 : 195,0,SPD_SHORT_V,SPD_SHORT_V,SPD_SHORT_A,SPD_SHORT_A);//蓝要多走一点
      //转身（ROBOT_Move 已阻塞到车停稳，不用再补 HAL_Delay(100)）
      mode_red ? ROBOT_Angle(90) : ROBOT_Angle(270);
      
      //往后慢退，直到测距测得合适距离（适合倒球的距离）
      ROBOT_MoveSpeed(0, -SPD_AVG_V);   //慢匀速
      WAIT_WHILE(GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin)>85,
                 "YuanPanJi back-to-wall(mm<=85)");//100有点远，距离小于90就退此循环，90也远
      SENSOR_STOP();      /* ★停车：速度环 target=0 → 主动反接刹车（不再自由滑行 6~20cm，
                             也不再需要"反向冲一下"或补固定位移那套补丁）
                             ★倒球距离若不对：把上面的 85 调大（如 95），或在这里补一条固定位移 */

      //加一次角度校准（这些地方的角度很重要；ROBOT_Angle 已阻塞到停稳，不必再补延时）
      mode_red ? ROBOT_Angle(90) : ROBOT_Angle(270);

      //定位操作：向左慢平移到左后光电感应到无障碍物
      //新：更改激光位置，让它在没对到障碍物时直接就已经是合适的位置，不需要调整
      ROBOT_MoveSpeed(-5, 0);   //★匀速靠近：SPD_AVG_V
      while(LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin));
      // WAIT_WHILE(LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin)==1, "YuanPanJi laser1 no-obstacle");

      SENSOR_STOP();      /* ★停车：速度环 target=0 主动反接刹车（不再自由滑行/不再反向冲）
                             ★若左边还差一点：在这里补一条固定左移
                               ROBOT_Move(-X, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A); */

      //加一次角度校准（ROBOT_Angle 已阻塞到停稳，不必再补延时）
      // mode_red ? ROBOT_Angle(90) : ROBOT_Angle(270);
      //往右走固定距离（刚到对上仓库的距离）
      if(mode_red) ROBOT_Move(5, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);//20太大，速度100会飘，12太远

      flag.angle = 0;
      runActionGroup(16, 1); 	//这里是倒球动作组
      delay_ms(2000);
      flag.angle = 1;
      //加一次角度校准（这些地方的角度很重要；ROBOT_Angle 已阻塞到停稳，不必再补延时）
      mode_red ? ROBOT_Angle(90) : ROBOT_Angle(270);

ZHENGMIAN_START:            //★调试入口(KEY0选成ZM+KEY3开始)：goto 跳到这一行往下执行 → 正面识别前
      ZhengMian_Flag = 1;//正面识别开始

      if(ZhengMian_Flag == 1){
        
        /*正面识别的通信流程：
          1. MCU分别发0xA2给主视觉和副视觉，告诉它们进入正面识别
          2. 副视觉(串口4)发来的单字节为0xAB/0xAC/0xAD/0xBC/0xBD/0xCD这6种之一时，
             就存下来并原样转发给主视觉(串口2)；不是这6种就不存；
          3. 存下来后，MCU发0xA7给副视觉，告诉它我收到了，副视觉可以结束识别了
          4. 主视觉收到MCU发送的，副视觉传来的原样信息后，发0xA7给MCU，告诉它正面识别结束了
          5. MCU收到主视觉发来的单字节0xA7，则退出本while循环，正面识别结束
        */

        /* ===== 正面识别 第1段：先复位显示，再"收倒球槽 + 走到阶梯对准位"；到位后才发0xA2 =====
           ★为什么到位才发0xA2：0xA2是两个视觉的"开始识别"标志。早发的话副视觉可能在车还没到位时
             就把结果发出来，甚至被后面的"清残留"清掉；到位后再发，这段时序就完全不用纠结。 */
        comm_rx4 = 0; comm_rx2 = 0; comm_t_oled = 0;               //本阶段收发计数清零
        zm_step = ZM_ST_PREP;                       //进度0：还没发0xA2(正在移动到识别位)
        zm_v4_res = ZM_RES_WAIT; zm_v4_byte = 0; zm_v4_len = 0;
        zm_v2_res = ZM_RES_WAIT; zm_v2_byte = 0;
        ZhengMian_Letter[0] = 0; ZhengMian_Letter[1] = 0;    //上一轮残留的字母清掉
        zm_t0 = HAL_GetTick();                      //本阶段起始时刻(第4行"秒数"用)
        ZM_ShowComm(comm_rx4, comm_rx2);            //先画一屏：进度0(移动中)，V4/V2 都是 WAIT

        runActionGroup(19, 1); 	//这里是收倒球槽
        delay_ms(2000);

        //右+前，移动到阶梯附近,要往右多走点，不然撞到了，y160太远了，不利于视觉识别（★长距(100,150)：120/120）
        //要拆开两段走了，否则会撞到，先走x后走y
        ROBOT_Move(70, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
        ROBOT_Move(0, 128, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

        //向左慢走，直到前面的两个光电都感应到障碍物（右边坏了，左边拆了，复用立柱的）4左2右
        ROBOT_MoveSpeed(-SPD_AVG_V, 0);   //★匀速靠近：SPD_AVG_V
        //while(LASER_Barrier(LASER2_GPIO_Port, LASER2_Pin)==0);
        while(LASER_Barrier(LASER4_GPIO_Port, LASER4_Pin)==0);//左后是1，右边为2，左前是3
        ROBOT_MoveSpeed(0, 0);      //★停车（激光触发后立即给 0 速，速度环反接刹车）

        //往左走一定距离，视觉中能完整看到两个字母（可以省去测距前后校准）（★短距 40cm：20/30）
        //短距离太快速，走的斜斜的，不要100速度，50还算可以
        ROBOT_Move(-40, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
        //停车准备识别
        ROBOT_MoveSpeed(0,0);

        //清掉主/副视觉串口的残留接收数据，避免把旧数据误当成有效信息
        UART2_RxFlag = 0;
        UART2_RxRealLength = 0;
        memset(UART2_RxBuf, 0, UART2_RxLength);
        UART4_RxFlag = 0;
        UART4_RxRealLength = 0;
        memset(UART4_RxBuf, 0, UART4_RxLength);   //UART4_RxBuf是extern数组，不能用sizeof

        /* ★把两个视觉串口的 DMA 接收强制重启一遍：万一之前因为出错(ORE/FE)停了接收，
           这里复位+重启，保证进循环时两个口都是真的"在听"状态 */
        HAL_UART_AbortReceive(&huart2);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RxBuf, UART2_RxLength);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
        HAL_UART_AbortReceive(&huart4);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_RxBuf, UART4_RxLength);
        __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);

        /* ===== 车已到位、串口残留已清、DMA已重启 → 现在才发0xA2开始识别(进度1)，然后进通信循环 ===== */
        UART2_Printf("%c", 0xA2);//发0xA2告诉主视觉进入正面识别
        UART4_Printf("%c", 0xA2);//发0xA2告诉副视觉进入正面识别
        UART1_Printf("TX 0xA2 -> V1(UART2) + V4(UART4)\r\n");      //日志：0xA2发出的时刻(和后面的RX日志对时间)
        zm_step = ZM_ST_SEND;                       //进度1完成：0xA2已发
        comm_t_oled = HAL_GetTick();                //显示节流重新计时
        ZM_ShowComm(comm_rx4, comm_rx2);            //第2行等副视觉字母，第3行等主视觉0xA7

        //流程2~5都在while里跑，收到主视觉发来的0xA7才退出
        while(ZhengMian_Flag == 1){ 
          /* ============ 流程2/3/5：副视觉六种字节处理 + 主视觉0xA7退出，见下方 ============ */

          /* 串口4收到一帧后的处理：副视觉 */
          if(UART4_RxFlag){                               //串口4（副视觉）DMA收到一帧
            UART4_RxFlag = 0;                             //必须立即清零
            comm_rx4++;                                   //监视：串口4(副视觉)收到帧数+1
            //副视觉发来的原始内容(整帧)打印到电脑串口1；屏幕上只显示状态与结果(见 ZM_ShowComm)
            UART1_Printf("RX4 len=%d:", UART4_RxRealLength);
            for(uint8_t k = 0; k < UART4_RxRealLength && k < 8; k++) UART1_Printf(" %02X", UART4_RxBuf[k]);
            UART1_Printf("\r\n");

            if(UART4_RxRealLength == 1){                  //这一帧是单字节
              uint8_t vision4_byte = UART4_RxBuf[0];      //取出这一帧的数据

              /* 流程2：副视觉发来的6种有效结果字节 0xAB/0xAC/0xAD/0xBC/0xBD/0xCD 之一 */
              if(vision4_byte == 0xAB || vision4_byte == 0xAC || vision4_byte == 0xAD ||
                 vision4_byte == 0xBC || vision4_byte == 0xBD || vision4_byte == 0xCD){
                //存下来：高4位=第一个字母，低4位=第二个字母（存到 ZhengMian_Letter，供后面阶梯阶段使用）
                ZhengMian_Letter[0] = (vision4_byte & 0xF0) >> 4;
                ZhengMian_Letter[1] = (vision4_byte & 0x0F);
                //原样发送给主视觉（串口2）
                UART2_Printf("%c", vision4_byte);
                //流程3：发0xA7给副视觉，告诉它我收到了，副视觉可以结束识别了
                UART4_Printf("%c", 0xA7);
                UART1_Printf("TX relay 0x%02X -> V1(UART2), TX 0xA7 -> V4(UART4)\r\n", vision4_byte);

                /* ===== 收到6种之一 → 第2行显示字母(A~D)+原始字节；拿到一次就锁死(之后无效字节不再覆盖) =====
                   ★字母已由上面两行的 ZhengMian_Letter[0]/[1] 存好(高4位/低4位)，显示函数直接取用 */
                zm_v4_res  = ZM_RES_OK;
                zm_v4_len  = 1;
                zm_v4_byte = vision4_byte;
                zm_step = ZM_ST_LETTER;                 //进度：第2步完成
                ZM_ShowComm(comm_rx4, comm_rx2);
              }else{
                /* 不是这6种的其他单字节：不存、忽略
                   ★只在"还没拿到有效结果"时才显示在第2行(带 BAD 标记)，拿到结果后不再改它 */
                if(zm_v4_res != ZM_RES_OK){
                  zm_v4_res = ZM_RES_BAD; zm_v4_len = 1; zm_v4_byte = vision4_byte;
                  ZM_ShowComm(comm_rx4, comm_rx2);
                }
              }
            }else{
              /* 帧长不是1(副视觉发了别的包)：不存、忽略；同样只在还没拿到有效结果时才显示长度 */
              if(zm_v4_res != ZM_RES_OK){
                zm_v4_res = ZM_RES_BAD; zm_v4_len = UART4_RxRealLength;
                ZM_ShowComm(comm_rx4, comm_rx2);
              }
            }
          }

          /* 流程5：主视觉(串口2)收到MCU转发的原样信息后，发0xA7给MCU → 收到则退出while */
          if(UART2_RxFlag){                               //串口2（主视觉）DMA收到一帧
            UART2_RxFlag = 0;                             //必须立即清零
            comm_rx2++;                                   //监视：串口2(主视觉)收到帧数+1
            //主视觉发来的原始内容也打印一份：和上面的RX4日志连起来看，就能判断"两个串口是不是收到同一个字节"
            UART1_Printf("RX2 len=%d got=0x%02X\r\n", UART2_RxRealLength, UART2_RxBuf[0]);
            if(UART2_RxRealLength == 1 && UART2_RxBuf[0] == 0xA7){
              /* ===== 收到主视觉0xA7 → 第3行显示 OK，正面识别结束，进入阶梯 =====
                 ★阶梯阶段屏幕上会保留"正面识别拿到的两个字母"，方便对照 */
              zm_v2_res  = ZM_RES_OK;
              zm_v2_byte = 0xA7;
              zm_step    = ZM_ST_RECV;                    //进度：3步全部完成
              ZM_ShowComm(comm_rx4, comm_rx2);

              ZhengMian_Flag = 0;                         //主视觉确认，正面识别结束
              JieTi_Flag = 1;                             //阶梯开始
              HAL_Delay(1000); //延时1s，不然正面识别切换到下一个状态太快了
              break;
            }
            /* 主视觉发来的其它帧：不算成功，只在"还没拿到0xA7"时把最新字节显示在第3行(带 BAD) */
            if(zm_v2_res != ZM_RES_OK){
              zm_v2_res  = ZM_RES_BAD;
              zm_v2_byte = (UART2_RxRealLength > 0) ? UART2_RxBuf[0] : 0;
              ZM_ShowComm(comm_rx4, comm_rx2);
            }
          }

          /* ===== 每100ms：自愈两个视觉串口的接收(出错/DMA停了就重启) + 定时刷屏(计数/串口状态/秒数) =====
             ★屏幕上四行的内容统一由 ZM_ShowComm() 负责，这里只做"定时刷新"，不再直接写屏幕 */
          if(HAL_GetTick() - comm_t_oled >= 100U){
            comm_t_oled = HAL_GetTick();

            /* 自愈1：串口4(副视觉)如果没在接收(出错停了 / DMA关了)就立刻重新拉起来 */
            if((huart4.RxState != HAL_UART_STATE_BUSY_RX) || ((hdma_uart4_rx.Instance->CR & DMA_SxCR_EN) == 0U)){
              HAL_UART_AbortReceive(&huart4);
              UART4_RxFlag = 0;
              HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_RxBuf, UART4_RxLength);
              __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
            }
            /* 自愈2：串口2(主视觉)同理 */
            if((huart2.RxState != HAL_UART_STATE_BUSY_RX) || ((hdma_usart2_rx.Instance->CR & DMA_SxCR_EN) == 0U)){
              HAL_UART_AbortReceive(&huart2);
              UART2_RxFlag = 0;
              HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RxBuf, UART2_RxLength);
              __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
            }

            ZM_ShowComm(comm_rx4, comm_rx2);           //心跳刷新(第4行计数/串口状态/秒数，前三行内容不变)
          }
        }
      }

      if(JieTi_Flag == 1){//阶梯开始
        /* ==================== 阶梯阶段（模块化，序号与下面注释一一对应）====================
           M1 视觉通信 : 收帧/解析(JieTi_VisionPoll) + 清帧 + 发 0xA3 + 读每坑的 cmd
           M3 走位/计数: 每坑走固定距离(JIETI_STEP_CM / JIETI_STEP_CROSS_CM) + 计第几个坑
           M5 动作组   : 识别位(54) / 夹取(57·60·63) / 回识别位(66) / 收尾抬臂(151)
           M6 显示/收尾: OLED + 停车 + 按 JIETI_GO_LIZHU 交棒给"立柱"或"回家"
           （原 M2 左右视觉对准、M4 前后距离校准 2026-09-21 都已删 —— 实测效果不好；
             阶梯只走固定距离：位置 = 进阶梯时走近到 90mm 附近 + 每个坑走固定距离）
           ============================================================================== */

        /* ---- M6.1 屏幕初始化：字母留着，第2行标"step only"，第4行先显示 BLK -/8 ---- */
        OLED_Clear();                                 //4行一起清掉（顺带清掉正面识别留在第3行的状态字）
        OLED_Printf(0,  0, OLED_8X16_HALF, "JIETI letter %c %c",
                    ZM_Nibble2Char(ZhengMian_Letter[0]), ZM_Nibble2Char(ZhengMian_Letter[1]));
        OLED_Printf(0, 16, OLED_8X16_HALF, "step only");   //本阶段只走固定距离，不做逐坑校准
        OLED_Update();
        JieTi_ShowBlockNo(0, 8, 0);   //第4行先显示"BLK -/8"，处理到第1个坑后就变成 1/8

        /* ---- 到位①：左移找激光4（激光一离开障碍物就停）---- */
        ROBOT_MoveSpeed(-SPD_AVG_V, 0);
        /* ★超时保护：激光4一直报"有障碍物"时原来会无限往左走(顶住左边不动=卡死)，
           现在最多等 WAIT_TIMEOUT_MS，超时打 TIMEOUT 后照常停车继续走流程 */
        WAIT_WHILE(LASER_Barrier(LASER4_GPIO_Port,LASER4_Pin)==1, "JIETI laser4 no-obstacle");
        ROBOT_MoveSpeed(0, 0);

        /* ---- M3.1 朝向基准：转到"正对阶梯"的绝对角，并记住它当整段阶梯的锁向目标 ----
           正常流程：红90/蓝270（两边差180°）；跳转测阶梯：红方 shift=90 / 蓝方 shift=270，
           Yaw_Abs() 换算后都是"按摆车姿态不转"（摆车时 0° 已经就是该段的车头方向）；
           后面每次设速都用 JieTi_MoveSpeed() 把 target_yaw 恢复成这个值，角度环才会一直按它纠偏。 */
        mode_red ? ROBOT_Angle(Yaw_Abs(90)) : ROBOT_Angle(Yaw_Abs(270));
        jieti_keep_yaw = chassis.target_yaw;

        /*激光校准，往右 5cm（激光刚离开时位置偏左,步往右测距出去了）*/
        ROBOT_Move(11, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

        /* ---- 到位②：走近阶梯，前测距到 90mm 就停 ---- */
        JieTi_MoveSpeed(0, SPD_AVG_V);
        /* ★提前 20mm 停（不是读到 90 才停）：GY-53 一次读数 ≈200ms，10cm/s 逼近时"读到 ≤90mm"
           那一刻车还会再往前冲 ≈2cm；提前量补上，停稳后就正好落在 90mm 附近。
           （后面不再做任何测距校准，就按这个位置走） */
        WAIT_WHILE(GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin) > (JIETI_FWD_TARGET_MM + 20),
                   "JIETI walk-to-ladder(stop at 110mm=90+lead20)");
        JieTi_MoveSpeed(0, 0);

        /* ---- M5.1 机械臂到识别位 ---- */
        runActionGroup(54, 1);
        HAL_Delay(2000);

        /* ---- M1.4 视觉通信：清残留帧 → 发 0xA3 告诉主视觉"进阶梯阶段"（之后它会按 A3 包上报每个物块的 cmd）---- */
        JieTi_FlushVision();
        UART2_Printf("%c", 0xA3);

        /* ==================== 8 个坑循环 ====================
           每坑：走固定距离(M3) → 读这一坑"夹不夹"(M1.5) → 计数/显示(M3.3) → 按需夹取(M5.2)
           · 位置：全靠"进阶梯时的到位 + 坑间距"决定（不做任何逐坑校准）。
           · 朝向：底盘一直锁向(target_yaw = 进阶梯时校好的那个)，走位 20cm/s 期间角度环也在纠偏。
           · cmd：站定后等一帧（视觉"有目标"才会发帧）；等不到 = 视野里没目标 = 这一坑不夹。
           夹取动作组：第1~2个→57(中阶梯) / 第3~6个→60(高阶梯) / 第7~8个→63(矮阶梯)，夹完一律 66 回识别位。
           数据包(7字节)：A3 | cmd | x低 | x高 | y低 | y高 | 0x0B；cmd 含义看全局 JieTi_Grab_Mode。 */
        for(uint8_t idx = 1; idx <= 8; idx++){                  //第1~8个坑
          /* ---- M3.2 到下一个坑：走固定距离 ---- */
          if(idx > 1){
            JieTi_MoveSpeed(0, 0);
            /* 换阶梯那一步(第2→3个、第6→7个，就是动作组 57/60/63 切换处)隔得远，走 JIETI_STEP_CROSS_CM(10cm)；
               同一个阶梯里相邻坑走 JIETI_STEP_CM(8cm) */
            int32_t step_cm = (idx == 3 || idx == 7) ? JIETI_STEP_CROSS_CM : JIETI_STEP_CM;
            ROBOT_Move(step_cm, 0, JIETI_STEP_SPEED, 0, JIETI_STEP_ACC, 0);
          }

          /* ---- M1.5 读这一坑"夹不夹"：先按"不夹"，清掉走位途中的旧帧，再等一帧 cmd ----
             （视觉协议：视野里没目标就一个字节都不发 → 等不到 = 这一坑没东西/不用夹） */
          jieti_cmd = 0;
          JieTi_FlushVision();
          JieTi_GetVision(JIETI_VIS_MS);

          /* ---- M3.3 计数 + 屏幕：处理到第 idx 个坑 ---- */
          jieti_blk_now = idx;
          JieTi_ShowBlockNo(idx, 8, jieti_cam_x);

          /* ---- M1.5 要不要夹：Mode1 cmd==0x01；Mode2 cmd低4位非0 ----
             ★进来前已把 jieti_cmd 清 0（上面那 3 行），所以这一坑一帧都没收到 → 这里就是 0 → 不夹 */
          uint8_t need = (JieTi_Grab_Mode == 1) ? (jieti_cmd == 0x01)
                                                : ((jieti_cmd & 0x0F) != 0);
          if(JIETI_VIS_LOG)                          //日志：这一坑视觉给的是啥 + 最终判定(对着上面几行RX2看)
            UART1_Printf("BLK %u/8 cmd=0x%02X need=%u\r\n",
                         (unsigned)idx, (unsigned)jieti_cmd, (unsigned)need);

          /* ---- M5.2 夹取：按坑号选动作组 ---- */
          if(need){
            uint8_t act = (idx <= 2) ? 57 : ((idx <= 6) ? 60 : 63);   //57矮 / 60高 / 63中阶梯
            runActionGroup(act, 1);
            delay_ms(7500);                          //夹取动作约7.5秒(矮阶梯更慢)
            runActionGroup(66, 1);                   //夹完回到识别状态
            delay_ms(3000);                          //识别动作约3秒
            JieTi_FlushVision();                     //清掉夹取期间滞留的旧帧
          }
        }

        /* ---- M6.2 收尾：停车 + 抬臂（M6.3 接着按 JIETI_GO_LIZHU 交棒给"立柱"或"回家"）---- */
        JieTi_MoveSpeed(0, 0);
       
        //单独的机械臂抬起
        runActionGroup(151, 1);
        HAL_Delay(1000);
        
        JieTi_Flag = 0;                     //M6.3 阶梯结束
        /* ★阶梯跑完去哪儿：就按上面那个开关 JIETI_GO_LIZHU 走（改一个数就能切回来）
           1 → 先进"立柱"段（绕柱 → 仓库倒方块 → 回家；★正常流程）   0 → 跳过立柱、直接进"回家"段
           ★这里必须用"赋值"，别写成 ==（历史上写成 == 导致阶段标志全是 0、车停在阶梯不动） */
        if(JIETI_GO_LIZHU) LiZhu_Flag = 1;  //去立柱（立柱段末尾自己会置 HuiJia_Flag=1 接回家）
        else               HuiJia_Flag = 1; //直接回家

        if(LiZhu_Flag == 1)//立柱开始
        {
          
          runActionGroup(0, 1);//复位
          ROBOT_Move(30, -150, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

          //更换新跑图逻辑
          //ROBOT_Move(-45, -35, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
          //好像前面把角度清0了，此时正对阶梯为0（不对，依然是正对阶梯为90呢）
          //ROBOT_Angle(Yaw_Abs(270));   //正对阶梯/立柱(跳转测试时自动换算)
          //ROBOT_Angle(180);

    /* ★调试入口(KEY0选成 LZ + KEY3开始)：goto 跳到这一行往下执行"立柱"段。
       上面那条"走到立柱附近(-45,-35) + 转向"是给"阶梯→立柱"用的；单独测立柱时车已经按
       "立柱起点姿态"(红方车头朝左、就在立柱右边一点)摆好了，所以跳过它们，直接进下面的校准。 */
LIZHU_START:

          /* ====== 立柱前校准（2026-09-21 简化：只有两步）======
             前提(现场保证)：车在立柱右边一点。
             ① 往左慢走(5cm/s)，直到"左右两个激光都有障碍物" → 已经左右正对柱面
               左4右2
             ② 再以 5cm/s 向前逼近，直到前测距 ≤200mm → 前后到位
             然后直接 LiZhu_Circle_Run() 开始绕圈（它拿这时读到的距离当参考半径）。
             ★两处都用 5cm/s：GY-53 是阻塞读(≈200ms)、激光也要车慢慢靠过去才不会冲过头
               （10cm/s 会冲过头，和阶梯那套"走多"是同一个原因）。
             ★WAIT_WHILE 自带 6s 超时兜底：超时打一行 TIMEOUT 后继续走，不会卡死。 */
          ROBOT_MoveSpeed(-5.0f, 0.0f);                     //① 往左慢走
          //左激光看到障碍物就往往左走固定距离，左4右2
          WAIT_WHILE(!(LASER_Barrier(LASER4_GPIO_Port, LASER4_Pin)),
                     "LiZhu both lasers on pillar");
          ROBOT_MoveSpeed(0.0f, 0.0f);                      //停车
          ROBOT_Move(-4, 0, SPD_SHORT_V, 0, SPD_SHORT_A, 0);

          ROBOT_MoveSpeed(0.0f, 5.0f);                      //② 向前慢逼近到 200mm，180也太远
          WAIT_WHILE(GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin) > 170U,
                     "LiZhu front distance -> 170mm");
          ROBOT_MoveSpeed(0.0f, 0.0f);                      //停车，准备转圈

          
          //立柱转圈（2026-09-26 重写：开环三旋钮 + 测距/激光两路可选反馈，见 LiZhu_Circle_Run 函数头）
          //★前提：车头已经正对着柱子（函数开头就是静止采测距定参考距离）
          LiZhu_Circle_Run();

          /*
          识别钩子已搬进 LiZhu_Circle_Run() 的绕圈 while 里：
          遇到可以夹的就在那里 break，车停下、收尾照常执行
          */

          //转完一圈，收起机械臂，然后往左转身走到仓库中间倒方块（更换新逻辑）
          //ROBOT_Move(-60, 0, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);
          //ROBOT_Angle(Yaw_Abs(90));//车子前面朝右
          //ROBOT_Move(0, -138, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);
          //ROBOT_Move(-45, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

          HAL_Delay(1000);

          //转完一圈，纠正角度
          //ROBOT_Angle(Yaw_Abs(90));//车子前面朝右

          //先后退到合适距离
          /* ★9.25 修方向：原来是 +SPD_AVG_V(v_y>0=前进)，但注释写"后退"、判据又是"后测距(GY53_1)
             ≤85mm(贴后墙)" —— 前进只会让后测距变大、永远不满足，只能靠 6s 超时兜底往前冲 60cm。
             改成 -SPD_AVG_V：真后退，后测距一路变小，到 85mm 自然停。 */
          ROBOT_MoveSpeed(0.0f, -SPD_AVG_V);
          WAIT_WHILE(GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin)>85,
                 "LiZhu back-to-wall(mm<=85)");

          //定位操作：向左慢平移到左后光电感应到无障碍物，之后再往右走固定距离（刚到仓库中间的距离）
          ROBOT_MoveSpeed(-SPD_AVG_V, 0);   //★匀速靠近：SPD_AVG_V
          WAIT_WHILE(LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin)==1, "LiZhu laser1 no-obstacle");
          ROBOT_MoveSpeed(0,0);

          ROBOT_Move(25, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

          //得走远一点才能转身倒方块
          ROBOT_Move(0, 10, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
          ROBOT_Angle(Yaw_Abs(270));//车子前面朝左
          UART1_Printf("10");
          
          //这里放倒方块的代码
          runActionGroup(16, 1);//倒方块动作组(复用圆盘机)
          HAL_Delay(2000);//倒方块动作约2s
          runActionGroup(19, 1);//收倒球槽

          LiZhu_Flag = 0;//立柱结束，回家开始
          HuiJia_Flag = 1;
        }



    /* ★调试入口(KEY0选成HOME+KEY3开始)：goto 跳到下面 HUIJIA_START 标签往下执行"回家"
       （HuiJia_Flag 已在跳转前立好；正常流程走到这里时一字不变） */
HUIJIA_START:
    if(HuiJia_Flag == 1)//回家开始
    {

          //往前走确保转方向不卡脚（★短距 15cm：20/30）
          ROBOT_Move(0, 15, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A); 

          //倒完方块转正再回家
          ROBOT_Angle(Yaw_Abs(0));
          //往后多走一点，必须保证，前后在左右移动后能进入红色区域（★长距 230cm：120/120）
          ROBOT_Move(mode_red ? 30 : -30, -230, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);//60，-240能进

      /*先校准左右再校准前后，左右走可能会抖，而且前后比左右的反馈更准
    注意！！！必须先让颜色传感器在左右移动之后一定能进入红/蓝区域，
    即前后距离必须能确保在红/蓝区域内（在哪里无所谓，后面再校准）
    */
    //如果为黑色，匀速往右走，直到传感器进入红/蓝区域
    //去抖：连续3次(约150ms)都读到红/蓝才确认，交界处"红黑红黑"抖动不会误停
    ROBOT_MoveSpeed(5, 0);   //★匀速靠近：SPD_AVG_V,10太大了
    {
      uint8_t stable = 0;
      while(1){
        if(TCS_PollColor(&tcs_rgbc) != TCS_COLOR_BLACK){
          if(++stable >= 3) break;
        } else {
          stable = 0;
        }
        HAL_Delay(50);
      }
    }
    
    //因为加了消抖，所以会稍微多走一小点，再减少一点盲走的距离

    //进入红/蓝后，继续向右多走11.5，确保停在红/蓝区域内部（避免停在边缘抖动；距离按区域宽度调整），而且确保车身左右都在红/蓝区域内
    //★短距 11.5cm：按两档标准 20/30（<13cm 的三角波峰值<20cm/s，若现场发现走不到位，把这里的第5/6个参数(a)单独加大到 100~200）
    ROBOT_Move(mode_red ? 8 : -8, 0, 5, 5, 5, 5);
    
    //往前走，走到颜色传感器一定在黑色区域内（同样连续3次确认）
    ROBOT_MoveSpeed(0, SPD_AVG_V);   //★匀速靠近：SPD_AVG_V
    {
      uint8_t stable = 0;
      while(1){
        if(TCS_PollColor(&tcs_rgbc) == TCS_COLOR_BLACK){
          if(++stable >= 3) break;
        } else {
          stable = 0;
        }
        HAL_Delay(50);
      }
    }
    
    //颜色传感器校准前后：如果为黑色，匀速往后走，直到进入红/蓝区域（连续3次确认）
    ROBOT_MoveSpeed(0, -SPD_AVG_V);   //★匀速靠近：SPD_AVG_V
    {
      uint8_t stable = 0;
      while(1){
        if(TCS_PollColor(&tcs_rgbc) != TCS_COLOR_BLACK){
          if(++stable >= 3) break;
        } else {
          stable = 0;
        }
        HAL_Delay(50);
      }
    }
    
    //识别为红/蓝后继续向后多走2，确保停在红/蓝区域内部（避免停在边缘抖动；距离按区域宽度调整），而且确保车身前后都在红/蓝区域内
    ROBOT_Move(0, -2, 5, 5, 5, 5);
    ROBOT_MoveSpeed(0, 0);
        }
      }

    }
    /* 原来这里有一句无条件的 UART1_Data[0]=0;，它把刚解析出来的指令(6/7)提前清成 0，
       所以"发指令没反应"。已删除：每条指令在处理时自己会清零。 */

    //立柱转圈(2026-09-26 重写：开环三旋钮 + 测距/激光两路可选反馈)：正式流程在立柱阶段(LiZhu_Flag==1)里调 LiZhu_Circle_Run()；
    //  单独调试：在上方 UART1_DebugCmd() 里，串口发 "7,0,0,0,0,0,0,0" 就地跑一遍
    //  （停在"红蓝方选择"菜单界面也能用，菜单 while 里也调了 UART1_DebugCmd）：
    //      if(UART1_Data[0]==7){ UART1_Data[0]=0; LiZhu_Circle_Run(); }
    //  前提：车头已正对柱子（函数开头会静止采12次测距取中值当目标距离 d_ref，不是写死的 180mm）
    //  绕法（★2026-09-26 重写：不用增益，只有“速度”）：
    //       开环基准：切向 v_x=V_TAN + 车头摆速 w=W_TURN(°/s) → 圆半径 r=V_TAN/(W_TURN×π/180)，天然以柱子为圆心
    //       测距反馈(FB_DIST=1)：测距偏大→往前靠、偏小→往后退，满幅 3cm 误差 → v_y=V_RAD cm/s
    //       双激光反馈(FB_LASER=1)：左4右2，一个有一个没有=横向偏了 → 摆速加 LAS_YAW、切向速度加 LAS_TAN
    //       两个开关都置 0 = 纯开环（只有三个旋钮）
    //  串口(200ms/条)：d=测距mm e=径向误差mm l4/r2=左/右激光 vx=切向速度 vy=径向速度(0.1cm/s) w=角速度×100 yaw=已绕角度(°)
    if(UART1_Data[0]==6)
    {
      UART1_Data[0]=0;
      /*先校准左右再校准前后，左右走可能会抖，而且前后比左右的反馈更准
      注意！！！必须先让颜色传感器在左右移动之后一定能进入红/蓝区域，
      即前后距离必须能确保在红/蓝区域内（在哪里无所谓，后面再校准）
      */
      //如果为黑色，匀速往右走，直到传感器进入红/蓝区域
      //去抖：连续3次(约150ms)都读到红/蓝才确认，交界处"红黑红黑"抖动不会误停
      ROBOT_MoveSpeed(SPD_AVG_V, 0);   //★匀速靠近：SPD_AVG_V
      {
        uint8_t stable = 0;
        while(1){
          if(TCS_PollColor(&tcs_rgbc) != TCS_COLOR_BLACK){
            if(++stable >= 3) break;
          } else {
            stable = 0;
          }
          HAL_Delay(50);
        }
      }
      
      //因为加了消抖，所以会稍微多走一小点，再减少一点盲走的距离

      //进入红/蓝后，继续向右多走12，确保停在红/蓝区域内部（避免停在边缘抖动；距离按区域宽度调整），而且确保车身左右都在红/蓝区域内
      //★短距 12cm：按两档标准 20/30（<13cm 的峰值<20cm/s，若走不到位就把 a 单独加大到 100~200）
      ROBOT_Move(12, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
      
      //往前走，走到颜色传感器一定在黑色区域内（同样连续3次确认）
      ROBOT_MoveSpeed(0, SPD_AVG_V);   //★匀速靠近：SPD_AVG_V
      {
        uint8_t stable = 0;
        while(1){
          if(TCS_PollColor(&tcs_rgbc) == TCS_COLOR_BLACK){
            if(++stable >= 3) break;
          } else {
            stable = 0;
          }
          HAL_Delay(50);
        }
      }
      
      //颜色传感器校准前后：如果为黑色，匀速往后走，直到进入红/蓝区域（连续3次确认）
      ROBOT_MoveSpeed(0, -SPD_AVG_V);   //★匀速靠近：SPD_AVG_V
      {
        uint8_t stable = 0;
        while(1){
          if(TCS_PollColor(&tcs_rgbc) != TCS_COLOR_BLACK){
            if(++stable >= 3) break;
          } else {
            stable = 0;
          }
          HAL_Delay(50);
        }
      }
      
      //识别为红/蓝后继续向后多走3，确保停在红/蓝区域内部（避免停在边缘抖动；距离按区域宽度调整），而且确保车身前后都在红/蓝区域内
      //★短距 3cm：按两档标准 20/30（<13cm 的峰值<20cm/s，若走不到位就把 a 单独加大到 100~200）
      ROBOT_Move(0, -3, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
      ROBOT_MoveSpeed(0, 0);
    }

    // /* 串口打印角度环数据（目标/实际/输出w + 里程计位置x/y），SerialPlot 观察走直线纠偏/转向收敛
    //    并解析串口调参指令：kp/ki/kd/target 角度环、vx/vy 手动、mx/my 走距、mv/mvacc 规划速度 */
    // UART1_Printf("%f %f %f %f %f\r\n",
    //              chassis.target_yaw,
    //              HWT101CT_Data.yaw,
    //              chassis.yaw_pid.out,
    //              chassis.pos_x,
    //              chassis.pos_y);
    // if(UART1_RxFlag){
    //   UART1_RxFlag = 0;
    //   SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
    // }
   
    // UART1_Printf("GY1:%d  GY2:%d\r\n",GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin),GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin));
    // OLED_Printf(0, 0, OLED_8X16_HALF, "ni%d %d",GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin),GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin));
    // OLED_Update();
    // HAL_Delay(10);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
