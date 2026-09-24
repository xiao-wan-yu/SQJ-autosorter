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
#define DBG_START_ALL        0      //完整流程：圆盘机→仓库倒球→正面识别→阶梯→回家(默认)
#define DBG_START_ZHENGMIAN  1      //从"正面识别前"开始：跳过圆盘机+仓库倒球，车自己走到阶梯识别位
#define DBG_START_HUIJIA     2      //从"回家"开始：跳过前面全部，直接跑回家那段(红蓝区找色→停进红蓝区)
#define DBG_START_COLORCAL   3      //★颜色传感器单独校准：黑→红→蓝→白 四色重标定判色阈值(与比赛流程无关，跑完回菜单)
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
static const char * const DBG_START_NAME[] = { "ALL", "ZM", "HOME", "CAL" };
static const char * const DBG_START_DESC[] = { "full flow", "front recog", "go home", "color calib" };


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
/* ---------------- ★阶梯跑完去哪儿(修"跑完阶梯不回家/卡死在阶梯") ----------------
   1 = 阶梯跑完直接进"回家"段(跳过被 if(0) 禁用的立柱段)；
   0 = 老设计：不在这里置位，靠"立柱"段末尾置 HuiJia_Flag(前提是把立柱段的 if(0) 恢复成 if(LiZhu_Flag == 1))
   ★注意"回家"段的头几步位移是按"立柱/仓库那边跑完"的位置写的，从阶梯旁直接进回家段时，
     若落点不对，改回家段最前面那几条 ROBOT_Move 即可(见 HUIJIA_START 标签)。 */
#define JIETI_GO_HOME_DIRECT    1     //阶梯跑完直接回家(1) / 走完立柱再回家(0)
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

/* ================== ★调试跳转的"角度基准换算"（2026-09-21）==================
   为什么需要它（"单独测阶梯/回家角度乱套"的根因）：
     · 正常发车：上电时车头朝前 → HWT101CT 的 0° 就是"前"，所以程序里的绝对角
       0=前 / 90=右 / 180=后 / 270=左 都是按"场地"说的。
     · 单独测阶梯/回家：车是**按"倒完球那一步的姿态"摆好再上电**的（正常流程在那一步车头朝"右"），
       于是 0° 变成了"右" —— 整套绝对角相对场地都转过了 DBG_YAW_SHIFT(90°)，
       再直接拿去 ROBOT_Angle 就会白转一个角度、越走越乱。
   做法：跳转入口把 dbg_yaw_shift 置 1，之后**每一处绝对角都过一遍 Yaw_Abs()**：
     Yaw_Abs(90)→0、Yaw_Abs(270)→180、Yaw_Abs(0)→270 …… 也就是"跳转进来就用另一套角度转"。
   正常流程(dbg_yaw_shift=0)一律原样返回，行为一个字都不变。
   ★摆车姿态必须是"倒完球那一步的样子"（红方车头朝右 / 蓝方朝左）SHIFT 才是 90；
     换别的摆法只改 DBG_YAW_SHIFT 这一个数。 */
#define DBG_YAW_SHIFT   90U          //跳转测试时"摆车姿态"相对正常"车头朝前"转过的角度(°)
static uint8_t  dbg_yaw_shift = 0;   //1=本次是调试跳转进来的 → 绝对角按上面那套换算
static uint32_t Yaw_Abs(uint32_t normal_angle){   //把"正常基准角"换算成本次实际该转的角度
  if(!dbg_yaw_shift) return normal_angle;
  return (normal_angle + (360U - DBG_YAW_SHIFT)) % 360U;
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
/* ==================== 立柱转圈：8.28"绕柱闭环"原版（commit b6fc0ab「2026.8.28好像看到转圈希望」）====================
   来源：那段代码原来在 main 循环里，用串口指令 5 触发"单独测试转圈"。这里原封不动搬成函数：
   立柱阶段(LiZhu_Flag==1)直接调用；串口调试指令 7 也调它 —— 两处跑的是同一套代码。

   思路（不依赖里程计位置，只用 陀螺仪 + 实时测距）：
     ① 开圈前静止采 12 次测距，只收 8~22cm 的有效值，冒泡排序取中值 → 目标测距 d_ref
        （前提：车头已正对柱子，GY53_2 读到的是"传感器→柱面"的距离）
     ② 闭环绕圈：切向 v_x = v_t 恒定；径向 v_y = KP_R×(实测-目标) 纠偏保半径（远了前进、近了后退）；
        w = v_t/实时半径 前馈（车头随圈转、始终指向圆心）+ KD_W×e 车头修正
        （★9.13 改：原来是「测距变化率→w 阻尼」，符号反了 + 拿差分做微分，既抽风又保不住半径，详见函数内说明）
     ③ 绕圈进度 = 陀螺仪累积转角，绕满 355° 停；测距连续 20 次无效 → 保护停车（打印 LOST! stop）

   参数（就下面这四个 const，改完重新编译；每个"调大/调小会怎样、看哪一列、建议范围"
        见函数里 const 上方那段详细说明）：
     v_t    = 6.0f   切向速度 cm/s（>0 逆时针 / <0 顺时针）
                     大：绕得快，但 GY-53 数据只有 5Hz、w 也更大；小：稳、一圈更久（35~42s）
     KP_R   = 1.0f   径向纠偏增益 1/s（误差cm → v_y cm/s）：只管小误差的快速微调
     VY_MAX = 2.5f   径向速度限幅 cm/s：兜住突然的半径偏差（≈v_t/2，不抢切向）
     KD_W   = 0.05f  半径误差→w 修正 (rad/s)/cm：收半径的主力（纯前馈 w=v_t/r 只是临界稳定，
                     没有这一路半径必跑飞；符号+来源 9.13 都改过，别照老注释理解）

   现场排查（串口1，每 200ms 一条）：d=测距mm  e=径向误差mm  vy=径向速度(0.1cm/s)  w=角速度×100  yaw=已绕角度(°)
     · yaw 不涨 → 车头压根没转（w 符号/麦轮/控制循环的问题），先别调参数
     · d 往一个方向单调跑（慢慢爬出圈或一路收进来）→ KD_W 太小或方向不对，★先查它
     · e 长期同号 → KP_R 太小；e 在 0 附近来回跳 → KP_R/VY_MAX 太大，或就是 GY-53 的 ±1cm 噪声
     · 一直打印 LOST! stop → 车头没对着柱子（测距跑出 8~22cm 窗口）或半径已经崩了
   ★立柱阶段要求"遇到可以夹的就停下转圈"：钩子在下面 while 里，插视觉判断后 break 即可。 */
static void LiZhu_Circle_Run(void)
{
  if(!flag.chassis){                          // 前置条件：底盘控制循环在跑，否则车不会动、while 会一直空转
    UART1_Printf("no chassis!\r\n");
    return;
  }
  UART1_Printf("circle start\r\n");

  /* ===== 测距定参考距离：多次采样只收有效值(目标区间10~20cm)，取中值抗杂散 =====
     前提：车头已正对柱子（GY53_2 读到的是 传感器→柱面 的距离） */
  uint16_t d_ok[10];                          // 有效采样缓存
  uint8_t  n = 0;                             // 有效采样个数
  for(uint8_t i = 0; i < 12; i++){            // 最多采12次
    uint16_t dd = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
    if(dd >= 80 && dd <= 220){                // 只收8~22cm：丢目标返回大值/杂散直接丢弃
      d_ok[n++] = dd;
      if(n >= 10) break;
    }
    HAL_Delay(30);                            // 采样间隔，避开电机/震动噪声
  }
  UART1_Printf("valid=%d\r\n", n);
  if(n < 3){                                  // 有效采样太少：保护退出，绝不乱转
    UART1_Printf("no pipe!\r\n");
    return;
  }

  /* 冒泡排序取中值：比平均更抗单次大值/小值 */
  for(uint8_t i = 0; i < n-1; i++)
    for(uint8_t j = i+1; j < n; j++)
      if(d_ok[j] < d_ok[i]){ uint16_t t = d_ok[i]; d_ok[i] = d_ok[j]; d_ok[j] = t; }
  uint16_t d_ref = d_ok[n/2];               // 目标测距 mm
  UART1_Printf("ref=%dmm\r\n", d_ref);

  /* 几何常量（实测尺寸，不是手感参数；只进"半径"计算，量准了就别动）：
     r = 测距 + 传感器到车心 + 管半径，用来算前馈 w=v_t/r（车头每秒该转多少度）。
     改大 → r 算大 → w 偏小 → 车头转得比实际绕圈慢；改小反之。对 355° 一圈是几十度的累积差。
     标准起步位置：传感器→管壁 20cm，此时车心到管心 = 20+14+4 = 38cm，打印 ref≈200mm。 */
  const float GY53_2_OFFSET_CM = 14.0f;     // 前测距传感器到车中心的纵向距离(cm)（实测14.0cm）
  const float PIPE_RADIUS_CM   = 4.0f;      // 柱子(水管)半径4cm（外径8cm）

  /* ===== 绕柱闭环 v2（不依赖里程计位置，只用 陀螺仪+实时测距） ===== */
  flag.angle = 0;                           // 角度环让位，w 由本闭环接管
  chassis.v_x = 0.0f;  chassis.v_y = 0.0f;  chassis.w = 0.0f;
  chassis.x_speed_plan_flag = 0;
  chassis.y_speed_plan_flag = 0;
  chassis.x_set_speed_flag  = 1;            // 手动设速，防控制循环归零
  chassis.y_set_speed_flag  = 1;

  float yaw0     = HWT101CT_Data.yaw;       // 起点朝向（车头指向圆心）
  float yaw_last = yaw0;
  float yaw_acc  = 0.0f;                    // 陀螺仪累积转角(°)
  float d_ref_cm = (float)d_ref/10.0f;      // 目标测距 cm
  float d_cm     = d_ref_cm;                // 当前有效测距 cm（★9.13 起不再留 d_prev：GY-53 数据只有 5Hz，差分全是尖峰）
  uint8_t  lost      = 0;                   // 连续无效计数
  uint8_t  lost_stop = 0;                   // 丢目标停车标志
  uint32_t t_prt     = HAL_GetTick();       // 打印节拍
  /* ================= 绕圈 4 个可调参数（现场就调这四行）=================
     调参顺序（9.13 版循环已经闭环，4 个旋钮各管一件事，别一起动）：
       ① 先看 yaw 涨不涨、d 会不会一路往一个方向跑：
          yaw 不涨   → 车头压根没转（w 符号/麦轮/控制循环），先别调参数；
          d 一路单调往外/往里跑（不是围着目标摆）→ 先把 KD_W 取负试一次：
                       方向对了就该能收住；方向对但摆得太大 → KD_W 减 0.02。
       ② 再按 e 定 KP_R：e 长期同号(>1cm) → 加 0.2；e 在 0 附近来回跳 → 减 0.2。
       ③ 最后按 vy 定 VY_MAX：vy 常年顶在 ±VY_MAX → 说明残差一直很大，
          先回去加 KD_W（收半径的大头在它），VY_MAX 只做小误差微调，不用给太大。
     每次只动一个，跑完一圈看串口那 5 列（d e vy w yaw）再决定下一动。

     v_t    切向速度 cm/s（车沿圈往前蹭的快慢；>0 逆时针 / <0 顺时针 = 绕行方向反过来）
              调大 → 一圈更快（半径33cm：5cm/s≈42s，6cm/s≈35s，8cm/s≈26s），但 GY-53 的 PWM 数据
                     更新只有 5Hz(默认高精度档 T≈200ms)：8cm/s 时两次有效测距之间车已经蹭出去 1.6cm，
                     等于闭着眼走一段；前馈 w=v_t/r 也大，离散步进大、偏差来不及纠。
              调小 → 稳、丢目标少，但一圈变慢，比赛时间紧就别太小。
              看现象：d 一路往下掉、最后 LOST → 往下调到 5 甚至 4；干净跑完还想快 → 上调到 7~8。
              建议 5~7，先 6（等 6 能干净跑完一圈、四个参数都稳了，再往上试速度）。

     KP_R   径向纠偏增益 1/s（测距与目标差 1cm → 产生几 cm/s 的"往圈里/往圈外"速度）
              只管"小误差的快速微调"；收半径的大头在 KD_W 那一路（见下），别指望它把大偏差拉回来。
              调大 → 半径拉回快；但 GY-53 精度只有 ±1cm(默认档)，乘大后 v_y 跟着抖，车径向一顿一顿。
              调小 → 平顺不抖；但几厘米的偏差要很久才拉回来，e 会长期同号。
              看现象：e 长期同号(>1cm) → 加 0.2；e 在 0 附近来回跳、vy 抖 → 减 0.2。
              建议 0.8~1.5，先 1.0。

     VY_MAX 径向速度限幅 cm/s（"往圈里/往圈外"这一路最多给多快，硬顶）
              调大 → 大偏差时拉回快；但径向速度一旦超过切向的一半，车就斜着往圈里插/往圈外退，
                     麦轮横向擦地打滑，姿态一乱测距跟着乱（原版 5 比切向 8 的一半还多，就是这个毛病）。
              调小 → 修正顺、姿态稳；但被撞偏几厘米时要很久才拉回，期间 d 一直偏、偏多了会丢目标。
              看现象：vy 常年顶在 ±VY_MAX 上 → 说明残差一直很大：先去加 KD_W，仍然顶再放到 3。
              建议 2~3，先 2.5（约 v_t 的一半）。

     KD_W   半径误差→w 修正 (rad/s)/cm（★9.13 改，替代原来的"测距变化率→w 阻尼"）
              怎么算：w = v_t/r_est + KD_W×e（e = 实测-目标，cm），修正量限幅到前馈的一半。
              为什么必须靠它：只给前馈 w=v_t/r 时车头是"开环转"的——yaw 差 0.3°/s(约2%)，半径就会以
              二次曲线一路跑飞（你那份 log 正是如此：先缓缓往外爬到 184mm，再往内崩到 93mm 丢目标）。
              KD_W>0 = 给"偏了多少"装了个自动回正：远了多转一点 → 车头偏进圈 → 切向速度在径向上就
              带出 v_t·sinψ 的分量（最多能有 v_t 这么大，比 VY_MAX 那一路大好几倍，才是收半径的主力）。
              调大 → 半径收得快、圈贴得紧；太大 → 车头来回拧、d 在目标附近大幅度摆动（欠阻尼振荡）。
              调小/置0 → 退回"纯前馈"：半径只剩 KP_R×e 那点速度扛，车头稍有没对正(3~5°)就会
                     一路往外/往里跑，几十秒内丢目标（这就是原来 KD_W=0 的结果）。
              看现象：d 长期偏一边不收 → 加 0.02；d 在目标附近来回过冲 → 减 0.02。
              建议 0.03~0.06，先 0.05（≈ ωn=√(v_t·KD_W)=0.55rad/s、阻尼比 0.9，约 10s 收敛）。
              ★万一接上后反而"越绕越偏"（越转越往一个方向跑）→ 说明这台车的 w/麦轮符号链跟我推的
                相反，把 KD_W 取负(-0.05)再试一次，哪边能收住就用哪边。
     ==================================================================== */
  const float v_t    = 6.0f;                // 切向速度 cm/s（>0逆时针 / <0顺时针）
  const float KP_R   = 1.0f;                // 径向纠偏增益 1/s（半径误差cm → 径向速度cm/s）
  const float VY_MAX = 2.5f;                // 径向速度限幅 cm/s（别超 v_t/2，径向不抢切向）
  const float KD_W   = 0.05f;               // 半径误差→w 修正 (rad/s)/cm（★9.13 换符号+换来源）

  while(fabsf(yaw_acc) < 355.0f){           // 绕满一整圈
    uint16_t dd = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
    if(dd >= 80 && dd <= 220){              // 有效读数
      d_cm = (float)dd/10.0f;
      lost = 0;
    }else{                                  // 无效/丢目标：保持上次值，连续20次→停车
      if(++lost >= 20){ lost_stop = 1; break; }
    }

    /* 径向纠偏：远了前进(朝圆心)、近了后退 */
    float e = d_cm - d_ref_cm;
    chassis.v_x = v_t;                      // 切向（车身x）
    chassis.v_y = KP_R * e;                 // 径向（车身y，车头朝圆心）
    if(chassis.v_y >  VY_MAX) chassis.v_y =  VY_MAX;
    else if(chassis.v_y < -VY_MAX) chassis.v_y = -VY_MAX;
    /* w = 切向速度/实时半径 前馈 + 半径误差→车头修正（KD_W，★9.13 改：符号 + 来源）
       ── 为什么原来"w = v_t/r - KD_W×(d-d_prev)"又抽风又保不住半径（看你那份 log）：
       ① 符号反了：远了(e>0)要让车头多转一点(w 加大)才能往圈里收；
          原版"远了反而少转"→ ψ 越偏越大 → 半径越跑越远/越收越紧，是正反馈（KD_W=0.3 时 w=-175/140 的抽风）。
       ② 来源是差分：GY-53 PWM 数据更新只有 5Hz(默认高精度档 T≈200ms)，两拍之间读数常是"同一次测量重复值"，
          差分要么 0 要么整段 2cm 跳变 → 微分尖峰。改成拿"半径误差 e"乘 KD_W，等于对误差积分，抗噪、稳态无静差。
       ③ 物理意义：车头多偏 ψ 角 → 切向速度 v_t 在径向上就带出 v_t·sinψ 的分量（最多 v_t 本身），
          这才是收半径的主力；KP_R×e 那一路只有 VY_MAX 这么点速度，只管小误差的快速微调。
       ④ 纯前馈(w=v_t/r) 那一路是"临界稳定"：yaw 只要差 0.3°/s，半径就会以二次曲线跑飞（10s 掉 6cm，正是 log 的样子）。 */
    float r_est = d_cm + GY53_2_OFFSET_CM + PIPE_RADIUS_CM;
    if(r_est < 10.0f) r_est = 10.0f;        // 防小半径产生过大 w
    float w_corr     = KD_W * e;            // 半径误差 → 车头角速度修正(rad/s)
    float w_corr_max = 0.5f * v_t / r_est;  // 修正量最多到前馈的一半，别把车头拧歪
    if(w_corr >  w_corr_max) w_corr =  w_corr_max;
    else if(w_corr < -w_corr_max) w_corr = -w_corr_max;
    chassis.w = v_t / r_est + w_corr;
    if(chassis.w >  YAW_PID_OUT_MAX) chassis.w =  YAW_PID_OUT_MAX;
    else if(chassis.w < -YAW_PID_OUT_MAX) chassis.w = -YAW_PID_OUT_MAX;

    /* 实时打印（每200ms；整数，避免%f不支持问题）
       d = 测距mm   e = 半径误差mm(正=远了)   vy = 径向速度(0.1cm/s，读数12就是1.2cm/s)
       w = 角速度×100(rad/s)   yaw = 已绕角度(°)
       ★原来 vy 打的是整数 cm/s：KP_R=1 时 e 要 1cm 才出 1，平时整列全是 0，等于看不见纠偏在不在动，
         所以改成 ×10。vx 恒等于 v_t 没信息量，删掉，空出一列打"绕圈进度 yaw"（355° 就该停，
         它不涨就是卡住了/在局部打转；配合 w 就能判断车头到底转没转）。 */
    if(HAL_GetTick() - t_prt >= 200){
      UART1_Printf("d=%d e=%d vy=%d w=%d yaw=%d\r\n",
                   (int)(d_cm * 10.0f), (int)(e * 10.0f),
                   (int)(chassis.v_y * 10.0f),
                   (int)(chassis.w * 100.0f),
                   (int)fabsf(yaw_acc));
      t_prt = HAL_GetTick();
    }

    /* ===== 识别钩子（原来立柱阶段那句"遇到可以夹的就停下转圈"搬到这里）=====
       要边绕边等视觉：在这里查一次视觉结果，命中就 break；
       break 后下面的停车/恢复角度环照常执行，车就停在原地不动 */
    /* if(视觉命中){ break; } */

    /* 陀螺仪累积转角判断已绕角度 */
    float ddg = HWT101CT_Data.yaw - yaw_last;
    yaw_last = HWT101CT_Data.yaw;
    if(ddg > 180.0f)       ddg -= 360.0f;
    else if(ddg < -180.0f) ddg += 360.0f;
    yaw_acc += ddg;

    HAL_Delay(10);                          // 本循环节拍(底盘控制周期已改 20ms，这里 10ms 只是读测距/打印节奏)
  }
  /* 停车 + 恢复角度环（重新锁向当前朝向） */
  chassis.v_x = 0.0f;  chassis.v_y = 0.0f;  chassis.w = 0.0f;
  chassis.x_set_speed_flag = 0;
  chassis.y_set_speed_flag = 0;
  flag.angle = 1;
  chassis.target_yaw = YAW_TARGET_NONE;
  if(lost_stop) UART1_Printf("LOST! stop\r\n");
  else          UART1_Printf("circle done\r\n");
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
       7 = 立柱绕圈 LiZhu_Circle_Run()（8.28"绕柱闭环"原版，实现在本函数上方）
   返回 1 = 这一帧已被本函数处理掉（7 绕圈已在里面阻塞跑完）；0 = 只是解析/回显，交给主循环原逻辑
   要改绕圈参数（v_t/KP_R/VY_MAX/KD_W）就去改上面 LiZhu_Circle_Run() 函数里的 const，别改散落的其它地方
   ------------------------------------------------------------------------------------------
   ★2026-09-15 新增【单键手动测试指令】（专门用来单独测 前/后/左/右 / 原地转）：
     帧里没有逗号就按单键解释（外面那套 "S,A,B,..." 数值指令完全不受影响）：
       w/s/a/d = 前进/后退/左移/右移（持续走，发 x 停）   W/S/A/D = 同上但只走 1 秒自动停
       x 或空格或回车 = 停车     +/- = 测试速度 ±10cm/s（默认 30，10~100）
       1/2/3/4 = 原地转到 0°/90°/180°/270°      p = 打印状态   o = 里程清零   h = 帮助
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
      case 'h': case '?':
        UART1_Printf("single-key: w/s/a/d=前进/后退/左移/右移  W/S/A/D=走1秒  x=停  +/-=调速\r\n");
        UART1_Printf("            1/2/3/4=原地转0/90/180/270  o=里程清零  p=状态\r\n");
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

  if(UART1_Data[0]==7){                             // 立柱绕圈（8.28 绕柱闭环原版，见上方 LiZhu_Circle_Run）
    UART1_Data[0] = 0;                              // 立即清指令，防止循环重复触发
    UART1_Printf("lizhu circle start\r\n");
    /* 直接调立柱阶段用的那个函数（两边同一套代码，方便先单独测）：
       车头先对着柱子 → 静止采12次测距取中值当目标距离 → 切向 v_t + 径向闭环(KP_R/VY_MAX)保半径
       + w=前馈(v_t/实时半径) + KD_W×半径误差 车头修正，陀螺仪累计转角满355°停；测距连续20次无效则保护停车 */
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
       切回完整流程(dbg_start=ALL)时还会带着 dbg_yaw_shift=1 → 所有绝对角都被换算，那就错了。 */
    dbg_yaw_shift = 0;
    if(dbg_start == DBG_START_ZHENGMIAN){
      /* 从"正面识别前"开始：跳过圆盘机+仓库倒球，直接跳到 ZHENGMIAN_START 标签往下执行
         （往下依次是：收倒球槽 → 走到阶梯识别位 → 左前光电对准 → 清串口残留 → 发0xA2通信流程 → 阶梯 → 回家）
         ★车会先自己走到阶梯识别位，所以先把车放在圆盘机/仓库方向随便一点的位置即可 */
      UART1_Printf("DEBUG: start from ZHENGMIAN (before front recognize)\r\n");
      /* ★★ 角度基准：本次车是"按倒完球那一步的姿态"摆好再上电的（正常流程在那里车头朝右），
         HWT101CT 的 0° 就变成了"右"，所有绝对角整体差 90° → 置位 dbg_yaw_shift，
         之后每一处 ROBOT_Angle 都走 Yaw_Abs() 换算过的那一套角度（见文件上方说明）。
         摆车要求：位置按"倒完球"那附近放、**车头朝向按那一步的姿态摆正**（红方朝右/蓝方朝左），
         因为基准就是这么来的、程序不会再纠正你。 */
      dbg_yaw_shift = 1;
      UART1_Printf("DEBUG: yaw base shift ON (%u deg) for this run\r\n", (unsigned)DBG_YAW_SHIFT);
      //跳过了圆盘机，就没走"告诉视觉红(0xAA)蓝(0xBB)方"那一步，这里补上（视觉也得知道红/蓝方）
      UART2_Printf("%c", mode_red ? 0xAA : 0xBB);//告诉视觉红(0xAA)蓝(0xBB)方
      goto ZHENGMIAN_START;
    }
    else if(dbg_start == DBG_START_HUIJIA){
      /* 从"回家"开始：跳过前面全部阶段，直接跳到 HUIJIA_START 标签往下执行（红蓝区找色→停进红蓝区）
         ★同 ZM 起点：车是按"倒完球那一步的姿态"摆的，绝对角整体差 90° → 置位换算标志 */
      UART1_Printf("DEBUG: start from HUIJIA (go home)\r\n");
      dbg_yaw_shift = 1;  //本段里的绝对角(90/270/0)都走 Yaw_Abs() 换算（见文件上方说明）
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
           M6 显示/收尾: OLED + 停车 + 交棒给"回家"
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
           正常流程：红90/蓝270；跳转测阶梯(dbg_yaw_shift=1)：Yaw_Abs() 换算成 0/180（摆车时 0° 已是"右"）。
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

        /* ---- M6.2 收尾：停车 + 抬臂（M6.3 接着交棒给"回家"）---- */
        JieTi_MoveSpeed(0, 0);
       
        //单独的机械臂抬起
        runActionGroup(151, 1);
        HAL_Delay(1000);
        
        JieTi_Flag = 0;//M6.3 阶梯结束（下面按 JIETI_GO_HOME_DIRECT 交棒给"回家"）
        /* ★2026-09-15 修复"跑完阶梯不回家、车停着像卡死在阶梯"：
           原来这一行写的是 `HuiJia_Flag == 1;` —— 是"比较"，不是"赋值"，整条语句没有任何效果
           (编译器其实早就警告过：main.c 里那句 "statement with no effect [-Wunused-value]")。
           于是下面 `if(HuiJia_Flag == 1)` 的"回家"段永远不成立；
           而唯一会把它置 1 的地方在"立柱"段末尾(本段更下面)，那段又被 `if(0)` 整段禁用 →
           阶梯跑完后所有阶段标志全是 0，主循环一路空转回最上面的"红蓝方选择"菜单：
           车停着不动、屏幕停在最后画面、串口也不再打印，看起来就是"卡死在阶梯"。
           现在按 JIETI_GO_HOME_DIRECT 真正赋值：默认=阶梯跑完直接进"回家"段。
           想走"阶梯→立柱→回家"的老流程：① 把 JIETI_GO_HOME_DIRECT 置 0
                                        ② 打开下一行 //LiZhu_Flag = 1;
                                        ③ 把下面 if(0) 改回 if(LiZhu_Flag == 1) */
        if(JIETI_GO_HOME_DIRECT) HuiJia_Flag = 1;   //★必须是"赋值"：写成 == 就永远回不了家
        //LiZhu_Flag = 1;//因为中途没去仓库，所以两个状态需要同时切换

        if(0)
        //if(LiZhu_Flag == 1)//立柱开始
        {
          
          runActionGroup(0, 1);//复位
          //右前光电无障碍物，就开始向左走固定距离（走到阶梯平面中间）（★短距(45,35)：20/30）
          ROBOT_Move(-45, -35, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
          
          //好像前面把角度清0了，此时正对阶梯为0（不对，依然是正对阶梯为90呢）
          ROBOT_Angle(270);
          //ROBOT_Angle(180);
          
          //这里是校准到正对立柱
          /* ====== 立柱前校准：先左右(激光)再前后(测距)，校准完再转圈 ======
             前提(现场保证)：车在立柱右边、一开始前测距 > 200mm。
             左右：10cm/s 往左走，右激光(激光3)看到柱面就停；停下检验两个激光：
                   两个都有=左右正对；只有右有(往左走多了)→往右补 2cm；只有左有/都没有→往左补 2cm；
                   每次停下都再检验一遍，直到两个都有。
             前后：往前走，测距进 190~210mm 就停；停下再验一次，不在 200 附近就按差值用 ROBOT_Move 补。 */
          {
            float    lz_yaw = chassis.target_yaw;      //270°(ROBOT_Angle 校好的朝向)：置零后锁回来，
                                                       //  否则 ROBOT_MoveSpeed 的哨兵会把"当前歪掉的朝向"当新目标
            uint32_t lz_t0  = HAL_GetTick();

            //---- 第一步：20cm/s 往左走，右激光(激光3)有障碍物就停 ----
            ROBOT_MoveSpeed(-(float)SPD_AVG_V, 0.0f);
            while(LASER_Barrier(LASER3_GPIO_Port, LASER3_Pin) == 0){   //右激光看到柱面 → 跳出
              if(HAL_GetTick() - lz_t0 > 5000U) break;                  //5s 还没看到：别一直往左跑(防异常)
            }
            ROBOT_MoveSpeed(0.0f, 0.0f);
            chassis.target_yaw = lz_yaw;
            HAL_Delay(150);                            //停稳

            //---- 停下检验：两个激光都有障碍物才算左右正对，不然往另一边补 2cm，每次停下都再验 ----
            for(uint8_t k = 0; k < 6; k++){            //最多补 6 次(防死循环)
              uint8_t b3 = LASER_Barrier(LASER3_GPIO_Port, LASER3_Pin);   //右激光
              uint8_t b4 = LASER_Barrier(LASER4_GPIO_Port, LASER4_Pin);   //左激光
              if(b3 && b4) break;                                         //★两个都有 → 左右校准完成
              if(b3) ROBOT_Move( 2, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);   //只有右有(往左走多了) → 往右 2cm
              else   ROBOT_Move(-2, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);   //只有左有/都没有 → 往左 2cm
              HAL_Delay(150);
            }

            //---- 第二步：往前走，测距进 190~210mm 就停 ----
            lz_t0 = HAL_GetTick();
            ROBOT_MoveSpeed(0.0f, (float)SPD_AVG_V);
            while(1){
              uint16_t d = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
              if(d <= 210U) break;                     //进 190~210 窗口(或更近) → 停，下面再验/补
              if(HAL_GetTick() - lz_t0 > 5000U) break; //5s 还没进窗口：别一直往前(防异常)
            }
            ROBOT_MoveSpeed(0.0f, 0.0f);
            chassis.target_yaw = lz_yaw;
            HAL_Delay(250);                            //停稳 + 等 GY-53 出新读数

            //---- 停下再验：不在 200 附近就按差值用 ROBOT_Move 往前/往后补，补完再验 ----
            for(uint8_t k = 0; k < 4; k++){            //最多补 4 次
              uint16_t d  = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
              if(d >= 190U && d <= 210U) break;        //★200±10mm → 前后校准完成
              int32_t  cm = ((int32_t)d - 200) / 10;   //差几厘米(正=离远了 → 往前走)
              ROBOT_Move(0, cm, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
              HAL_Delay(250);
            }
          }

          
          //立柱转圈：绕柱闭环（测距定半径 + 陀螺仪累计转角 + 径向闭环 KP_R/VY_MAX + 航向前馈/KD_W 修正）
          //★前提：车头已经正对着柱子（函数开头就是静止采测距定参考距离）
          LiZhu_Circle_Run();

          /*
          识别钩子已搬进 LiZhu_Circle_Run() 的绕圈 while 里：
          遇到可以夹的就在那里 break，车停下、收尾照常执行
          */

          //转完一圈，收起机械臂，然后往左转身走到仓库中间倒方块（★长距 60/138cm：120/120）
          ROBOT_Move(-60, 0, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);
          ROBOT_Angle(90);//车子前面朝右
          ROBOT_Move(0, -138, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);
          ROBOT_Move(-45, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

          //定位操作：向左慢平移到左后光电感应到无障碍物，之后再往右走固定距离（刚到仓库中间的距离）
          ROBOT_MoveSpeed(-SPD_AVG_V, 0);   //★匀速靠近：SPD_AVG_V
          while (LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin)==1);
          ROBOT_MoveSpeed(0,0);

          ROBOT_Move(25, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

          //得走远一点才能转身倒方块
          ROBOT_Move(0, 10, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
          ROBOT_Angle(270);//车子前面朝左
          UART1_Printf("10");
          
          //这里放倒方块的代码
          runActionGroup(16, 1);//倒方块动作组(复用圆盘机)
          HAL_Delay(2000);//倒方块动作约2s
          runActionGroup(19, 1);//收倒球槽

          //倒完方块转个身再回家
          ROBOT_Angle(0);
          //往后多走一点，必须保证，前后在左右移动后能进入红色区域（★长距(35,225)：120/120）
          ROBOT_Move(35, -225, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);//60，-240能进
          LiZhu_Flag = 0;//立柱结束，回家开始
          HuiJia_Flag = 1;
        }




    /* ★调试入口(KEY0选成HOME+KEY3开始)：goto 跳到下面 HUIJIA_START 标签往下执行"回家"
       （HuiJia_Flag 已在跳转前立好；正常流程走到这里时一字不变） */
HUIJIA_START:
    if(HuiJia_Flag == 1)//回家开始
    {
    //暂时的
          //ROBOT_Move(60, 0, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);
          runActionGroup(0, 1);//复位
          mode_red ? ROBOT_Angle(Yaw_Abs(90)) : ROBOT_Angle(Yaw_Abs(270));//车子前面朝右/朝左（★长距 (30,-145)：120/120）
          ROBOT_Move(30, -165, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_V, SPD_SHORT_A);
          ROBOT_Move(-70, 0, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_V, SPD_SHORT_A);

//往后慢退，直到测距测得合适距离（适合倒球的距离）
      ROBOT_MoveSpeed(0, -SPD_AVG_V);   //★匀速靠近：SPD_AVG_V
      WAIT_WHILE(GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin)>85,
                 "HUIJIA back-to-wall(mm<=85)");//100有点远，距离小于90就退此循环，90也远
      SENSOR_STOP();      /* ★停车：速度环 target=0 → 主动反接刹车（不再自由滑行/不再反向冲）
                             ★倒球距离若不对：把上面的 85 调大（如 95），或在这里补一条固定位移 */

          //定位操作：向左慢平移到左后光电感应到无障碍物
      //新：更改激光位置，让它在没对到障碍物时直接就已经是合适的位置，不需要调整
      ROBOT_MoveSpeed(-SPD_AVG_V, 0);   //★匀速靠近：SPD_AVG_V
      WAIT_WHILE(LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin)==1, "HUIJIA laser1 no-obstacle");
      SENSOR_STOP();      /* ★停车（同上）；★若左边还差一点：
                             ROBOT_Move(-X, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A); */

      //加一次角度校准（ROBOT_Angle 已阻塞到停稳，不必再补延时）
      mode_red ? ROBOT_Angle(Yaw_Abs(90)) : ROBOT_Angle(Yaw_Abs(270));
      //往右走固定距离（刚到对上仓库的距离）（★短距 20cm：20/30）
      if(mode_red) ROBOT_Move(22, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

      runActionGroup(16, 1); 	//这里是倒球动作组（复用圆盘机）
      delay_ms(2000);

      //★"前后抖"是故意快抖（不是走位），按两档标准里的特例保留 100
      for(uint8_t i = 0; i < 2; i++){//前后抖
        ROBOT_Move(0, 4, 0, 160, 0, 160);
        ROBOT_Move(0, -4, 0, 160, 0, 160);
      }

          runActionGroup(19, 1);//收倒球槽
          //往前走确保转方向不卡脚（★短距 15cm：20/30）
          ROBOT_Move(0, 15, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
          ROBOT_Angle(Yaw_Abs(0));//车子前面朝左
          

          //倒完方块转个身再回家
          ROBOT_Angle(Yaw_Abs(0));
          //往后多走一点，必须保证，前后在左右移动后能进入红色区域（★长距 230cm：120/120）
          ROBOT_Move(mode_red ? 30 : -30, -230, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);//60，-240能进

          ROBOT_Angle(Yaw_Abs(0));
          HuiJia_Flag = 1;

          //暂时的
    
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
        TCS34725_GetRawData(&tcs_rgbc);
        if(TCS34725_ClassifyColor(&tcs_rgbc) != TCS_COLOR_BLACK){
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
    ROBOT_Move(mode_red ? 11.5 : -11.5, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
    
    //往前走，走到颜色传感器一定在黑色区域内（同样连续3次确认）
    ROBOT_MoveSpeed(0, SPD_AVG_V);   //★匀速靠近：SPD_AVG_V
    {
      uint8_t stable = 0;
      while(1){
        TCS34725_GetRawData(&tcs_rgbc);
        if(TCS34725_ClassifyColor(&tcs_rgbc) == TCS_COLOR_BLACK){
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
        TCS34725_GetRawData(&tcs_rgbc);
        if(TCS34725_ClassifyColor(&tcs_rgbc) != TCS_COLOR_BLACK){
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
      }

    }
    /* 原来这里有一句无条件的 UART1_Data[0]=0;，它把刚解析出来的指令(6/7)提前清成 0，
       所以"发指令没反应"。已删除：每条指令在处理时自己会清零。 */

    //立柱转圈(8.28"绕柱闭环"原版)：正式流程在立柱阶段(LiZhu_Flag==1)里调 LiZhu_Circle_Run()；
    //  单独调试：在上方 UART1_DebugCmd() 里，串口发 "7,0,0,0,0,0,0,0" 就地跑一遍
    //  （停在"红蓝方选择"菜单界面也能用，菜单 while 里也调了 UART1_DebugCmd）：
    //      if(UART1_Data[0]==7){ UART1_Data[0]=0; LiZhu_Circle_Run(); }
    //  前提：车头已正对柱子（函数开头静止采12次测距、只收8~22cm有效值取中值当目标距离）
    //  绕法：切向 v_x=v_t 恒定 + 径向 v_y=KP_R×(测距-目标) 保半径
    //        + w=v_t/实时半径 前馈(车头始终指圆心) + KD_W×(测距-目标) 车头修正（★9.13 改：原为"变化率阻尼"）
    //  进度：陀螺仪累积转角，满355°停；测距连续20次无效 → 保护停车(打印 LOST! stop)
    //  调参：LiZhu_Circle_Run() 里的 v_t / KP_R / VY_MAX / KD_W 四个 const（函数头有调大调小口诀）
    //  串口(200ms/条)：d=测距mm e=径向误差mm vy=径向速度(0.1cm/s) w=角速度×100 yaw=已绕角度(°)
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
          TCS34725_GetRawData(&tcs_rgbc);
          if(TCS34725_ClassifyColor(&tcs_rgbc) != TCS_COLOR_BLACK){
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
          TCS34725_GetRawData(&tcs_rgbc);
          if(TCS34725_ClassifyColor(&tcs_rgbc) == TCS_COLOR_BLACK){
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
          TCS34725_GetRawData(&tcs_rgbc);
          if(TCS34725_ClassifyColor(&tcs_rgbc) != TCS_COLOR_BLACK){
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
