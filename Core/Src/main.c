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


    //车体10ms控制周期：速度闭环（第一阶段）
    static uint16_t count2 = 0;
    if(flag.chassis && ++count2 >= 10){
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

/* ================= 阶梯阶段参数 ================= */
#define JIETI_ALIGN_SPEED     5.0f    //对准阶段左右移动速度(cm/s)：左移、右移都用它
#define JIETI_ALIGN_BAND       30     //对准带：|cam_x|<=30 就算对准(停车夹取)；原来10太严，50太宽
#define JIETI_IMG_CX          160     //视觉x是0~320像素，减它转成相对画面中心的偏移
/* ---------------- 前后距离：实时监测 + 超标才校准 ----------------
   车向右扫描时容易前后漂移(离阶梯越来越远/越来越近)，夹取和视觉对准都会受影响。
   做法：扫描途中实时读前测距(GY53_2)，只有偏离目标超过 JIETI_FWD_TRIG_MM 才停下来校准：
     ① 先 ROBOT_MoveSpeed(0,0) 直接速度置零（增量式速度环的输出/积分随之清零，不挂速度、不留累计）
     ② 脉冲式一步步微调：给一小段速度 → 立刻置零 → 等车停稳+测距刷新 → 再判断。
        单步只走1~2mm，不会像"连续给速度+每10ms判断"那样被测距滞后(几十ms)带着冲过头——
        那正是"校准完又往后再往前"的原因：车还没停、测距还是旧值就反向给速度，加上底盘
        低速精细档(≤15cm/s)起步会"启动整形"顶PWM，冲得更明显。
     ③ 回到 目标±容差 或 超过 JIETI_FWD_MS 就结束(结束时同样置零)，调用方重新给 JIETI_ALIGN_SPEED
   测距说明：GY53_2是前面的、GY53_1是后面的；GY53_GetDistance_PWM()返回mm，超量程/无目标返回2000 */
#define JIETI_FWD_TARGET_MM   80      //目标前后距离(mm)：100靠太近，120太远,105也远，80才是合适
#define JIETI_FWD_TOL_MM        5     //容差(mm)：|实测-目标|<=5 视为已校准好
#define JIETI_FWD_TRIG_MM      10     //触发阈值(mm)：偏离超过该值才停下校准(避免频繁停车)
#define JIETI_FWD_MS        1000U     //单次校准最长耗时(ms)：到点先退出，剩下偏差下一轮扫描继续修
#define JIETI_FWD_SPEED       5.0f    //校准时的前后速度(cm/s，慢一点不容易冲过头)
#define JIETI_FWD_STEP_MS      80U    //微调每步给速度的时间(ms)：5cm/s×80ms≈4mm/步（80合适，100好像多了）
#define JIETI_FWD_SETTLE_MS    60U    //每步后置零等车停稳+测距刷新的时间(ms)
#define JIETI_FWD_CHK_MS       100U    //扫描中每隔多少ms实时测一次距离（50太频繁）
#define JIETI_FWD_OLED_MS     100U    //屏幕上实时测距的刷新周期(ms)

/* ---------------- 8个坑固定位移步进(计数就靠它) ----------------
   ★JIETI_STEP_CM = "坑与坑之间的距离"，现场拿尺量一下把值改准：
     每走完一段固定位移 = 到了下一个坑(粗定位)，再用视觉x精细对准；对准了就算数到这一个 */
#define JIETI_STEP_CM         10      //★坑间距(10cm)：每个坑之间固定走多远(现场量)
#define JIETI_STEP_SPEED      5      //走固定位移时的最大速度(cm/s)
#define JIETI_STEP_ACC        5      //走固定位移时的加减速(cm/s^2)
#define JIETI_ALIGN_MS      3000U     //单个坑"视觉对准"总超时(ms)：超时就按已对准处理，继续下一个
#define JIETI_VIS_MS         200U     //等主视觉一帧的超时(ms)

/* ================== 阶梯阶段：保持锁向目标（否则越走越斜） ==================
   ★ROBOT_MoveSpeed() 内部会把 chassis.target_yaw 置成哨兵(YAW_TARGET_NONE)，
     让底盘控制循环下一拍"锁定当前朝向"。角度环本身一直是开的(flag.angle=1)，
     但阶梯阶段车一直在左右/前后动、视觉每来一帧就设一次速度，
     于是"已经被走歪的朝向"被反复当成新目标锁住 → 角度环只保持歪掉的朝向、
     不再往原目标纠偏 → 现象就是"越来越斜、像没开角度环"。
   ★所以阶梯阶段统一用下面的 JieTi_MoveSpeed() 设速：设完速度立刻把目标朝向恢复回
     进入阶梯时校好的那个值，角度环就能一直按它纠偏。 */
static float jieti_keep_yaw = -1.0f;   //阶梯阶段目标朝向(°)，-1=还没锁(没锁就不动它)

//在原有基础上加了锁定目标朝向的代码：设完速度立刻把目标朝向恢复回进入阶梯时校好的那个值
static void JieTi_MoveSpeed(float x_speed, float y_speed){
  ROBOT_MoveSpeed(x_speed, y_speed);
  if(jieti_keep_yaw >= 0.0f) chassis.target_yaw = jieti_keep_yaw;  //恢复锁向目标，继续纠偏
}

/* 实时测距显示：只占第3行(y=32, 8x16)，先清这一行再画，
   不动第1/2行("JIETI letter x y" / "scan & grab...")，也不动下面几行 */
static void JieTi_ShowFwdMm(uint16_t d_mm){
  const char *tag = (d_mm > (JIETI_FWD_TARGET_MM + JIETI_FWD_TRIG_MM)) ? "FAR "
                  : ((d_mm + JIETI_FWD_TRIG_MM) < JIETI_FWD_TARGET_MM) ? "NEAR" : "OK  ";
  OLED_ClearArea(0, 32, 128, 16);                              //只清第3行
  OLED_Printf(0, 32, OLED_8X16_HALF, "D:%4umm %s", (unsigned)d_mm, tag);
  OLED_Update();
}

/* ---------------- 阶梯阶段运行时状态(显示/对准/测距共用) ---------------- */
static int16_t  jieti_cam_x     = 0;       //最近一帧：目标距画面中心偏移(负=偏左 / 正=偏右)
static uint8_t  jieti_cmd       = 0;       //最近一帧：cmd(要不要夹)
static uint8_t  jieti_blk_now   = 0;       //当前是第几个坑(1~8，0=还没对准第一个)
static uint32_t jieti_chk_tick  = 0;       //实时测距节流时刻(ms)
static uint32_t jieti_oled_tick = 0;       //屏幕刷新节流时刻(ms)

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

/* 清掉主视觉残留帧：只认之后的实时坐标(走完固定位移后 / 夹取动作后调用) */
static void JieTi_FlushVision(void){
  VISION1_RxFlag = 0;
  VISION1_RxRealLength = 0;
  memset(VISION1_RxBuf, 0, VISION1_RxLength);
}

/* 取一帧主视觉A3包(等不到就返回0)；结果存进 jieti_cam_x / jieti_cmd */
static uint8_t JieTi_GetVision(uint32_t wait_ms){
  uint32_t t0 = HAL_GetTick();
  while((HAL_GetTick() - t0) < wait_ms){
    if(VISION1_RxFlag){
      VISION1_RxFlag = 0;                                        //必须立即清零
      for(uint8_t i = 0; i + 7 <= VISION1_RxRealLength; i++){    //在缓冲里找 A3...0B 完整包
        if(VISION1_RxBuf[i] == 0xA3 && VISION1_RxBuf[i + 6] == 0x0B){
          jieti_cam_x = (int16_t)((uint16_t)VISION1_RxBuf[i + 2]
                                | ((uint16_t)VISION1_RxBuf[i + 3] << 8));  //x低字节在前(0~320像素)
          jieti_cam_x -= JIETI_IMG_CX;                                     //减160 → 距画面中心偏移
          jieti_cmd   = VISION1_RxBuf[i + 1];
          return 1;
        }
      }
    }
  }
  return 0;
}

/* 实时监测 + 超标才校准：入参是刚读到的距离(mm)；返回 1=确实停下校准过(调用方要重新给速度) */
static uint8_t JieTi_FwdFixIfNeeded(uint16_t d_now){
  if(d_now >= 2000U || d_now < 50U) return 0;                        //超量程/读数离谱：不处理，继续扫描
  int16_t err = (int16_t)d_now - (int16_t)JIETI_FWD_TARGET_MM;
  if(err <= JIETI_FWD_TRIG_MM && err >= -JIETI_FWD_TRIG_MM) return 0; //没超标：车照常走，不停不校

  /* ---- 超标：先直接速度置零(清掉速度环输出/积分)，再脉冲式一步步微调 ---- */
  JieTi_MoveSpeed(0, 0);
  uint32_t t0 = HAL_GetTick();
  uint32_t oled_tick = t0;
  while((HAL_GetTick() - t0) < JIETI_FWD_MS){                        //单次最多校准1秒
    uint16_t d = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
    if(d >= 2000U || d < 50U) break;                                 //读数异常：不瞎走，直接结束
    if(HAL_GetTick() - oled_tick >= JIETI_FWD_OLED_MS){              //校准时屏幕也实时刷测距
      oled_tick = HAL_GetTick();
      JieTi_ShowFwdMm(d);
    }
    err = (int16_t)d - (int16_t)JIETI_FWD_TARGET_MM;
    if(err <= JIETI_FWD_TOL_MM && err >= -JIETI_FWD_TOL_MM) break;    //已回到目标±容差 → 立即结束
    JieTi_MoveSpeed(0, (err > 0) ? JIETI_FWD_SPEED : -JIETI_FWD_SPEED);//离太远→前进 / 离太近→后退
    HAL_Delay(JIETI_FWD_STEP_MS);                                    //只走一小步
    JieTi_MoveSpeed(0, 0);                                           //立刻置零：不连续憋速度、不留累计
    HAL_Delay(JIETI_FWD_SETTLE_MS);                                  //等车停稳 + 等测距刷出新值再判断
  }
  JieTi_MoveSpeed(0, 0);                                             //到位/超时/异常：统一置零
  return 1;                                                          //告诉调用方"停过车了，要重新给速度"
}

/* 阶梯阶段"边走边服务"：按周期实时测距 + 刷屏幕 + 超标就停下校准
   返回1 = 这次真停过车校准(调用方按需把速度再给回去) */
static uint8_t JieTi_FwdService(void){
  if(HAL_GetTick() - jieti_chk_tick < JIETI_FWD_CHK_MS) return 0;          //没到测距周期：先不测
  jieti_chk_tick = HAL_GetTick();
  uint16_t d_mm = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);     //实时测距(mm)
  if(HAL_GetTick() - jieti_oled_tick >= JIETI_FWD_OLED_MS){               //屏幕按自己的周期刷
    jieti_oled_tick = HAL_GetTick();
    JieTi_ShowFwdMm(d_mm);                                               //第3行：实时测距
    JieTi_ShowBlockNo(jieti_blk_now, 8, jieti_cam_x);                    //第4行：第几个坑 + 视觉x
  }
  return JieTi_FwdFixIfNeeded(d_mm);
}

/* 走向并对准当前坑：用视觉x对准，|cam_x|<=JIETI_ALIGN_BAND 就算对准(对准完停车)
   first_pit=1：第1个坑要"先看到目标出现在画面右侧(x>对准带)"，再等它进入对准带(防止把别的东西当目标)
   对准过程中反复调用 JieTi_FwdService()：前后距离偏了就停下校准，校准完继续对准 */
static void JieTi_GoAlign(uint8_t first_pit){
  uint32_t t0 = HAL_GetTick();
  uint8_t  found = 0;                                                  //第1个坑：是否已看到目标在右侧
  while((HAL_GetTick() - t0) < JIETI_ALIGN_MS){                        //单坑对准总超时(超时按已对准处理)
    JieTi_FwdService();                                               //实时测距/屏幕/超标校准(会临时停车)
    if(!JieTi_GetVision(JIETI_VIS_MS)) continue;                      //这一拍没收到帧：接着等
    if(first_pit && !found){
      if(jieti_cam_x > JIETI_ALIGN_BAND) found = 1;                   //目标已经在右边出现，开始对准
      JieTi_MoveSpeed(JIETI_ALIGN_SPEED, 0);                          //没确认前一直往右找
      continue;
    }
    if(jieti_cam_x > JIETI_ALIGN_BAND)        JieTi_MoveSpeed(JIETI_ALIGN_SPEED, 0);   //目标偏右 → 右移
    else if(jieti_cam_x < -JIETI_ALIGN_BAND)  JieTi_MoveSpeed(-JIETI_ALIGN_SPEED, 0);  //目标偏左 → 左移
    else { JieTi_MoveSpeed(0, 0); break; }                            //进了对准带 → 对准完成
  }
  JieTi_MoveSpeed(0, 0);                                              //对准完/超时：停车
  JieTi_GetVision(JIETI_VIS_MS);                                      //再取最新一帧：拿它对这一坑的判断
}
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

    HAL_Delay(10);                          // 控制周期10ms
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
   要改绕圈参数（v_t/KP_R/VY_MAX/KD_W）就去改上面 LiZhu_Circle_Run() 函数里的 const，别改散落的其它地方 */
static uint8_t UART1_DebugCmd(void){
  if(!UART1_RxFlag) return 0;                       // DMA空闲中断没收到帧：什么都不做
  UART1_RxFlag = 0;                                 // 必须立即清零
  char line[UART1_RxLength + 1];                    // 拷贝一份并补'\0'（DMA缓冲末尾没有结束符）
  uint16_t len = UART1_RxRealLength;
  if(len > UART1_RxLength) len = UART1_RxLength;    // 防越界
  memcpy(line, UART1_RxBuf, len);
  line[len] = '\0';
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
  //     ROBOT_Move(-50, 390, 100, 100, 100, 100);
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
  // ROBOT_Move(0, 200, 30, 30, 30, 30);
//
  /*// ==================== TCS34725 颜色识别（替换原 1 号灰度传感器 GRAY1） ====================
    // SCL=PB9、SDA=PB4（原 GRAY1 的 CLK/DAT 线位），软件 I2C 100kHz，无需改 CubeMX；
    // 模块供电 3.3V，LED/INT 悬空。GRAY3 仍走串行接口用于循线（GRAY_Data[GRAY3] 依旧可用）。
    // 判色由 TCS34725_ClassifyColor() 完成：只分 黑/非黑 两类（红蓝统称非黑），
    // 串口打印原始 C/R/G/B + HSV，可接 SerialPlot 观察并按实际物体标定阈值（见 tcs34725.h）。 */
  //uint8_t tcs_online = TCS34725_Init();
  //UART1_Printf("TCS34725 %s, ID=0x%02X\r\n",
    //           tcs_online ? "ONLINE" : "OFFLINE", TCS34725_GetID());

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

    //完整走
    //if(UART1_Data[0]==4)

    //红蓝方选择
    while(1){
      /* 菜单界面也能用串口调试指令：发 "7,0,0,0,0,0,0,0" 就地绕圈，
         不用先按 KEY3 进流程。解析/回显/执行都在 UART1_DebugCmd() 里（见文件上方） */
      UART1_DebugCmd();
      OLED_Printf(0, 0, OLED_8X16_HALF, "mode:%s", mode_red?"red ":"blue");
      OLED_Printf(0, 16, OLED_8X16_HALF, "key2:change mode");
      OLED_Printf(0, 32, OLED_8X16_HALF, "key3:next");
      OLED_Printf(0, 48, OLED_8X16_HALF, "key0:jump ZM");
      OLED_Update();
      if(KEY_ONE(KEY2_GPIO_Port, KEY2_Pin)){//按键2--更改模式
        mode_red = !mode_red;
      }
      if(KEY_ONE(KEY3_GPIO_Port, KEY3_Pin)){//按键3--下一步
        break;
      }

      //0和1专用于调试
      if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
        /* 【调试入口】按键0：跳过圆盘机、跳过仓库倒球，直接跳到"正面识别"开头执行
           （就是跳到下面 ZhengMian_Flag = 1 那一行，从那里往下依次是：
             发0xA2给主/副视觉 → 收倒球槽 → 走到阶梯附近 → 左前光电对准 → 清串口残留
             → 正面识别通信流程(5步) → 阶梯抓取 → 立柱 → 回红蓝区）
           ★用法：在"红蓝方选择"界面按 KEY2 先选好红/蓝，再按 KEY0 就从中途开始跑；
             车会先自己走到阶梯附近对准，所以先把车放在圆盘机/仓库方向随便一点的位置即可 */
        UART1_Printf("DEBUG: KEY0 jump to ZhengMian start\r\n");
        OLED_Printf(0, 48, OLED_8X16_HALF, "jump to ZM...");
        OLED_Update();
        //如果跳过了发送红蓝方给视觉的步骤，直接告诉视觉红(0xAA)蓝(0xBB)方
        UART2_Printf("%c", mode_red ? 0xAA : 0xBB);//告诉视觉红(0xAA)蓝(0xBB)方
        goto ZHENGMIAN_START;

        /* ---- 原来的按键0调试(手动把车开到阶梯对准位)已由上面跳转代替，保留备用 ----
        //右+前，移动到阶梯附近,要往右速度快点，不然撞到了
        ROBOT_Move(100, 160, 100, 50, 100, 50);

        //向左慢走，直到前面的两个光电都感应到障碍物
        ROBOT_MoveSpeed(-10, 0);
        //while(LASER_Barrier(LASER2_GPIO_Port, LASER2_Pin)==0);
        while(LASER_Barrier(LASER3_GPIO_Port, LASER3_Pin)==0);//左后是1，右边为2，左前是3


        //往左走一定距离，视觉中能完整看到两个字母（可以省去测距前后校准）
        ROBOT_Move(-50, 0, 10, 10, 10, 10);
        //停车准备识别
        ROBOT_MoveSpeed(0,0);
        HAL_Delay(100);//延时一下提高稳定性
        -------------------------------------------------------------------------- */
      }

      if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){

        OLED_Printf(0, 0, OLED_8X16_HALF, "barrier:%1d", LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin));
        OLED_Update();

        /* ================= 旧代码开始（原来的按键1"倒球调试"，先整段注释保留）=================
           要恢复原来的倒球调试：把本行与下面"旧代码结束"那一行的注释符去掉，并把上面的等待接收测试删掉即可。

        //往后慢退，直到测距测得合适距离（适合倒球的距离）
        UART1_Printf("3");
        ROBOT_MoveSpeed(0, -10);
        while(GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin)>90);//100有点远，距离小于90就推出此循环
        ROBOT_MoveSpeed(0,0);
        
        //定位操作：向左慢平移到左后光电感应到无障碍物
        UART1_Printf("4");
        ROBOT_MoveSpeed(-10, 0);
        while (LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin)==1);
        ROBOT_MoveSpeed(0,0);

        //往右走固定距离（刚到对上仓库的距离）
        ROBOT_Move(10, 0, 10, 0, 10, 0);//20太大，速度100会飘

        runActionGroup(16, 1); 	//这里是倒球动作组
	      delay_ms(2000);
        ================= 旧代码结束 ================= */

      }
    }
    //？后面，左蓝右红
    
      
    /**************圆盘机****************/
  
    YuanPanJi_Flag = 1;//圆盘机开始
    UART2_Printf("%c", mode_red ? 0xAA : 0xBB);//告诉视觉红(0xAA)蓝(0xBB)方

    //路上就先把机械臂举起来
    runActionGroup(1, 1);//不需要延时，因为和出发一起
    
    //先盲走到圆盘机中心+面向
    ROBOT_Move(mode_red?-60:60,417,100,120,100,120);
    HAL_Delay(100);
    UART1_Printf("1");
    mode_red ? ROBOT_Angle(270) : ROBOT_Angle(90);
    UART1_Printf("2");

    
    //向前慢走，直到灰度传感器第三路(探头3)感应到白线
    GRAY_Update();
    ROBOT_MoveSpeed(0, 15);
    while(GRAY_Data[GRAY3][2] == 0)//探头3为0(黑)继续走，读到1(白)即停

    {
      GRAY_Update();
    }
    ROBOT_MoveSpeed(0, 0);

    //往后走一点点
    ROBOT_Move(0, mode_red ? -2 : -2, 0, 10, 0, 10);
    HAL_Delay(100);

    //往后退到可以拍球，要快（不需要后退了，直接灰度校准更准确）
    //ROBOT_Move(0, mode_red ? -11 : -14, 0, 10, 0, 10);//20太多，12擦球

    //走到位后，放到识别状态
    runActionGroup(4, 1);
    delay_ms(1400);
    UART2_Printf("%c", 0xA1);//发0xA1告诉主视觉进入圆盘机识别

    /******************** 圆盘机视觉处理代码 ********************/
    /* 与视觉约定的数据包格式：包头0xA1 | 第一个数据(0x00/0x01/0x02) | 第二个数据x坐标(2字节) | 第三个数据y坐标(2字节) | 包尾0x0B
        触发条件：第二个字节==0x01或0x02，且第三个数(x,2字节)==100~200，且第四个数(y,2字节)==80~160 → 触发动作组1
        结束条件：收到单字节指令0xA6，则退出本while循环 
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
       */

    /* 实测：圆盘8s/圈、12球 → 相邻两球间隔≈667ms；动作组7/10执行400ms。
       冷却窗口须满足 400ms < 冷却 < 667ms（>动作耗时保证上一拍跑完不连拍，
       <球间隔保证不漏下一个球）。取500ms＝已验证过"连拍解决、连续球能拍"的参数，
       与动作组改回400ms保持同一套基线，方便你继续调"偏左"。 */
    const uint32_t YPJ_HIT_COOLDOWN_MS = 300;   /* 冷却窗口：400 < 值 < 667 */
    const uint8_t  YPJ_HIT_LOG = 1;            /* 1=开日志 0=关 */
    uint32_t ypj_last_hit_t   = 0;             /* 上一次真正触发"拍"的时刻(ms) */
    uint32_t ypj_last_frame_t = 0;             /* 上一次收到"有球"帧的时刻(ms)，日志看发帧间隔 */
    uint8_t  ypj_first_skipped = 0;            /* 0=还没跳过第一个球；1=第一个球已跳过，之后正常拍 */


    while(YuanPanJi_Flag == 1){
      if(VISION1_RxFlag){                              // 2号串口（主视觉）DMA收到一帧
        VISION1_RxFlag = 0;                            // 必须立即清零

        /* 唯一退出条件：收到单字节指令0xA6，则退出本while循环 */
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
          ypj_last_frame_t = now;

          if((cam_x >= 10 && cam_x <= 310)      // x为10~300，直接不限制，只要在屏幕内就拍
              && (cam_y >= 10 && cam_y <= 230)){  // y为10~240
            if((now - ypj_last_hit_t) < YPJ_HIT_COOLDOWN_MS){
              /* ===== 冷却期：还是同一个球 / 上一拍动作还没执行完 → 忽略（防连拍关键）===== */
              if(YPJ_HIT_LOG)
                UART1_Printf("SKIP b=%u x=%u y=%u d=%lu\r\n",
                             (unsigned)CAM_Data[1], (unsigned)cam_x, (unsigned)cam_y,
                             (unsigned long)d_last);
            }else{
              /* ===== 冷却已过：这一帧当作"新球"，真正拍一次 ===== */
              ypj_last_hit_t = now;
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
              if(YPJ_HIT_LOG)
                UART1_Printf("HIT  b=%u x=%u y=%u d=%lu\r\n",
                             (unsigned)CAM_Data[1], (unsigned)cam_x, (unsigned)cam_y,
                             (unsigned long)d_last);
              }
            }
          }
        }
      }
    }

    /***************去仓库倒球*************/

    if(YuanPanJi_Flag == 0)//圆盘机结束时
    {

      //退后固定距离
      ROBOT_Move(0,-25,50,50,50,50);
      //收起机械臂
      //重复了runActionGroup(0, 1);
      //向左平行到仓库
      ROBOT_Move(mode_red ? -185 : 192,0,100,0,100,0);
      //转身
      HAL_Delay(100);
      mode_red ? ROBOT_Angle(90) : ROBOT_Angle(270);
      
      //往后慢退，直到测距测得合适距离（适合倒球的距离）
      ROBOT_MoveSpeed(0, -10);
      while(GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin)>85);//100有点远，距离小于90就退此循环，90也远
      ROBOT_MoveSpeed(0,0);
      
      HAL_Delay(100);//延时一下提高稳定性

      //加一次角度校准（这些地方的角度很重要）
      mode_red ? ROBOT_Angle(90) : ROBOT_Angle(270);

      //定位操作：向左慢平移到左后光电感应到无障碍物
      //新：更改激光位置，让它在没对到障碍物时直接就已经是合适的位置，不需要调整
      ROBOT_MoveSpeed(-5, 0);
      while (LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin)==1);
      ROBOT_MoveSpeed(0,0);
      HAL_Delay(100);//延时一下提高稳定性

      //加一次角度校准
      mode_red ? ROBOT_Angle(90) : ROBOT_Angle(270);
      //往右走固定距离（刚到对上仓库的距离）
      //if(mode_red) ROBOT_Move(5, 0, 10, 0, 10, 0);//20太大，速度100会飘，12太远

      runActionGroup(16, 1); 	//这里是倒球动作组
      delay_ms(2000);

ZHENGMIAN_START:            //★按键0调试入口：开机"红蓝方选择"界面按 KEY0，goto 跳到这一行往下执行
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

        //右+前，移动到阶梯附近,要往右多走点，不然撞到了，y160太远了，不利于视觉识别
        ROBOT_Move(100, 150, 100, 50, 100, 50);

        //向左慢走，直到前面的两个光电都感应到障碍物（右边坏了）
        ROBOT_MoveSpeed(-10, 0);
        //while(LASER_Barrier(LASER2_GPIO_Port, LASER2_Pin)==0);
        while(LASER_Barrier(LASER3_GPIO_Port, LASER3_Pin)==0);//左后是1，右边为2，左前是3
        HAL_Delay(100);//延时一下提高稳定性

        //往左走一定距离，视觉中能完整看到两个字母（可以省去测距前后校准）
        //短距离太快速，走的斜斜的，不要100速度，50还算可以
        ROBOT_Move(-35, 0, 50, 0, 50, 0);
        //停车准备识别
        ROBOT_MoveSpeed(0,0);
        HAL_Delay(100);//延时一下提高稳定性

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
        /* 屏幕切到"阶梯"阶段：正面识别拿到的两个字母留在第1行(方便对照)，其余清掉；
           阶梯阶段下面不再刷新屏幕，想加信息就在这几行后面加 */
        OLED_Clear();
        OLED_Printf(0,  0, OLED_8X16_HALF, "JIETI letter %c %c",
                    ZM_Nibble2Char(ZhengMian_Letter[0]), ZM_Nibble2Char(ZhengMian_Letter[1]));
        OLED_Printf(0, 16, OLED_8X16_HALF, "scan & grab...");
        OLED_Update();
        JieTi_ShowBlockNo(0, 8, 0);   //第4行先显示"BLK -/8"，对准第1个坑后就变成 1/8
        
        //先左右再前后，因为激光太近的话，会直接以为是没有障碍物
        //定位操作：向左慢平移到左前光电感应到无障碍物
        ROBOT_MoveSpeed(-10, 0);//这里容易识别到字母上的黑，然后停下来
        //HAL_Delay(1000);//避免路上识别到字母然后停下来
        while(LASER_Barrier(LASER3_GPIO_Port,LASER3_Pin)==1);
        ROBOT_MoveSpeed(0, 0);
        HAL_Delay(100);//延时一下提高稳定性

        //加一次角度校准，否则很歪影响测距（如果是start过来的，那就要改为如下的0）
        ROBOT_Angle(0);
        //正常流程
        //mode_red ? ROBOT_Angle(90) : ROBOT_Angle(270);
        /* 记住这次校好的朝向 = 整个阶梯阶段的目标朝向：后面每次设速都用 JieTi_MoveSpeed()
           把它恢复回去，角度环才会一直按这个朝向纠偏(否则每设一次速就把当前歪掉的朝向当新目标) */
        jieti_keep_yaw = chassis.target_yaw;
        HAL_Delay(1000);//延时一下提高稳定性

        //走近阶梯，走到合适距离（适合识别的距离）
        JieTi_MoveSpeed(0, 10);
        while(GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin)>JIETI_FWD_TARGET_MM);//105mm：100靠太近，120太远(和校准用的是同一个值)
        JieTi_MoveSpeed(0, 0);
        HAL_Delay(100);//延时一下提高稳定性

        //激光校准（激光刚离开时，位置偏右）
        ROBOT_Move(-10, 0, 10, 0, 10, 0);//走多也没事

        //机械臂变成识别状态
        runActionGroup(54, 1);
        HAL_Delay(2000);

        //清掉主视觉残留帧：只认进入对准后的实时坐标，避免拿校准途中收到的旧数据误判/误停
        VISION1_RxFlag = 0;
        VISION1_RxRealLength = 0;
        memset(VISION1_RxBuf, 0, VISION1_RxLength);

        /* 阶梯目标对准逻辑见下方：主视觉A3数据包 0x00→继续右走；0x01→按x对直到|x|<=50停下 */
        UART2_Printf("%c", 0xA3);//发0xA3告诉主视觉进入"阶梯目标对准"阶段(惯例同0xA1圆盘机/0xA2正面识别；若视觉端在收到0xA7后已自行上报A3帧，此行可删)

        //左前光电无障碍物就开始从左往右走，边走边找目标，抓完一个继续抓下一个
        JieTi_MoveSpeed(JIETI_ALIGN_SPEED, 0);   //右扫描起步(原来10太快，容易走歪)

        /* ============ 阶梯连续抓取：8个坑固定位移步进 ============
           数据包(7字节)：A3 | cmd | x低 | x高 | y低 | y高 | 0x0B包尾
           x为像素坐标(16位,低字节在前,0~320)，程序里减160 = 距画面中心偏移(-160~160)；y是0~240(本阶段不用)
           cmd含义由全局变量 JieTi_Grab_Mode 决定：
             Mode=1 cmd==0x00 → 这个坑不用夹；cmd==0x01 → 这个坑要夹
             Mode=2 cmd==0xMN(M=第几个坑1~8，N=低4位1要夹/0不要)，例：0x11=第1个要夹、0x60=第6个不要
           流程(8个坑逐个来)：
             第1个坑：从左往右边走边找，等目标进入对准带(|x|<=50) = 第1个
             第2~8个：先走固定位移(JIETI_STEP_CM)到下一个坑(粗定位)，再用视觉x对准(|x|<=50)
             对准之后(不管夹不夹都已经对准)：要夹就按坑号选动作组夹 + 66回识别状态；不夹直接下一个
           计数：对准第1个=1，之后每走一段固定位移就+1(屏幕第4行显示 "BLK n/8 X±xx")；
                 记满8且第8个处理完 → 阶梯结束(不再看光电)
           夹取动作组：第1~2个(中阶梯)→57；第3~6个(高阶梯)→60；第7~8个(矮阶梯)→63
        */
        {
          /* ===== 8个坑固定位移步进：对准(计数) → 夹/不夹 → 走固定位移 → 下一个 =====
             第1个坑：从左往右边走边找，等目标进入对准带 = 第1个
             第2~8个：先 ROBOT_Move 走 JIETI_STEP_CM 到下一个坑(粗定位)，再用视觉x对准
             对准之后不论夹不夹都已经对准；要夹就按坑号选动作组(57/60/63)+66回识别状态 */
          for(uint8_t idx = 1; idx <= 8; idx++){                  //第1~8个坑
            if(idx > 1){
              /* 到下一个坑：先停稳，再走一段固定位移 */
              JieTi_MoveSpeed(0, 0);
              ROBOT_Move(JIETI_STEP_CM, 0, JIETI_STEP_SPEED, 0, JIETI_STEP_ACC, 0);
              JieTi_FlushVision();                                //丢掉移动途中的旧帧，只用站定后的新坐标
            }

            /* ---- 视觉对准(内部边走边实时测距，前后偏了就停下校准) ---- */
            JieTi_GoAlign(idx == 1);

            /* ---- 对准了就是第 idx 个坑：计数 + 屏幕显示 ---- */
            jieti_blk_now = idx;
            JieTi_ShowBlockNo(idx, 8, jieti_cam_x);

            /* ---- 要不要夹：Mode1 cmd==0x01；Mode2 cmd低4位非0 ---- */
            uint8_t need = (JieTi_Grab_Mode == 1) ? (jieti_cmd == 0x01)
                                                  : ((jieti_cmd & 0x0F) != 0);
            if(need){
              /* 第1~2个→57中阶梯夹；第3~6个→60高阶梯夹；第7~8个→63矮阶梯夹 */
              uint8_t act = (idx <= 2) ? 57 : ((idx <= 6) ? 60 : 63);
              runActionGroup(act, 1);                             //夹取动作组
              delay_ms(7500);                                     //夹取动作约7.5秒(矮阶梯更慢)
              runActionGroup(66, 1);                              //夹完回到识别状态
              delay_ms(3000);                                     //识别动作约3秒
              JieTi_FlushVision();                                //清掉夹取期间滞留的旧帧
            }
          }

        }
        JieTi_MoveSpeed(0, 0);
        HAL_Delay(100);
       
        //单独的机械臂抬起
        runActionGroup(151, 1);
        HAL_Delay(1000);
        
        JieTi_Flag = 0;//阶梯结束，立柱开始
        LiZhu_Flag = 1;//因为中途没去仓库，所以两个状态需要同时切换

        if(LiZhu_Flag == 1)//立柱开始
        {
          //右前光电无障碍物，就开始向左走固定距离（走到阶梯平面中间）
          ROBOT_Move(-45, -35, 50, 50, 50, 50);
          
          ROBOT_Angle(270);
          //立柱转圈：绕柱闭环（测距定半径 + 陀螺仪累计转角 + 径向闭环 KP_R/VY_MAX + 航向前馈/KD_W 修正）
          //★前提：车头已经正对着柱子（函数开头就是静止采测距定参考距离）
          LiZhu_Circle_Run();

          /*
          识别钩子已搬进 LiZhu_Circle_Run() 的绕圈 while 里：
          遇到可以夹的就在那里 break，车停下、收尾照常执行
          */

          //转完一圈，收起机械臂，然后往左转身走到仓库中间倒方块
          ROBOT_Move(-60, 0, 100, 100, 100, 100);
          ROBOT_Angle(90);//车子前面朝右
          ROBOT_Move(0, -138, 50, 50, 50, 50);
          ROBOT_Move(-45, 0, 50, 50, 50, 50);
          UART1_Printf("9");

          //定位操作：向左慢平移到左后光电感应到无障碍物，之后再往右走固定距离（刚到仓库中间的距离）
          ROBOT_MoveSpeed(-20, 0);
          while (LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin)==1);
          ROBOT_MoveSpeed(0,0);

          ROBOT_Move(25, 0, 20, 0, 50, 0);

          //得走远一点才能转身倒方块
          ROBOT_Move(0, 10, 50, 50, 50, 50);
          ROBOT_Angle(270);//车子前面朝左
          UART1_Printf("10");
          /*
            这里放倒方块的代码
          */

          //倒完方块转个身再回家
          ROBOT_Angle(0);
          //往后多走一点，必须保证，前后在左右移动后能进入红色区域
          ROBOT_Move(40, -225, 50, 100, 100, 100);//60，-240能进
          UART1_Printf("11");

    if(1){
    /*先校准左右再校准前后，左右走可能会抖，而且前后比左右的反馈更准
    注意！！！必须先让颜色传感器在左右移动之后一定能进入红/蓝区域，
    即前后距离必须能确保在红/蓝区域内（在哪里无所谓，后面再校准）
    */
    //如果为黑色，匀速往右走，直到传感器进入红/蓝区域
    //去抖：连续3次(约150ms)都读到红/蓝才确认，交界处"红黑红黑"抖动不会误停
    ROBOT_MoveSpeed(10, 0);
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
    ROBOT_Move(11.5, 0, 10, 10, 100, 100);
    
    //往前走，走到颜色传感器一定在黑色区域内（同样连续3次确认）
    ROBOT_MoveSpeed(0, 10);
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
    ROBOT_MoveSpeed(0, -10);
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
    ROBOT_Move(0, -3, 10, 10, 100, 100);
    ROBOT_MoveSpeed(0, 0);
        }
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
      ROBOT_MoveSpeed(10, 0);
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
      ROBOT_Move(12, 0, 10, 10, 100, 100);
      
      //往前走，走到颜色传感器一定在黑色区域内（同样连续3次确认）
      ROBOT_MoveSpeed(0, 10);
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
      ROBOT_MoveSpeed(0, -10);
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
      ROBOT_Move(0, -3, 10, 10, 100, 100);
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
