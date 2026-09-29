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
/* FLAG 结构体定义在 Mycode/chassis.h（main.c 定义变量）*/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* UART1 帧格式 S,A,B,C,D,E,F,G（8 个整数，逗号分隔）*/
#define UART1_DATA_NUM 8                     // 每帧数据个数

/* ==================== ★★ 调试用“起始阶段”选择(KEY0 切、KEY3 开始) ★★ ====================
   新增一个起点：① 加 DBG_START_xxx；② 在 DBG_START_NAME/DESC 表加一项；
   ③ main() 的“调试起点跳转”处加一个 goto；④ 在阶段开头打 xxx_START: 标签。
   默认 ALL 时主流程一条语句不变。 */
#define DBG_START_ALL        0      //完整流程(默认)：圆盘机→倒球→正面识别→阶梯→立柱→倒方块→回家
#define DBG_START_ZHENGMIAN  1      //从“正面识别前”开始(跳过圆盘机+倒球)
#define DBG_START_LIZHU      2      //从“立柱”开始：立柱前校准→绕柱→倒方块→回家
#define DBG_START_HUIJIA     3      //从“回家”开始(红蓝区找色→停进红蓝区)
#define DBG_START_COLORCAL   4      //颜色单独校准(黑→红→蓝→白，跑完回菜单)
#define DBG_START_MAX        DBG_START_COLORCAL  //KEY0 切换上限(=最后一个起点)

/* ==================== ★★ 移动速度三档标准（主流程统一用这三个宏，不再手写数字）★★ ====================
   ① SPD_SHORT_V/A = 40/50  定点走位：有终点的 ROBOT_Move（两轴较大距离 <50cm）；
      位移 d < v²/2a = 16cm 走三角波，峰值 √(a·d)，d≤7cm 时 <20cm/s 可能推不动 ⇒
      现场“走不到位/起步发肉”就把那一行的 max_a 单独加到 100~200。
   ② SPD_AVG_V = 10  匀速靠近/边判边走：等激光、等测距、等灰度/颜色、视觉对准等，
      只给恒速、没有终点；整体调速只改这一个数。
   ③ SPD_LONG_V/A = 120/120  长距跑图（较大距离 ≥50cm；上限 160 = SPEED_TARGET_MAX）。
   ★判断类用 10 而非 40：GY53 测距约 5Hz、判色带 150ms 去抖 ⇒ 条件成立后车还要多走
     速度×0.2~0.35s，慢一点换落点稳定。距离为 0 的轴速度/加速度填什么都一样，填同一组。 */
#define SPD_SHORT_V   40     // ① 定点走位 目标速度 cm/s
#define SPD_SHORT_A   50     // ① 定点走位 加减速 cm/s²
#define SPD_AVG_V     10     // ② 匀速靠近/边判边走 恒速 cm/s
#define SPD_LONG_V   120     // ③ 长距高速 目标速度 cm/s(别超160)
#define SPD_LONG_A   120     // ③ 长距高速 加减速 cm/s^2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FLAG flag;

int16_t data_encoder = 0;

int32_t UART1_Data[UART1_DATA_NUM] = {0};
uint8_t CAM_Data[7] = {0};    // 视觉坐标帧：A1|类型|x低x高|y低y高|0x0B

bool mode_red = true;  // true=红方 false=蓝方

//阶梯夹取模式：1=按序号计数(cmd 0x00不夹/0x01夹)；2=视觉编号(0xMN: M=第几个1~8, N=1夹/0不夹)
uint8_t JieTi_Grab_Mode = 1;   // 现场切换改这里

/* 调试起点的名字/说明(顺序同 DBG_START_xxx)：OLED 第3/4行显示，8x16半高一行最多16字符 */
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

  if(htim == &htim7){ //1ms 定时中断

    // (已废弃) ICM42688 5ms 更新 与 CAR_Control_Loop 10ms 车体控制周期



    //hwt101ct陀螺仪数据更新
    static uint8_t count1 = 0;
    if(flag.hwt101ct && ++count1 >= 5){
      count1 = 0;
      if(HWT101CT_RxFlag){
        HWT101CT_RxFlag = 0;
        HWT101CT_Update();
      }
    }


    //车体20ms控制周期(须与 chassis.h 的 ENCODER_TIME_S=0.020f 同步)
    static uint16_t count2 = 0;
    if(flag.chassis && ++count2 >= 20){
      count2 = 0;
      CHASSIS_Control_Loop();      // 4轮速度环（PID + 编码器 + PWM）
    }

  }
  /* 编码器用清零法，无 htim3/htim4 溢出中断分支 */

}





/* USER CODE END 0 */

/* ================= 正面识别通信显示(本阶段只有这一个函数动屏幕) =================
   一屏四行：y=0  8x16 “ZM 0/3 TO POS”→“ZM 1/3 SEND OK”→“ZM 2/3 LETTER OK”→“ZM DONE 0xA7 OK”
             y=16 “V4 A,B 0xAB OK” 副视觉(UART4)：字母(A~D)+原始字节 / WAIT / BAD
             y=32 “V2 0xA7 OK”     主视觉(UART2)：WAIT / OK / 其它字节 BAD
             y=48 6x8  收帧数 + 串口4接收状态(RX/ERR) + 本阶段秒数(不动=死等在某一步)
   ★状态一变或 100ms 心跳调用一次：函数内四行一次画完再刷新，不会花屏/留旧字。 */
#define ZM_ST_PREP    0      //进度0：还没发0xA2(正在移动到识别位)
#define ZM_ST_SEND    1      //进度1：0xA2已发给主视觉+副视觉
#define ZM_ST_LETTER  2      //进度2：副视觉字母已收→已转发主视觉+已回执副视觉
#define ZM_ST_RECV    3      //进度3：主视觉的0xA7已收到，正面识别结束
#define ZM_RES_WAIT   0      //结果：还没收到
#define ZM_RES_OK     1      //结果：收到有效内容
#define ZM_RES_BAD    2      //结果：收到无效内容

static uint8_t  zm_step     = ZM_ST_PREP;   //通信进度(0=还没发0xA2)
static uint8_t  zm_v4_res   = ZM_RES_WAIT;  //副视觉(串口4)结果状态
static uint8_t  zm_v4_byte  = 0;            //副视觉原始字节
static uint8_t  zm_v4_len   = 0;            //副视觉帧长(≠1 时显示长度)
static uint8_t  zm_v2_res   = ZM_RES_WAIT;  //主视觉(串口2)结果状态
static uint8_t  zm_v2_byte  = 0;            //主视觉原始字节
static uint32_t zm_t0       = 0;            //进入本阶段的时刻(第4行"秒数"用)

/* 副视觉结果(0xAB~0xCD)：高/低 4 位各是一个字母(0xA~0xD)；放文件顶部供 ZM_ShowComm() 读 */
uint8_t ZhengMian_Letter[2] = {0, 0};

/* nibble(0xA~0xD)→字母(A~D)；其它值原样成字符，便于看出收到了什么 */
static char ZM_Nibble2Char(uint8_t nib){
  return (nib >= 0x0A && nib <= 0x0D) ? (char)('A' + nib - 0x0A) : (char)('0' + (nib & 0x0F));
}

static void ZM_ShowComm(int rx4, int rx2){
  /* 整屏清 0~63 行：新串比旧的短时不留旧字，也清掉 y=48 用 8x16 时残留的下半截 */
  OLED_Clear();

  /* 第1行：通信进度 */
  if(zm_step == ZM_ST_PREP)        OLED_Printf(0,  0, OLED_8X16_HALF, "ZM 0/3 TO POS");   //还没发0xA2(移动中)
  else if(zm_step == ZM_ST_SEND)   OLED_Printf(0,  0, OLED_8X16_HALF, "ZM 1/3 SEND OK");
  else if(zm_step == ZM_ST_LETTER) OLED_Printf(0,  0, OLED_8X16_HALF, "ZM 2/3 LETTER OK");
  else                             OLED_Printf(0,  0, OLED_8X16_HALF, "ZM DONE 0xA7 OK");

  /* 第2行：副视觉(UART4)字母结果 */
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

  /* 第4行(6x8)：收帧数+串口4接收状态+本阶段秒数(不动=死等在某一步) */
  OLED_Printf(0, 48, OLED_6X8_HALF, "r4:%d r2:%d U4:%s %02ds",
              rx4, rx2, (huart4.RxState == HAL_UART_STATE_BUSY_RX) ? "RX" : "ERR",
              (int)((HAL_GetTick() - zm_t0) / 1000U % 100U));

  OLED_Update();
}

/* ================= 阶梯阶段参数（固定位移 + 逐坑校准）=================
   本阶段只做：读每坑“夹不夹”(cmd) + 动作组 + 第2~8坑走固定距离 + 每坑都逐坑校准并计数；
   第1坑就在“进阶梯到位”那个点(不走位)，**第1~8坑一律做左右+前后校准，跟这一坑夹不夹无关**；
   朝向：走位前 ROBOT_Angle 转一次“正对阶梯”的绝对角，之后一律直接 ROBOT_MoveSpeed(底盘原生锁当前朝向)。
   ★视觉协议：没目标时也可能发帧(坐标填0)，cmd 只决定夹不夹，不再拿坐标挪车。 */
#define JIETI_IMG_CX          160     //视觉x 0~320，减它=相对画面中心偏移(只用于显示)
#define JIETI_VIS_MS         200U     //等主视觉一帧的超时(ms)：读本坑 cmd 用(帧间隔约20~50ms)
#define JIETI_CMD_WAIT_MS    1000U     //★第1坑读 cmd 的总时限(ms)：按 JIETI_VIS_MS 反复等，到它就收手
                                       //  (第1坑守着“进阶梯到位”点，等帧前没有走位+校准那段余量，一轮 200ms 太紧)
/* ---- “这一帧有没有目标”：没目标时视觉也发帧、坐标填0，只看 cx 会被当成“目标在最左” ---- */
#define JIETI_X_BAD_MAX      1000U    //★原始x>它=无效帧(0xFFFF)；x=0 也算没检测到
#define JIETI_XFIX_MAX_MS    3000U    //★单坑左右校准时限(ms)：坐标一直不满足容差就收手
                                      //  (0=不限时)
/* ---- 前测距目标：①“走近阶梯”到 50mm 附近停，实测是50；②第1~8坑逐坑前后校准(JieTi_Adjust_Y)的目标 ---- */
#define JIETI_FWD_TARGET_MM   50      //★目标前后距离(mm)：既是“走近阶梯”的停止距离，也是逐坑前后校准的目标

/* ---- 8 个坑固定位移：STEP=同阶梯相邻坑间距；CROSS=换阶梯那一步(第2→3、第6→7个坑) ---- */
#define JIETI_STEP_CM         14      //★阶梯内坑间距(cm)：实际8，要给15
#define JIETI_STEP_CROSS_CM  18      //★换阶梯那一步(cm)：实际10，要给16
#define JIETI_STEP_SPEED      (float)SPD_SHORT_V   //走位速度=短距档(要≥20 才压得过起转PWM)
#define JIETI_STEP_ACC        (float)SPD_SHORT_A   //★加速度=短距档：8cm 峰值 √(50×8)=20cm/s 刚好够起转(想一拍就起转可提到100~150)
/* ---- 阶梯跑完去哪儿：1=先去立柱(绕柱→倒方块→回家) 0=直接回家；两段的绝对角都过 Yaw_Abs() ---- */
#define JIETI_GO_LIZHU          1     //★1=阶梯跑完先去立柱   0=阶梯跑完直接回家
/* ---- “等条件”类死等的超时保护：WAIT_WHILE(条件,标识) 最长 WAIT_TIMEOUT_MS(6000ms)，超时打 TIMEOUT 后继续 ---- */
#define WAIT_TIMEOUT_MS     6000U     //“等条件”类循环的最长等待(ms)
#define WAIT_WHILE(cond, tag)                                            \
  do{                                                                    \
    uint32_t _wait_t0 = HAL_GetTick();                                   \
    while(cond){                                                         \
      SERIALPLOT_WheelActualPump();      /* 串口1实时发四轮实际值：死等循环里也要有数据(见 serialplot.c) */ \
      if(HAL_GetTick() - _wait_t0 > WAIT_TIMEOUT_MS){                    \
        UART1_Printf("TIMEOUT: %s\r\n", tag);                            \
        break;                                                           \
      }                                                                  \
    }                                                                    \
  }while(0)

/* ================== 阶梯阶段朝向：用底盘原生锁向（2026-09-28 删掉专用封装）==================
   ROBOT_MoveSpeed() 内部把 target_yaw 置哨兵(YAW_TARGET_NONE) → 控制循环把“调用那一刻的朝向”
   锁成目标角，平移全程角度环照它纠偏 —— 这就是全工程通用的“平时的速度环 + 角度环”用法。
   ★原来本阶段还套了一层“保持锁向”的专用设速封装：每次设完速度把 target_yaw 改回“进阶梯时
     校好的绝对角”，防“本阶段频繁设速、把已经走歪的朝向反复当新目标锁住”。那套封装连同它的
     目标角变量已按用户要求于 2026-09-28 整块删除 —— 阶梯段现在与其他段完全一样：走位前
     ROBOT_Angle 转一次，之后一律直接调 ROBOT_MoveSpeed（哨兵锁当前朝向）。 */

/* ================== ★调试跳转的“角度基准换算”（每个起点一套，红蓝差180°）==================
   正常发车时上电车头朝前，HWT 的 0° 就是“前”；单独测某段时车按该段姿态摆好再上电，
   HWT 的 0° 变成“该段车头方向” ⇒ 绝对角都要过 Yaw_Abs() 换算。
   跳转入口调 SetYawShift(红方姿态角)，蓝方自动 +180°（场地镜像）；红方按“倒完球/立柱起点
   姿态”车头朝右=90。★立柱入口也用 90（不是270）：LIZHU_START 前那句“先转到270”已注释掉，
   写成 270 会让 Yaw_Abs(90)=180°，绕完一圈还要再转半圈。
   正常流程 yaw_shift_deg=0 时 Yaw_Abs() 原样返回，行为一个字不变。 */
static uint16_t yaw_shift_deg = 0;   //0=不换算；否则=摆车姿态相对“车头朝前”转过的角度(°)
static uint32_t Yaw_Abs(uint32_t normal_angle){   //把“正常基准角”换算成本次该转的角度
  if(yaw_shift_deg == 0) return normal_angle;
  return (normal_angle + (360U - yaw_shift_deg)) % 360U;
}
/* 角度基准统一入口：红方传红方摆车姿态角，蓝方自动 +180°(场地镜像) */
static void SetYawShift(uint16_t red_shift_deg){
  yaw_shift_deg = (uint16_t)((red_shift_deg + (mode_red ? 0U : 180U)) % 360U);
  UART1_Printf("DEBUG: yaw base shift = %u deg (mode %s)\r\n",
               (unsigned)yaw_shift_deg, mode_red ? "RED" : "BLUE");
}

/* ---- 阶梯阶段运行时状态(视觉/显示共用) ---- */
static int16_t  jieti_cam_x     = 0;       //最近一帧：目标距画面中心偏移(负=偏左 正=偏右)
static uint16_t jieti_cam_px    = 0;       //最近一帧原始x(0~320)：判“有没有目标”用
static uint8_t  jieti_cmd       = 0;       //最近一帧：cmd(要不要夹)
static uint8_t  jieti_blk_now   = 0;       //当前第几个坑(1~8，0=还没对准第一个)

/* 显示当前第几个坑+最近一帧视觉x：只占第4行(6x8)，不动第1/2/3行 */
static void JieTi_ShowBlockNo(uint8_t no, uint8_t total, int16_t cam_x){
  OLED_ClearArea(0, 48, 128, 8);                                  //只清第4行
  if(no >= 1 && no <= total){
    /* X+12/X-30 = 目标离画面中心多远 */
    OLED_Printf(0, 48, OLED_6X8_HALF, "BLK %u/%u X%+4d", (unsigned)no, (unsigned)total, (int)cam_x);
  }else{
    OLED_Printf(0, 48, OLED_6X8_HALF, "BLK -/%u", (unsigned)total);  //还没对准第一个坑
  }
  OLED_Update();
}

/* ================= 阶梯阶段：主视觉(UART2)每来一帧就把原样字节+解析结果打到串口1 =================
   RX2 #12 len=7: A3 01 20 00 40 00 0B | A3 cmd=0x01 x=32 y=64 cx=-128
   帧里没有完整 A3..0x0B 时前半段照样打，后面跟 “ | no A3”；JIETI_VIS_LOG=0 关。 */
#define JIETI_VIS_LOG         1        //1=开 0=关
#define JIETI_VIS_LOG_BYTES   8        //每条日志最多原样打几个字节

static uint32_t jieti_vis_cnt = 0;     //本阶段累计收帧数

/* 处理主视觉一帧：先原样转发串口1，再找 A3..0x0B 包，解析 cmd/x/y 存 jieti_cmd/jieti_cam_x；返回1=有合法包 */
static uint8_t JieTi_VisionPoll(void){
  if(!VISION1_RxFlag) return 0;
  VISION1_RxFlag = 0;
  uint8_t len = VISION1_RxRealLength;
  jieti_vis_cnt++;

  if(JIETI_VIS_LOG){
    UART1_Printf("RX2 #%u len=%u:", (unsigned)jieti_vis_cnt, (unsigned)len);
    for(uint8_t k = 0; k < len && k < JIETI_VIS_LOG_BYTES; k++)
      UART1_Printf(" %02X", VISION1_RxBuf[k]);
  }

  for(uint8_t i = 0; i + 7 <= len; i++){
    if(VISION1_RxBuf[i] == 0xA3 && VISION1_RxBuf[i + 6] == 0x0B){
      uint16_t px = (uint16_t)VISION1_RxBuf[i + 2]
                  | ((uint16_t)VISION1_RxBuf[i + 3] << 8);          //x像素(低字节在前，0~320)
      uint16_t py = (uint16_t)VISION1_RxBuf[i + 4]
                  | ((uint16_t)VISION1_RxBuf[i + 5] << 8);          //y像素(本阶段不用)
      /* ★不做值域过滤(真坐标原样收)；原始 x 另存 jieti_cam_px，对准逻辑用它区分“有目标/没检测到” */
      jieti_cam_x  = (int16_t)px - JIETI_IMG_CX;        //减160 → 目标距画面中心偏移
      jieti_cam_px = px;                                //原始x(0~320)：判定有没有目标
      jieti_cmd   = VISION1_RxBuf[i + 1];               //cmd(要不要夹，含义看 JieTi_Grab_Mode)
      if(JIETI_VIS_LOG)
        UART1_Printf(" | A3 cmd=0x%02X x=%u y=%u cx=%d\r\n",
                     (unsigned)VISION1_RxBuf[i + 1], (unsigned)px, (unsigned)py,
                     (int)jieti_cam_x);
      return 1;
    }
  }
  if(JIETI_VIS_LOG) UART1_Printf(" | no A3\r\n");      //不是A3包/帧不完整也打出来，便于查“没反应”
  return 0;
}

/* 清残留帧：只认之后的实时坐标(走完固定位移后/夹取动作后调用) */
static void JieTi_FlushVision(void){
  VISION1_RxFlag = 0;
  VISION1_RxRealLength = 0;
  memset(VISION1_RxBuf, 0, VISION1_RxLength);
}

/* 取一帧 A3 包(超时返回0)，结果存 jieti_cam_x/jieti_cmd；每帧已由 JieTi_VisionPoll 转发串口1 */
static uint8_t JieTi_GetVision(uint32_t wait_ms){
  uint32_t t0 = HAL_GetTick();
  while((HAL_GetTick() - t0) < wait_ms){
    if(JieTi_VisionPoll()) return 1;     //这一帧有A3包：已转发+已更新 jieti_cam_x/jieti_cmd
  }
  return 0;
}

/*前后测距校准(JieTi_Adjust_Y)*/
static void JieTi_Adjust_Y(int Target_Y_Distance){
  while(1){
                uint16_t dis = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
                //校准容差：x 给 30 像素，y 给 20mm
                if(dis > (Target_Y_Distance + 5)){
                  for(uint8_t i = 0; i < 5; i++)
                  {
                    ROBOT_MoveSpeed(0, 20);
                    HAL_Delay(15);
                    ROBOT_MoveSpeed(0, 0);
                    HAL_Delay(25);

                  }
                }
                else if(dis  < (Target_Y_Distance - 5)){
                  for(uint8_t i = 0; i < 5; i++)
                  {
                    ROBOT_MoveSpeed(0, -20);
                    HAL_Delay(15);
                    ROBOT_MoveSpeed(0, 0);
                    HAL_Delay(25);
                  }
                }
                else break;
            }
}

/* 阶梯左右校准(JieTi_Adjust_X)：按视觉 x 偏移对准，容差±30 像素
   ★没东西就不动(2026-09-27 加)：① 等不到帧 → 不校；② 帧在但原始 x=0/超量程 → 也不校
     （否则 x=0 → cx=-160 会被当成“目标在最左”，把没物块的坑一路往左带）
   ★JIETI_XFIX_MAX_MS(默认3s) 兜底收手；每轮调整量 = 5 次脉冲，直接 ROBOT_MoveSpeed(锁当前朝向)。 */
static void JieTi_Adjust_X(void){
  uint32_t t0 = HAL_GetTick();
  while(1){
                if(!JieTi_GetVision(JIETI_VIS_MS)){                 //等不到这一坑的帧
                  UART1_Printf("JX skip: no frame (no target)\r\n");
                  break;                                            //视野里没目标 → 不校，去下一个坑
                }
                if(jieti_cam_px == 0 || jieti_cam_px > JIETI_X_BAD_MAX){   //帧在，但坐标是"没检测到"的填充值
                  UART1_Printf("JX skip: x=%u bad (no target)\r\n", (unsigned)jieti_cam_px);
                  break;                                            //同样不校(别把填充值当"目标在左边")
                }
                int16_t cx = jieti_cam_x;
                if(cx > 15){
                  for(uint8_t i = 0; i < 5; i++)
                  {
                    ROBOT_MoveSpeed(30, 0);
                    HAL_Delay(15);
                    ROBOT_MoveSpeed(0, 0);
                    HAL_Delay(25);

                  }
                }
                else if(cx < -15){
                  for(uint8_t i = 0; i < 5; i++)
                  {
                    ROBOT_MoveSpeed(-30, 0);
                    HAL_Delay(15);
                    ROBOT_MoveSpeed(0, 0);
                    HAL_Delay(25);
                  }
                }
                else break;                                          //进容差带：对准完成
#if (JIETI_XFIX_MAX_MS > 0)
                if(HAL_GetTick() - t0 >= JIETI_XFIX_MAX_MS){           //兜底：到点收手，别再往一边挪
                  UART1_Printf("JX stop: timeout %ums cx=%d\r\n", (unsigned)JIETI_XFIX_MAX_MS, (int)cx);
                  break;
                }
#endif
            }
}

/* 2026-09-21：删掉旧的左右对准/脉冲走一步/第3行测距显示，改为固定距离走位 + 底盘锁向 */

/* ★逐坑校准（**第1~8坑都做**：第1坑守着“进阶梯到位”点做，第2~8坑走完固定距离后做；
   顺序固定：先左右 JieTi_Adjust_X，再前后 JieTi_Adjust_Y）：
   X 用视觉 x(容差±30像素)、Y 用前测距(容差±20mm)；都是 while(1)，5 次脉冲=一次调整量；
   没物块/坐标无效时不校(左右那步内部跳过)；★校准与“这一坑夹不夹”无关：不夹也先校准再读 cmd。 */
/* ==================== 立柱阶段：视觉通信（主视觉/串口2）====================
   与“正面识别(0xA2)/阶梯(0xA3)”同一套通信，只换字节：
     ① 我→视觉(UART2) 0xA4：进立柱阶段、开始识别（开圈前发一次，同“到位才发0xA2”）
     ② 视觉→我：A4 | cmd | x低 | x高 | y低 | y高 | 0x0B（7字节，布局同 A3 包）
        cmd = 0x00 不用拍 / ≠0 要拍（判据只在 LiZhu_Circle_Run 的钩子里那一行）
     ③ 我→视觉(UART2) 0xA9：绕完一整圈、本阶段结束
   流程：清残留+重启串口2 → 发 0xA4 → 边绕边收包 → cmd≠0 停车 → 动作组107 → 等 LIZHU_PIC_MS
     → 清残留 → 接着绕；绕满 355° 停车 → 发 0xA9
   ★2026-09-27 实测视觉回包包头还是 0xA3，故 LIZHU_ACCEPT_A3=1 时 A3 包一样触发停车拍；
     确认视觉只发 A4 后置 0 即恢复只认 A4。
   日志(串口1)：每帧一行 “RX2 #12 len=7: A3 01 ...”，每停一次拍一行 “PIC #1 cmd=.. yaw=..”。
   屏幕第4行 “LZ rx481 n481 p0” = 收帧数 / 没解析出包的帧数 / 已拍次数 */
#define LIZHU_VIS_LOG          1        //串口1：1=每帧一行 0=不打
#define LIZHU_VIS_LOG_NOA4     1        //    1=每帧都打 0=只打能解析出包的帧(≈20帧/s 防刷屏)
#define LIZHU_VIS_LOG_BYTES    8        //    每行最多原样打几个字节
#define LIZHU_VIS_SNAP_BYTES   32       //    每帧先抄进局部数组再解析/打印(为何要抄见 LiZhu_VisionPoll)
#define LIZHU_DIST_LOG         0        //串口1：1=绕圈每200ms打一行测距；0=不打
#define LIZHU_VIS_HEAD         0xA4     //我→视觉：进立柱阶段（视觉回包包头也用它）
#define LIZHU_VIS_END          0xA9     //我→视觉：立柱结束
#define LIZHU_ACCEPT_A3        1        //1=也认阶梯的 0xA3 包(实测视觉还发A3)；
                                        //  0=只认 A4 包
#define LIZHU_PIC_MS           2500U    //跑完动作组107后等它做完的时间(ms)
#define LIZHU_PIC_COOLDOWN_MS  1000U    //两次"停拍"的最短间隔(ms)：防同一目标连拍；0=不防
#define LIZHU_VIS_HEAL_MS      100U     //串口2自愈+屏幕刷新的节拍(ms)

/* ==================== ★★ 2026-09-29 新增：绕完一圈后的“原地夹取”任务 ★★ ====================
   流程（就在绕完一圈的那个位置，先不走位、不转向；实现在 LiZhu_Circle_Run() 收尾调用的
   LiZhu_PostCircleTask() 里 ⇒ “正式流程的立柱段”和“菜单里单独发指令7 / 单键测试绕圈”两边都会跑）：
     ① 动作组110 → 等 LIZHU_POST_ACT_MS；② 视觉识别（要不要夹）→ 需要就 动作组113 + 等 LIZHU_GRAB_ACT_MS；
     ③ 无论夹不夹都 动作组160 复位 + 等 LIZHU_RESET_ACT_MS → 发 0xA9 收尾本阶段。
   下面的等待时间常量就是按现场标定的动作时长填的，动作组本身在舵机板上跑，这里只是等它跑完。 */
#define LIZHU_POST_VIS        1         //1=绕完后再发一次 0xA4 让主视觉继续识别“要不要夹”
                                        //  (绕圈末尾 LiZhu_Circle_Run 已发 0xA9 结束上一轮，不重发视觉可能不再发包)
                                        //  0=不重发握手(仍会等一帧实时包，收不到就按“不夹”处理)
#define LIZHU_GRAB_VIS_MS    1000U      //绕完后等“要不要夹”这一帧包的总时限(ms)：超时/没帧=不夹
#define LIZHU_POST_ACT_MS    2500U      //绕完后：动作组110(摆到识别位)的等待时间(ms)
#define LIZHU_GRAB_ACT_MS    6500U      //绕完后：动作组113(夹取)的等待时间(ms)
#define LIZHU_RESET_ACT_MS   4000U      //绕完后：动作组160(复位)的等待时间(ms)★夹不夹都要复位

static uint32_t lizhu_vis_cnt = 0;      //本阶段累计收帧数
static uint32_t lizhu_pic_cnt = 0;      //本阶段停下来拍了几次
static uint8_t  lizhu_pic_cmd = 0;      //最近一个包的 cmd(0=不用拍 / ≠0=要拍)
static uint16_t lizhu_pic_x   = 0;      //最近一包的 x(只进日志)
static uint16_t lizhu_pic_y   = 0;
static uint32_t lizhu_pic_t   = 0;      //上一次"停拍"的时刻(冷却计时用)
static uint32_t lizhu_heal_t  = 0;      //串口2自愈 / 屏幕刷新的计时
static uint32_t lizhu_nopkt_cnt = 0;    //收到帧但没解析出包的次数

/* 只在第4行(6x8)刷状态：OLED 刷新约几 ms，会拖慢绕圈 10ms 节拍 */
static void LiZhu_ShowComm(void){
  OLED_ClearArea(0, 48, 128, 8);
  OLED_Printf(0, 48, OLED_6X8_HALF, "LZ rx%u n%u p%u",
              (unsigned)lizhu_vis_cnt, (unsigned)lizhu_nopkt_cnt, (unsigned)lizhu_pic_cnt);
  OLED_Update();
}

/* 清残留帧(发0xA4前/拍完都清)：只认之后的实时帧，防旧包重复触发 */
static void LiZhu_FlushVision(void){
  VISION1_RxFlag = 0;
  VISION1_RxRealLength = 0;
  memset(VISION1_RxBuf, 0, VISION1_RxLength);
}

/* 串口2自愈：出错停/DMA关了就重新拉起来 */
static void LiZhu_VisionHeal(void){
  if((huart2.RxState != HAL_UART_STATE_BUSY_RX) || ((hdma_usart2_rx.Instance->CR & DMA_SxCR_EN) == 0U)){
    HAL_UART_AbortReceive(&huart2);
    UART2_RxFlag = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RxBuf, UART2_RxLength);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }
}

/* 合法包头：0xA4=立柱包；LIZHU_ACCEPT_A3=1 时 0xA3 也算 */
static uint8_t LiZhu_IsHead(uint8_t b){
  return (uint8_t)((b == LIZHU_VIS_HEAD) || (LIZHU_ACCEPT_A3 && b == 0xA3));
}

/* 处理主视觉一帧：原样打一行日志 + 找包(A4|cmd|x低x高|y低y高|0x0B，也有3字节简包)；返回1=有合法包 */
static uint8_t LiZhu_VisionPoll(void){
  if(!VISION1_RxFlag) return 0;
  VISION1_RxFlag = 0;
  /* ★先把这一帧“抄一份”再解析/打印(两者同源)：收帧回调已把 DMA 重新武装到同一个 RxBuf，
     视觉约20帧/s 一直在发，“解析(µs)”到“日志打完(约4ms)”之间缓冲就可能被覆盖；
     不抄会出现“原始字节与解析结果来自两帧”的假故障(实测 535 帧里 5~6 行)。 */
  uint8_t buf[LIZHU_VIS_SNAP_BYTES];
  uint8_t len = VISION1_RxRealLength;
  if(len > (uint8_t)LIZHU_VIS_SNAP_BYTES) len = (uint8_t)LIZHU_VIS_SNAP_BYTES;  //超长帧只取前32字节(包都在7字节内)
  memcpy(buf, VISION1_RxBuf, len);
  lizhu_vis_cnt++;

  uint8_t hit = 0, head = 0;
  for(uint8_t i = 0; i + 2 <= len; i++){
    if(!LiZhu_IsHead(buf[i])) continue;
    if(i + 7 <= len && buf[i + 6] == 0x0B){            //7字节标准包
      head = buf[i];
      lizhu_pic_cmd = buf[i + 1];
      lizhu_pic_x   = (uint16_t)buf[i + 2] | ((uint16_t)buf[i + 3] << 8);
      lizhu_pic_y   = (uint16_t)buf[i + 4] | ((uint16_t)buf[i + 5] << 8);
      hit = 1;
    }else if(i + 3 <= len && buf[i + 2] == 0x0B){      //3字节简包(只报要不要拍、不带坐标)
      head = buf[i];
      lizhu_pic_cmd = buf[i + 1];
      lizhu_pic_x   = 0;
      lizhu_pic_y   = 0;
      hit = 1;
    }
    if(hit) break;
  }

  if(LIZHU_VIS_LOG && (LIZHU_VIS_LOG_NOA4 || hit)){    //日志：原样字节+解析结果
    UART1_Printf("RX2 #%u len=%u:", (unsigned)lizhu_vis_cnt, (unsigned)len);
    for(uint8_t k = 0; k < len && k < LIZHU_VIS_LOG_BYTES; k++)
      UART1_Printf(" %02X", buf[k]);
    if(hit)
      UART1_Printf(" | 0x%02X cmd=0x%02X x=%u y=%u\r\n", (unsigned)head, (unsigned)lizhu_pic_cmd,
                   (unsigned)lizhu_pic_x, (unsigned)lizhu_pic_y);
    else
      UART1_Printf(" | no pkt\r\n");
  }

  if(!hit) lizhu_nopkt_cnt++;
  return hit;
}

/* 清残留帧 + 重启串口2接收（两个入口共用：进立柱 LiZhu_VisionStart / 绕完后重新识别 LiZhu_VisionReopen） */
static void LiZhu_RxReopen(void){
  LiZhu_FlushVision();                                 //清掉上一阶段/上一轮留下的旧帧
  HAL_UART_AbortReceive(&huart2);                      //★强制复位重启，保证串口2真的在听
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RxBuf, UART2_RxLength);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

/* 立柱阶段开始：清残留 → 重启串口2接收 → 发 0xA4 → 计数复位 */
static void LiZhu_VisionStart(void){
  LiZhu_RxReopen();

  lizhu_vis_cnt   = 0;
  lizhu_nopkt_cnt = 0;
  lizhu_pic_cnt = 0;
  lizhu_pic_cmd = 0;
  lizhu_pic_x   = 0;
  lizhu_pic_y   = 0;
  lizhu_pic_t   = HAL_GetTick() - LIZHU_PIC_COOLDOWN_MS;   //第一拍不受冷却限制
  lizhu_heal_t  = HAL_GetTick();

  UART2_Printf("%c", LIZHU_VIS_HEAD);                  //发 0xA4：告诉主视觉进入立柱阶段、开始识别
  UART1_Printf("LZ TX 0xA4 -> V1(UART2)\r\n");
  LiZhu_ShowComm();
}

/* ★2026-09-29：绕完一圈后的“再识别”握手。绕圈末尾 LiZhu_VisionStop 已发过 0xA9 结束上一轮，
   这里再发一次 0xA4 请主视觉继续认“柱子这边要不要夹”；★各计数不清零，日志/屏幕仍是本阶段累计值。
   ★只在 LIZHU_POST_VIS=1 时编译（关掉握手就不需要这个函数，免得 -Wall 报“定义了没用”） */
#if LIZHU_POST_VIS
static void LiZhu_VisionReopen(void){
  LiZhu_RxReopen();
  lizhu_pic_cmd = 0;                                       //丢掉旧包的 cmd，别拿绕圈时的结论当这一轮的
  lizhu_pic_t   = HAL_GetTick() - LIZHU_PIC_COOLDOWN_MS;

  UART2_Printf("%c", LIZHU_VIS_HEAD);                      //再发 0xA4
  UART1_Printf("LZ TX 0xA4 -> V1(UART2) again (post-circle)\r\n");
  LiZhu_ShowComm();
}
#endif  /* LIZHU_POST_VIS */

/* ★2026-09-29：绕完一圈后判“要不要夹”：清残留 → 在 wait_ms 内等一帧包 → cmd≠0 才算要夹。
   判据与绕圈钩子里那一行完全一致(见 LiZhu_Circle_Run)；★等不到帧/超时/包都是 cmd=0 → 返回 0(不夹，照样复位)。
   等待期间顺带做串口2自愈(与绕圈里同一套)，视觉掉线也不会把这里卡死。 */
static uint8_t LiZhu_NeedGrab(uint32_t wait_ms){
  lizhu_pic_cmd = 0;
  LiZhu_FlushVision();                                     //只认“发完 0xA4 之后”的新帧

  uint32_t t0 = HAL_GetTick();
  while((HAL_GetTick() - t0) < wait_ms){
    if(LiZhu_VisionPoll()){                                //收到一帧合法包(已更新 lizhu_pic_cmd)
      if(lizhu_pic_cmd != 0U) return 1;                    //cmd≠0 → 需要夹
      /* cmd==0：这一帧说的是“不用夹”，继续在剩余时限里等下一帧 */
    }
    LiZhu_VisionHeal();                                    //串口2出错停了就重新拉起来
  }
  return 0;
}

/* ★2026-09-29：绕完一圈后的“原地夹取”任务（★正式流程的立柱段 与 菜单里单独发指令7 都走这里）
   就在绕完停下来的那个位置原地做，不插任何走位/转向：
     ① 动作组110 摆到识别位 → 等 LIZHU_POST_ACT_MS(2500ms)；
     ② 视觉识别“要不要夹”：再发一次 0xA4(LIZHU_POST_VIS=1 时；绕圈结束后主视觉已收到 0xA9，
        不重新握手可能不再发包) → 在 LIZHU_GRAB_VIS_MS(1000ms) 内等一帧包，
        cmd≠0 = 需要夹(判据与绕圈钩子里那个 107 完全相同)；等不到/超时/包是 cmd=0 → 按不夹处理；
     ③ 需要夹 → 动作组113 夹取 → 等 LIZHU_GRAB_ACT_MS(6500ms)；不需要夹 → 直接跳过；
     ④ ★无论夹不夹都跑动作组160 复位 → 等 LIZHU_RESET_ACT_MS(4s)。
   串口1 每次打一行 “LZ post-circle need=0/1”。要改时长/时限就改上面那几个 LIZHU_* 常量。 */
static void LiZhu_PostCircleTask(void){
  runActionGroup(110, 1);                                 //① 摆到识别位
  HAL_Delay(LIZHU_POST_ACT_MS);

#if LIZHU_POST_VIS
  LiZhu_VisionReopen();                                   //② 再发 0xA4：请主视觉继续识别
#endif
  {
    uint8_t lz_need = LiZhu_NeedGrab(LIZHU_GRAB_VIS_MS);  //   等一帧包判“要夹/不夹”
    UART1_Printf("LZ post-circle need=%u\r\n", (unsigned)lz_need);
    if(lz_need){
      runActionGroup(113, 1);                             //③ 夹取
      HAL_Delay(LIZHU_GRAB_ACT_MS);
    }
  }

  runActionGroup(160, 1);                                 //④ 复位(夹不夹都做)
  HAL_Delay(LIZHU_RESET_ACT_MS);
}

/* 立柱结束：发 0xA9(绕完一圈发；丢测距提前退出也发) */
static void LiZhu_VisionStop(void){
  UART2_Printf("%c", LIZHU_VIS_END);
  UART1_Printf("LZ TX 0xA9 -> V1(UART2)\r\n");
  LiZhu_ShowComm();
}


/* ==================== 立柱转圈：绕柱（开环三旋钮 + 两路可选反馈）====================
   原理：车头一直指着柱子、车身横着走 ⇒ 轨迹天然是以柱为圆心的圆，r = V_TAN/(W_TURN×π/180)。
   本车 r 由几何定死：测距 17 + 传感器到车心 14 + 柱半径 4 = 35cm。
     W_TURN = V_TAN/r×57.3 ⇒ r=35cm 时 W_TURN = 1.64×V_TAN（V_TAN=10→16.4°/s，整圈约22s）；
     反算 r = 57.3×V_TAN/W_TURN。★闭环就填这个几何值：半径被测距压到目标后平衡点要求 w=V_TAN/r；
     纯开环实测平均半径小 13%(靠柱前轮跑不满)，按实测比例凑 W_TURN 只是临时手段。
   要调的只有函数开头几行：① V_TAN / W_TURN / V_RAD；② FB_DIST / FB_LASER；③ LAS_YAW / LAS_TAN；
     其余 DEFAULT 段是定死的几何常数，不用动。
   两路反馈(可单独开关)：FB_DIST 调径向速度 v_y(车头正对柱子 ⇒ 测距就是半径，满幅3cm → V_RAD cm/s)；
     FB_LASER 调车头摆速 w(左4右2 平行打柱，一个有一个没有 = 横向偏了)。
     ★全写成“速度”而不是“误差×增益”：增益一顶限幅就变成开关环(9.25 实测 1.96s 周期呼吸)。
   怎么调(一次只动一个值)：e 绕一圈一直在同一符号变大 → 改 W_TURN；
     e 绕一圈正好摆一次(最低点 yaw≈180) → 开环固有摆动，改 W_TURN 没用，交给 FB_DIST；
     V_RAD = 半径环阻尼 ζ ≈ V_RAD/1.7(V_TAN=10)：1.7≈临界、3≈过阻尼(先用它)，别超 V_TAN 一半；
     激光抖 → LAS_YAW 调小，偏了不回来 → 调大；两路都开互相打架就先关一路。
   ★靠柱那对前轮天生慢(0.18×V_TAN)：绕圈几何决定，不是故障；V_TAN<8 才真推不动。
   串口每200ms一行：d=测距mm e=半径误差mm(正=远) l4/r2=左右激光 vx/vy=切向/径向(0.1cm/s)
                     w=角速度×100 yaw=已绕角度°(不涨=卡住；355° 该停) */

static void LiZhu_Circle_Run(void)
{
  if(!flag.chassis){                          // 前置条件：底盘控制循环在跑，否则车不会动、while 会一直空转
    UART1_Printf("no chassis!\r\n");
    return;
  }
  UART1_Printf("circle start\r\n");

  /* ===== ① 三个可调参数（开环基本圆 + 反馈强度都在这三行）===== */
  const float V_TAN  = 10.0f;               // 切向速度 cm/s(>0 逆时针 <0 顺时针)
  const float W_TURN = 16.4f;               // 车头摆速 °/s(只填正的，大小由上面公式算)
  const float V_RAD  = 3.0f;                // 径向速度 cm/s(仅 FB_DIST=1 有效：路线像椭圆→加；一冲一停→减)
  /* ★W_TURN = V_TAN/r×57.3，r 由几何定死(见函数头) */
  /* r 由几何定死：车心到柱轴 = 前测距(立柱校准停在 170mm) + 传感器到车心 14 + 柱半径 4 = 35cm */

  /* ===== ② 两个反馈开关（0=关 1=开；都置0 = 只有三个旋钮的纯开环圆）===== */
  const uint8_t FB_DIST  = 1;               // 测距反馈：测距偏大→往前靠、偏小→往后退（调 v_y）
  const uint8_t FB_LASER = 1;               // 双激光反馈：一个有一个没有→横向偏了(调 w)

  /* ===== ③ 激光反馈参数（仅 FB_LASER=1 用）===== */
  const float LAS_YAW = 4.0f;               // 偏了时额外加的车头摆速 °/s（纠偏主力；方向反了取负）
  const float LAS_TAN = -2.0f;               // 额外切向速度 cm/s(默认不用；想试改成 1~3，别超 V_TAN 一半)

  /* ===== DEFAULT：定死常数（不用调，理由都写在这）===== */
  const float RAD_FULL_CM = 3.0f;           // 测距反馈满幅误差：≥3cm 都按满幅算（斜率=V_RAD/3cm，写死）
  const float RAD_DEAD_CM = 0.3f;           // 测距反馈死区 ±3mm
  const float D_ALPHA     = 0.5f;           // 测距一阶低通系数（压掉 GY-53 的 ±5~10mm 抖动）
  const float GY53_2_OFFSET_CM = 14.0f;     // 前测距(GY53_2)到车心纵向距离 cm(几何常数)
  const float PIPE_RADIUS_CM   = 4.0f;      // 柱子半径 4cm(外径8；几何常数)

  /* ★靠柱那对前轮天生很慢(0.18×V_TAN)：绕圈几何决定，不是故障；V_TAN<8 才真推不动 */
  float v_tan  = V_TAN;                                         // 切向速度 cm/s（= 旋钮值，不做任何隐藏修改）
  float dir    = (v_tan < 0.0f) ? -1.0f : 1.0f;                 // 绕向：+1 逆时针 / -1 顺时针（由 V_TAN 符号定）
  float w_base = dir * W_TURN * 0.0174532925f;                  // 开环基准角速度 rad/s（w>0 逆时针）
  float r_knob = (W_TURN > 0.05f) ? fabsf(V_TAN) / (W_TURN * 0.0174532925f) : 999.0f;  // 旋钮隐含半径 cm(串口对照用)

  /* ===== 测距定参考：开圈前静止采 12 次，只收有效值(8~22cm)取中值当目标半径 ⇒ 起步 e≈0，不会先往柱里冲；FB_DIST=0 时只用于串口显示 ===== */
  uint16_t d_ok[10];                            // 有效采样缓存
  uint8_t  n = 0;                               // 有效采样个数
  for(uint8_t i = 0; i < 12; i++){              // 最多采12次
    uint16_t dd = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
    if(dd >= 80 && dd <= 220){                  // 只收 8~22cm(丢目标返回2000/杂散直接丢)
      d_ok[n++] = dd;
      if(n >= 10) break;
    }
    HAL_Delay(30);                              // 采样间隔，避开噪声
  }
  UART1_Printf("valid=%d\r\n", n);
  uint8_t have_ref = (n >= 3) ? 1 : 0;          // 有没有真实的半径参考（FB_DIST=0 时只影响打印）
  if(n < 3){                                    // 有效采样太少
    if(FB_DIST){                                // 测距反馈开着 → 没有半径参考，绝不乱转
      UART1_Printf("no pipe! (dist fb needs it)\r\n");
      return;
    }
    UART1_Printf("no pipe; open-loop anyway\r\n");   // 纯开环不需要测距，照跑（e 只作显示）
    d_ok[0] = 180; n = 1;                       // 显示用名义值 180mm(e 的零点随便，看趋势)
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
  /* ★半径对照：车心到柱轴 = 测距+14+4 = 35cm —— 这行直接告诉你本圈 W_TURN 该填多少 */
  if(have_ref){
    float r_tgt = d_ref_cm + GY53_2_OFFSET_CM + PIPE_RADIUS_CM;
    UART1_Printf("ref=%dmm -> r_tgt=%dcm -> W_TURN_ideal=%d (x0.1deg/s; now=%d)\r\n",
                 d_ref, (int)r_tgt, (int)(v_tan / r_tgt * 572.9578f), (int)(W_TURN * 10.0f));
  }else{
    UART1_Printf("ref=? (no valid reading) -> e 与 W_TURN 的绝对值没意义，只看趋势\r\n");
  }
  if(FB_LASER) UART1_Printf("laser: LAS_YAW=%d deg/s LAS_TAN=%d cm/s\r\n",
                            (int)LAS_YAW, (int)LAS_TAN);

  /* ===== 接管底盘：角度环让位 + 手动设速标志 ===== */
  flag.angle = 0;
  chassis.v_x = 0.0f;  chassis.v_y = 0.0f;  chassis.w = 0.0f;
  chassis.x_speed_plan_flag = 0;
  chassis.y_speed_plan_flag = 0;
  chassis.x_set_speed_flag  = 1;
  chassis.y_set_speed_flag  = 1;

  float yaw_last = HWT101CT_Data.yaw;           // 起点朝向（此时车头正对柱子）
  float yaw_acc  = 0.0f;                        // 陀螺仪累积转角°=已绕角度(车头一直跟着柱子转)
  uint8_t  lost      = 0;                       // 连续无效测距计数
  uint8_t  lost_stop = 0;                       // 丢目标保护停车标志
  uint32_t t_prt     = HAL_GetTick();           // 打印节拍

  /* ===== 视觉通信开始：清残留→重启串口2接收→发 0xA4（放这里是因为“静止采测距定参考”要先跑完，同“到位才发0xA2”）===== */
  LiZhu_VisionStart();

  while(fabsf(yaw_acc) < 355.0f){           // 绕满一整圈
    /* ===== 读一次测距(显示+测距反馈都用)：有效窗口 80~220mm，丢目标/杂散保持上次值；只有 FB_DIST=1 才做连续20次保护停车；杂散靠“固定斜率+V_RAD封顶”消化 ===== */
    uint16_t dd = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
    if(dd >= 80 && dd <= 220){
      d_cm = d_cm + D_ALPHA * ((float)dd / 10.0f - d_cm);   // 一阶低通(几十 ms 滞后，可忽略)
      lost = 0;
    }else if(FB_DIST && ++lost >= 20){
      lost_stop = 1; break;
    }
    /* ★读数出窗口时 d_cm 保持上次值不更新：串口看到“d=80 卡住、l4/r2 在闪”是读数被丢了，不是半径稳住了 */
    float e = d_cm - d_ref_cm;                              // 半径误差 cm（正 = 离柱子比目标远）

    /* ===== 读两个激光(仅串口显示；纠不纠由 FB_LASER 决定)：正对柱子应一直 1 1 ===== */
    uint8_t l4 = LASER_Barrier(LASER4_GPIO_Port, LASER4_Pin);   // 左激光（有障碍=1）
    uint8_t r2 = LASER_Barrier(LASER2_GPIO_Port, LASER2_Pin);

    /* ===== ① 基准：开环圆（三个旋钮）===== */
    float vx = v_tan;                                       // 切向：横向沿圈走(圆心在车头正前方)
    float vy = 0.0f;                                        // 径向：默认不动
    float w  = w_base;                                      // 车头摆速：开环基准(r=V_TAN/W_TURN 靠它)

    /* ===== ② 测距反馈：偏大→往前靠、偏小→往后退(调 v_y)；±3mm 死区内不动 ===== */
    if(FB_DIST){
      float u = e / RAD_FULL_CM;                            // 归一化：±1 封顶 = 满幅
      if(u >  1.0f)      u =  1.0f;
      else if(u < -1.0f) u = -1.0f;
      if(fabsf(e) < RAD_DEAD_CM) u = 0.0f;
      vy = V_RAD * u;                                       // 满幅 V_RAD cm/s(斜率写死 V_RAD/3cm)
    }

    /* ===== ③ 双激光反馈：左4右2 平行打柱(间距≈柱半径，柱宽8cm) =====
             · 1 1 → 正对，不纠；0 0 → 偏了6cm以上/丢失，这一拍不纠
             · 左4有右2无 → 柱偏在车头轴线左边 → 车头往左摆(w 加正)；反之往右摆
           ★纠偏方向只跟“柱子在左还是右”有关，与绕向无关；切向那一路只有 LAS_TAN≠0 才用 */
    if(FB_LASER){
      int8_t s = 0;                                         // +1 = 柱子偏左
      if(l4 && !r2)      s = +1;
      else if(r2 && !l4) s = -1;
      w  += (float)s * LAS_YAW * 0.0174532925f;             // 车头纠偏（默认走这一路）
      vx += (float)s * LAS_TAN;                             // 切向纠偏(默认不用)
    }

    /* ===== 下发：三个速度解耦，互不干涉 ===== */
    if(w >  YAW_PID_OUT_MAX) w =  YAW_PID_OUT_MAX;          // 摆速不超角度环限幅
    else if(w < -YAW_PID_OUT_MAX) w = -YAW_PID_OUT_MAX;
    chassis.v_x = vx;
    chassis.v_y = vy;
    chassis.w   = w;

    /* 测距打印(每200ms，LIZHU_DIST_LOG=1 才打)：d=测距mm e=误差mm l4/r2=激光 vx/vy=速度 w yaw° */
    if(LIZHU_DIST_LOG && HAL_GetTick() - t_prt >= 200){
      UART1_Printf("d=%d e=%d l4=%d r2=%d vx=%d vy=%d w=%d yaw=%d\r\n",
                   (int)(d_cm * 10.0f), (int)(e * 10.0f), l4, r2,
                   (int)(vx * 10.0f), (int)(vy * 10.0f),
                   (int)(w * 100.0f), (int)fabsf(yaw_acc));
      t_prt = HAL_GetTick();
    }

    /* ===== 识别钩子：边绕边收包 → cmd≠0 就停车拍一张(够不够格当包由 LiZhu_VisionPoll 判)===== */
    if(LiZhu_VisionPoll()){
      /* 判据就这一行：cmd≠0=要拍；冷却 LIZHU_PIC_COOLDOWN_MS 内再来也不停(防同一目标连拍) */
      if(lizhu_pic_cmd != 0U && (HAL_GetTick() - lizhu_pic_t) >= LIZHU_PIC_COOLDOWN_MS){
        chassis.v_x = 0.0f;  chassis.v_y = 0.0f;  chassis.w = 0.0f;   //① 停车
        HAL_Delay(100);                                               //   等速度环把车刹稳(≈5个控制周期)

        runActionGroup(107, 1);                                       //② 拍：动作组 107
        HAL_Delay(LIZHU_PIC_MS);                                      //   等它跑完(按现场标定改 LIZHU_PIC_MS)
        //   ★若 107 跑完不自带回“识别状态”，放开这行: runActionGroup(104, 1); HAL_Delay(2500);

        LiZhu_FlushVision();                                          //③ 拍期间滞留的旧帧全清
        yaw_last = HWT101CT_Data.yaw;                                 //   拍时车没走：这段陀螺仪抖动不算进“已绕角度”
        lizhu_pic_t = HAL_GetTick();
        lizhu_pic_cnt++;
        UART1_Printf("PIC #%u cmd=0x%02X x=%u y=%u yaw=%d\r\n",
                     (unsigned)lizhu_pic_cnt, (unsigned)lizhu_pic_cmd,
                     (unsigned)lizhu_pic_x, (unsigned)lizhu_pic_y, (int)fabsf(yaw_acc));
        lizhu_pic_cmd = 0;
        t_prt = HAL_GetTick();
        LiZhu_ShowComm();                                             //   第4行刷新"拍了几次"
      }
    }

    /* ===== 串口2自愈 + 屏幕心跳（100ms 一次）===== */
    if(HAL_GetTick() - lizhu_heal_t >= LIZHU_VIS_HEAL_MS){
      lizhu_heal_t = HAL_GetTick();
      LiZhu_VisionHeal();
    }

    /* 用陀螺仪累积转角判断已绕角度 */
    float ddg = HWT101CT_Data.yaw - yaw_last;
    yaw_last = HWT101CT_Data.yaw;
    if(ddg > 180.0f)       ddg -= 360.0f;
    else if(ddg < -180.0f) ddg += 360.0f;
    yaw_acc += ddg;

    HAL_Delay(10);                          // 本循环节拍(读数都是阻塞读)
  }
  /* 停车 + 恢复角度环（重新锁向当前朝向） */
  chassis.v_x = 0.0f;  chassis.v_y = 0.0f;  chassis.w = 0.0f;
  chassis.x_set_speed_flag = 0;
  chassis.y_set_speed_flag = 0;
  flag.angle = 1;
  chassis.target_yaw = YAW_TARGET_NONE;
  if(lost_stop) UART1_Printf("LOST! stop\r\n");
  else          UART1_Printf("circle done (yaw=%d)\r\n", (int)fabsf(yaw_acc));

  /* ★2026-09-29：绕完一圈后的“原地夹取”任务(110 → 视觉识别 → 需要就113 → 160复位)就挂在这里：
     所以【正式流程的立柱段】和【菜单里单独发指令 7 / 单键测试】调的都是同一段代码，两边都有这一段。
     它必须在 LiZhu_VisionStop()(发 0xA9) 之前跑，因为那里面要视觉继续认“要不要夹”。
     要单测夹取动作本身，就把 LIZHU_POST_VIS 设 0(不重发 0xA4，仍会等一帧实时包)或临时注释这一行。 */
  LiZhu_PostCircleTask();

  LiZhu_VisionStop();                        //★发 0xA9 通知主视觉(本轮识别结束；丢测距提前退出也发)
}



/* ==================== ★★ 串口1“单键传感器单独测试” ★★ ====================
   发一个字母 = 持续打印那一路，再发一次关(车停在菜单界面就能看)：
     c=颜色 TCS34725(原始RGBC+HSV+判色)   g=双测距 GY53(front/back，2000=超量程)
     l=双激光(L4/R2/L3，1=有障碍)          e=四轮编码器(脉冲+里程计位置/速度)
     y=陀螺仪(yaw/目标角/环开关)           r=灰度 GRAY3 八路(1=白 0=黑)
     m=颜色采样一次(打一行 CAL，标定判色阈值用)  i=全传感器快照一次
   实现：本块只有“六路打印 + 泵 + 开关的公共写法”，开关在 UART1_DebugCmd 的单键分支。
     ★泵 = 非阻塞 + 各自按 DBG_xxx_MS 节流，调用点频繁调即可，不占控制周期；
       调用点：主循环开头 + “红蓝方选择”菜单循环(紧跟 SERIALPLOT_WheelActualPump)。
     ★故意不放进 robot.c 的阻塞等待循环里：单次 GY53 最坏 75ms、双激光各最长 5ms，塞进
       “等激光/等颜色”这类循环会把停车时机拖后几厘米 —— 调试输出不能改变被调对象的行为。
       想在跑动时也看 e/y，可把 DBG_SensorLogPump() 加到 robot.c 那三处泵旁边(只读寄存器，代价小)。
     ★本块不动 OLED：屏幕已被菜单/流程占用，传感器数值一律走串口1。 */
#define DBG_COL_MS   200U   /* 颜色   打印间隔(ms)：一次 I2C 读≈1~2ms，200ms 足够看稳定值 */
#define DBG_DIS_MS   300U   /* 双测距 打印间隔(ms)：GY53 本身约 5Hz(200ms) 才更新一次 */
#define DBG_LAS_MS   200U   /* 双激光 打印间隔(ms)：单次读取最长 5ms(内部消抖) */
#define DBG_ENC_MS   100U   /* 编码器 打印间隔(ms)：只读寄存器，几乎不占时间 */
#define DBG_YAW_MS   100U   /* 陀螺仪 打印间隔(ms)：数据由串口3中断刷新，这里纯打印 */
#define DBG_GRAY_MS  200U   /* 灰度   打印间隔(ms)：8 位串行读 ≈1ms */

static uint8_t  dbg_on_col  = 0, dbg_on_dis  = 0, dbg_on_las  = 0;
static uint8_t  dbg_on_enc  = 0, dbg_on_yaw  = 0, dbg_on_gray = 0;
static uint32_t dbg_t_col   = 0, dbg_t_dis   = 0, dbg_t_las   = 0;
static uint32_t dbg_t_enc   = 0, dbg_t_yaw   = 0, dbg_t_gray  = 0;

/* --- 六路打印：只读传感器/底盘状态，绝不改控制量 --- */
static void DBG_PrintColor(void)          /* 颜色：原始 RGBC + H/S/V + 判色结果 */
{
  TCS34725_RGBC rgbc;
  if(!TCS34725_GetRawData(&rgbc) || rgbc.c == 0){
    UART1_Printf("COL read fail (check VCC=3.3V / SCL=PB9 / SDA=PB4)\r\n");
    return;
  }
  UART1_Printf("COL C=%5u R=%5u G=%5u B=%5u H=%6.1f S=%.3f V=%.3f -> %s\r\n",
               (unsigned)rgbc.c, (unsigned)rgbc.r, (unsigned)rgbc.g, (unsigned)rgbc.b,
               (double)rgbc.h, (double)rgbc.s, (double)rgbc.v,
               TCS34725_ColorName(TCS34725_ClassifyColor(&rgbc)));
}

static void DBG_PrintDist(void)           /* 双测距：front=前(GY53_2) / back=后(GY53_1)，单位 mm */
{
  uint16_t d_f = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
  uint16_t d_b = GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin);
  UART1_Printf("DIS front=%4umm back=%4umm\r\n", (unsigned)d_f, (unsigned)d_b);
}

static void DBG_PrintLaser(void)          /* 双激光：1=有障碍(L3 是备用那一路，一起看一眼) */
{
  UART1_Printf("LAS L4=%u R2=%u L3=%u\r\n",
               (unsigned)LASER_Barrier(LASER4_GPIO_Port, LASER4_Pin),
               (unsigned)LASER_Barrier(LASER2_GPIO_Port, LASER2_Pin),
               (unsigned)LASER_Barrier(LASER3_GPIO_Port, LASER3_Pin));
}

static void DBG_PrintEnc(void)            /* 四轮编码器：★非破坏读 CNT（不清零，速度环照常读走） */
{
  UART1_Printf("ENC LF=%4d LB=%4d RB=%4d RF=%4d pos=(%+.1f,%+.1f) v=(%+.1f,%+.1f)\r\n",
               (int)(int16_t)__HAL_TIM_GET_COUNTER(&htim2), (int)(int16_t)__HAL_TIM_GET_COUNTER(&htim3),
               (int)(int16_t)__HAL_TIM_GET_COUNTER(&htim4), (int)(int16_t)__HAL_TIM_GET_COUNTER(&htim5),
               (double)chassis.pos_x, (double)chassis.pos_y,
               (double)chassis.now_v_x, (double)chassis.now_v_y);
}

static void DBG_PrintYaw(void)            /* 陀螺仪：实测 yaw / 航向环目标角 / 航向环开关 */
{
  UART1_Printf("YAW act=%.2f tgt=%.2f angle_loop=%u\r\n",
               (double)HWT101CT_Data.yaw, (double)chassis.target_yaw, (unsigned)flag.angle);
}

static void DBG_PrintGray(void)           /* 灰度 GRAY3：探头1~8，"1"=浅(白) "0"=深(黑) */
{
  GRAY3_Serial_Update();
  UART1_Printf("GRAY3 %u%u%u%u%u%u%u%u\r\n",
               (unsigned)(GRAY_Data[GRAY3][0] & 1U), (unsigned)(GRAY_Data[GRAY3][1] & 1U),
               (unsigned)(GRAY_Data[GRAY3][2] & 1U), (unsigned)(GRAY_Data[GRAY3][3] & 1U),
               (unsigned)(GRAY_Data[GRAY3][4] & 1U), (unsigned)(GRAY_Data[GRAY3][5] & 1U),
               (unsigned)(GRAY_Data[GRAY3][6] & 1U), (unsigned)(GRAY_Data[GRAY3][7] & 1U));
}
/* --- 六路打印函数结束（下面接 泵 / 开关公共写法 / 快照 / 颜色采样）--- */

/* 单键开关公共写法：翻转标志 + 报 ON/OFF + 打开时立刻来一行 */
static void DBG_ToggleLog(uint8_t *on, uint32_t *t, void (*print_f)(void), const char *name, uint32_t ms)
{
  *on = (uint8_t)(!(*on));
  *t  = HAL_GetTick();
  UART1_Printf("%s log %s (%ums/line)\r\n", name, *on ? "ON" : "OFF", (unsigned)ms);
  if(*on) print_f();
}

/* 节流泵：非阻塞，谁开着、谁到点就打一行(主循环/菜单循环里频繁调) */
static void DBG_SensorLogPump(void)
{
  uint32_t now = HAL_GetTick();
  if(dbg_on_col  && (uint32_t)(now - dbg_t_col ) >= DBG_COL_MS ){ dbg_t_col  = now; DBG_PrintColor(); }
  if(dbg_on_dis  && (uint32_t)(now - dbg_t_dis ) >= DBG_DIS_MS ){ dbg_t_dis  = now; DBG_PrintDist();  }
  if(dbg_on_las  && (uint32_t)(now - dbg_t_las ) >= DBG_LAS_MS ){ dbg_t_las  = now; DBG_PrintLaser(); }
  if(dbg_on_enc  && (uint32_t)(now - dbg_t_enc ) >= DBG_ENC_MS ){ dbg_t_enc  = now; DBG_PrintEnc();   }
  if(dbg_on_yaw  && (uint32_t)(now - dbg_t_yaw ) >= DBG_YAW_MS ){ dbg_t_yaw  = now; DBG_PrintYaw();   }
  if(dbg_on_gray && (uint32_t)(now - dbg_t_gray) >= DBG_GRAY_MS){ dbg_t_gray = now; DBG_PrintGray();  }
}

/* “i”：全传感器快照一次(六路各一行) */
static void DBG_SensorSnapshot(void)
{
  DBG_PrintColor(); DBG_PrintDist(); DBG_PrintLaser();
  DBG_PrintEnc();   DBG_PrintYaw();  DBG_PrintGray();
}

/* “m”：颜色采样一次，打一行 CAL(字段同 KEY0 的 CAL 模式)，用来改 tcs34725.h 的判色阈值 */
static void DBG_PrintColorCal(void)
{
  TCS34725_RGBC rgbc;
  if(!TCS34725_GetRawData(&rgbc) || rgbc.c == 0){
    UART1_Printf("CAL READ FAIL (C=0): check VCC=3.3V / SCL=PB9 / SDA=PB4\r\n");
    return;
  }
  UART1_Printf("CAL C=%5u R=%5u G=%5u B=%5u H=%6.1f S=%.3f V=%.3f -> %s\r\n",
               (unsigned)rgbc.c, (unsigned)rgbc.r, (unsigned)rgbc.g, (unsigned)rgbc.b,
               (double)rgbc.h, (double)rgbc.s, (double)rgbc.v,
               TCS34725_ColorName(TCS34725_ClassifyColor(&rgbc)));
}



/* ==================== 串口1调试指令：解析 + 回显 + 就地执行（菜单界面也能用）====================
   ★为什么要提成函数：车平时停在下面“红蓝方选择”菜单里，主循环体不往下走，菜单里没人解析串口 ⇒
     发指令毫无反应；主循环与菜单循环各调一次本函数后，停在哪都能立刻生效。
   帧格式：“S,A,B,C,D,E,F,G”(逗号分隔 8 个整数，串口1发时带换行)，S = UART1_Data[0]：
     3 = 走固定距离 + 转到指定角度(带耗时)；5 = 原地转到任意角；7 = 立柱绕圈 Looping_Circle()；
     9 = 舵机动作组(9,组号,次数；组号0=停止所有)
   返回 1 = 本帧已被处理(3/5/7/9 都跑完动作才返回)；0 = 没收到帧或不认识的命令号(只回显)
   要改绕圈参数就去改 LiZhu_Circle_Run() 开头那几个 const，别改散落的地方。
   ------------------------------------------------------------------
   单键(整帧只有1个字符)：w/s/a/d=前后左右移动(持续，x 停)、W/S/A/D=同上前进1秒自动停、
     x/空格/回车=停车、+/-=测试速度±10cm/s(默认30，10~100)、o=里程清零、p=状态；
     c/g/l/e/y/r/v=传感器日志开关、m=颜色采样一次、i=全传感器快照、h/?=帮助表。
   ★原地转角不在单键里(单键 1/2/3/4 已删)：用 5,角度,0,... 或 at f 角度。
   ★函数体里的 ①②③ 是“帧的三种类型”，与这里的键分组不是一回事。 */
static uint8_t UART1_DebugCmd(void){
  /* ★单键手动测试状态(对应下面 ① 单键分支，静态变量在两次调用间保持) */
  static uint32_t cmd_stop_at = 0;      /* 定时自动停车的时刻（大写 W/S/A/D = 走 1 秒自动停） */
  static float    cmd_spd     = 30.0f;  /* 单键测试速度 cm/s（+/- 每档 10，范围 10~100） */
  if(cmd_stop_at != 0 && (int32_t)(HAL_GetTick() - cmd_stop_at) >= 0){
    cmd_stop_at = 0;
    ROBOT_MoveSpeed(0.0f, 0.0f);
    UART1_Printf("AUTO STOP (1s)\r\n");
  }
  if(!UART1_RxFlag) return 0;
  UART1_RxFlag = 0;
  char line[UART1_RxLength + 1];                    // 拷贝一份(DMA缓冲末尾无结束符，需自己补0)
  uint16_t len = UART1_RxRealLength;
  if(len > UART1_RxLength) len = UART1_RxLength;
  memcpy(line, UART1_RxBuf, len);
  line[len] = '\0';

  /* ==================== 帧的三种类型 ====================
       ① 单键 = 去掉首尾空白/换行后只剩 1 个字符(含传感器测试键 c/g/l/e/y/r/m/i)
       ② 调参 = 有空格、没有逗号 → “名字 类型 数值”，交给 serialplot.c 的 SERIALPLOT_ChangeParam
       ③ 数值 = 有逗号 → “S,A,B,C,D,E,F,G”，按 UART1_Data[0] 分发(3/5/7/9)
       ★为什么不再只按“有没有逗号”分：“vx f 30” 会被单键分支截胡、SerialPlot 调参一条都进不来。
       ★单键只做“设恒速/开关日志”：移动都走 ROBOT_MoveSpeed()，它锁住发指令那一刻的朝向；
         底盘速度 <20cm/s 角度环不介入，≥20cm/s 才纠偏 ⇒ 想验“纠偏灵不灵”就用 ≥20cm/s 走。
       ★原地转角用 5,角度,0,... 或 at f 角度。 */
  /* 先判帧类型：去掉首尾空白/换行后只剩 1 个字符才算单键 */
  char    *kc_s = line;
  while(*kc_s == ' ' || *kc_s == '\t') kc_s++;
  uint16_t kc_n = 0;
  while(kc_s[kc_n] != '\0' && kc_s[kc_n] != '\r' && kc_s[kc_n] != '\n') kc_n++;
  while(kc_n > 0 && (kc_s[kc_n-1] == ' ' || kc_s[kc_n-1] == '\t')) kc_n--;

  if(kc_n <= 1){
    char    c     = (kc_n == 1) ? kc_s[0] : 'x';   /* 纯回车/空格 → 按停车处理 */
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
      case 'o': chassis.pos_x = 0.0f; chassis.pos_y = 0.0f;
                chassis.dist_acc_x = 0.0f; chassis.dist_acc_y = 0.0f;
                UART1_Printf("odometry cleared (pos=0,0)\r\n"); return 1;
      case 'p': break;                          /* 只打印状态，见下面统一打印 */
      case 'v':                                 /* 串口1实时发四轮实际值 开/关(默认关，见 serialplot.h 的 WHEEL_ACT_SEND_EN)；打开后是纯数字流，接 SerialPlot 看4条曲线 */
        serialplot_wheel_on = !serialplot_wheel_on;
        UART1_Printf("wheel actual log %s (%u ms/line)\r\n",
                     serialplot_wheel_on ? "ON" : "OFF", (unsigned)WHEEL_ACT_SEND_MS);
        return 1;

      /* ---- 传感器单测键(c/g/l/e/y/r 开关，m/i 打一次)，实现在上方 DBG_ 块 ---- */
      case 'c': DBG_ToggleLog(&dbg_on_col,  &dbg_t_col,  DBG_PrintColor, "COL  ", DBG_COL_MS);  return 1;
      case 'g': DBG_ToggleLog(&dbg_on_dis,  &dbg_t_dis,  DBG_PrintDist,  "DIS  ", DBG_DIS_MS);  return 1;
      case 'l': DBG_ToggleLog(&dbg_on_las,  &dbg_t_las,  DBG_PrintLaser, "LAS  ", DBG_LAS_MS);  return 1;
      case 'e': DBG_ToggleLog(&dbg_on_enc,  &dbg_t_enc,  DBG_PrintEnc,   "ENC  ", DBG_ENC_MS);  return 1;
      case 'y': DBG_ToggleLog(&dbg_on_yaw,  &dbg_t_yaw,  DBG_PrintYaw,   "YAW  ", DBG_YAW_MS);  return 1;
      case 'r': DBG_ToggleLog(&dbg_on_gray, &dbg_t_gray, DBG_PrintGray,  "GRAY3", DBG_GRAY_MS); return 1;
      case 'm': DBG_PrintColorCal();     return 1;   /* 颜色采样一次(打一行 CAL，标定阈值用) */
      case 'i': DBG_SensorSnapshot();    return 1;   /* 全传感器快照一次(六路各一行) */

      case 'h': case '?':                       /* 帮助：三类指令一张表 */
        UART1_Printf("== 单键(整帧1个字符) ==========================================\r\n");
        UART1_Printf("动 w/s/a/d=前/后/左/右(持续)  W/S/A/D=只走1秒  x/空格/回车=停\r\n");
        UART1_Printf("+/-=测试速度  o=里程清零  p=状态(yaw/速度/里程)  转角用 5,角度 或 at f 角度\r\n");
        UART1_Printf("测 v=四轮实际值  c=颜色  g=双测距  l=双激光  e=编码器  y=陀螺仪  r=灰度\r\n");
        UART1_Printf("   c/g/l/e/y/r 再发一次=关   m=颜色采样一次   i=全传感器快照一次\r\n");
        UART1_Printf("== 数值(含逗号) S,A,B,C,D,E,F,G ================================\r\n");
        UART1_Printf("3,dx,dy,vx,vy,ax,ay,ang   走dx/dy(cm)再原地转到ang度(阻塞,走完才回)\r\n");
        UART1_Printf("5,ang                     原地转到任意角, 打印耗时ms\r\n");
        UART1_Printf("7,0,...                   立柱绕圈(参数在 LiZhu_Circle_Run 开头)\r\n");
        UART1_Printf("9,组号,次数,0,...         舵机动作组(组号0=停止; 次数0按1次处理)\r\n");
        UART1_Printf("== 调参(空格分隔, 来自 SerialPlot 指令区) ======================\r\n");
        UART1_Printf("vx/vy=手动速度  mx/my=走固定距离cm  at=转到角度  mv/mvacc=规划速度/加速\r\n");
        UART1_Printf("ang=航向环开关  target=目标角  w=整车角速度  pq=查参数  go/gb=长距直行\r\n");
        UART1_Printf("skp/ski/skd=四轮统一  kp1~4/ki1~4/kd1~4=单轮  tspd/tkp/tki/tkd=临时单轮(以上均为增量式)\r\n");
        UART1_Printf("pkp1~4/pki1~4/pkd1~4=位置式单轮 ★上电默认走位置式  pomax/pomin/pimax/psep/pidz/pkip=位置式限制\r\n");
        UART1_Printf("pmode i 1/0=位置式/增量式切换  ptgt f 40=四轮定速40  pauto i 0=退出定速\r\n");
        UART1_Printf("ykp1~3/yki1~3/ykd1~3/ybias1~3=航向环三档(1旋转 2低速平移 3高速平移)\r\n");
        UART1_Printf("用法：\"名字 f 数值\"（浮点用 f，整数用 i），如 vx f 30 / ykp2 f -0.02；pq f 0 查当前值\r\n");
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

  /* ==================== ② 调参指令：有空格、没有逗号（“名字 类型 数值”）====================
       交给 serialplot.c 的 SERIALPLOT_ChangeParam（发 “pq” 可列出全部参数）：
         vx/vy 手动速度 · mx/my 走距 · at 转角 · mv/mvacc 规划速度/加速度 · target 目标角 · ang 航向环
         skp/ski/skd 四轮统一 · kp1~4 单轮 · yk* 航向环 · go/gb 长距直行 · pkp/pki/pkd1~4 位置式单轮
         pomax/pomin/pimax/psep/pidz/pkip 位置式限制 · pmode i 1/0 切位置式(★上电默认) · ptgt/pauto 定速
       ★2026-09-26 修掉“单键截胡”后才真正能用（以前 “vx f 30” 只会被单键分支当成四轮日志开关）；
         ★只在主循环/菜单循环里解析：车在跑阻塞动作时发的调参要等那段跑完才生效。 */
  if(strchr(line, ',') == NULL){
    for(char *q = line; *q != '\0'; q++){            /* 去掉尾部回车/换行(只为回显好读) */
      if(*q == '\r' || *q == '\n'){ *q = '\0'; break; }
    }
    UART1_Printf("PARAM %s\r\n", line);              /* 回显：能看到这行=指令收到了 */
    SERIALPLOT_ChangeParam(line);
    return 1;
  }

  uint8_t i = 0;
  char *p = strtok(line, ",");
  while(p && i < UART1_DATA_NUM){                   // 转成32位整数(支持负数)
    UART1_Data[i++] = (int32_t)strtol(p, NULL, 10);
    p = strtok(NULL, ",");
  }
  /* 回显：能看到这条=指令收到并解析出来了 */
  UART1_Printf("S=%d A=%d B=%d C=%d D=%d E=%d F=%d G=%d\r\n",
               UART1_Data[0], UART1_Data[1], UART1_Data[2], UART1_Data[3],
               UART1_Data[4], UART1_Data[5], UART1_Data[6], UART1_Data[7]);

  /* ==================== ③ 数值指令：S = UART1_Data[0] ====================
       ★3/5/9 全部收进本函数（以前 3 只在主循环里判，车停在菜单发它毫无反应）；
         7 保持原样，调的是本函数上方的 LiZhu_Circle_Run。 */
  if(UART1_Data[0] == 3 || UART1_Data[0] == 5 || UART1_Data[0] == 9){
    int32_t cmd = UART1_Data[0];
    UART1_Data[0] = 0;                      /* 立即清指令，防止上层重复触发 */

    if(cmd == 3){                           /* 3,dx,dy,vx,vy,ax,ay,ang：走 dx/dy(cm) 再原地转到 ang 度 */
      UART1_Printf("MOVE dx=%d dy=%d -> then ANG %d\r\n",
                   (int)UART1_Data[1], (int)UART1_Data[2], (int)UART1_Data[7]);
      ROBOT_Move(UART1_Data[1], UART1_Data[2], UART1_Data[3], UART1_Data[4], UART1_Data[5], UART1_Data[6]);
      /* ★角度先挡越界：ROBOT_Angle 形参是 uint32_t，发 -90 会变成 42.9 亿 → 车永远转不停 */
      if(UART1_Data[7] < 0 || UART1_Data[7] > 360){
        UART1_Printf("ANG skip (range 0~360, got %d)\r\n", (int)UART1_Data[7]);
      }else{
        ROBOT_Angle((uint32_t)UART1_Data[7]);
      }
      UART1_Printf("MOVE+ANG done\r\n");
    }else if(cmd == 5){                     /* 5,ang,0,...：原地转到任意角 + 打印耗时（不用打空格版） */
      int32_t ang = UART1_Data[1];
      if(ang < 0 || ang > 360){
        UART1_Printf("ANG range 0~360, got %d\r\n", (int)ang);
      }else{
        uint32_t ang_t0 = HAL_GetTick();
        ROBOT_Angle((uint32_t)ang);
        UART1_Printf("ANG %d ok, %dms\r\n", (int)ang, (int)(HAL_GetTick() - ang_t0));
      }
    }else{                                  /* 9,组号,次数,0,...：舵机动作组 */
      int32_t grp = UART1_Data[1], times = UART1_Data[2];
      if(grp <= 0){
        stopActionGroup();
        UART1_Printf("ACT stop\r\n");
      }else{
        if(times <= 0) times = 1;           /* 协议里 0=无限循环，这里强制 1 次，防卡住出不来 */
        runActionGroup((uint8_t)grp, (uint16_t)times);
        UART1_Printf("ACT run group %d x%d\r\n", (int)grp, (int)times);
      }
    }
    return 1;
  }

  if(UART1_Data[0]==7){                             // 立柱绕圈(见上方 LiZhu_Circle_Run)
    UART1_Data[0] = 0;
    UART1_Printf("lizhu circle start\r\n");
    /* 直接调立柱阶段那个函数(和立柱阶段同一套代码，方便先单独测)：
           开圈发 0xA4 → 边绕边收包(A3 也算) → “要拍”就停车跑动作组 107 → 绕完发 0xA9；
           ★菜单里发 7 会连立柱视觉通信一起跑，单测时记得把视觉接上。 */
    LiZhu_Circle_Run();
    UART1_Printf("lizhu circle done\r\n");
    return 1;
  }
  return 0;                                         // 其它命令号：只回显，不执行
}

/* ==================== 传感器触发后的停车（2026-09-20 新底盘） ====================
   老底盘的 SENSOR_BRAKE()(反向速度 + 临时改 start_margin 造柔性力矩) 整段删除后，停车就一句
   ROBOT_MoveSpeed(0, 0)：速度环闭环，target=0 而四轮还在转时当拍就反向修正(≈主动反接刹车)，
   比断电滑行刹得快，也不用补固定位移；停稳后由底盘的“静止零漂保护”清零断电。
   ★2026-09-28：原来把这一句包了一层的停车封装已删除（只多一层嵌套、没别的动作），用到的地方
     直接写 ROBOT_MoveSpeed(0, 0)；紧跟着的 ROBOT_Angle 会阻塞到“航向到位且四轮停稳”。
   ★停车点位置不对就改那条固定位移。 */


/* ==================== ★★ 颜色传感器(TCS34725)校准模式（KEY0 选 CAL + KEY3 进入）★★ ====================
   只做一件事：黑/红/蓝/白 四种颜色各采 5 次，原始值+H/S/V 打到串口1；拿回来按实测数据重写
   tcs34725.h 的三个判色阈值（★不写 Flash、不改运行时逻辑，阈值永远是头文件里的编译期宏）。
   按键：K1=切颜色(黑→红→蓝→白，换色时计数清零)  K2=采一次(打一行 C/R/G/B+H/S/V+判色结果，屏幕同步)
         K3=返回菜单
   用法：颜色放好 → K2 采 5 次 → K1 换下一色 → 四色采完按 K3；串口那 20 行 CAL[...] 复制发回来即可。
   ★采的时候要和比赛时保持同样的距离/灯光，否则量出来的值没意义。 */
static const char * const COLORCAL_NAME[4] = { "BLACK", "RED", "BLUE", "WHITE" };  /* K1 的切换顺序 */

static void COLOR_Calib_Run(void)
{
  TCS34725_RGBC rgbc;
  uint8_t  idx    = 0;               /* 当前在采的颜色：0=黑 1=红 2=蓝 3=白 */
  uint8_t  n      = 0;               /* 当前颜色已经采了几次 */
  uint8_t  got    = 0;               /* 上次采样是否有效(无效就不显示旧数据) */
  uint32_t oled_t = 0;

  UART1_Printf("\r\n--- TCS34725 CAL: K1=switch color, K2=sample+print, K3=exit ---\r\n");
  UART1_Printf("order BLACK->RED->BLUE->WHITE, 5 samples each, then paste the CAL lines back\r\n");
  UART1_Printf("TH in code (tcs34725.h): TCS_BLUE_S_MAX=%.2f TCS_BLUE_V_MAX=%.2f TCS_WHITE_C_MIN=%.0f TCS_BLACK_V_THRESH=%.2f\r\n",
               (double)TCS_BLUE_S_MAX, (double)TCS_BLUE_V_MAX,
               (double)TCS_WHITE_C_MIN, (double)TCS_BLACK_V_THRESH);
  UART1_Printf("now target = %s (press K2)\r\n", COLORCAL_NAME[idx]);

  while(1){
    /* ---- K1：切换颜色（黑→红→蓝→白→黑…） ---- */
    if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){
      idx = (uint8_t)((idx + 1) & 0x03);
      n   = 0;                                     /* 换颜色 → 计数清零(每色都从 #1 开始) */
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

/* ================= 回家阶段：读一次颜色 + 把“当前判成什么色”显示到 OLED 第3行 =================
   回家段全靠颜色判断“进没进红/蓝区”，盯屏幕就能看出它现在判成什么色；
   第3行(y=32) 格式 “COL:BLACK/RED/BLUE/...”(判色规则见 tcs34725.h 的编译期阈值)。
   ★只有在“等颜色”的循环里调用：每次采样读一次、顺手刷一行屏，不额外占时间。 */
static uint8_t TCS_PollColor(TCS34725_RGBC *rgbc){
  TCS34725_GetRawData(rgbc);                     //读一次(不看返回值)
  uint8_t col = TCS34725_ClassifyColor(rgbc);    //按 S/V/C 判色
  OLED_ClearArea(0, 32, 128, 16);                //只清第3行
  OLED_Printf(0, 32, OLED_8X16_HALF, "COL:%s", TCS34725_ColorName(col));
  OLED_Update();
  return col;
}

/* ★当前“生效那套”速度环的 PWM 输出(2026-09-27 位置式接入后加)：
   两套状态(speed_pid_pos[] 位置式 / speed_pid[] 增量式，见 chassis.h)同一时刻只有一套在算，
   另一套的 out 是残值 —— 调试打印必须用本函数取，否则默认位置式时会一直打出恒 0，像“车没输出”。 */
static int CHASSIS_PwmOut(uint8_t wheel){
  return (int)(chassis.speed_pos_mode ? chassis.speed_pid_pos[wheel].out
                                      : chassis.speed_pid[wheel].out);
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
  /* 编码器清零法：每10ms读CNT后清零，不用溢出中断(以前开中断缺处理函数会卡死) */
  HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin, GPIO_PIN_SET);//TB6612使能

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RxBuf, UART1_RxLength);//调试串口
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);    //关掉DMA传输过半中断(开DMA+UART时默认打开)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RxBuf, UART2_RxLength);//主视觉串口
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  // UART5(步进电机串口) 未启用接收，故无对应 DMA 初始化

  UART4_RxInit();//串口4(副视觉)接收初始化(DMA+空闲中断，见 Mycode/uart.c)




  HAL_Delay(300);

  HWT101CT_Init();//陀螺仪初始化
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_RxBuf, UART3_RxLength);//陀螺仪串口
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  flag.hwt101ct = 1;

  CHASSIS_Init();//底盘初始化：4轮速度环 + 航向环(角度环)参数
  flag.chassis = 1;
  flag.angle   = 1;   // 航向环默认开启：上电锁定当前朝向，串口可遥控转向


/******************************上电测试位置******************************/
/*
for(uint8_t i=0;i<5;i++){ ROBOT_MoveSpeed(0,20); HAL_Delay(20); ROBOT_MoveSpeed(0,0); }
*/

//JieTi_Adjust_X();   //★要上电单独测校准就调 X/Y 这两个(旧名 JieTi_Adjust() 不存在，会链接不过)




  // （原“上电各传感器单测 / 调参打印”的注释代码已删，对应功能现在的入口：）
  //   副视觉字母 → VISION_ReceiveLetter() 或看串口1日志；主视觉 → 正面识别阶段的日志
  //   走距+转角 → 串口发 3,dx,dy,vx,vy,ax,ay,ang 或 5,ang,0,...
  //   角度环调参打印 → serialplot.c 的 SERIALPLOT_PIDAdjustParam()（临时调参+画图，见其函数头）
  // （速度环/激光/测距/灰度/陀螺仪/电机PWM+编码器 等上电单测的注释代码已删）
  //   入口：串口1单键 e/y/l/g/r 看数值、p 看状态、o 清里程；临时调参用上面那句 SERIALPLOT_PIDAdjustParam()。
  //   ★2026-09-27 位置式速度环接入后：要重开速度环调参，kp/ki/kd/target 读 chassis.speed_pid_pos[1]，
  //     PWM 用 CHASSIS_PwmOut(1)（默认位置式下 speed_pid[1].out 是残值 0）。
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

  /* ==================== TCS34725 颜色传感器上电初始化（★必须有，别跟着调试代码一起注释掉）====================
       ★2026-09-14 修复：“颜色传感器又坏了”的根因就在这里 —— cf20b8f 那次整理调试代码时把上面的
         TCS34725_Init()/打印两行连同整个调试循环一起注释掉了，驱动里 tcs_i2c_gpio_init() 从没跑过：
         PB9(SCL) 还是推挽输出(能翻转，看着像在工作)、PB4(SDA) 还是输入上拉 ⇒ 主机拉不低 SDA、起始
         条件发不出去 ⇒ GetRawData() 全失败 ⇒ C/R/G/B 全 0 ⇒ S=0、V=0 ⇒ 恒判 BLUE(不是黑)，回家
         阶段“读到红/蓝才停”“等到黑才停”全部判错，看起来就是传感器坏了。
       这里调 TCS34725_Init() 一次做完：PB9/PB4 配开漏+内部上拉、50ms 积分+1x 增益、PON+AEN 使能。
       上电串口1 会打 “TCS34725 ONLINE, ID=0x44”；打成 OFFLINE 先查供电/接线(VCC=3.3V、SCL=PB9、SDA=PB4)。
       另：GRAY1 的 GRAY1_Serial_Update() 也不能恢复 —— 它拿串口时序驱动 PB9/PB4，会把 I2C 打坏。 */
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
        标定 = 进 KEY0 起点里的 CAL 模式四色各采 5 次，按实测值重写那 4 个宏后重新编译烧录。 */

  /* ★★ 固件版本标记（开机就打，用来确认“烧进去的到底是不是最新代码”）★★
       ★2026-09-15 踩坑：改了 main.c 但没重新 build+flash（或被旧内容覆盖回去），串口看到的还是旧行为。
       ★2026-09-20 换新底盘：每轮独立速度环PID + 4倍频878编码器 + 20ms控制周期 + 航向环三档/静摩擦
         bias + 停车由速度环闭环反接；主流程里的反冲刹车(SENSOR_BRAKE)、白线反向急刹(LINE_BRAKE)、
         start_margin/brake_decel、move/angle 后的稳定延时全部删除。看到 “NEW-CHASSIS 09-20” 才是这版。 */
  UART1_Printf("FW: NEW-CHASSIS 09-20 (per-wheel PID, 878enc, 20ms, no brake-patch) built %s %s\r\n", __DATE__, __TIME__);

  while (1)
  {
    /* ===== 串口1实时发四轮实际值（2026-09-25 加，实现在 serialplot.c）=====
           每 WHEEL_ACT_SEND_MS(默认20ms) 打一行 4 个数：左前/左后/右后/右前 实际速度 cm/s，接 SerialPlot
           看四条曲线；串口1发单键 “v” 可随时开关。★非阻塞、自己按时间节流，车在跑时的那几个阻塞等待
           循环里也各调了一次(见 robot.c)，所以整趟动作都有数据。 */
    SERIALPLOT_WheelActualPump();

    // （原上电测试的注释代码已删：GRAY3 循线更新、TCS 颜色 OLED 显示、KEY1/2/3 颜色采样、
    static TCS34725_RGBC tcs_rgbc = {0};
    //   测距/激光 OLED 显示。入口：串口1单键 r=灰度GRAY3、c=颜色、g=测距、l=激光；颜色标定用 KEY0 的 CAL 或单键 m）
    //测距，2是前面的，1是后面的
    //OLED_Printf(0,16 , OLED_8X16_HALF, "dis_1:%4d", GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin));
    OLED_Printf(0,32 , OLED_8X16_HALF, "dis_2:%4d", GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin));
    // （原激光 OLED 显示已删：用串口1单键 l 看 L3/L2 状态）
    OLED_Update();


    /* ===== UART1 指令：解析/回显/就地执行都在文件上方 UART1_DebugCmd 里（菜单循环也调了一次），这里不用再写 if；
           2026-09-26 整理：原先放在这里那段 3 的块已搬进函数，所以菜单界面也能发了。 */
    UART1_DebugCmd();
    DBG_SensorLogPump();       /* 串口1单键传感器日志（c/g/l/e/y/r，非阻塞节流，见文件上方 DBG_ 块） */

    /* 例：3,-50,390,100,100,100,100,270 → 左移50cm+前进390cm+原地转到270° */
    

    /* ---------- 正面识别通信的屏幕显示(实现在文件上方的 ZM_ShowComm()，这里只说“屏幕上该看到什么”) ----------
           y=0/16/32/48 四行的文字与含义见上方 ZM_ShowComm() 的函数头注释（状态一变就整屏重画，不花屏）。
           流程：0 走到识别位 → 1 发 0xA2 → 2 收副视觉字母(6种之一) → 3 转发主视觉+回执副视觉 →
           4 等主视觉 0xA7 → 5 结束。★排查用电脑串口1(115200)：完整收发时序都会打印(“TX 0xA2 …/RX4 …/RX2 …”)。 */
    int comm_rx4 = 0, comm_rx2 = 0;                //副视觉(串口4)收帧数 / 主视觉(串口2)收帧数
    uint32_t comm_t_oled = 0;                      //OLED刷新节流用时刻(ms)

    //进入各部分的标志位，红蓝可共用
    uint8_t YuanPanJi_Flag = 0;
    uint8_t ZhengMian_Flag = 0;
    uint8_t JieTi_Flag = 0;
    uint8_t LiZhu_Flag = 0;
    uint8_t HuiJia_Flag = 0; 

    uint8_t dbg_start = DBG_START_ALL;   //★KEY0 选好的调试起始阶段(取值见上方 DBG_START_xxx)
    //完整走

    //红蓝方选择
    while(1){
      /* 菜单界面就能用全部串口指令（不用先按 KEY3 进流程）：解析/回显/执行都在 UART1_DebugCmd 里，
               单键 w/s/a/d、日志 c/g/l/e/y/r、数值 3/5/7/9、SerialPlot 调参都在这里生效。 */
      UART1_DebugCmd();
      DBG_SensorLogPump();            /* 串口1单键传感器日志（非阻塞节流，见文件上方 DBG_ 块） */
      SERIALPLOT_WheelActualPump();   // 串口1实时发四轮实际值(菜单里也发)
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

      /* ============== ★★ 按键0：选择“调试起始阶段”（想单独调哪一段，就让车从哪一段开始跑）★★ ==============
               每按一次 KEY0 往后切一个：0=ALL → 1=ZM → 2=HOME → 回到 0=ALL
                 0=ALL  完整流程：圆盘机→仓库倒球→正面识别→阶梯→回家（默认，主流程一字不变）
                 1=ZM   从“正面识别前”开始：跳过圆盘机+倒球，车走到识别位再开始正面识别（车放哪都行）
                 2=HOME 从“回家”开始：跳过前面全部，直接跑回家那段（车放红蓝区旁边即可）
               ★当前选择显示在第3/4行；选好后按 KEY3 才开始跑；真正的跳转代码在菜单 while 外面。 */
      if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
        dbg_start++;
        if(dbg_start > DBG_START_MAX) dbg_start = DBG_START_ALL;    //切到最后一个再按 → 回到"完整流程"
        UART1_Printf("DEBUG: KEY0 start stage = %u (%s) %s\r\n",
                     (unsigned)dbg_start, DBG_START_NAME[dbg_start], DBG_START_DESC[dbg_start]);
      }
      //按键0/1专用于调试(按键0=选起始阶段；按键1=显示左后激光)

        /* ---- 原来的按键0调试(手动把车开到阶梯对准位)已由上面跳转代替，保留备用 ----
                ROBOT_Move(100,160,长距档) → 左移 SPD_AVG_V 等 LASER3(左后) 触发 → ROBOT_Move(-50,0,长距档) → 停车
                （注释里那句是老写法：左后是 1、右边为 2、左前是 3） ---- */

      if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){

        OLED_Printf(0, 0, OLED_8X16_HALF, "barrier:%1d", LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin));
        OLED_Update();

        /* ================= 旧代码（原来的按键1“倒球调试”，整段注释保留）=================
                要恢复原来的倒球调试：把本行与“旧代码结束”那一行的注释符去掉，并把上面的等待接收测试删掉。
                流程：打串口“3” → 后退到前测距 ≤90 → 打“4” → 左移到 LASER1(左后) 无遮挡 →
                      ROBOT_Move(10,0,短距档) → runActionGroup(16,1) 倒球 → delay_ms(2000)
                ================= 旧代码结束 ================= */

      }
    }
    /* ============== ★★ 调试起点跳转（在“红蓝方选择”界面按 KEY0 选、再按 KEY3 开始后才会走到这里）★★ ==============
           dbg_start==DBG_START_ALL(0)：一个字都不执行，原样往下跑完整流程 → 和“没有这套调试入口”时完全一致；
           ★跳转代码放在菜单 while 外面，就是为了让“完整流程”连一条语句都不变。 */
    /* ★每次运行开始先把“角度基准换算”复位：它是静态变量，跑过一次跳转起点后若不清，切回完整流程
           (dbg_start=ALL)时还会带着上一段的换算 → 所有绝对角都被换算，那就错了。 */
    yaw_shift_deg = 0;
    if(dbg_start == DBG_START_ZHENGMIAN){
      /* 从“正面识别前”开始：跳过圆盘机+仓库倒球，直接跳到 ZHENGMIAN_START 往下执行
               （往下依次是：收倒球槽 → 走到阶梯识别位 → 左前光电对准 → 清串口残留 → 发0xA2 → 阶梯 → 回家）
               ★车会先自己走到阶梯识别位，所以放在圆盘机/仓库方向随便一点的位置即可 */
      UART1_Printf("DEBUG: start from ZHENGMIAN (before front recognize)\r\n");
      /* ★★ 角度基准：本次车按“倒完球那一步的姿态”摆好再上电（红方车头朝右=基准90°，蓝方自动+180°=270°）
               → SetYawShift() 按红蓝自动设好 yaw_shift_deg，之后每一处 ROBOT_Angle 都走 Yaw_Abs()。 */
      SetYawShift(90);
      //跳过了圆盘机，没走“告诉视觉红(0xAA)蓝(0xBB)方”那一步，这里补上
      UART2_Printf("%c", mode_red ? 0xAA : 0xBB);//告诉视觉红(0xAA)蓝(0xBB)方
      goto ZHENGMIAN_START;
    }
    else if(dbg_start == DBG_START_LIZHU){
      /* 从“立柱”开始：直接跳到 LIZHU_START 往下执行
               （立柱前校准：左移 5cm/s 直到两激光都有障碍 → 前进 5cm/s 到前测距 200mm → LiZhu_Circle_Run()
                 绕柱 → 走到仓库中间倒方块 → 回家）
               ★★2026-09-26 改 270 → 90（修“单独测立柱时绕完一圈车直接转身180°”）：9.26 起 LIZHU_START
                 前那句“先转到270”已注释掉，正式流程走到立柱起点的朝向＝阶梯跑完那个朝向＝红90/蓝270；
                 本入口的摆车姿态必须与之一致 —— 写 270 会让 Yaw_Abs(90)=180°(正后方)，绕完 355° 还要再转
                 半圈；改成 90 后 Yaw_Abs(90)=0，那句只剩“把绕圈攒下的几度掰回来”(正对柱子/朝右)。 */
      UART1_Printf("DEBUG: start from LIZHU (pillar)\r\n");
      SetYawShift(90);
      LiZhu_Flag = 1;     //"立柱"段的进入条件
      goto LIZHU_START;
    }
    else if(dbg_start == DBG_START_HUIJIA){
      /* 从“回家”开始：跳过前面全部阶段，直接跳到 HUIJIA_START（红蓝区找色→停进红蓝区）
               ★角度基准和“单独测阶梯(ZM)”完全一样：红方按“倒完球姿态”(车头朝右)摆、蓝方自动+180°(朝左)，
                 本段所有绝对角都走 Yaw_Abs() 换算。 */
      UART1_Printf("DEBUG: start from HUIJIA (go home)\r\n");
      SetYawShift(90);   //和 ZM 起点同一套（红90 / 蓝270）
      HuiJia_Flag = 1;   //“回家”段的进入条件是 HuiJia_Flag==1：正常流程由前面阶段置位，跳转时先自己立好
                         //(其余阶段标志出菜单时本来就都是0，不用管)
      goto HUIJIA_START;
    }
    else if(dbg_start == DBG_START_COLORCAL){
      /* 从“颜色校准”开始：跟比赛流程完全无关（只读颜色传感器 + 按键 + 打串口），跑完 continue 回菜单；
                用法：KEY0 切到 CAL → KEY3 进入 → K1 切颜色、K2 采一次并打串口、K3 返回。 */
      UART1_Printf("DEBUG: start COLOR CALIBRATION\r\n");
      COLOR_Calib_Run();
      continue;                 /* 回菜单：想继续跑比赛流程，把起点切回 ALL 再按 KEY3 */
    }

    //？后面，左蓝右红
    
      
    /**************圆盘机****************/
  
    YuanPanJi_Flag = 1;//圆盘机开始
    UART2_Printf("%c", mode_red ? 0xAA : 0xBB);//告诉视觉红(0xAA)蓝(0xBB)方

    //路上就先把机械臂举起来
    runActionGroup(1, 1);//不需要延时，因为和出发一起
    
    //先盲走到圆盘机中心+面向(★长距档 120/120)
    ROBOT_Move(mode_red?-88:88,420,SPD_LONG_V,SPD_LONG_V,SPD_LONG_A,SPD_LONG_A);//58太靠右
    /* ★2026-09-20 新底盘：ROBOT_Move / ROBOT_Angle 都是阻塞式，且 ROBOT_Angle 会等到
           “航向到位 且 四轮真正停稳”才返回 —— 后面不用再补 HAL_Delay(100) 提高稳定性了 */
    mode_red ? ROBOT_Angle(270) : ROBOT_Angle(90);

    
    /* ==================== 向前慢走，直到灰度(探头1)感应到白线 ====================
           ★★★ 2026-09-15 修复“停不下来”的根因：运算符优先级 ★★★
             上一版写成 while(GRAY_Data[GRAY3][0] | GRAY_Data[GRAY3][1] == 0)，而 == 优先级高于 |，
             实际等价于 GRAY_Data[GRAY3][0] | (GRAY_Data[GRAY3][1] == 0)：只要探头1那一位恒为 1
             (探头没压到线/通道坏/DAT 线松或模块没电，输入上拉全读1)，表达式恒真 → 车永不停、一直前冲。
             编译日志本来就有这条警告：suggest parentheses around comparison in operand of '|'。
             现在只用探头1(序号0)：读到黑(0)继续走，读到白(1)立刻停。 */
    /* ==================== 恒速逼近白线 + 检测到白线就停车 ====================
           ★2026-09-20 新底盘(4倍频878编码器 + 20ms控制周期 + 每轮独立速度环PID + 航向环三档/静摩擦bias)
             后，本节原来的“反冲急刹”补丁已整段删除：· 逼近速度回到命令值本身(命令 10cm/s 就是 10cm/s，
             老底盘的破静摩擦整形会把低速命令顶到 16~29cm/s)；· 停车直接用 ROBOT_MoveSpeed(0,0)：速度环
             target=0 而四轮还在转 → 当拍反向修正(≈主动反接刹车)，比断电滑行刹得快、落点重复。
           ★现场核对：若落点整体偏前/偏后，直接改下面几条 ROBOT_Move 的固定位移量(别再加反向冲补丁)。 */
    const uint8_t  LINE_DBG_LOG  = 1;    /* ★诊断日志：1=开 0=关（把停车调好后改0） */
    /* 逼近速度直接用 SPD_AVG_V（见文件上方速度档）；原来的 LINE_CRUISE_V 已被取代、删掉 */
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
        /* ① 每150ms一行：逼近时的实测速度 + 4轮PWM。正常应看到 tgt=10、实测 v≈10、PWM 稳定在能维持
                      10cm/s 的小值区间；若 tgt=0/pwm 全 0 → 车没动，先查底盘。
                      ★若开始 0.3s 左右不动：那是速度环积分在爬升，想让它立刻就走就把逼近速度(SPD_AVG_V)提到 20。 */
        if((now - line_dbg_t) >= 150){
          line_dbg_t = now;
          float v_now = sqrtf(chassis.now_v_x*chassis.now_v_x + chassis.now_v_y*chassis.now_v_y);
          UART1_Printf("CRUISE v=%.1f tgt=%.0f pwm=%d/%d/%d/%d g3=0x%02X\r\n",
                       (double)v_now, (double)chassis.speed_pid[CHASSIS_MOTOR_LF].target,
                       CHASSIS_PwmOut(CHASSIS_MOTOR_LF),
                       CHASSIS_PwmOut(CHASSIS_MOTOR_LB),
                       CHASSIS_PwmOut(CHASSIS_MOTOR_RB),
                       CHASSIS_PwmOut(CHASSIS_MOTOR_RF),
                       (unsigned)g3);
        }
        /* ② 灰度 8 位数字量变化（压过白线时该看到某一位变1；p1/bit0 就是 while 里用的探头1）：
                      · 一路不打变化行 / g3 恒 0x00 → 模块没识别出白线（高度/角度、阈值、环境光）
                      · g3 恒 0xFF → DAT 线松/模块没供电（输入上拉全读1）
                      · g3 有 1 但不落在 p1 → 探头编号与竖装后的朝向不符，换 while 里的下标即可 */
        if(g3 != line_g3_last){
          line_g3_last = g3;
          UART1_Printf("LINE g3=0x%02X p1=%u\r\n", (unsigned)g3, (unsigned)GRAY_Data[GRAY3][0]);
        }
      }
    }


    /* ★★ 停车 = ROBOT_MoveSpeed(0, 0)（2026-09-20 新底盘；原“柔性反向刹车”补丁已删）★★
            新底盘速度环是闭环的：target=0 而四轮还在转时当拍就把 PWM 反向修正(≈主动反接刹车)，比断电
            滑行刹得快、落点重复；真停稳后由底盘的“静止零漂保护”清零断电。所以原来的 LINE_BRAKE_V/MARGIN/MS
            ＋ 临时改 chassis.start_margin ＋ LINE_TRIM_CM ＋ 临时改 chassis.brake_decel 全部删除。 */
    {
      /* 本段车头朝向固定(锁向保持)，把“全局速度/位移”投影到车头前方才是真正的前向量：
               body_forward = -global_x*sin(θ) + global_y*cos(θ)，θ = chassis.now_the
               （直接打全局 pos_y 是错的：本段车头朝 270/90°，前进方向对应全局 x） */
      line_c_th  = cosf(chassis.now_the);
      line_s_th  = sinf(chassis.now_the);
      line_v_hit = -chassis.now_v_x*line_s_th + chassis.now_v_y*line_c_th;  /* 触发瞬间前向速度 */
      line_x0    = chassis.pos_x;
      line_y0    = chassis.pos_y;
      ROBOT_MoveSpeed(0, 0);                     /* ★停车：速度环 target=0 → 主动反接刹车 */
    }

    //再次校准(★ROBOT_Angle 现在阻塞到“航向到位且四轮停稳”才返回，不用再补延时)
    mode_red ? ROBOT_Angle(270) : ROBOT_Angle(90);

    /* ★落点测量(STOP)：把“触发 → 停车 → 原地转正校准”整段的车头前向位移打出来，用来核对拍球落点
            (原地转正那一下麦轮会蹭掉一点位移，属正常)。
            ★落点若整体偏前/偏后：直接改下面几条 ROBOT_Move 的固定位移量补回来（别再加反向冲补丁）。 */
    if(LINE_DBG_LOG)
      UART1_Printf("STOP v_hit=%.1f dfwd=%+.2fcm dxy=%+.2f/%+.2f total=%lums\r\n",
                   (double)line_v_hit,
                   (double)(-((chassis.pos_x - line_x0)*line_s_th) + ((chassis.pos_y - line_y0)*line_c_th)),
                   (double)(chassis.pos_x - line_x0), (double)(chassis.pos_y - line_y0),
                   (unsigned long)(HAL_GetTick() - line_t0));

    //往后走一点点(★短距 2cm；往前会拍不到球)
    ROBOT_Move(0, mode_red ? -6 : -6, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

    //(不需要后退了，直接灰度校准更准确)

    //走到位后，放到识别状态
    runActionGroup(4, 1);
    delay_ms(1400);
    UART2_Printf("%c", 0xA1);//发0xA1告诉主视觉进入圆盘机识别

    /******************** 圆盘机视觉处理代码 ********************/
    /* 与视觉约定的数据包：包头0xA1 | 第1个数据(0x00/0x01/0x02) | x坐标(2字节) | y坐标(2字节) | 包尾0x0B
            触发：第1个数据==0x01或0x02，且 x==100~200、y==80~160 → 动作组1
            结束：收到单字节 0xA6 退出本 while；★本机自己给“拍球”计数，累计到 YPJ_HIT_MAX(10) 个后不论
            视觉有没有发 0xA6 都强制退出；★★连续 YPJ_NO_HIT_TIMEOUT_MS(9s) 一次都没触发过拍球也强制退出。
            视觉屏幕 320*240 */
    /* ==================== 圆盘机拍球防连拍（重点！） ====================
           ★先看现象再调参：① “一个球拍两次” = 视觉按 0.02s 连续上报、同一个球在画面里占好几帧，
             旧代码每帧都 runActionGroup；② “上一球要拍拍了下一个” = runActionGroup 只是给舵机控制板发一条
             启动指令就返回(约7ms)，动作组本身要在控制板上跑几百ms，上一拍没跑完下一帧(已是下一个球)又触发。
             核心是在 MCU 侧“同一球只认第一帧 + 一次动作没执行完不再触发”，下面用冷却时间实现。
           ★YPJ_HIT_COOLDOWN_MS 调法：≥ 动作组7/10 从开始到复位可再拍的实测耗时。调大→更不易连拍，但若
             相邻两球间隔比它还短，后一个球会漏拍；调小→能跟上连着的球，但小于动作实际耗时又会连拍。
             用串口1日志里相邻两次 “HIT” 行的时间戳就能标定真实动作耗时。
           ★YPJ_HIT_LOG=1 时串口1打印 HIT/SKIP 时序(先在电脑上分析帧间隔和连拍)，调好后改0。
           ★实测：圆盘 8s 一圈、12个球，两球间隔 2/3 秒(≈667ms)；动作组7/10 都是 400ms。
           ★★拍够10个强制退出（防视觉一直不发/漏发0xA6导致卡在圆盘机）：ypj_hit_cnt 每次真拍 7/10 后 +1，
             累计到 YPJ_HIT_MAX(10) 就不再等视觉“需要夹”，直接抬臂(151)+延时+收臂(0)并 break 出本 while
             （与收到0xA6时收尾完全一致）。· 想改拍几个球只改 YPJ_HIT_MAX；
             · ★冷却值决定“计数准不准”：冷却 < 一个球在画面里的停留时长 → 同一个球被拍2次 → 数到10时其实
               只过了5个球（比赛实测踩过），所以必须 动作耗时400ms < 冷却 < 球间隔667ms，当前取650。
             · 验证(YPJ_HIT_LOG=1)：HIT 行 c= 是两次“拍”的间隔，应≈667ms（≈650 说明只是节拍在拍、球还没换）；
               RUN 行 pres=球在画面停留多久、gap=两球之间空多久；EXIT 行 t=总耗时，10个球应≈6.7s
               （只有3.4s说明又是同一球拍两次）。
             · 若希望第一拍就计数：把下面 ypj_first_skipped“第一个球只跳过不拍”那段去掉即可。 */

    /* 实测：圆盘 8s/圈、12球 → 相邻两球间隔≈667ms；动作组7/10 执行 400ms。冷却窗口须满足
            400ms < 冷却 < 球间隔667ms（>动作耗时保证上一拍跑完不连拍，<球间隔保证不漏下一个球）。
            ★★ 冷却窗口＝一次“拍”到下一次“拍”的最短间隔，同时也是本机“拍球计数”的节拍 ★★
            ★为什么从 300 改成 650（比赛实测：记够10个却只拍了5个球就走了）：300ms 比“一个球在画面里停
              留的时间”还短 → 同一个球被拍了2次，10次计数其实只过了约5个球（667/300≈2.2拍/球）；
              650≈667 时同一个球不会被拍两次，球流不断时基本“一球一拍”。 */
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
    uint8_t  ypj_hit_cnt    = 0;               /* ★实拍计数：每执行一次动作组7/10就+1(冷却期忽略的帧、跳过的第一个球都不计) */


    while(YuanPanJi_Flag == 1){
      if(VISION1_RxFlag){                              // 2号串口（主视觉）DMA收到一帧
        VISION1_RxFlag = 0;                            // 必须立即清零

        /* 退出条件1：收到单字节指令0xA6（视觉确认），则退出本while循环
                    退出条件2：★本机拍球计数 ypj_hit_cnt 达到 YPJ_HIT_MAX(10)，见while末尾“强制退出”段 */
        if(VISION1_RxRealLength == 1 && VISION1_RxBuf[0] == 0xA6){
          YuanPanJi_Flag = 0;//只是圆盘机结束，不代表阶梯开始，还要倒球
          //单独的机械臂抬起
          runActionGroup(151, 1);
          HAL_Delay(1000);
      
          runActionGroup(160, 1);  // 收起机械臂
          //不需要延时，和跑图一起
          break;
        }

        memset(CAM_Data, 0, sizeof(CAM_Data));         // 清空上一帧数据
        if(VISION1_RxRealLength <= sizeof(CAM_Data)){
          memcpy(CAM_Data, VISION1_RxBuf, VISION1_RxRealLength);
        }
        if(CAM_Data[0] == 0xA1                                              // 包头
            && (CAM_Data[1] == 0x01 || CAM_Data[1] == 0x02)                  // 有球：0x01本色 / 0x02黄
            && CAM_Data[6] == 0x0B){                                         // 包尾
          uint16_t cam_x = (uint16_t)CAM_Data[2] | ((uint16_t)CAM_Data[3] << 8); // x坐标(2字节)
          uint16_t cam_y = (uint16_t)CAM_Data[4] | ((uint16_t)CAM_Data[5] << 8); // y坐标(2字节)
          uint32_t now    = HAL_GetTick();
          uint32_t d_last = now - ypj_last_frame_t; // 距上一帧“有球”帧的间隔(ms)，日志用于看视觉真实发帧间隔
          /* ★标定日志（确认 YPJ_HIT_COOLDOWN_MS 取值是否合适）：
                       d_last ≥ 100ms → 上一个球已离开画面，这时打印上一轮球的“停留时长(pres)”和“空窗(gap)”。
                       · pres 明显大于冷却值 → 就是本次“10次只拍5个球”的原因，需要把冷却调大（现已取650）；
                       · 几乎没有 RUN 行 → 视觉一直连着报“有球”(画面里同时有 2 个以上球)，只能按 667ms 的
                         节拍计球，此时日志里的 c=（两次拍之间的间隔）应≈667ms。 */
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
              && (cam_y >= 0 && cam_y <= 240)){
            if((now - ypj_last_hit_t) < YPJ_HIT_COOLDOWN_MS){
              /* ===== 冷却期：还是同一个球 / 上一拍动作还没执行完 → 忽略（防连拍关键）===== */
              if(YPJ_HIT_LOG)
                UART1_Printf("SKIP b=%u x=%u y=%u d=%lu\r\n",
                             (unsigned)CAM_Data[1], (unsigned)cam_x, (unsigned)cam_y,
                             (unsigned long)d_last);
            }else{
              /* ===== 冷却已过：这一帧当作"新球"，真正拍一次 ===== */
              uint32_t c_last = now - ypj_last_hit_t;   // ★距上一次“拍”的间隔：正常应≈球间隔667ms；若≈冷却值说明只是节拍在拍、不是新球
              ypj_last_hit_t = now;
              /* ★9s无触发超时的基准时刻在这里刷新：
                                不管是下面“第一个球只跳过不拍”还是真发了动作组，都算“触发过拍球”，
                                所以球流不断就永远不会走到 9s 强制退出；球流一断，9s 后自动收摊。 */
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
                视觉那边可能一直不发（或晚发、漏发）“需要夹”的 0xA6，这里由本机自己兜底：
                条件1：拍够 YPJ_HIT_MAX(10) 个球 —— ypj_hit_cnt 每真拍一球 +1，数到10就不再等 0xA6；
                条件2：★连续 YPJ_NO_HIT_TIMEOUT_MS(9s) 一次都没触发过拍球（视觉掉线/球流断了/车没对上），
                       由 ypj_no_hit_ref_t 计时。两个条件走同一套收尾：等最后一拍动作组跑完 → 抬臂(151) →
                       延时 → 收臂(0) → break 出 while。退出后 YuanPanJi_Flag=0，下面“去仓库倒球”照常执行。 */
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

        runActionGroup(160, 1);  // 收起机械臂
        break;                 // 跳出 while(YuanPanJi_Flag == 1)
      }
    }

    /***************去仓库倒球*************/

    if(YuanPanJi_Flag == 0)//圆盘机结束时
    {

      //退后固定距离(★短距 25cm：20/30)
      ROBOT_Move(0,-20,SPD_SHORT_V,SPD_SHORT_V,SPD_SHORT_A,SPD_SHORT_A);
      //收起机械臂
      //重复了runActionGroup(160, 1);
      //向左平行到仓库(★长距档 120/120)
      ROBOT_Move(mode_red ? -190 : 195,0,SPD_SHORT_V,SPD_SHORT_V,SPD_SHORT_A,SPD_SHORT_A);//蓝要多走一点
      //转身(ROBOT_Move 已阻塞到车停稳，不用再补 HAL_Delay(100))
      mode_red ? ROBOT_Angle(90) : ROBOT_Angle(270);
      
      //往后慢退，直到测距到适合倒球的距离
      ROBOT_MoveSpeed(0, -SPD_AVG_V);   //慢匀速
      WAIT_WHILE(GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin)>45,
                 "YuanPanJi back-to-wall(mm<=45)");//实测45
      
      //直接接激光，不然距离会很远
                 //ROBOT_MoveSpeed(0, 0);   /* ★停车：速度环 target=0 → 主动反接刹车（★这处现在注释掉：直接接激光，不然距离会很远）
                 //                            ★倒球距离若不对：把上面的 45 改大(停远)/改小(停近)，或在这里补一条固定位移 */

      //加一次角度校准(此处角度很重要；ROBOT_Angle 已阻塞到停稳，不必再补延时)
      //mode_red ? ROBOT_Angle(90) : ROBOT_Angle(270);

      //向左慢平移到左后光电感应到无障碍物
      //(激光移位后：没对到障碍物时就已经是合适位置，不需要再调整)
      ROBOT_MoveSpeed(-5, 0);   //★匀速靠近：SPD_AVG_V
      
      {
            uint32_t lz_t0 = HAL_GetTick();  uint8_t lz_hit = 0;   //连续"看到"计数
            while(lz_hit < 3){//从有到无
              lz_hit = !LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin) ? (uint8_t)(lz_hit + 1) : 0;  //断一次就重新数
              if(lz_hit >= 3) break;
              HAL_Delay(50);                                       //每次检测间隔 50ms
            }
          }
      

      ROBOT_MoveSpeed(0, 0);   /* ★停车：速度环 target=0 主动反接刹车（不再自由滑行/不再反向冲）
                                  ★若左边还差一点：在这里补一条固定左移
                                    ROBOT_Move(-X, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A); */

      //加一次角度校准(ROBOT_Angle 已阻塞到停稳，不必再补延时)
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
                  2. 副视觉(串口4)发来的单字节为0xAB/0xAC/0xAD/0xBC/0xBD/0xCD这6种之一时，就存下来并原样
                     转发给主视觉(串口2)；不是这6种就不存；
                  3. 存下来后，MCU发0xA7给副视觉，告诉它我收到了，副视觉可以结束识别了
                  4. 主视觉收到MCU转发的副视觉信息后，发0xA7给MCU，告诉它正面识别结束了
                  5. MCU收到主视觉发来的单字节0xA7，则退出本while循环，正面识别结束
                */

        /* ===== 正面识别 第1段：先复位显示，再“收倒球槽 + 走到阶梯对准位”；到位后才发0xA2 =====
                   ★为什么到位才发0xA2：0xA2是两个视觉的“开始识别”标志。早发的话副视觉可能在车还没到位时
                     就把结果发出来，甚至被后面的“清残留”清掉；到位后再发，这段时序就完全不用纠结。 */
        comm_rx4 = 0; comm_rx2 = 0; comm_t_oled = 0;               //本阶段收发计数清零
        zm_step = ZM_ST_PREP;                       //进度0：还没发0xA2(正在移动到识别位)
        zm_v4_res = ZM_RES_WAIT; zm_v4_byte = 0; zm_v4_len = 0;
        zm_v2_res = ZM_RES_WAIT; zm_v2_byte = 0;
        ZhengMian_Letter[0] = 0; ZhengMian_Letter[1] = 0;    //上一轮残留的字母清掉
        zm_t0 = HAL_GetTick();                      //本阶段起始时刻(第4行"秒数"用)
        ZM_ShowComm(comm_rx4, comm_rx2);            //先画一屏：进度0(移动中)，V4/V2 都是 WAIT

        runActionGroup(19, 1); 	//这里是收倒球槽
        delay_ms(2000);

        //右+前移动到阶梯附近:要往右多走点不然会撞；y160太远不利于视觉识别(★长距档 120/120)
        //要拆开两段走，否则会撞到：先走x后走y
        ROBOT_Move(80, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
        ROBOT_Move(0, 128, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

        //向左慢走，直到前面的两个光电都感应到障碍物(右2坏了/左4拆了，复用立柱的)，4左2右
        ROBOT_MoveSpeed(-SPD_AVG_V, 0);   //★匀速靠近：SPD_AVG_V
        
        {
            uint32_t lz_t0 = HAL_GetTick();  uint8_t lz_hit = 0;   //连续"看到"计数
            while(lz_hit < 3){//从无到有
              lz_hit = LASER_Barrier(LASER4_GPIO_Port, LASER4_Pin) ? (uint8_t)(lz_hit + 1) : 0;  //断一次就重新数
              if(lz_hit >= 3) break;
              HAL_Delay(50);                                       //每次检测间隔 50ms
            }
          }
        
        ROBOT_MoveSpeed(0, 0);      //★停车（激光触发后立即给 0 速，速度环反接刹车）

        //往左走一定距离，视觉里能完整看到两个字母(可省去测距前后校准)(★短距 40cm：20/30)
        //短距档太快速会走斜，不要100速度，50还算可以
        ROBOT_Move(-45, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
        //停车准备识别
        ROBOT_MoveSpeed(0,0);

        //清掉主/副视觉串口的残留数据，避免把旧数据误当成有效信息
        UART2_RxFlag = 0;
        UART2_RxRealLength = 0;
        memset(UART2_RxBuf, 0, UART2_RxLength);
        UART4_RxFlag = 0;
        UART4_RxRealLength = 0;
        memset(UART4_RxBuf, 0, UART4_RxLength);   //UART4_RxBuf是extern数组，不能用sizeof

        /* ★把两个视觉串口的 DMA 接收强制重启一遍：万一之前因为出错(ORE/FE)停了接收，
                    这里复位+重启，保证进循环时两个口都是真的“在听”状态 */
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
                //存下来：高4位=第一个字母，低4位=第二个字母(供后面阶梯阶段使用)
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
                                   ★只在“还没拿到有效结果”时才显示在第2行(带 BAD 标记)，拿到结果后不再改它 */
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
                               ★阶梯阶段屏幕上会保留“正面识别拿到的两个字母”，方便对照 */
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
                       ★屏幕上四行的内容统一由 ZM_ShowComm() 负责，这里只做“定时刷新”，不再直接写屏幕 */
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
                   M3 走位/计数: 第2~8坑每坑走固定距离(JIETI_STEP_CM / JIETI_STEP_CROSS_CM) + 计第几个坑
                                (第1坑守着“进阶梯到位”点，不走位，但照样校准/读数)
                   M5 动作组   : 识别位(54) / 夹取(57·60·63) / 回识别位(66) / 收尾抬臂(151)
                   M6 显示/收尾: OLED + 停车 + 按 JIETI_GO_LIZHU 交棒给“立柱”或“回家”
                   M2 逐坑校准 : **第1~8坑都做**：先左右(JieTi_Adjust_X 视觉x±30px) → 再前后(JieTi_Adjust_Y 前测距±20mm)
                                ★与夹不夹无关(不夹的坑也先校准)；左右等不到帧/坐标填0 → 内部跳过不动
                   ============================================================================== */

        /* ---- M6.1 屏幕初始化：字母留着，第2行标"step only"，第4行先显示 BLK -/8 ---- */
        OLED_Clear();                                 //4行一起清掉(顺带清掉正面识别留在第3行的状态字)
        OLED_Printf(0,  0, OLED_8X16_HALF, "JIETI letter %c %c",
                    ZM_Nibble2Char(ZhengMian_Letter[0]), ZM_Nibble2Char(ZhengMian_Letter[1]));
        OLED_Printf(0, 16, OLED_8X16_HALF, "step only");   //本阶段=固定位移走位 + 第1~8坑逐坑校准（屏上字样沿用旧文本，未改）
        OLED_Update();
        JieTi_ShowBlockNo(0, 8, 0);   //第4行先显示"BLK -/8"，处理到第1个坑后就变成 1/8

        /* ---- 到位①：左移找激光4（激光一离开障碍物就停）---- */
        ROBOT_MoveSpeed(-SPD_AVG_V, 0);
        /* ★超时保护：激光4一直报“有障碍物”时原来会无限往左走(顶住左边不动=卡死)，
                    现在最多等 WAIT_TIMEOUT_MS，超时打 TIMEOUT 后照常停车继续走流程 */

          {
            uint32_t lz_t0 = HAL_GetTick();  uint8_t lz_hit = 0;   //连续"看到"计数
            while(lz_hit < 3){
              lz_hit = !LASER_Barrier(LASER4_GPIO_Port, LASER4_Pin) ? (uint8_t)(lz_hit + 1) : 0;  //断一次就重新数
              if(lz_hit >= 3) break;
              HAL_Delay(50);                                       //每次检测间隔 50ms
            }
          }

        ROBOT_MoveSpeed(0, 0);

        /* ---- M3.1 朝向基准：转到“正对阶梯”的绝对角 ----
                    正常流程：红90/蓝270（两边差180°）；跳转测阶梯：红方 shift=90 / 蓝方 shift=270，
                    Yaw_Abs() 换算后都是“按摆车姿态不转”（摆车时 0° 已经就是该段的车头方向）。
                    ★2026-09-28：这里原来还把该角记下来当整段阶梯的锁向目标、后面每次设速都恢复它；
                      那套封装已删除（速度环/角度环已调稳）→ 后面按常规直接用 ROBOT_MoveSpeed 锁当前朝向。 */
        mode_red ? ROBOT_Angle(Yaw_Abs(90)) : ROBOT_Angle(Yaw_Abs(270));

        /*激光校准，往右 5cm（激光刚离开时位置偏左,不往右测距出去了）*/
        ROBOT_Move(7, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

        /* ---- 到位②：走近阶梯，前测距到 90mm 就停 ---- */
        ROBOT_MoveSpeed(0, SPD_AVG_V);
        /* ★提前 20mm 停（不是读到 90 才停）：GY-53 一次读数 ≈200ms，10cm/s 逼近时“读到 ≤90mm”
                    那一刻车还会再往前冲 ≈2cm；提前量补上，停稳后就正好落在 90mm 附近。
                    （这里只是“从远处走到车能看清单个坑”的粗到位；后面第1~8坑都按
                      JIETI_FWD_TARGET_MM 再做一次前后校准(M2)，第1坑也不例外） */
        WAIT_WHILE(GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin) > (JIETI_FWD_TARGET_MM + 20),
                   "JIETI walk-to-ladder(stop at 110mm=90+lead20)");
        ROBOT_MoveSpeed(0, 0);

        /* ---- M5.1 机械臂到识别位 ---- */
        runActionGroup(54, 1);
        HAL_Delay(2000);

        /* ---- M1.4 视觉通信：清残留帧 → 发 0xA3 告诉主视觉"进阶梯阶段"（之后它会按 A3 包上报每个物块的 cmd）----
             ★发完先等 300ms：视觉切到“阶梯上报节奏”要一点时间，而第1坑紧接着就读 cmd(M1.5)，
               没有第2~8坑那种“走位+校准”的余量，等一下才不会把第1坑读成“不夹”。 */
        JieTi_FlushVision();
        UART2_Printf("%c", 0xA3);
        HAL_Delay(300);

        /* ==================== 8 个坑循环 ====================
                    每坑：走固定距离(M3，第1坑不走位) → 逐坑校准(M2，先左右后前后，★第1~8坑都做) → 读这一坑“夹不夹”(M1.5)
                          → 计数/显示(M3.3) → 按需夹取(M5.2)
                    · 位置：第1坑守着“进阶梯到位”点先校准再读数；第2~8坑每坑走完固定距离后校准(左右看视觉x、前后看前测距)。
                    · 朝向：M3.1 转到“正对阶梯”的绝对角一次；之后一律直接 ROBOT_MoveSpeed(底盘原生锁当前朝向)，平移期间角度环照常纠偏。
                    · cmd：校准完再等一帧(★第1坑等到 JIETI_CMD_WAIT_MS，其余坑等一轮 JIETI_VIS_MS)；
                           帧里坐标可能填0(没目标也发帧) → 等不到/坐标0 按不夹处理，★但这不影响校准：校准已经先做完了。
                    夹取动作组：第1~2个→57 / 第3~6个→60 / 第7~8个→63（对应哪种阶梯见动作组那行注释），夹完一律 66 回识别位。
                    数据包(7字节)：A3 | cmd | x低 | x高 | y低 | y高 | 0x0B；cmd 含义看全局 JieTi_Grab_Mode。 */
        for(uint8_t idx = 1; idx <= 8; idx++){                  //第1~8个坑
          /* ---- M3.2 到下一个坑：走固定距离（★第1坑守着“进阶梯到位”点，不走位）---- */
          if(idx > 1){
            ROBOT_MoveSpeed(0, 0);
            /* 换阶梯那一步(第2→3个、第6→7个，就是动作组 57/60/63 切换处)隔得远，走 JIETI_STEP_CROSS_CM；
                            同一个阶梯里相邻坑走 JIETI_STEP_CM */
            int32_t step_cm = (idx == 3 || idx == 7) ? JIETI_STEP_CROSS_CM : JIETI_STEP_CM;
            ROBOT_Move(step_cm, 0, JIETI_STEP_SPEED, 0, JIETI_STEP_ACC, 0);
            HAL_Delay(750);                       //停稳再校准
          }

          /* ---- M2 逐坑校准：★第1~8坑都做（先左右后前后），跟“这一坑要不要夹”无关 ---- */
          JieTi_FlushVision();                  //先清掉走位途中的旧帧：校准只看停下来之后的新帧
          JieTi_Adjust_X();                     //先校准左右：视觉x，容差30（等不到帧/坐标填0 → 内部跳过，不动）
          HAL_Delay(750);
          JieTi_Adjust_Y(JIETI_FWD_TARGET_MM);  //再校准前后：前测距，容差20
          HAL_Delay(750);

          /* ---- M1.5 读这一坑"夹不夹"：先按"不夹"，清掉校准期间的旧帧，再等一帧 cmd ----
             （视觉协议：帧里坐标可能填0(没目标也会发帧，见文件头)→“等不到/坐标0”一律按不夹处理）
             ★第1坑：等帧前没有“走位+校准”那段余量，按 JIETI_VIS_MS 反复等到总时限 JIETI_CMD_WAIT_MS；
               第2~8坑照旧只等一轮 JIETI_VIS_MS。 */
          jieti_cmd = 0;
          JieTi_FlushVision();
          if(idx == 1){
            uint32_t vis_t0 = HAL_GetTick();
            do{
              if(JieTi_GetVision(JIETI_VIS_MS)) break;      //拿到带 A3 包的一帧就走
            }while(HAL_GetTick() - vis_t0 < JIETI_CMD_WAIT_MS);
          }else{
            JieTi_GetVision(JIETI_VIS_MS);
          }

          /* ---- M3.3 计数 + 屏幕：处理到第 idx 个坑 ---- */
          jieti_blk_now = idx;
          JieTi_ShowBlockNo(idx, 8, jieti_cam_x);

          /* ---- M1.5 要不要夹：Mode1 cmd==0x01；Mode2 cmd低4位非0 ----
             ★读 cmd 前已把 jieti_cmd 清 0（上面那段），所以“这一坑一帧都没收到”时这里就是 0 → 不夹；
               ★不夹不等于不校准：校准已经在上面 M2 做完了(第1坑也一样) */
          uint8_t need = (JieTi_Grab_Mode == 1) ? (jieti_cmd == 0x01)
                                                : ((jieti_cmd & 0x0F) != 0);
          if(JIETI_VIS_LOG)                          //日志：这一坑视觉给的是啥(含原始x) + 最终判定(对着上面几行RX2看)
            UART1_Printf("BLK %u/8 cmd=0x%02X px=%u need=%u\r\n",
                         (unsigned)idx, (unsigned)jieti_cmd,
                         (unsigned)jieti_cam_px, (unsigned)need);   //px=0 → 这一帧视觉没检测到目标(填充值)

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
        ROBOT_MoveSpeed(0, 0);
       
        //单独的机械臂抬起
        runActionGroup(151, 1);
        HAL_Delay(1000);
        
        JieTi_Flag = 0;                     //M6.3 阶梯结束
        /* ★阶梯跑完去哪儿：就按上面那个开关 JIETI_GO_LIZHU 走（改一个数就能切回来）
                    1 → 先进“立柱”段（绕柱 → 仓库倒方块 → 回家；★正常流程）   0 → 跳过立柱、直接进“回家”段
                    ★这里必须用“赋值”，别写成 ==（历史上写成 == 导致阶段标志全是 0、车停在阶梯不动） */
        if(JIETI_GO_LIZHU) LiZhu_Flag = 1;  //去立柱（立柱段末尾自己会置 HuiJia_Flag=1 接回家）
        else               HuiJia_Flag = 1; //直接回家

        if(LiZhu_Flag == 1)//立柱开始
        {
          
          runActionGroup(160, 1);//复位
          ROBOT_Move(20, -150, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

          //更换新跑图逻辑
          // （原“阶梯→立柱”的走位/转向测试行已删：-45,-35 走位 + ROBOT_Angle(Yaw_Abs(270)) / 180）

    /* ★调试入口(KEY0选成 LZ + KEY3开始)：goto 跳到这一行往下执行“立柱”段。
            上面那条“走到立柱附近(-45,-35) + 转向”是给“阶梯→立柱”用的；单独测立柱时车已经按
            “立柱起点姿态”(红方车头朝右=正对柱子、就在立柱右边一点)摆好了，所以跳过它们，直接进下面的校准。
            ★摆车姿态就是红方车头朝右(90) / 蓝方车头朝左(270)，与 DBG_START_LIZHU 里的 SetYawShift(90) 对应；
              所以绕完一圈后那句 ROBOT_Angle(Yaw_Abs(90)) 只把绕圈攒下的几度掰回来(正对柱子/朝右)，
              不会再出现“绕完 355° 后又多转 180°、车头朝反方向”的情况。 */
LIZHU_START:

          runActionGroup(101, 1);//机械臂抬起

          /* ====== 立柱前校准（2026-09-21 简化：只有两步）======
                       前提(现场保证)：车在立柱右边一点。
                       ① 往左慢走(5cm/s)，直到“左激光(LASER4)连续 3 次(每次间隔30ms)看到障碍物” → 已经左右正对柱面
                         左4右2（LASER4=左、LASER2=右；这一步只判左4，右2不参与——右2是绕圈那套 l4/r2 反馈用的）
                       ② 再以 5cm/s 向前逼近，直到前测距 ≤200mm → 前后到位
                       然后直接 LiZhu_Circle_Run() 开始绕圈（它拿这时读到的距离当参考半径）。
                       ★两处都用 5cm/s：GY-53 是阻塞读(≈200ms)、激光也要车慢慢靠过去才不会冲过头
                         （10cm/s 会冲过头，和阶梯那套“走多”是同一个原因）。
                       ★两处等待都带 6s 超时兜底(WAIT_TIMEOUT_MS)：超时打一行 TIMEOUT 后继续走，不会卡死。 */
          ROBOT_MoveSpeed(-5.0f, 0.0f);                     //① 往左慢走
          /* ★左激光(LASER4)连续 3 次都“看到障碍物”才停车（只判左4，右2不参与）：
                       单次可能被反光/噪声误触，所以连看到 3 次才算数、每次间隔 30ms（≈90ms 去抖，
                       判定期间车 5cm/s 只多走 4.5mm）。仍带 WAIT_TIMEOUT_MS 兜底：激光没接/坏了不会永久卡死 */
          {
            uint32_t lz_t0 = HAL_GetTick();  uint8_t lz_hit = 0;   //连续"看到"计数
            while(lz_hit < 3){
              lz_hit = LASER_Barrier(LASER4_GPIO_Port, LASER4_Pin) ? (uint8_t)(lz_hit + 1) : 0;  //断一次就重新数
              if(lz_hit >= 3) break;
              HAL_Delay(50);                                       //每次检测间隔 50ms
            }
          }
          ROBOT_MoveSpeed(0.0f, 0.0f);                      //停车

          ROBOT_MoveSpeed(0.0f, 5.0f);                      //② 向前慢逼近到 200mm，180也太远
          WAIT_WHILE(GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin) > 170U,
                     "LiZhu front distance -> 170mm");
          ROBOT_MoveSpeed(0.0f, 0.0f);                      //停车，准备转圈

          runActionGroup(104, 1);//识别状态
          HAL_Delay(2500);

          //立柱转圈（2026-09-26 重写：开环三旋钮 + 测距/激光两路可选反馈，见 LiZhu_Circle_Run 函数头）
          //★前提：车头已经正对着柱子（函数开头就是静止采测距定参考距离）
          LiZhu_Circle_Run();

          /* ★★ 2026-09-29：绕完一圈后的“原地夹取”任务(动作组110 → 视觉识别 → 需要就113 → 160复位)
             ★不写在这里：它已经挂进 LiZhu_Circle_Run() 的收尾(绕圈循环之后、发 0xA9 之前)，这样
               【正式流程的立柱段】和【菜单里单独发指令 7 / 单键测试绕圈】跑的是同一段代码，两边都会夹。
               顺序/时长/开关见该函数头 + “立柱阶段：视觉通信”那一段的 LIZHU_POST_* 常量(main.c:492~503)。
             ★本段下面(原来的 HAL_Delay(1000) + ROBOT_Angle(Yaw_Abs(90)) + 走到仓库倒方块)顺序不变。 */

          /*
                    识别钩子已搬进 LiZhu_Circle_Run() 的绕圈 while 里：
                    遇到可以夹的就在那里 break，车停下、收尾照常执行

                    ★立柱视觉通信(2026-09-27 加入)同样在 LiZhu_Circle_Run() 里，本段不用再管：
                      开圈前发 0xA4 → 边绕边收视觉包(A3 也算) → cmd≠0 就停车跑动作组 107 → 接着绕
                      → 绕完一整圈发 0xA9。协议/开关/日志见该函数上方“立柱阶段：视觉通信”那一段。
                    */

          //转完一圈，收起机械臂，然后往左转身走到仓库中间倒方块（更换新逻辑）
          // （原立柱→仓库的几条走位测试行已删：-60 左移 / ROBOT_Angle(Yaw_Abs(90)) / -138 后退 / -45 左移）

          HAL_Delay(1000);

          //转完一圈，纠正角度
          ROBOT_Angle(Yaw_Abs(90));//车子前面朝右

          //先后退到合适距离
          /* ★9.25 修方向：原来是 +SPD_AVG_V(v_y>0=前进)，但注释写“后退”、判据又是“后测距(GY53_1)
                       ≤85mm(贴后墙)” —— 前进只会让后测距变大、永远不满足，只能靠 6s 超时兜底往前冲 60cm。
                       改成 -SPD_AVG_V：真后退，后测距一路变小，到 85mm 自然停。 */
          ROBOT_MoveSpeed(0.0f, -SPD_AVG_V);
          WAIT_WHILE(GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin)>45,
                 "LiZhu back-to-wall(mm<=45)");

          //向左慢平移到左后光电感应到无障碍物，之后再往右走固定距离(刚到仓库中间的距离)
          ROBOT_MoveSpeed(-5, 0);   //★匀速靠近：SPD_AVG_V
          
          {
            uint32_t lz_t0 = HAL_GetTick();  uint8_t lz_hit = 0;   //连续"看到"计数
            while(lz_hit < 3){
              lz_hit = !LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin) ? (uint8_t)(lz_hit + 1) : 0;  //断一次就重新数
              if(lz_hit >= 3) break;
              HAL_Delay(50);                                       //每次检测间隔 50ms
            }
          }
          
          ROBOT_MoveSpeed(0,0);
          ROBOT_Move(30, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);

          ROBOT_Angle(Yaw_Abs(90));//校准一下
          
          //这里放倒方块的代码
          runActionGroup(16, 1);//倒方块动作组(复用圆盘机)
          HAL_Delay(200);//倒方块动作约200ms

          for(uint8_t i = 0; i < 3; i++)
          {
            ROBOT_MoveSpeed(0, 30);
            HAL_Delay(90);
            ROBOT_MoveSpeed(0, -30);
            //往后速度稍微大点，抵消掉往前的一小点位移
            HAL_Delay(90);
          }
          ROBOT_MoveSpeed(0, 0);
          HAL_Delay(500);

          for(uint8_t i = 0; i < 3; i++)
          {
            ROBOT_MoveSpeed(0, 30);
            HAL_Delay(90);
            ROBOT_MoveSpeed(0, -30);
            //往后速度稍微大点，抵消掉往前的一小点位移
            HAL_Delay(90);
          }

          ROBOT_MoveSpeed(0, 0);
          runActionGroup(19, 1);//收倒球槽
          HAL_Delay(1000);

          LiZhu_Flag = 0;//立柱结束，回家开始
          HuiJia_Flag = 1;
        }



    /* ★调试入口(KEY0选成HOME+KEY3开始)：goto 跳到下面 HUIJIA_START 标签往下执行“回家”
           （HuiJia_Flag 已在跳转前立好；正常流程走到这里时一字不变） */
HUIJIA_START:
    if(HuiJia_Flag == 1)//回家开始
    {

          //往前走确保转方向不卡脚（★短距 15cm：20/30）
          ROBOT_Move(0, 17, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A); 

          //倒完方块转正再回家
          ROBOT_Angle(Yaw_Abs(0));
          //往后多走一点(★长距档 120/120)，必须保证前后在左右移动后能进入红/蓝区域
          ROBOT_Move(mode_red ? 28 : -28, -228, SPD_LONG_V, SPD_LONG_V, SPD_LONG_A, SPD_LONG_A);//60，-240能进

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

    //进入红/蓝后，继续向右/左多走 7cm，确保停在红/蓝区域内部（避免停在边缘抖动；距离按区域宽度调整），而且确保车身左右都在红/蓝区域内
    //★短距档 40/50（2026-09-27 统一改用宏；7cm 三角波峰值 √(50×7)=19cm/s，若现场发现走不到位，把这里的第5/6个参数(a)单独加大到 100~200）
    ROBOT_Move(mode_red ? 9 : -9, 0, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
    
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
    
    //识别为红/蓝后继续向后多走3cm，确保停在红/蓝区域内部（避免停在边缘抖动；距离按区域宽度调整），而且确保车身前后都在红/蓝区域内
    //★短距档 40/50（原手写 5/5：5cm/s 第一拍根本推不动，白走一趟；3cm 三角波峰值 √(50×3)=12cm/s，
    //  仍在起转阈值边缘 ⇒ 若现场发现这条走不动，把它第5/6个参数(a)单独加大到 200，峰值就有 24cm/s）
    ROBOT_Move(0, -3, SPD_SHORT_V, SPD_SHORT_V, SPD_SHORT_A, SPD_SHORT_A);
    ROBOT_MoveSpeed(0, 0);
        }
      }

    }
    /* 原来这里有一句无条件的 UART1_Data[0]=0;，它把刚解析出来的指令提前清成 0，
           所以“发指令没反应”。已删除：每条指令在处理时自己会清零。 */

    //立柱转圈 / 走距 / 转角 这些调试指令现在全部在 UART1_DebugCmd() 里解析执行（发 7,0,0,0,0,0,0,0 就地绕圈），
    //  所以本处不需要任何 if；另外原来紧跟这里的那段"6 号指令"（回家找红蓝区的旧版副本，与回家阶段代码重复）已删除。
    //  下面的注释只留 LiZhu_Circle_Run 的绕法说明：
    //  前提：车头已正对柱子（函数开头会静止采12次测距取中值当目标距离 d_ref，不是写死的 180mm）
    //  绕法（★2026-09-26 重写：不用增益，只有“速度”）：
    //       开环基准：切向 v_x=V_TAN + 车头摆速 w=W_TURN(°/s) → 圆半径 r=V_TAN/(W_TURN×π/180)，天然以柱子为圆心
    //       测距反馈(FB_DIST=1)：测距偏大→往前靠、偏小→往后退，满幅 3cm 误差 → v_y=V_RAD cm/s
    //       双激光反馈(FB_LASER=1)：左4右2，一个有一个没有=横向偏了 → 摆速加 LAS_YAW、切向速度加 LAS_TAN
    //       两个开关都置 0 = 纯开环（只有三个旋钮）
    //  串口(200ms/条)：d=测距mm e=径向误差mm l4/r2=左/右激光 vx=切向速度 vy=径向速度(0.1cm/s) w=角速度×100 yaw=已绕角度(°)

    // /* 原“每100ms打印角度环数据(目标/实际/输出w + 里程计x/y) 并解析串口调参指令”的调试代码已删：
    //      角度环 kp/ki/kd/target、vx/vy、mx/my、mv/mvacc 等调参现在走 UART1_DebugCmd / SerialPlot */
   
    // （原“串口1打两个测距 + OLED 打印测距 + OLED_Update + HAL_Delay(10)”的调试行已删）


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
