/**
  * @brief 该文件专门放置各类模块的测试代码，方便快速回顾某一模块的用法
  */
#include "stm32f4xx_hal.h"

//led测试
// LED_ON(LED0_GPIO_Port, LED0_Pin); 
// HAL_Delay(500);
// LED_OFF(LED0_GPIO_Port, LED0_Pin);
// HAL_Delay(500);

//uart测试
// if(UART1_RxFlag){
//   OLED_Printf(0, 0, OLED_8X16, "%X %X %X", UART1_RxBuf[0], UART1_RxBuf[1], UART1_RxBuf[2]);
//   OLED_Update();
//   UART1_RxFlag = 0;
// }

//key测试
// if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
//   OLED_ShowString(0, 0, "key0", OLED_6X8);
//   OLED_Update();
// }
// b = KEY_ALL();
// if(b != KEY_NO){
//   OLED_Printf(0, 0, OLED_8X16, "key%d", b);
//   OLED_Update();
// }

//mpu6050测试
// MPU6050DMP_GetAngle();
// OLED_ShowFloatNum(0, 0, MPU6050_Data.pitch, 2, 1, OLED_8X16);
// OLED_ShowFloatNum(0, 20, MPU6050_Data.roll, 2, 1, OLED_8X16);
// OLED_ShowFloatNum(0, 40, MPU6050_Data.yaw, 2, 1, OLED_8X16);
// OLED_Update();
// MPU6050_GetAcc();
// MPU6050_GetGyros();
// OLED_ShowFloatNum(0, 0, MPU6050_Data.accx, 2, 1, OLED_6X8);
// OLED_ShowFloatNum(0, 6, MPU6050_Data.accy, 2, 1, OLED_6X8);
// OLED_ShowFloatNum(0, 12, MPU6050_Data.accz, 2, 1, OLED_6X8);
// OLED_ShowFloatNum(0, 18, MPU6050_Data.acc, 2, 1, OLED_6X8);
// OLED_ShowFloatNum(0, 24, MPU6050_Data.gyrosx, 2, 1, OLED_6X8);
// OLED_ShowFloatNum(0, 30, MPU6050_Data.gyrosy, 2, 1, OLED_6X8);
// OLED_ShowFloatNum(0, 36, MPU6050_Data.gyrosz, 2, 1, OLED_6X8);
// OLED_Update();
// HAL_Delay(100);

//灰度传感器测试
// OLED_ShowNum(0, 0, Gray_ONE(GRAY1), 1, OLED_6X8);
// OLED_ShowNum(8, 0, Gray_ONE(GRAY2), 1, OLED_6X8);
// OLED_ShowNum(16, 0, Gray_ONE(GRAY3), 1, OLED_6X8);
// OLED_ShowNum(24, 0, Gray_ONE(GRAY4), 1, OLED_6X8);
// OLED_ShowNum(32, 0, Gray_ONE(GRAY5), 1, OLED_6X8);
// OLED_ShowNum(40, 0, Gray_ONE(GRAY6), 1, OLED_6X8);
// OLED_ShowNum(48, 0, Gray_ONE(GRAY7), 1, OLED_6X8);
// OLED_ShowNum(56, 0, Gray_ONE(GRAY8), 1, OLED_6X8);
// OLED_Update();

//w25q64测试--失败，根本就没法建立通信，可能是模块本身有问题
// if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
//   a++;
//   UART1_printf("%c\r\n", a);
// }
// if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){
//   W25Q64_Save(a);
//   UART1_printf("Save\r\n", a);
// }
// if(KEY_ONE(KEY2_GPIO_Port, KEY2_Pin)){
//   W25Q64_Read();
//   b = W25Q64_Read();
//   UART1_printf("Read :%c\r\n", b);
// }

//buzzer测试
// BUZZER_On();
// HAL_Delay(500);
// BUZZER_OFF();
// HAL_Delay(500);    

//laser测试（激光漫反射传感器）--识别颜色成功，识别物体电位器调不出来（可尝试采用看起来较为容易调试的第五代激光漫反射传感器）
// if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
//   UART1_printf("%d", (uint32_t)LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin));
// }
// if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){
//   UART1_printf("%d", (uint32_t)LASER_White(LASER2_GPIO_Port, LASER2_Pin));
// }

//infrared测试（红外传感器）
// if(INFRARED_Barrier(SENSOR3_GPIO_Port, SENSOR3_Pin)){
//   OLED_ShowString(0, 0, "Have", OLED_8X16);
//   OLED_Update();
// }else{
//   OLED_ShowString(0, 0, "No  ", OLED_8X16);
//   OLED_Update();
// }
// delay_ms(100);

//蓝牙间通信测试
// if(BT_RxFlag){
//   OLED_ShowHexNum(0, 0, BT_RxBuf[0], 2, OLED_8X16);
//   OLED_ShowHexNum(0, 20, BT_RxBuf[1], 2, OLED_8X16);
//   OLED_Update();
//   BT_RxFlag = 0;
// }

//测距模块GY53测试
// // （连续模式+串口中断接收）
// if(GY53_RxFlag){
//   UART1_Printf("%d\r\n", GY53_Distance);
//   GY53_RxFlag = 0;
// }
// //（连续模式+PWM模式）    
// OLED_ShowNum(0, 0, GY53_GetDistance_PWM(SENSOR1_GPIO_Port, SENSOR1_Pin), 5, OLED_8X16);
// OLED_Update();

//ultrasonic测试
// if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
//   OLED_ShowFloatNum(0, 0, ULTRASONIC_GetDistance(), 4, 2, OLED_8X16);
//   OLED_Update();
// }

//servo测试(记得先开启TIM的PWM功能)
// HAL_Delay(500);
// SERVO_Control(SERVO1, angle);
// angle += 20;
// if(angle > 180) angle = 0;

//MG513X电机测试（tb6612+霍尔encoder）
// if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
//   OLED_Printf(0, 0, OLED_8X16, "Left: %+04d", speed_l);
//   OLED_Update();
//   TB6612_Control(MOTOR_Left, speed_l);
//   speed_l += 20;
//   if(speed_l > 100) speed_l = -100;
// }
// if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){
//   OLED_Printf(0, 16, OLED_8X16, "Right:%+04d", speed_r);
//   OLED_Update();
//   TB6612_Control(MOTOR_Right, speed_r);
//   speed_r += 20;
//   if(speed_r > 100) speed_r = -100;
// }    
// OLED_Printf(0, 32, OLED_8X16, "L-Enco:%+05d", ENCODER_GetPulse(ENCODER_Left));
// OLED_Printf(0, 48, OLED_8X16, "R-Enco:%+05d", ENCODER_GetPulse(ENCODER_Right));
// OLED_Update();

//步进电机测试（X42S驱动模块+EMM固件）（使用时利用DMA+串口空闲中断，注意关闭DMA半传输中断）
// HAL_Delay(500);   //等待系统稳定
// Emm_V5_Origin_Set_O(1, 0);  //设置当前位置为零点
// while(!STEP_RxFlag); STEP_RxFlag = 0;
// HAL_Delay(500);
// Emm_V5_Pos_Control(1, 0, 2000, 0, 1600, 0, 0);  //设置位置
// while(!STEP_RxFlag); STEP_RxFlag = 0;
// HAL_Delay(1000);
// Emm_V5_Origin_Trigger_Return(1, 0, 0);  //设置回零
// while(!STEP_RxFlag); STEP_RxFlag = 0;
// HAL_Delay(1000);
// Emm_V5_Read_Sys_Params(1, S_CPOS);  //读取实时位置
// while(!STEP_RxFlag); STEP_RxFlag = 0;
// if(STEP_RxRealLength == 8 && STEP_RxBuf[0] == 1 && STEP_RxBuf[1] == 0x36){
//   uint32_t pos;
//   float angle;
//   pos = (uint32_t) (
//     (uint32_t)STEP_RxBuf[3]<<24 |
//     (uint32_t)STEP_RxBuf[4]<<16 |
//     (uint32_t)STEP_RxBuf[5]<<8  |
//     (uint32_t)STEP_RxBuf[6]<<0
//   );
//   angle = (float)pos * 360.0f / 65536.0f;
//   OLED_ShowFloatNum(0, 0, angle, 4, 3, OLED_8X16);
//   OLED_Update();
// }
// Emm_V5_Vel_Control(1, 0, 2000, 200, 0); //设置为速度模式
// while(!STEP_RxFlag); STEP_RxFlag = 0;
// HAL_Delay(500);
// Emm_V5_Stop_Now(1, 0);  //设置为立即停止
// while(!STEP_RxFlag); STEP_RxFlag = 0;
// Emm_V5_Vel_Control(1, 0, 2000, 200, 0); //设置为速度模式
// while(!STEP_RxFlag); STEP_RxFlag = 0;
// HAL_Delay(500);
// Emm_V5_En_Control(1, 0, 0); //失能电机
// while(!STEP_RxFlag); STEP_RxFlag = 0;
// Emm_V5_En_Control(1, 1, 0); //使能电机
// while(!STEP_RxFlag); STEP_RxFlag = 0;
// Emm_V5_Read_Sys_Params(1, S_VEL); //读取实时转速
// while(!STEP_RxFlag); STEP_RxFlag = 0;
// if(STEP_RxRealLength == 6 && STEP_RxBuf[0] == 1 && STEP_RxBuf[1] == 0x35){
//   uint16_t speed;
//   speed = (uint16_t)(
//     (uint16_t)STEP_RxBuf[3]<<8  |
//     (uint16_t)STEP_RxBuf[4]<<0
//   );
//   OLED_ShowNum(0, 16, speed, 5, OLED_8X16);
//   OLED_Update();
// }
// HAL_UARTEx_ReceiveToIdle_DMA(&huart5, STEP_RxBuf, STEP_RxLength);
// HAL_Delay(500);   //等待系统稳定
// Emm_V5_Origin_Set_O(1, 1);  //设置当前位置为零点
// HAL_Delay(10);
// Emm_V5_Origin_Set_O(2, 1);  //设置当前位置为零点
// HAL_Delay(10);
// Emm_V5_Synchronous_motion(0);
// HAL_Delay(10);
// HAL_Delay(500);
// /**************************************/
// Emm_V5_Stop_Now(1, 0);
// HAL_Delay(10);
// Emm_V5_MMCL_Vel_Control(1, 1, 2000, 0, 0);  //依靠多电机命令实现多电机运动
// Emm_V5_Multi_Motor_Cmd(0);
// /**************************************/

//pid测试
///*下面是变量定义*/
// int32_t speed, location;
// PID_POS pid_speed = {
//   .Kp = 1.5,
//   .Ki = 0.34,
//   .Kd = 0.0,
//   .out_max = 100,
//   .out_min = -100
// };
// PID_POS pid_position = {
//   .Kp = 0.12,
//   .Ki = 0.0,
//   .Kd = 0.11,
//   .out_max = 35,
//   .out_min = -35
// };
///*下面是tim中断*/ 
// count1++;
// if(count1 >= 10){ //10ms进行一次调节 速度环
//   count1 = 0;
//   speed = ENCODER_GetPulse(ENCODER_Left);
//   location += speed;
//   /*获取实际值*/
//   pid_speed.actual = speed;
//   PID_PosUpdate(&pid_speed);
//   /*执行控制*/
//   TB6612_Control(MOTOR_Left, pid_speed.out);    
// }
// count2++;
// if(count2 > 40){ //40ms进行一次调节 位置环
//   count2 = 0;
//   /*获取实际值*/
//   pid_position.actual = location;
//   /*PID运算*/
//   PID_PosUpdate(&pid_position);
//   pid_speed.target = pid_position.out;
// }
///*下面是while循环*/
// if(UART1_RxFlag){
//   SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
//   UART1_RxFlag = 0;
// }
// OLED_Printf(0, 0, OLED_6X8, "tar:%+03.0f", pid_position.target);
// OLED_Printf(0, 8, OLED_6X8, "act:%+03.0f", pid_position.actual);
// OLED_Printf(0, 16, OLED_6X8, "out:%+03.0f", pid_position.out);
// OLED_Printf(0, 24, OLED_6X8, "Kp:%+05.2f", pid_position.Kp);
// OLED_Printf(0, 32, OLED_6X8, "Ki:%+05.2f", pid_position.Ki);
// OLED_Printf(0, 40, OLED_6X8, "Kd:%+05.2f", pid_position.Kd);
// OLED_Printf(60, 0, OLED_6X8, "tar:%+03.0f", pid_speed.target);
// OLED_Printf(60, 8, OLED_6X8, "act:%+03.0f", pid_speed.actual);
// OLED_Printf(60, 16, OLED_6X8, "out:%+03.0f", pid_speed.out);
// OLED_Printf(60, 24, OLED_6X8, "Kp:%+05.2f", pid_speed.Kp);
// OLED_Printf(60, 32, OLED_6X8, "Ki:%+05.2f", pid_speed.Ki);
// OLED_Printf(60, 40, OLED_6X8, "Kd:%+05.2f", pid_speed.Kd);
// OLED_Printf(0, 48, OLED_6X8, "pos");
// OLED_Printf(60, 48, OLED_6X8, "speed");
// OLED_Update();
// UART1_Printf("%f %f %f %f\r\n", pid_position.target, pid_position.actual, pid_position.out, pid_position.errorint);

//icm42688测试
// OLED_ShowNum(0, 0, ICM42688Mahony_Init(), 1, OLED_8X16);   //初始化+查询部分
// OLED_Update();
// // HAL_TIM_Base_Start_IT(&htim7);//记得初始化结束再开启更新
// ICM42688Mahony_GetAngle();                                 
// UART1_Printf("%.1f %.1f %.1f\r\n", ICM42688Data.pitch, ICM42688Data.roll, ICM42688Data.yaw);
// if(count2 >= 5){                                           //tim部分
//   count2 = 0;
//   // ICM42688Mahony_Update();
// }
///*上面是用了mahony算法，下面为不加mahony算法*/
// OLED_ShowNum(0, 0, ICM42688_Init(), 1, OLED_8X16);
// OLED_Update();
// ICM42688_GetAcc();
// ICM42688_GetGyros();
// UART1_Printf("ax:%05.2f ay:%05.2f az:%05.2f a_all:%05.2f gx:%05.2f gy:%05.2f gz:%05.2f\r\n", ICM42688Data.accx, ICM42688Data.accy, ICM42688Data.accz,
//                     ICM42688Data.acc, ICM42688Data.gyrosx, ICM42688Data.gyrosy, ICM42688Data.gyrosz);

//myflash测试
// OLED_ShowHexNum(0, 0, MYFLASH_ReadByte(0x08009900), 2, OLED_8X16);
// OLED_ShowHexNum(0, 16, MYFLASH_ReadHalfWord(0x08009900), 4, OLED_8X16);
// OLED_ShowHexNum(0, 32, MYFLASH_ReadWord(0x08009900), 8, OLED_8X16);
// OLED_Update();
// MYFLASH_EraseSector(FLASH_SECTOR_7);
// MYFLASH_ProgramWord(0x08060000, 0x12345678);
// OLED_ShowHexNum(0, 48, MYFLASH_ReadWord(0x08060000), 8, OLED_8X16);
// OLED_Update();

//storage测试
// STORAGE_Init();//从flash加载数据到sram结构体
// while(1){
//   OLED_ShowHexNum(0, 0, STORAGE_Data.flag, 8, OLED_8X16);
//   OLED_ShowFloatNum(0, 16, STORAGE_Data.line_kp, 3, 3, OLED_8X16);
//   OLED_ShowFloatNum(0, 32, STORAGE_Data.line_ki, 3, 3, OLED_8X16);
//   OLED_ShowFloatNum(0, 48, STORAGE_Data.line_kd, 3, 3, OLED_8X16);
//   OLED_Update();
//   if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
//     STORAGE_Data.line_kp += 0.1;
//   }
//   if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){
//     STORAGE_Data.line_ki += 0.1;
//   }
//   if(KEY_ONE(KEY2_GPIO_Port, KEY2_Pin)){
//     STORAGE_Data.line_kd += 0.1;
//   }
//   if(KEY_ONE(KEY3_GPIO_Port, KEY3_Pin)){
//     STORAGE_Save();
//     OLED_Clear();
//     OLED_ShowString(0, 0, "OK!", OLED_8X16);
//     OLED_Update();
//     break;
//   }
// }

//oled_ui测试
//main函数中
// OLED_UI_Init(&MainMenuPage);
// while(1){
//   OLED_UI_MainLoop();
// }
//20ms中断中
// OLED_UI_InterruptHandler();

// 脱机调参功能（基于oled_ui+storage模块）
// STORAGE_Init();//把数据从flash加载到存储模块中
// OLED_UI_Init(&MainMenuPage);
// if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){//如果上电时按下了key0，则进入多级菜单调参
//   flag.oled_ui = true; //启动20ms调用更新函数
//   extern bool oled_ui_exit;
//   while(1){
//     OLED_UI_MainLoop();
//     if(oled_ui_exit == true){//退出oled菜单
//       flag.oled_ui = false;
//       STORAGE_Save(); //把调好的参数保存到flash中
//       break;
//     }
//   }
// }
// line.kp = STORAGE_Data.line_kd;//把存储模块中保存的数据放置到原来的变量中
// line.ki = STORAGE_Data.line_ki;
// line.kd = STORAGE_Data.line_kd;
// angle.kp = STORAGE_Data.angle_kp;
// angle.ki = STORAGE_Data.angle_ki;
// angle.kd =  STORAGE_Data.angle_kd;
// speed_left.kp = STORAGE_Data.speed_left_kp;
// speed_left.ki = STORAGE_Data.speed_left_ki;
// speed_left.kd = STORAGE_Data.speed_left_kd;
// speed_right.kp = STORAGE_Data.speed_right_kp;
// speed_right.ki = STORAGE_Data.speed_right_ki;
// speed_right.kd = STORAGE_Data.speed_right_kd;
// angle_offset = STORAGE_Data.angle_offset;
// encoder_cnt_odd  = STORAGE_Data.encoder_cnt_odd;
// encoder_cnt_even = STORAGE_Data.encoder_cnt_even;
// OLED_Clear();
// OLED_ShowString(0, 0, "Save Success", OLED_8X16_HALF);
// OLED_Update();
// while(1);

//jy901s陀螺仪测试
// HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RxBuf, UART2_RxLength);//jy901s串口
// __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
// while(1){
//   if(UART2_RxFlag){
//     UART2_RxFlag = 0;
//     JY901S_Update(UART2_RxBuf);
//     UART1_Printf("gyrox:%+6.2f gyroy:%6.2f gyroz:%6.2f pitch:%6.2f roll:%6.2f yaw:%6.2f\r\n", JY901S_Data.gyrosx,
//       JY901S_Data.gyrosx, JY901S_Data.gyrosz, JY901S_Data.pitch, JY901S_Data.roll, JY901S_Data.yaw);
//   }
// }

//pid调参测试
// while(1){
//   OLED_Printf(0, 0, OLED_8X16, "kp:%06.2f", vision_x.kp);
//   OLED_Printf(0, 16, OLED_8X16, "ki:%06.2f", vision_x.ki);
//   OLED_Printf(0, 32, OLED_8X16, "kd:%06.2f", vision_x.kd);
//   OLED_Printf(0, 48, OLED_8X16, "tar:%06.2f", vision_x.target);
//   OLED_Update();
//   UART1_Printf("%f %f %f %f\r\n", vision_x.target, vision_x.actual, vision_x.out, vision_y.errorint);
//   if(UART1_RxFlag){
//     UART1_RxFlag = 0;
//     SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
//   }
// }

/****************************模块测试***************************************/
