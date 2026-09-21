# 底盘速度环改造设计：增量式 PID（执行逻辑回归旧代码）

日期：2026-08-20

## 背景

今年把去年 F103（标准外设库）自动分拣机器人代码移植到 STM32F407 + HAL + VSCode。此前已用位置式 PI 实现速度环（`chassis.c`），但实测出现"停-冲"极限环震荡。用户确认硬件接线正常，问题是 PID 方案与参数。

用户决定：**不修改旧版底盘的执行思路**，只做结构/命名清晰化。速度环回归旧代码的**增量式 PID**（去年调好的算法与参数）。

## 已确认决策

| 决策 | 结论 |
|---|---|
| 改造范围 | 只改速度环（第一阶段）；麦轮/里程计/航向环第二阶段按旧思路再做 |
| 速度环算法 | 增量式 PID，执行逻辑逐行照搬旧代码 `IncrementalPID_Calculate` |
| 死区 | **去掉**（旧代码无死区，靠积分爬升越过静摩擦） |
| 参数初值 | `speed_p=5.2 / speed_i=0.03 / speed_d=0`（去年调好） |
| 位置式 PID_POS | **保留不删**（第二阶段航向环按旧代码也是位置式 `PositionPID_Calculate`） |
| 编码器 | 清零法（用户已确认，不变） |
| 速度换算 | 实测标定：`ENCODER_ACCURACY=878`、`WHEEL_DIAMETER=12.7`（不变） |
| 轮子编号 | 1~4 = TIM1 通道 = TB6612 编号 = 编码器编号（不变） |

## 旧代码速度环参考（`motor.c:IncrementalPID_Calculate`）

```c
void IncrementalPID_Calculate(Incremental_PID *pid,const float Target,const float Measure){
  pid->Err = Target - Measure;
  pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);
  pid->d_out = pid->Kd * (pid->Err - 2.0f*pid->Last_Err + pid->Previous_Err);
  pid->i_out += pid->Ki * pid->Err;
  if(pid->i_out > 10) pid->i_out = 10;          // 积分项限幅
  if(pid->i_out < -10) pid->i_out = -10;
  if(pid->Ki * pid->Err >-0.1 && pid->Ki*pid->Err<0.1){  // 防积死
    if(pid->i_out > 0.2) pid->i_out = 0.2;
    else if(pid->i_out <-0.2) pid->i_out = -0.2;
  }
  pid->Output += pid->p_out + pid->i_out + pid->d_out;
  pid->Output = limit(pid->Output, pid->OutputMax);  // ±1000
  pid->Previous_Err = pid->Last_Err;
  pid->Last_Err = pid->Err;
}
```

> 注意：`10.0 / 0.1 / 0.2` 三个魔法数是去年现场调出来的手感值，**必须原样保留**。

## 改动方案（方案 A：干净封装增量式）

### 1. `pid.h` —— 新增 `PID_INC` 结构体 + `PID_IncUpdate()`

```c
typedef struct{
  float target;      // 目标速度 cm/s
  float actual;      // 实测速度 cm/s
  float out;         // PWM 输出（增量累加）
  float kp, ki, kd;  // PID 参数（字段名小写，与位置式一致，serialplot 免改）
  float p_out;       // 比例项增量
  float i_out;       // 积分项（累计，限幅 ±10）
  float d_out;       // 微分项增量
  float err;         // 本次误差
  float last_err;    // 上次误差
  float prev_err;    // 上上次误差
  float out_max;     // 输出对称限幅 ±out_max
}PID_INC;

void PID_IncUpdate(PID_INC *pid);
```

位置式 `PID_POS` 与 `PID_PosUpdate` 保留。

### 2. `chassis.h` —— 类型替换 + 参数

- `Chassis.speed_pid[5]` 类型 `PID_POS` → `PID_INC`
- `SPEED_PID_KP=5.2 / KI=0.03 / KD=0.0 / OUT_MAX=1000`
- 删除 `MOTOR_DEAD_ZONE` 宏

### 3. `chassis.c` —— 控制循环改增量式

```c
void CHASSIS_Control_Loop(void){
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    PID_INC *pid = &chassis.speed_pid[i];
    int16_t pulse = ENCODER_GetPulse(i);
    pid->actual = (pulse / ENCODER_ACCURACY) * PERIMETER / ENCODER_TIME_S;
    PID_IncUpdate(pid);
    TB6612_Control(i, (int16_t)pid->out);   // 无死区
  }
}
```

### 4. `serialplot.c` —— 无需改动

`PID_INC` 字段名与位置式同名（`kp/ki/kd/target`），现有 param 表、4 轮联动赋值、5 通道发送原样可用。

### 5. `main.c` —— 不动

用户已加 `flag.chassis` 开关，`SERIALPLOT_PIDAdjustParam()` 保留。

## 验证方式

1. 编译 `cmake --build --preset Debug`
2. 车架空，SerialPlot 5 通道（target/LF/LB/RB/RF）
3. 目标 30cm/s 应平滑收敛（增量式 + 无死区，低速平滑微调而非"停-冲"）
4. 若响应偏慢/偏冲，串口在线改 `kp`/`ki`
