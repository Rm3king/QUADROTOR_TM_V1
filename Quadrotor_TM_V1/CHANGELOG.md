# Changelog

## [Stage3] 项目目录结构重组 (2026-04-30)

### 改动概述
将项目目录结构从 `App / FlightControl / Platform` 重组为符合嵌入式行业惯例的
`Application / Algorithm / Driver` 三层架构，并将算法层拆分为 Common（通用工具）、
Estimation（状态估计）、Control（闭环控制）三个子模块。

所有 `#include` 指令均为扁平文件名，因此本次重组**不涉及任何源代码改动**，
仅移动目录、更新 Keil 工程文件和源文件注释。

### 新目录结构
```
Application/  (原 App/)     — Core, Service, Mission, Vision
Algorithm/    (原 FlightControl/)  — Common, Estimation, Control
Driver/       (原 Platform/)      — BSP, Peripheral, Sensor, ExtDevice
TiDriver/     (不变)
```

### 改动详情
- **App/** → **Application/**：目录重命名，子目录结构不变
- **FlightControl/Base/** 拆分为：
  - **Algorithm/Common/**：FcData, config.h, Filter, Math, Pid
  - **Algorithm/Estimation/**：Imu, Sensor_Basic, MotionCal, Navigate
- **FlightControl/Control/** 拆分为：
  - **Algorithm/Estimation/**：MagProcess, FlightDataCal（状态估计，非控制器）
  - **Algorithm/Control/**：AltCtrl, AttCtrl, FlightCtrl, LocCtrl, MotorCtrl, Power
- **Platform/** → **Driver/**：Bsp→BSP, Device/Sensor+Misc→Sensor, Device/Nav→ExtDevice
- **Keil .uvprojx / .uvoptx**：更新全部 GroupName, FilePath, IncludePath
- **28 个源文件**：更新"所属模块"注释

### 致谢
本轮重构由 Claude (Anthropic) 辅助完成。
感谢匿名科创（ANO）提供原始飞控框架代码。
感谢指导老师对教学代码质量标准的持续指导。

---

## [Stage2-R10] 功能开关集中化 (2026-04-30)

### 改动概述
将散落在各 `.c` 文件中的编译期功能开关（`#define USE_XXX`）统一集中到
`sysconfig.h` 尾部的"功能开关"区域，方便教学时一眼看到项目启用了哪些功能。

### 改动详情
- **sysconfig.h** — 新增"功能开关 (Feature Toggles)"段落，包含：
  - `USE_MAG`（磁力计航向融合，已启用）
  - `USE_LENGTH_LIM`（四元数长度限制，已启用）
  - `USE_HEAT`（传感器加热，已注释）
  - `USE_THERMOSTATIC`（恒温 PID 控制，已注释）
- **Imu.c** — 删除文件顶部 `#define USE_MAG` / `#define USE_LENGTH_LIM`
- **Drv_heating.c** — 删除文件内 `//#define USE_THERMOSTATIC`，改为注释指向 sysconfig.h

### 致谢
本轮重构由 Claude (Anthropic) 辅助完成。
感谢匿名科创（ANO）提供原始飞控框架代码。
感谢指导老师对教学代码质量标准的持续指导。

---

## [Stage2-R9] 全局代码质量收尾 (2026-04-30)

### 改动概述
对全项目进行最终代码质量收尾：光流模块变量封装、typedef 命名统一、
LED 模块变量封装与常量化、死代码/注释清理、头文件保护符统一、宏改 typedef。
不改变任何控制算法、协议解析逻辑或外部数据接口。

### 改动详情

#### OF.c / OF.h — 变量封装（23 个 → static）
- **23 个仅内部使用的变量添加 `static`**：`OF_STATE`、`OF_DX`、`OF_DY`、`OF_ALT2`、
  `OF_GYR_X/Y/Z`、`OF_GYR_X2/Y2/Z2`、`OF_ACC_X/Y/Z`、`OF_ACC_X2/Y2/Z2`、
  `OF_ATT_ROL/PIT/YAW`、`OF_ATT_S1/S2/S3/S4`
- **OF.h 删除对应 extern 声明**，仅保留外部使用的 6 个变量：
  `OF_QUALITY`、`OF_DX2`、`OF_DY2`、`OF_DX2FIX`、`OF_DY2FIX`、`OF_ALT`

#### FcData.h / FcData.c — typedef 命名规范化
- `_flag` → `_flag_st`（与项目 `_xxx_st` 命名规范一致）

#### Filter.h / Filter.c — typedef 命名规范化
- `_lf_t` → `_lf_st`（3 处定义 + 6 处引用）
- `_jldf_t` → `_jldf_st`（1 处定义 + 1 处引用）
- `filter_s` → `_filter_st`（1 处定义 + 4 处引用）

#### MotorCtrl.h / MotorCtrl.c — typedef 命名规范化
- `motor_ctrl_t` → `_motor_ctrl_st`（R2 引入的命名不符合项目规范，修正）

#### LED.c — 变量封装 + 常量化
- **`LED_Brightness[4]` 和 `led_accuracy`** 添加 `static`（仅 LED.c 内部使用）
- **新增 5 个 LED 常量**：`LED_PWM_RESOLUTION (20)`、`LED_ERR_DISPLAY_MS (3000)`、
  `LED_FLASH_ON_MS (60)`、`LED_FLASH_CYCLE_MS (200)`、`LED_STATUS_PAUSE_MS (1000)`
- 替换全部状态显示裸数字

#### AttCtrl.c — 死代码清理
- **删除** 3 处 `kd_ex = 0;//0.000f ;` 旧参数注释
- **删除** `#define POS_V_DAMPING 0.02f` 及 2 处注释引用（功能未启用）

#### FlightCtrl.c — 死代码清理
- **删除** 2 处空 `//` 注释
- **删除** `//wxyz_fusion_reset();` 注释代码

#### Drv_RcIn.c — 死代码清理
- **删除** `//RCData_t SbusData;` 注释行

#### 头文件保护符统一（5 个文件）
- `FlightDataCal.h`：`__FLIGHT_DATA_COMP_H` → `__FLIGHT_DATA_COMP_H__`
- `sysconfig.h`：`_SYSCONFIG_H_` → `__SYSCONFIG_H__`
- `Drv_Uart.h`：`_DRV_UART_H_` → `__DRV_UART_H__`
- `Drv_RcIn.h`：`_DRV_RCIN_H_` → `__DRV_RCIN_H__`
- `RC.h`：`_RC_H_` → `__RC_H__`

#### RC.h — 宏改 typedef
- `#define _stick_f_lp_st u16` → `typedef u16 _stick_f_lp_st;`

### 涉及文件清单
| 文件 | 改动类型 |
|------|---------|
| `Platform/Device/Nav/OF.c` | 变量封装 |
| `Platform/Device/Nav/OF.h` | 删除多余 extern |
| `FlightControl/Base/FcData.h` | typedef 重命名 |
| `FlightControl/Base/FcData.c` | typedef 重命名 |
| `FlightControl/Base/Filter.h` | typedef 重命名 |
| `FlightControl/Base/Filter.c` | typedef 重命名 |
| `FlightControl/Control/MotorCtrl.h` | typedef 重命名 |
| `FlightControl/Control/MotorCtrl.c` | typedef 重命名 |
| `App/Service/LED.c` | 变量封装 + 常量化 |
| `FlightControl/Control/AttCtrl.c` | 死代码清理 |
| `FlightControl/Control/FlightCtrl.c` | 死代码清理 |
| `Platform/Peripheral/Drv_RcIn.c` | 死代码清理 |
| `FlightControl/Control/FlightDataCal.h` | 头文件保护符 |
| `App/Core/sysconfig.h` | 头文件保护符 |
| `Platform/Peripheral/Drv_Uart.h` | 头文件保护符 |
| `Platform/Peripheral/Drv_RcIn.h` | 头文件保护符 |
| `App/Service/RC.h` | 头文件保护符 + 宏改 typedef |

### 致谢
本轮重构由 Claude (Anthropic) 辅助完成代码审计、变量作用域分析与批量修改建议。
感谢匿名科创（ANO）提供原始飞控框架代码。
感谢指导老师对教学代码质量标准的持续指导。

### 验证方式
1. Keil 编译通过（0 Error, 0 Warning）
2. 上位机各数据显示正常
3. 遥控通道响应正常
4. 定高定点功能正常
5. 试飞确认无异常

---

## [Stage2-R8] 控制层清理（最终轮） (2026-04-30)

### 改动概述
对 FlightCtrl、AltCtrl、LocCtrl、AttCtrl 四个控制层模块进行清理：
内部变量 `static` 化、魔法数字常量化、死变量/死函数/死 extern 删除。
不改变任何 PID 参数值、控制算法、状态机逻辑或外部数据接口。

### 改动详情

#### FlightCtrl.c — 变量封装 + 常量化 + 死代码删除
- **4 个变量添加 `static`**：`flying_cnt`、`landing_cnt`、`speed_mode_old`、`flight_mode_old`
- **删除** 死变量 `float stop_baro_hpf`（全项目零读写）
- **删除** 空函数 `Speed_Mode_Switch()` 及其调用
- **新增 6 个飞行状态常量**：`TAKEOFF_DELAY_MS (1400)`、`LAND_DETECT_DELAY_MS (200)`、
  `LAND_THR_THRESHOLD (250)`、`LAND_DURATION_MS (1500)`、`FLY_CONFIRM_MS (1000)`、
  `TILT_PROTECT_COS (0.25f)`
- **新增 4 个光流判定常量**：`OF_QUALITY_THRESHOLD (50)`、`OF_QUALITY_DELAY_MS (500)`、
  `OF_MAX_ALT_CM (600)`、`OF_ALT_DELAY_MS (1000)`
- **修复** 错误注释 `//800ms`（实际阈值为 1000ms）

#### FlightCtrl.h — 删除死 extern 声明
- **删除** `extern float wifi_selfie_mode_yaw_vlue`（无定义、拼写错误、遗留代码）

#### AltCtrl.c — 变量封装 + 常量化
- **4 个 PID 变量添加 `static`**：`alt_arg_2`、`alt_val_2`、`alt_arg_1`、`alt_val_1`
- **新增 8 个高度控制常量**：`ALT_STEP_LIMIT_CM (200)`、`ALT_VEL_OUT_LIMIT (150)`、
  `TAKEOFF_TIMEOUT_MS (5000)`、`THR_CHECK_DELAY_MS (2000)`、`THR_ACTIVE_THRESHOLD (0.1f)`、
  `ALT_PID_INTE_LIM (100)`、`ALT_HOLD_DEADZONE_CM (20)`、`ALT_SPEED_FF_GAIN (0.6f)`

#### LocCtrl.c — 变量封装 + 常量化 + 死变量删除
- **6 个变量添加 `static`**：`loc_arg_1_fix[2]`、`loc_val_1_fix[2]`、`vel_fb_d_lpf[2]`、
  `vel_fb_h[2]`、`vel_fb_w[2]`（均仅本文件使用）
- **删除** 死变量 `float vel_fb_fix_w[2]`（全项目零读写）
- **新增 5 个位置控制常量**：`LOC_PID_INTE_ERR_LIM (50)`、`LOC_PID_OUT_SCALE (10)`、
  `ACC_LEAD_GAIN (0.03f)`、`GPS_DECEL_RATE (5)`、`GPS_POS_HOLD_DELAY (50)`

#### AttCtrl.c — 常量化
- **新增 6 个姿态控制常量**：`ATT_PID_INTE_ERR_LIM (5)`、`RATE_PID_INTE_ERR_LIM (200)`、
  `YAW_SPEED_MEDIUM (220)`、`YAW_SPEED_LOW (200)`、`MC_ROLL_PITCH_LIMIT (1000)`、
  `MC_YAW_LIMIT (400)`

### 涉及文件清单
| 文件 | 改动类型 |
|------|---------|
| `FlightControl/Control/FlightCtrl.c` | 变量封装 + 常量化 + 死代码删除 |
| `FlightControl/Control/FlightCtrl.h` | 删除死 extern 声明 |
| `FlightControl/Control/AltCtrl.c` | 变量封装 + 常量化 |
| `FlightControl/Control/LocCtrl.c` | 变量封装 + 常量化 + 死变量删除 |
| `FlightControl/Control/AttCtrl.c` | 常量化 |

### 验证方式
1. Keil 编译通过（0 Error, 0 Warning）
2. 上位机各控制数据显示正常
3. 遥控通道与飞行模式切换正常
4. 定高定点功能正常
5. 试飞确认无异常

---

## [Stage2-R7] 传感器/驱动层清理 (2026-04-30)

### 改动概述
对 ICM20602、SPL06、RcIn、OF、OF_DecoFusion 五个传感器/驱动模块进行清理：
内部变量 `static` 化、裸数字常量化、死代码删除、头文件修复、缓冲区安全修复。
不改变任何传感器配置、采样率、协议解析逻辑或外部数据接口。

### 改动详情

#### Drv_icm20602.c — 变量封装 + 常量化 + 死代码删除
- **`mpu_buffer[14]`** 添加 `static`（仅本文件使用）
- **初始化函数常量化**：9 处裸数字替换为已有头文件宏
  （`BIT_PWR_MGMT_1_DEVICE_RESET`、`BIT_PWR_MGMT_1_CLK_XGYRO`、
  `BITS_GYRO_FS_2000DPS`、`BITS_ACCEL_FS_16`、`MPU_WHOAMI_20602` 等）
- **新增 3 个 ICM20602 特有寄存器常量**：`MPUREG_ACCEL_CONFIG_2 (0x1D)`、
  `MPUREG_ACCEL_INTEL_CTRL (0x1E)`、`ICM20602_ACCEL_LPF_20HZ (0x04)`
  （原头文件沿用 MPU6050 寄存器表，0x1D/0x1E 在 ICM20602 中功能不同）
- **新增温度转换常量**：`ICM_TEMP_SENSITIVITY (326.8f)`、`ICM_TEMP_OFFSET (25.0f)`
- **删除** `icm20602_writeBits()` 注释块（多位写入函数，零调用）

#### Drv_spl06.c — 变量封装 + 寄存器常量 + 死变量/死代码删除
- **6 个文件级变量添加 `static`**：`baro_Offset`、`alt_3`、`height`、
  `temperature`、`alt_high`、`baro_pressure`（全部仅在 `Drv_Spl0601_Read()` 使用）
- **删除** 死变量 `unsigned char baro_start`（全项目零引用）
- **删除** `#define uint32 unsigned int` → 改用项目统一的 `u32` 类型
- **新增 8 个 SPL06 寄存器常量**：`SPL06_REG_PSR_B2 (0x00)` ~ `SPL06_REG_COEF_BASE (0x10)`
  替换 `spl0601_rateset()`、`spl0601_get_calib_param()`、`spl0601_get_raw_temp()`、
  `spl0601_get_raw_pressure()`、`Drv_Spl0601Init()` 中的裸寄存器地址
- **新增** `SEA_LEVEL_PRESSURE_PA (101400.0f)` 替换气压高度公式中的裸数字
- **删除** `Drv_Spl0601Init()` 末尾注释测试代码和 `test_spi[5]` 注释行

#### Drv_RcIn.c — 协议常量化 + 变量封装 + 安全修复
- **新增 PPM 协议常量**：`PPM_SYNC_THRESHOLD_US (5000)`、`SYS_CLK_MHZ (80)`、
  `TIMER_COUNTER_MASK (0xFFFFFF)`
- **新增 SBUS 协议常量**：`SBUS_FRAME_LENGTH (25)`、`SBUS_HEADER (0x0F)`、
  `SBUS_FOOTER (0x00)`、`SBUS_FLAG_FAILSAFE (0x10)`、`SBUS_USED_CHANNELS (8)`
- **`sbus_flag`** 添加 `static`（仅本文件使用，头文件未声明 extern）
- **修复 SBUS 缓冲区溢出**：原 `SUBS_RawData[DataCnt++]` 先写入再检查长度，
  连续无效字节可导致 `DataCnt` 超过 25 溢出。
  改为写入前检查 `DataCnt >= SBUS_FRAME_LENGTH` 上界

#### OF_DecoFusion.c — 变量封装 + 常量化
- **3 个变量添加 `static`**：`of_buf_update_flag`、`of_fus_err[2]`、`of_fus_err_i[2]`
  （仅本文件使用）
- **新增** `OF_VALID_FLAG (0xF5)` 替换 3 处光流有效标记裸数字

#### OF_DecoFusion.h — 删除错误的 static 函数声明
- **删除 6 个 `static` 函数声明**（`ANO_OF_Data_Get`、`OF_INS_Get`、
  `ANO_OF_Decouple`、`ANO_OF_Fusion`、`OF_State`、`OF_INS_Reset`）
  `static` 函数不应出现在公共头文件中——对其他翻译单元无意义，
  `.c` 文件中已有前置声明

#### OF.c — 协议常量化
- **新增 5 个匿名光流 V3 协议常量**：`ANO_OF_FRAME_HEAD1 (0xAA)`、
  `ANO_OF_FRAME_HEAD2 (0x22)`、`ANO_OF_MSG_MOTION (0x51)`、
  `ANO_OF_MSG_RANGE (0x52)`、`ANO_OF_MSG_IMU (0x53)`
- **新增 2 个在线判定常量**：`OF_OFFLINE_THRESHOLD_MS (1000)`、
  `OF_CHECK_CNT_MAX (10000)`

### 涉及文件清单
| 文件 | 改动类型 |
|------|---------|
| `Platform/Device/Sensor/Drv_icm20602.c` | 变量封装 + 常量化 + 死代码删除 |
| `Platform/Device/Sensor/Drv_spl06.c` | 变量封装 + 常量化 + 死变量/死代码删除 |
| `Platform/Peripheral/Drv_RcIn.c` | 常量化 + 变量封装 + 安全修复 |
| `App/Vision/OF_DecoFusion.c` | 变量封装 + 常量化 |
| `App/Vision/OF_DecoFusion.h` | 删除 static 函数声明 |
| `Platform/Device/Nav/OF.c` | 常量化 |

### 验证方式
1. Keil 编译通过（0 Error, 0 Warning）
2. 上位机传感器数据（加速度/陀螺/气压）显示正常
3. 遥控通道响应正常（PPM/SBUS）
4. 光流定点功能正常
5. 试飞确认无异常

---

## [Stage2-R6] DT 遥测模块重构 (2026-04-30)

### 改动概述
对 `DT.c` 进行结构性重构：合并重复的 UART/USB 接收状态机、修复 USB 接收缓冲区溢出隐患、
协议裸数字全部常量化、私有缓冲区 static 化、统一所有发送函数的组帧方式。
不改变任何协议帧格式、功能码、发送优先级或参数映射语义。

### 改动详情

#### DT.c — 接收状态机合并与安全修复
- **新增 `dt_rx_ctx_t` 结构体**：封装接收缓冲区、字节计数、状态机状态和完成标志
- **新增 `dt_rx_feed()` 通用接收函数**：参数化处理，UART/USB 共用同一逻辑
- **修复 USB 接收缓冲区溢出**：原 USB 状态机 state==5 缺少载荷长度上界检查
  （`_data_len` 可达 255，溢出 100 字节缓冲区），现统一使用 `ANO_DT_RX_MAX_PAYLOAD (80)` 限制
- **删除 6 个散落的 static 变量**：`DT_RxBuffer[100]`、`DT_data_cnt`、`ano_dt_data_ok`
  及 USB 侧对应的 3 个变量，由 `s_rx_uart` / `s_rx_usb` 结构体替代
- **消除约 50 行重复代码**（两套状态机合并为一套）

#### DT.c — 协议常量化
- **新增 17 个消息 ID 常量**：`ANO_MSG_STATUS (0x01)` ~ `ANO_MSG_STRVAL (0xA1)`
- **新增 4 个命令功能码**：`ANO_CMD_CALI`、`ANO_CMD_RESET`、`ANO_CMD_READ_PARAM`、`ANO_CMD_FLYCTRL`
- **新增 4 个校准子码**：`ANO_CALI_ACC`、`ANO_CALI_GYRO`、`ANO_CALI_MAG`、`ANO_CALI_READ_VER`
- **新增 3 个复位子码**：`ANO_RESET_PID`、`ANO_RESET_PARAM`、`ANO_RESET_ALL`
- **新增 11 个发送周期常量**：`DT_PERIOD_SENSOR (10)` ~ `DT_PERIOD_LOCATION (500)`
- **新增 `DT_CNT_WRAP`、`ANO_RC_NEUTRAL`、`ANO_DT_RX_MAX_PAYLOAD`** 替换散落裸数字
- 所有 Send 函数中的裸消息 ID 替换为对应常量

#### DT.c — 变量清理
- `data_to_send[50]` 加 `static`（仅模块内部使用）
- **删除** `checkdata_to_send` 和 `checksum_to_send`（全项目零引用）
- **删除** `extern float ultra_dis_lpf`（声明后未使用）

#### DT.c — 发送函数规范化
- `ANO_DT_Send_User()` 从手动组帧改为使用 `FrameStart()/FrameSend()` 统一流程
- `ANO_DT_Send_VER()` 裸 `0xAA` 替换为 `ANO_DT_FRAME_HEAD`
- `ANO_DT_Send_Data()` USART 路径修正：使用 `dataToSend` 参数替代硬编码的 `data_to_send`
- `SendString()` / `SendStrVal()` 缓冲区边界 `> 50` 改为 `>= ANO_DT_TX_BUFFER_SIZE`

#### Drv_gps.c — 删除死 extern
- 删除 `extern u8 data_to_send[50]`（声明后未使用）

### 涉及文件清单
| 文件 | 改动类型 |
|------|---------|
| `App/Service/DT.c` | 主要重构 |
| `Platform/Device/Nav/Drv_gps.c` | 删除死 extern |

### 验证方式
1. Keil 编译通过（0 Error, 0 Warning）
2. 上位机连接正常，各数据帧显示正确
3. 上位机参数读写正常（PID 调参往返）
4. 校准命令正常触发
5. USB 和 UART 两条通路均正常收发

---

## [Stage2-R5] RC 通道数据封装 (2026-04-29)

### 改动概述
将 RC 模块的 `CH_N[8]`（归一化遥控通道值）和 `chn_en_bit`（通道有效位图）
收为文件内 `static` 变量，新增 getter 函数接口，删除未使用的 `signal_intensity`，
消除三个控制函数的 `s16 *CH_N` 冗余参数传递。

### 改动详情

#### RC.c / RC.h — 模块接口封装
- `CH_N[CH_NUM]` 和 `chn_en_bit` 改为 `static`
- 删除未使用的 `signal_intensity` 变量（定义、extern、别名、唯一写入点）
- 新增三个公开 getter 函数：
  - `RC_GetChannel(u8 ch)` — 单通道读取，含越界保护
  - `RC_GetAllChannels(s16 *out, u8 count)` — 批量复制
  - `RC_GetChannelEnableMask(void)` — 读取通道有效位图
- 清理语义别名宏块（`g_rc_channels`、`g_rc_signal_intensity`、`g_rc_channel_enable_mask`）

#### FlightCtrl.h / FlightCtrl.c — 函数签名简化
- `Flight_State_Task(u8 dT_ms, s16 *CH_N)` → `Flight_State_Task(u8 dT_ms)`
- `Flight_Mode_Set` 内 AUX1/AUX2 读取改用 `RC_GetChannel()` + 局部变量

#### AttCtrl.h / AttCtrl.c — 函数签名简化
- `Att_2level_Ctrl(float dT_s, s16 *CH_N)` → `Att_2level_Ctrl(float dT_s)`
- YAW 通道读取改用 `RC_GetChannel(CH_YAW)`
- 新增 `#include "RC.h"`

#### LocCtrl.h / LocCtrl.c — 删除无用参数
- `Loc_1level_Ctrl(u16 dT_ms, s16 *CH_N)` → `Loc_1level_Ctrl(u16 dT_ms)`
- `CH_N` 参数在函数体内从未被引用，纯死参数删除

#### Scheduler.c — 调用点同步
- 三处调用去除 `CH_N` 实参

#### DT.c — 遥测数据读取
- `CH_N[i]` → `RC_GetChannel(i)`
- `chn_en_bit` → `RC_GetChannelEnableMask()`

#### MotorCtrl.c — ESC 校准路径
- `CH_N[CH_THR]` → `RC_GetChannel(CH_THR)`（`#ifdef Cali_Set_ESC` 内）

#### User_control.c — 任务控制
- `CH_N[AUX3]` / `CH_N[AUX4]` → `RC_GetChannel(AUX3)` / `RC_GetChannel(AUX4)`

### 涉及文件清单
| 文件 | 改动类型 |
|------|---------|
| `App/Service/RC.c` | 封装 + 新增 getter + 删除废弃变量 |
| `App/Service/RC.h` | 接口重定义 |
| `FlightControl/Control/FlightCtrl.h` | 签名变更 |
| `FlightControl/Control/FlightCtrl.c` | 签名变更 + getter 迁移 |
| `FlightControl/Control/AttCtrl.h` | 签名变更 |
| `FlightControl/Control/AttCtrl.c` | 签名变更 + getter + 新增 include |
| `FlightControl/Control/LocCtrl.h` | 签名变更 |
| `FlightControl/Control/LocCtrl.c` | 签名变更（死参数删除） |
| `App/Core/Scheduler.c` | 调用点同步 |
| `FlightControl/Control/MotorCtrl.c` | getter 迁移 |
| `App/Service/DT.c` | getter 迁移 |
| `App/Mission/User_control.c` | getter 迁移 |

### 验证方式
1. Keil 编译通过（0 Error, 0 Warning）
2. 烧录后遥控通道响应正常（PPM/SBUS）
3. 上位机遥测通道数据显示正确
4. AUX 开关飞行模式切换正常
5. 试飞确认无异常

---

## [Stage2-R4] 姿态解算与高度融合深度重构 (2026-04-29)

### 改动概述
对 `Imu.c`/`Imu.h`（姿态解算）和 `FlightDataCal.c`（传感器调度与高度融合）
进行深度重构：补充算法说明、整理模块结构、消除冗余变量、收窄作用域。

### 改动详情

#### Imu.c — 姿态解算模块重构
- **新增算法概览注释**：互补滤波完整流程说明（四元数、DCM、叉积误差、PI融合）
- **新增带编号的分步注释** ①~⑩ 贯穿 `IMU_update()` 函数
- **提取 `adjust_fusion_gains()`**：将增益自适应逻辑（~60行）从 `IMU_update()` 末尾
  抽取为独立的 `static` 函数，附带增益策略说明
- **`kp_use`/`ki_use`/`mkp_use`** 从函数内 `static` 局部变量提升为文件级
  `s_kp_use`/`s_ki_use`/`s_mkp_use`，配合函数提取
- **`att_matrix[3][3]`** 添加 `static` 限定（仅 Imu.c 内部使用）
- **`a2w_3d_trans()`** 添加 `static` 限定（仅 Imu.c 内部调用）
- **删除未使用的 `float imu_test[3]`**（全局声明但零引用）
- 所有 `static` 变量按用途分组并添加用途注释
- 添加坐标变换、状态变量、欧拉角计算的分节标题

#### Imu.h — 头文件整理
- **`_imu_st` 结构体**：为全部 17 个字段添加中文用途注释
- **`_imu_state_st` 结构体**：为全部 9 个字段添加用途注释
- **删除孤立声明 `void IMU_duty(float dT)`**（无定义、零调用）

#### FlightDataCal.c — 传感器调度与高度融合重构
- **9 个内部全局变量添加 `static`**：`baro_h_offset`、`ref_height_get_1`/`_2`、
  `ref_height_used`、`baro2tof_offset`/`tof2baro_offset`、`baro_fix1`/`baro_fix2`/`baro_fix`、
  `wcz_f_pause`、`wcz_acc_use`
  （`baro_height` 和 `ref_tof_height` 保留外部链接，DT.c 遥测使用）
- **新增高度融合算法注释**：气压计参考状态机 (0→1→2) 完整状态转移说明、
  ToF/光流切换策略说明
- 标注 `BARO_FIX` 当前值为 `0`，相关修正计算实际无效果
- 添加传感器读取、姿态更新、磁力计处理、高度融合的分节标题
- 所有 `static` 变量按用途分组并添加注释

### 涉及文件清单
| 文件 | 改动类型 |
|------|---------|
| `FlightControl/Base/Imu.c` | 深度重构 + 函数提取 |
| `FlightControl/Base/Imu.h` | 注释补充 + 删除孤立声明 |
| `FlightControl/Control/FlightDataCal.c` | 作用域收窄 + 注释补充 |

### 验证方式
1. Keil 编译通过（0 Error, 0 Warning）
2. 烧录后姿态解算和高度融合功能正常
3. 试飞确认无异常

---

## [Stage2-R3] 全项目编码统一：GBK → UTF-8 + 乱码注释修复 (2026-04-29)

### 改动概述
将全项目约 90 个 `.c`/`.h` 源文件从 GBK 编码批量转换为 UTF-8，
并手工修复因早期编码转换损坏（`?` 替换或 U+FFFD 替换）的注释。

### 改动详情

#### 批量 iconv 转换（~81 个文件）
- 使用 `iconv -f GBK -t UTF-8` 一次性转换所有纯 GBK 文件
- 覆盖 `FlightControl/`、`App/`、`Platform/` 三个目录
- 转换后所有中文注释均可在 UTF-8 编辑器中正常显示

#### 手工修复混合编码文件（3 个文件）
- **FlightCtrl.c**：55 行乱码注释重写（原始中文已被 `?`/U+FFFD 替代，不可恢复）
- **AltCtrl.c**：31 行乱码注释重写（GBK 与 UTF-8 混合，iconv 失败）
- **config.h**：3 行模块头注释修复

#### 手工修复 `?` 残留（8 个文件）
- `FlightDataCal.h`、`Imu.c`、`AttCtrl.c`、`MagProcess.c`
- `Sensor_Basic.c`、`Pid.h`、`AltCtrl_2.h`
- `OF.c`、`OF.h`、`Drv_spl06.c`
- `sysconfig.h`、`RC.c`

#### 未转换文件（2 个，非项目源码）
- `Legacy/` 下的历史冲突备份文件
- `TiDriver/` 下的 TI 官方驱动头文件

### 验证方式
1. Python 脚本扫描全部 196 个 `.c`/`.h` 文件，确认零 GBK / 零 `???` / 零 U+FFFD
2. Keil 编译通过（0 Error, 0 Warning）
3. 烧录后功能正常

---

## [Stage2-R2] 命名规范化 + 魔法数字清理 + 死代码删除 (2026-04-29)

### 改动概述
跨 9 个文件进行命名规范化、魔法数字替换和死代码清理，
提升代码可读性和教学价值。

### 改动详情

#### config.h — 新增命名常量
- `GRAVITY_CMSS (981)` — 重力加速度，cm/s^2
- `RAD_TO_DEG (57.2957795f)` — 弧度转角度
- `ACC_NORM_MAX (1060)` / `ACC_NORM_MIN (900)` — IMU 加速度模值有效范围

#### MotorCtrl.h — 结构体重命名
- `_mc_st` → `motor_ctrl_t`（类型名规范化）
- `ct_val_rol` → `roll`
- `ct_val_pit` → `pitch`
- `ct_val_yaw` → `yaw`
- `ct_val_thr` → `throttle`

#### MotorCtrl.c — 重写整理
- 应用上述字段重命名
- 修复 `flag.motor_preparation == 0` 的重复判断
- 预转延时从魔法数字 `300/600/900/1200` 改为 `MOTOR_PREP_TIME * N`
- `1000` → `MOTOR_PWM_MAX` 常量
- 添加 X 型混控矩阵注释图（标注每个电机的 roll/pitch/yaw 符号）

#### AttCtrl.c — 应用 mc 字段重命名
- `mc.ct_val_rol` → `mc.roll`（及 pitch、yaw）

#### AltCtrl.c — 应用 mc 字段重命名
- `mc.ct_val_thr` → `mc.throttle`

#### FlightCtrl.c — 清理
- `mc.ct_val_thr` → `mc.throttle`
- 删除 `ctrl_parameter_change_task()` 中的 `if(0)` 死代码块
- 删除 `Swtich_State_Task()` 中的 `if(0)//(Laser_height_mm<1900)` 死代码块
  → 简化为 `switchs.tof_on = 0;`（原逻辑等价）
- 删除未使用的 `extern s32 ref_height_get;` 声明

#### Imu.c — 魔法数字替换
- `981` → `GRAVITY_CMSS`
- `1060` / `900` → `ACC_NORM_MAX` / `ACC_NORM_MIN`
- `57.30f` → `RAD_TO_DEG`（3 处）

#### FlightDataCal.c — 清理
- 删除 `IMU_Update_Task()` 中两层 `if(0)` 死代码（24 行 → 3 行）
- 删除未使用的 `extern s32 sensor_val_ref[];`
- 删除未使用的 `u16 test_time_cnt` 及其自增

#### FlightDataCal.h — 修复
- 删除不存在的 `ref_height_get` extern 声明（变量实际名为 `ref_height_get_1`）

### 涉及文件清单
| 文件 | 改动类型 |
|------|---------|
| `FlightControl/Control/config.h` | 新增常量 |
| `FlightControl/Control/MotorCtrl.h` | 结构体重命名 |
| `FlightControl/Control/MotorCtrl.c` | 重写整理 |
| `FlightControl/Control/AttCtrl.c` | 字段重命名 |
| `FlightControl/Control/AltCtrl.c` | 字段重命名 |
| `FlightControl/Control/FlightCtrl.c` | 重命名 + 死代码删除 |
| `FlightControl/Base/Imu.c` | 魔法数字替换 |
| `FlightControl/Control/FlightDataCal.c` | 死代码 + 废弃变量删除 |
| `FlightControl/Control/FlightDataCal.h` | 修复错误 extern |

### 验证方式
1. Keil 编译通过（0 Error, 0 Warning）
2. 烧录后所有控制功能正常
3. 试飞确认无异常

---

## [Stage2-R1] UART 驱动重构 (2026-04-28)

### 改动概述
对 `Platform/Peripheral/Drv_Uart.c` 和 `Drv_Uart.h` 进行深度重构，
消除 5 路串口之间的重复代码，统一代码风格和命名。

### 改动详情

#### 新增：通用串口通道抽象
- 定义 `uart_ch_t` 结构体，封装 UART 基地址、发送缓冲区和读写指针
- 5 路串口各持有一个静态实例（`s_ch_gps`、`s_ch_dt`、`s_ch_openmv`、`s_ch_optflow`、`s_ch_laser`）
- 提取通用 `uart_tx_check()` 和 `uart_send_buf()` 内部函数

#### 删除：重复和遗留代码
- **删除 5 套独立的 `s_xxx_tx_buf` / `s_xxx_tx_write_idx` / `s_xxx_tx_read_idx` 变量**
  → 替换为 `uart_ch_t` 结构体内的统一字段
- **删除 5 套独立的 `Drv_UartXxx_TxCheck()` 公开函数**
  → 替换为内部 `uart_tx_check()`，TxCheck 不再暴露到头文件
- **删除 5 套独立的 `Drv_UartXxx_SendBuf()` 重复实现**
  → 公开接口保留，内部调用通用 `uart_send_buf()`
- **删除全部 15 个历史兼容包装函数** `Drv_Uart1Init/SendBuf/TxCheck` ~ `Drv_Uart5Init/SendBuf/TxCheck`
  → 链接器确认零外部调用，安全删除
- **从 `Drv_Uart.h` 删除** 15 个旧接口声明和 5 个 TxCheck 声明

#### 修改：ISR 命名
- `UART1_IRQHandler` → `UartGps_IRQHandler`（实际操作 UART0）
- `UART2_IRQHandler` → `UartDt_IRQHandler`（实际操作 UART4）
- `UART3_IRQHandler` → `UartOpenMv_IRQHandler`（实际操作 UART2）
- `UART4_IRQHandler` → `UartOptFlow_IRQHandler`（实际操作 UART7）
- `UART5_IRQHandler` → `UartLaser_IRQHandler`（实际操作 UART5）
- ISR 均通过 `UARTIntRegister()` 函数指针注册，不依赖固定名称

#### 补充：文档注释
- 文件头部添加硬件映射表（底板编号 → TM4C UART → 外设功能）
- 每路串口代码段添加清晰的分隔注释
- 激光串口初始化中的历史遗留 GPIO 解锁操作添加说明注释

### 代码量变化
- 重构前：408 行（含 15 个重复包装函数）
- 重构后：~315 行
- 净减少 ~90 行，同时增加了注释和结构化

### 未改动内容（保持兼容）
- 各串口 UART 基地址、GPIO 复用、中断优先级配置不变
- 发送缓冲区大小（256 字节）不变
- 收发流程和中断处理逻辑不变
- 激光串口初始化中的 PD7 解锁历史遗留操作保留

### 已知遗留问题
- 激光串口 `Drv_UartLaser_Init()` 中有一段对 `UART2_PORT` (GPIOD) 的解锁操作，
  但激光实际使用 `UART5_PORT` (GPIOE)，疑似是为 OpenMV 的 PD7 准备的历史代码。
  当前可正常工作，后续可考虑移至 `Drv_UartOpenMv_Init()` 中。

### 验证方式
1. Keil 编译通过（0 Error, 0 Warning）
2. 烧录后各串口功能正常（GPS、数传、OpenMV、光流、激光）
3. 试飞确认无异常
