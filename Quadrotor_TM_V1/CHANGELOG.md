# Changelog

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
