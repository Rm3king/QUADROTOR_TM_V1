# Changelog

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
