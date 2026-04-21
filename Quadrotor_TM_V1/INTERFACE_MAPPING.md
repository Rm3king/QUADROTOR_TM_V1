# 接口映射说明

## 1. 目的

本文件用于固定当前工程中“历史命名”和“实际硬件用途”之间的对应关系。

目标只有两个：

- 帮助后续重构时统一接口命名。
- 避免把“底板串口编号”误读成 “TM4C 芯片 UART 编号”。

本文件只做说明，不改变任何驱动行为。

## 2. UART 映射

| 底板接口名 | TM4C UART | 当前实际用途 | 推荐语义名 |
| --- | --- | --- | --- |
| `Drv_Uart1*` | `UART0` | GPS | `Drv_UartGps_*` |
| `Drv_Uart2*` | `UART4` | 数传 / DT | `Drv_UartDt_*` |
| `Drv_Uart3*` | `UART2` | OpenMV | `Drv_UartOpenMv_*` |
| `Drv_Uart4*` | `UART7` | 光流模块 | `Drv_UartOpticalFlow_*` |
| `Drv_Uart5*` | `UART5` | 激光模块预留 | `Drv_UartLaser_*` |

## 3. RC 输入映射

| 历史接口名 | 含义 | 推荐语义名 |
| --- | --- | --- |
| `Drv_PpmInit` | PPM 输入初始化 | `Drv_RcPpm_Init` |
| `Drv_SbusInit` | SBUS 输入初始化 | `Drv_RcSbus_Init` |

## 4. 当前阶段约定

- 旧接口名继续保留，保证现有工程和调用点兼容。
- 新代码优先使用推荐语义名。
- 若后续继续迁移调用点，只做“命名语义替换”，不改寄存器配置、中断流程和收发逻辑。

## 5. 当前已发现的历史不一致

- `Driver/Drv_Uart.c` 文件头部的旧注释写到“TM4C 串口 2 对应底板串口 5”。
- 但 `Drv_Uart5Init()` 当前实现实际使用的是 `UART5_BASE`。
- 这类问题先记录，不在本轮做行为修正。
- 后续若要继续核查，必须结合原理图、`sysconfig.h` 宏定义和实机连线一起确认。

## 6. 现阶段已迁移到语义名的典型位置

- `Application/Ano_DT.c`
- `Application/Ano_RC.c`
- `Driver/Drv_Bsp.c`
- `Driver/SenserDriver/Drv_gps.c`
- `Driver/SenserDriver/Drv_UP_Flow.c`
