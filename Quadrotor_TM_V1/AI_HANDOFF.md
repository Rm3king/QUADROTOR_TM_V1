# 飞控项目重构交接文档

## 1. 文档用途

本文件用于把当前这轮飞控项目重构的背景、已完成工作、验证结果、环境坑点和后续建议完整交给下一个 AI 或开发者。

如果后续需要继续推进，请优先阅读本文件，而不是直接根据工作区表象做判断。

---

## 2. 当前结论

- 当前重构分支代码已经完成了第一大轮“浅层重构收口”。
- 用户已明确反馈：**代码已经完整运行，并且已经试飞成功**。
- 因此可以确认：
  - 当前重构没有破坏核心飞行行为。
  - 这一轮“先统一风格、先收拢结构、先去历史脏内容”的目标已经基本达成。
- 后续工作不再是“救火式整理”，而是进入：
  - 第二轮更深层命名统一
  - 局部结构继续收拢
  - 更工程化的模块边界整理

---

## 3. 硬约束

后续任何 AI 继续工作时，都必须继续遵守：

- 不改算法
- 不改控制流程
- 不改状态机逻辑
- 不改参数含义
- 不改变外部行为
- 优先兼容现有 Keil 工程和编译环境
- 注释统一使用中文
- 风格保持 TI / 嵌入式 C 项目习惯
- 不搞过度面向对象化

---

## 4. 当前分支与远端状态

- 当前分支：`refactor/stage1-ano-dt`
- 最近验证通过的代码基线提交：`f1b5784`
- 远端分支：`origin/refactor/stage1-ano-dt`
- 当前这轮工作**没有合并到 `master`**

建议：

- 后续继续在 `refactor/stage1-ano-dt` 上推进
- 等下一轮整理和再次实机验证稳定后，再考虑合并主分支

---

## 5. 已完成的核心工作

### 5.1 工程结构层

已经完成目录结构重组，当前工程已不再维持最早的散乱布局。

当前主要层次为：

- `App/Core`
- `App/Service`
- `App/Mission`
- `App/Vision`
- `Platform/Bsp`
- `Platform/Peripheral`
- `Platform/Device/Sensor`
- `Platform/Device/Nav`
- `Platform/Device/Misc`
- `FlightControl/Base`
- `FlightControl/Control`

说明：

- 目录职责已经比原工程清晰很多。
- Keil 工程路径和 include 路径已经配合调整过。

### 5.2 文件命名层

已经完成一轮比较大的文件名去历史前缀整理：

- 大量 `Ano_*` 文件已改成更直接的模块名
- 同时更新了 `#include`、Keil 工程文件、相关路径引用

### 5.3 接口语义层

已经完成一轮重要的“语义名澄清”：

- UART 历史接口名与实际硬件用途不一致的问题，已经建立映射和兼容语义别名
- RC 输入相关接口已经做了语义化整理

参考文档：

- [INTERFACE_MAPPING.md](./INTERFACE_MAPPING.md)

### 5.4 注释与风格层

已经做过多轮清理，主要包括：

- 删掉大量历史废弃注释代码
- 删掉旧版权块、无意义分隔线、冗余块注释
- 补充统一风格的中文模块说明
- 统一一批关键函数注释
- 收口一批 `static` 私有函数和私有变量

### 5.5 低风险结构收拢

已处理过的高收益模块包括但不限于：

- `DT`
- `LED`
- `Usb/FcUsbCdc`
- `RC`
- `Parameter`
- `FcData`
- `FlightCtrl`
- `AttCtrl`
- `AltCtrl`
- `LocCtrl`
- `MagProcess`
- `Power`
- `MotionCal`
- `Sensor_Basic`
- `Imu`
- `OF`
- `UWB`
- `Drv_Bsp`
- `Drv_Uart`
- `Drv_gps`
- `OpenMV` 相关控制层

### 5.6 编译修复

在后期曾出现两处会导致 Keil 真实编译失败的源码问题，已经修复：

- `Platform/Device/Nav/OF.c`
  - 文件曾被截断，导致函数不完整
- `FlightControl/Control/AltCtrl.c`
  - 混入坏字符，导致语法错误

修复提交：

- `f1b5784` `fix(build): restore OF parser and altitude control syntax`

---

## 6. 已验证结果

### 6.1 编译

已多次通过本机 Keil 命令行编译：

```powershell
& 'D:\robomaster\keil\keil_v5\UV4\UV4.exe' -b 'D:\my_data\niming_flying\Quadrotor_TM_V1\Quadrotor_TM_V1\Quadrotor_TM_V1.uvprojx' -j0 -t 'Quadrotor_TM'
```

当前确认结果：

- `0 Error(s), 0 Warning(s)`
- `build\Quadrotor_TM_V1i.axf` 可生成

### 6.2 下载

曾出现过两类问题：

1. VS Code 直接执行 `-f` 下载，但 `axf` 不存在  
   原因：先下载后编译，导致找不到可烧录文件

2. Keil 打开工程时一度报编译错误  
   原因：源码里有真实损坏，已修复

当前结论：

- **现在 Keil 可以正常编译**
- **现在 Keil 可以正常下载**

### 6.3 实机

用户已明确反馈：

- **代码已经试飞成功**
- **当前版本可以完整运行**

这条信息非常关键，后续 AI 不要再把当前分支当成“未验证的纯整理代码”。

---

## 7. 重要提交时间线

以下提交是当前重构主线里最重要的一批检查点：

- `965cba8` `baseline: import current flight controller project`
- `4f979b8` `refactor(stage1): clean up Ano_DT frame packing`
- `2414894` `refactor(stage1): tidy usb cdc interface`
- `603d9b0` `refactor(stage2): clarify uart and rc input interfaces`
- `d26511f` `refactor(stage7): drop ano prefixes from file names`
- `f4d843f` `refactor(stage7): reorganize project directory layout`
- `761d3d8` `refactor(stage8): trim legacy comment blocks in control and nav`
- `834a34a` `refactor(stage8): clean remaining control and driver headers`
- `050bed6` `refactor(stage8): finalize header and style cleanup`
- `80a2014` `refactor(stage8): polish uart and optical-flow interfaces`
- `f1b5784` `fix(build): restore OF parser and altitude control syntax`

如果需要回溯本轮重构的关键节点，优先看这些提交。

---

## 8. 仍然存在的现实情况

### 8.1 编码问题

本工程中仍有大量历史 `GBK/GB2312` 编码文件。

现状：

- VS Code 工作区已配置 `gbk`
- 有些终端命令输出会把中文显示成 `????`
- 这**不一定代表文件内容真的坏了**

处理原则：

- 查看文件内容优先用编辑器，不要只凭终端 `rg` 输出判断
- 修改老文件时尽量保持原编码，不要随手全项目转 UTF-8

### 8.2 工作区脏文件

当前工作区通常会有大量非源码脏内容：

- `.vscode/*`
- `build/*`
- `*.uvoptx`
- `*.uvguix.*`

这些一般是：

- 编译产物
- Keil 用户本地配置
- VS Code 扩展日志

后续提交时：

- **不要把这些产物混进源码提交**
- 只提交真正的源码和文档

### 8.3 仍有少量历史命名残留

虽然文件层已经大幅整理，但下列残留仍然存在：

- `ANO_DT_*`
- `ANO_CBTracking_*`
- `ANO_LTracking_*`
- `ANO_OF_*`
- 少量 `AnoOF_*`

说明：

- 这些不再是必须立刻清理的内容
- 若继续处理，建议走“兼容包装 + 渐进迁移”路线
- 不建议一次性硬改全项目符号名

---

## 9. 下一阶段建议

当前这轮可以视为“第一轮浅层重构完成”。  
后续建议转入第二轮，但继续保持小步推进。

### 9.1 最推荐的继续方向

1. 继续做接口级命名统一
   - 重点是保留旧接口兼容的前提下，新增更工程化的语义入口

2. 做大文件内部结构拆分
   - 只拆 `static` 子函数
   - 不改调用顺序
   - 不改状态推进条件

3. 继续收敛全局变量访问
   - 从 `FcData`、`Parameter`、控制层共享状态入手
   - 优先做注释、访问边界和声明位置统一

4. 清理少量遗留头文件和视觉层接口
   - `App/Vision/*`
   - `DT.h / DT.c`
   - 少量仍带旧接口风格的头文件

### 9.2 暂时仍要谨慎的区域

这些区域可以看，但不要轻易做深改：

- 调度器与时基
- 中断处理
- PWM 输出链
- 电机混控
- 姿态解算核心公式
- 状态机转移条件
- Flash 参数存储时序

---

## 10. 下一个 AI 的建议工作方式

如果下一个 AI 是 Claude Code、另一个 Codex、或其他编码代理，建议它这样接手：

### 10.1 第一步先读这些文件

优先阅读：

- [AI_HANDOFF.md](./AI_HANDOFF.md)
- [REFACTOR_CONTEXT.md](./REFACTOR_CONTEXT.md)
- [INTERFACE_MAPPING.md](./INTERFACE_MAPPING.md)

说明：

- `REFACTOR_CONTEXT.md` 更像早期规则和计划
- 本文件是当前阶段的最新交接总结

### 10.2 第二步先确认 Git 状态

建议先执行：

```powershell
git status --short --branch
git log --oneline --decorate -n 20
```

然后确认：

- 当前仍在 `refactor/stage1-ano-dt`
- `HEAD` 至少不早于 `f1b5784`

### 10.3 第三步先做只读判断

在继续改之前，先确认：

- 哪些文件只是编码显示成问号
- 哪些文件是真的还没整理
- 哪些工作区改动只是 `build/` 和 `.vscode`

### 10.4 默认工作风格

推荐下一个 AI：

- 不要重新大改目录结构
- 不要重复做第一轮注释大扫除
- 以“第二轮命名和结构收拢”为主
- 每轮都编译
- 每轮都做 Git 检查点

---

## 11. 推荐给下一个 AI 的起始提示词

可以直接把下面这段发给下一个 AI：

```text
你现在接手一个已经完成第一轮浅层重构的嵌入式飞控项目。

请先阅读：
1. AI_HANDOFF.md
2. REFACTOR_CONTEXT.md
3. INTERFACE_MAPPING.md

当前分支是 refactor/stage1-ano-dt。
当前代码已经试飞成功，Keil 可编译、可下载。

硬约束：
- 不改算法
- 不改控制流程
- 不改状态机逻辑
- 不改参数含义
- 不改变外部行为
- 注释统一中文
- 风格保持 TI / 嵌入式 C 工程化风格

请先做只读判断，再继续推进第二轮命名统一和结构收拢。
优先避免重复做已经完成的大面积注释清理。
```

---

## 12. 当前工作区说明

在本文件写入时，通常会看到以下脏文件：

- `.vscode/*`
- `build/*`
- `Quadrotor_TM_V1.uvoptx`
- `Quadrotor_TM_V1.uvguix.*`

它们大多不是源码问题。

若后续要提交，只建议提交：

- 真正的源码修改
- 说明文档

不要顺手把 Keil 用户文件和构建产物也提交进去。

---

## 13. 当前建议

如果接下来是为了继续开发：

- 继续在 `refactor/stage1-ano-dt` 上小步推进

如果接下来是为了继续飞控验证：

- 优先使用当前已验证分支做台架和短飞
- 不要在测试前再做大面积重构

---

## 14. 一句话总结

当前分支已经完成了第一轮大规模浅层重构，并且经过了真实试飞验证。  
后续 AI 不应该再把它当作“未验证的整理草稿”，而应该把它当作“可运行、可编译、可继续小步演进的稳定基线”。
