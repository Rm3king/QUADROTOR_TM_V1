# 5 分钟测试上手文档

## 1. 适用场景

本文件用于：

- 换一台电脑后快速拉取当前稳定重构分支
- 用 Keil 完成编译与下载
- 在最短时间内完成台架和短时试飞前检查

当前推荐测试分支：

- `refactor/stage1-ano-dt`

---

## 2. 第一步：拉取代码

如果新电脑上还没有仓库：

```powershell
git clone https://github.com/Rm3king/QUADROTOR_TM_V1.git
cd QUADROTOR_TM_V1\Quadrotor_TM_V1
git checkout refactor/stage1-ano-dt
```

如果新电脑上已经有仓库：

```powershell
git fetch origin
git checkout refactor/stage1-ano-dt
git pull origin refactor/stage1-ano-dt
```

拉完后先确认当前分支：

```powershell
git status --short --branch
```

期望看到类似：

```text
## refactor/stage1-ano-dt...origin/refactor/stage1-ano-dt
```

---

## 3. 第二步：打开工程

用 Keil 打开：

- `Quadrotor_TM_V1.uvprojx`

确认目标工程为：

- `Quadrotor_TM`

---

## 4. 第三步：编译

推荐直接在 Keil 图形界面里操作：

1. 点击 `Rebuild`
2. 等待编译完成
3. 确认没有报错

成功后应生成：

```text
build\Quadrotor_TM_V1i.axf
```

如果想用命令行编译：

```powershell
& "D:\robomaster\keil\keil_v5\UV4\UV4.exe" -b "D:\my_data\niming_flying\Quadrotor_TM_V1\Quadrotor_TM_V1\Quadrotor_TM_V1.uvprojx" -j0 -t "Quadrotor_TM"
```

---

## 5. 第四步：下载

编译通过后，再执行下载。

推荐方式：

1. 在 Keil 中连接下载器
2. 点击 `Download`
3. 等待烧录完成

不要先下载再补编译。

---

## 6. 常见下载失败处理

### 6.1 报错：找不到 `.axf`

典型现象：

- `cannot open file`
- `Flash Download failed`

说明：

- 不是源码坏了
- 而是还没有先成功编译出 `build\Quadrotor_TM_V1i.axf`

处理：

1. 先 `Rebuild`
2. 确认 `build\Quadrotor_TM_V1i.axf` 已生成
3. 再 `Download`

### 6.2 报错：`Cannot Load Flash Device Description Flash\TM4C123_256.flm`

说明：

- 这通常不是代码问题
- 一般是 Keil 下载环境或 Flash Algorithm 配置问题

优先检查：

1. `Options for Target -> Utilities -> Settings`
2. `Flash Download` 页签里是否存在 `TM4C123_256.flm`
3. Tiva / TM4C 相关 Device Pack 是否安装完整
4. 下载器连接是否正常

---

## 7. 第五步：上电前检查

烧录成功后，不要直接起飞，先做下面这些：

1. 上电后观察 LED 是否正常
2. 看串口或上位机通信是否正常
3. 看遥控输入是否正常
4. 看传感器状态是否正常
5. 看模式切换是否正常

---

## 8. 第六步：台架检查

建议先无桨验证：

1. 解锁 / 上锁是否正常
2. 油门响应是否正常
3. 模式切换是否正常
4. 串口、OpenMV、光流、GPS、UWB 等外设链路是否正常
5. 电机输出方向和响应是否符合预期

---

## 9. 第七步：短时试飞

只有在前面都正常后，再做短时起飞测试。

建议顺序：

1. 低风险场地
2. 短时离地
3. 先验证基本姿态和油门响应
4. 再逐步验证更复杂模式

---

## 10. 这轮代码的已知结论

- 当前 `refactor/stage1-ano-dt` 分支已经完成第一大轮浅层重构
- 本分支代码已经：
  - 编译通过
  - 下载通过
  - 真实试飞成功

所以换电脑测试时，默认目标不是“排查重构是否完全不可用”，而是确认：

- 新电脑环境是否完整
- 下载链路是否正常
- 当前硬件连接是否正常

---

## 11. 如果要继续开发

继续重构前，建议先读：

1. `AI_HANDOFF.md`
2. `REFACTOR_CONTEXT.md`
3. `INTERFACE_MAPPING.md`

如果只是测试，本文件就够用了。
