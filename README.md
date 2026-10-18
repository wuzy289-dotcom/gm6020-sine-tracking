# GM6020 正弦跟随（RoboMaster 电控第一阶段考核）

用 RoboMaster 开发板 C 型驱动一台 GM6020 电机，分别跟随**正弦速度给定**与**正弦位置给定**，
并用独立看门狗（IWDG）做运行保护。

> 到基地现场的操作顺序见 [CHECKLIST.md](CHECKLIST.md)。

- 控制频率：1 kHz（TIM6）
- 通信：CAN1 @ 1 Mbps（电压控制模式，`0x1FF` / 反馈 `0x205`）
- 上位机曲线：USART1 @ 256000，DMA 发送，每 2 ms 一帧 `ref,fdb`
- 看门狗：IWDG 64 分频 / 400 重装 ≈ **0.8 s**

## 一、控制方案

### 1. 速度环（`kModeSpeed`）

```
ω_ref(t) = A_ω · sin(2π f_ω t)                 A_ω = 100 rpm, f_ω = 0.2 Hz
u         = K_ff · ω_ref + PID_ω(ω_ref − ω_fb)  K_ff = 78 电压单位/rpm
```

前馈 `K_ff` 来自电机空载特性：`25000` 电压单位对应 24 V，13.33 rpm/V，因此
`1 rpm ≈ 25000 / (24 × 13.33) ≈ 78` 个单位。前馈给主量，PID 只修残差，
这样正弦顶部的相位滞后会明显变小。

### 2. 位置环（`kModePosition`）

```
θ_ref(t) = θ_0 + A_θ · sin(2π f_θ t)           A_θ = 60°, f_θ = 0.1 Hz
ω_ref    = dθ_ref/dt + PID_θ(θ_ref − θ_fb)     位置外环，输出速度给定
u        = K_ff · ω_ref + PID_ω(ω_ref − ω_fb)  速度内环，输出即电压
```

- `θ_0` 取进入位置模式瞬间的角度，所以给定从 0 开始，上电不会突然甩一下。
- `dθ_ref/dt` 是位置给定的微分前馈，用来抵消“先有误差再动作”的相位滞后。
- 位置环输出限幅 ±20 rad/s，速度环输出限幅 ±15000（约 14.4 V），都留了安全余量。

### 3. 起振渐变

前 1.5 s 幅值从 0 线性升到满幅（`kRampInSec`），避免第一拍就满幅给定导致电流冲击。

## 二、看门狗保护

`.ioc` 里 IWDG 默认使能：LSI 32 kHz ÷ 64 分频 = 500 Hz 计数，重装值 400
→ 超时时间 `400 / 500 = 0.8 s`。

喂狗策略（`ControlTaskLoop`，主循环里调用）：

```c
if (g_task_tick != s_fed) {     // 1 kHz 任务心跳有推进
    s_fed = g_task_tick;
    HAL_IWDG_Refresh(&hiwdg);
}
```

即**只有 1 kHz 控制任务还活着，主循环才喂狗**。若控制任务卡死（死循环、HardFault 后
中断被关掉等），心跳不再推进，0.8 s 后芯片自动复位，电机输出随之清零。

另外两层保护：

- **CAN 掉线**：超过 100 ms 没有收到 `0x205` 反馈，输出直接置 0，红灯 2 Hz 闪。
- **长按 KEY（>1.5 s）**：先把电压写 0，再进入死循环并关中断，用来现场演示看门狗复位。

> 调试注意：`ControlTaskInit()` 里调用了 `__HAL_DBGMCU_FREEZE_IWDG()`，
> 挂调试器单步时看门狗会暂停，不会因为断点停久了把板子重启掉。

## 三、硬件接线

| 开发板 C 型 | 接到 | 说明 |
|---|---|---|
| CAN1（2-pin） | GM6020 CAN 口 | 板端 pin1 = CANL，pin2 = CANH；电机线 A 红 = CANH，B 黑 = CANL，**别接反** |
| UART（4-pin，丝印 UART2） | USB-TTL 模块 | 板端 1=RXD 2=TXD 3=GND 4=5V；TTL 的 TX 接板 RXD |
| SWD（4-pin） | ST-Link / J-Link | 3.3V / SWDIO / SWCLK / GND |
| 24V 输入 XT30 | 24 V 电源 | GM6020 也从 24 V 取电，建议单独一路或确认线径够 |

注意两点：

1. 外壳丝印和实际串口是**反的**：丝印 `UART2` 的 4-pin 口才是 STM32 的 USART1（PA9/PB7），
   本工程用这个口；丝印 `UART1` 的 3-pin 口是 USART6。
2. 电机拨码开关前三位是 ID：`001` → ID=1，对应反馈帧 `0x205`。若你的电机不是 ID=1，
   改 `Tasks/ControlTask.cpp` 里的 `cfg::kMotorId`。
   第四位是 120 Ω 终端电阻，单电机短距离测试建议拨到 ON。

## 四、编译与烧录

环境：CMake ≥ 3.22、Ninja、`arm-none-eabi-gcc`、STM32CubeMX 6.x（仅改外设时才需要）。

```bash
cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/gm6020_sine.elf`。

烧录（OpenOCD，按你的下载器选 interface）：

```bash
# ST-Link
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/gm6020_sine.elf verify reset exit"

# J-Link
openocd -f interface/jlink.cfg -f target/stm32f4x.cfg \
  -c "program build/gm6020_sine.elf verify reset exit"

# CMSIS-DAP (DAP-Link)
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "program build/gm6020_sine.elf verify reset exit"
```

也可以用仓库里的脚本：`tools\flash.cmd`（默认 J-Link，
换下载器就加 `-Interface stlink` 或 `-Interface cmsis-dap`；它内部用 `-ExecutionPolicy Bypass` 绕过执行策略）。

## 五、上位机曲线与截图

固件在 USART1 上每 2 ms 发一行 ASCII：`ref,fdb\r\n`

- 速度模式：单位 rpm
- 位置模式：单位 度（相对进入位置模式时的角度）

```bash
pip install pyserial matplotlib

# 先确认串口有数据
python tools/plot_live.py --port COM5 --print-only

# 采集 25 s 并截图
python tools/plot_live.py --port COM5 --mode speed    --seconds 25 --out figures/speed_follow.png
python tools/plot_live.py --port COM5 --mode position --seconds 35 --out figures/position_follow.png
```

串口助手（VOFA+ / SerialPlot）也能直接看，选择 `256000, 8N1`，
两通道 CSV 即可。

## 六、板载指示

| 现象 | 含义 |
|---|---|
| 绿灯常亮 | 速度模式，CAN 正常 |
| 蓝灯常亮 | 位置模式，CAN 正常 |
| 红灯 2 Hz 闪 | 超过 100 ms 没收到电机反馈（CAN 没通 / ID 不对 / 电机没上电） |
| 灯全灭后 0.8 s 重启 | 触发了长按 KEY 的看门狗演示 |
| 电机橙灯常亮 | 电机被设成了电流控制模式，需要用 RoboMaster Assistant 改回电压模式 |

短按 KEY（< 1 s）在速度/位置模式之间切换。

## 七、调参步骤（到基地后照这个顺序做）

1. **先看反馈对不对**：`--print-only`，用手转电机，第一列不动、第二列（速度）跟着变，
   说明 CAN 通了。若两列都是 0 且红灯闪 → 查 ID、CANH/CANL、电机供电。
2. **速度环**：从 `kSpdKp=25, kSpdKi=150` 开始。曲线整体“慢半拍” → 加大 `kSpdKp`；
   顶部削平/抖动 → 减小 `kSpdKp`；零速附近有台阶（摩擦力）→ 加大 `kSpdKi`。
   前馈 `kVoltPerRpm` 基本不用动。
3. **位置环**：`kPosKp=8` 起步。曲线整体滞后 → 加大 `kPosKp`；
   过零处来回抖 → 减 `kPosKp` 或加 `kPosKd=0.02~0.05`；
   顶点附近“顶不上去” → 给位置环加一点 `kPosKi=1~3`。
4. 每改一次参数重新 `cmake --build --preset Debug` 并烧录，采集一次截图。

## 八、目录结构

```
Tasks/ControlTask.cpp      1 kHz 调度、速度/位置环、串口上报、喂狗
Tasks/Gm6020.cpp           GM6020 CAN 协议（角度多圈累计、电压下发）
Tasks/Pid.cpp              位置式 PID（条件积分抗饱和 + 微分低通）
Tasks/*.h                  对应头文件
Core/                      CubeMX 生成：main.c / can.c / tim.c / usart.c / iwdg.c ...
tools/plot_live.py         串口实时曲线 + 截图
tools/sim_control.py       离线闭环仿真（验证参数方向）
tools/flash.ps1            一键编译 + 烧录（PowerShell 版）
tools/flash.cmd            同上，双击/直接运行的包装器
figures/                   曲线图
```

手写代码全部放在 `Tasks/`，`Core/` 保持"CubeMX 专属"——重新生成代码不会碰到 `Tasks/`。

构建用的是组里的 CMake 模板：它用 `GLOB_RECURSE` 自动收集 `Core/` 和 `Tasks/` 下的源文件，
所以**新增 .c/.cpp 不需要改 CMakeLists**，放进这两个目录即可（所在目录会自动加入 include 路径）。
但要注意 glob 是在**配置期**展开的：新增或删除了源码文件，必须重新跑一次 `cmake --preset Debug`
（或删掉 `build/` 重新配置），否则新文件不会参与编译。

## 九、离线仿真预览（非实测）

没有硬件时用 `tools/sim_control.py` 按同一套控制律跑一阶+摩擦模型，
用来确认增益方向与限幅，结果仅作参考：

![速度跟随仿真](figures/sim_speed.png)
![位置跟随仿真](figures/sim_position.png)

## 十、实测截图

<!-- 采集后把文件放到 figures/ 下并取消下面两行注释 -->
<!-- ![速度跟随](figures/speed_follow.png) -->
<!-- ![位置跟随](figures/position_follow.png) -->
