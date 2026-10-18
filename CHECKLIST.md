# 到基地现场的操作清单

> 目标是 1 小时内拿到两张截图。按顺序做，每步都有"判定标准"和"不对怎么办"。

## 0. 出发前（电脑上）

- [ ] `git pull` 或者确认 `D:\stm32\gm6020_sine` 是最新的
- [ ] 试编译一次：`cmake --build --preset Debug`，应输出 Flash ≈ 40 KB
- [ ] `pip install pyserial matplotlib`（基地可能没网）
- [ ] 确认 OpenOCD 能找到：`openocd -v`

## 1. 接线（先断电）

| 项目 | 检查点 |
|---|---|
| 24V | XT30 接开发板 24V 输入；电机 XT30 也接 24V（或板载 24V 输出） |
| CAN | 板 CAN1 pin1=CANL 接电机黑线，pin2=CANH 接电机红线 |
| 串口 | 板 4-pin UART（丝印 UART2）1=RXD 2=TXD 3=GND 4=5V，接 USB-TTL；TTL-TX → 板 RXD |
| SWD | 3.3V/SWDIO/SWCLK/GND 接下载器 |
| 拨码 | 前三位 `001`（ID=1），第四位拨 ON（120 Ω 终端） |
| 机械 | 电机**夹紧在桌面上**，轴上不要挂东西——100 rpm 甩起来会打人 |

## 2. 上电顺序

1. 只给开发板上电（不接电机电源），看板子 LED 是否红闪（说明程序在跑、但没收到 CAN）
2. 再给电机上 24V，电机绿灯 1 秒闪 1 次 = ID 正常
3. 此时红灯应停止闪烁，变成绿灯常亮（速度模式、CAN 已通）

## 3. 烧录

```bash
cmake --build --preset Debug
tools\flash.cmd -Interface jlink     # 基地用 J-Link；换下载器改 stlink / cmsis-dap
```

判定：OpenOCD 打印 `** Verified OK **` 和 `** Resetting Target **`。

也可以在 Ozone 里烧：`File → Open` 选 `build/gm6020_sine.elf`，Device 选 STM32F407IG，
然后 `Download & Run`（务必是全速运行，停住就没有数据）。

## 4. 串口自检（关键一步）

没有 USB-TTL 就直接用 Ozone：把 `g_dbg_fdb` 加进 Live Watch（或 Timeline），
手转电机看数值变化——能看到非零转速就说明 CAN 通了。

```bash
python tools/plot_live.py --port COM5 --print-only
```

| 现象 | 判定 | 不对怎么办 |
|---|---|---|
| 每秒约 500 行 `0.00,0.00` | 串口通了，但没电机反馈 | 看第 5 步 |
| 有非零第二列，手转电机时变化 | CAN + 串口都通 | 继续第 6 步 |
| 完全没有输出 | 串口线/波特率/端口号错 | 换 COM 口，确认 256000 8N1 |
| 乱码 | 波特率不对或 TX/RX 接反 | 交换 TX/RX |

## 5. 电机没反馈时排查

- 电机橙灯**常亮** → 电机在电流模式，用 RoboMaster Assistant 改回电压模式（出厂默认就是电压模式，一般不会）
- 电机灯不亮 → 没供电
- 红灯一直闪、第二列恒 0 → 检查 CANH/CANL 是否接反（最常见）、ID 拨码是否与 `cfg::kMotorId` 一致
- 用万用表量 CANH-CANL 之间应有约 60 Ω（两个 120 Ω 并联）

## 6. 速度环

默认参数就能跑：`±100 rpm, 0.2 Hz`。

截图可以直接截 Ozone 的 Timeline（把 `g_dbg_ref`、`g_dbg_fdb` 拖进去，
采样周期 10 ms，跑满 3 个正弦周期），也可以走下面的脚本。

```bash
python tools/plot_live.py --port COM5 --mode speed --seconds 25 --out figures/speed_follow.png
```

| 曲线现象 | 调整 |
|---|---|
| 整体慢半拍、峰值低 | `kSpdKp` 25 → 35~45 |
| 峰值附近抖动/毛刺 | `kSpdKp` 降到 15~20 |
| 零速附近有平台（走不动） | `kSpdKi` 150 → 250 |
| 来回振荡停不下来 | `kSpdKi` 降到 50，先只留 P |
| 幅值不够（只到 ±80） | `kVoltPerRpm` 78 → 85 |

改完 `cmake --build --preset Debug` + 重新烧录，再采一次。

**截图标准**：两条线重叠、能看到至少 3 个完整正弦周期、图例清晰。

## 7. 位置环

短按板载 KEY 切到位置模式（蓝灯常亮），再采集：

```bash
python tools/plot_live.py --port COM5 --mode position --seconds 35 --out figures/position_follow.png
```

| 曲线现象 | 调整 |
|---|---|
| 整体滞后（相位差明显） | `kPosKp` 8 → 12~15 |
| 过零处来回抖 | 加 `kPosKd = 0.02`，或 `kPosKp` 降到 5 |
| 顶点顶不到位、削顶 | 加 `kPosKi = 1~3` |
| 抖动剧烈/异响 | 立刻断电，`kPosKp` 减半，`kVoltPerRpm` 检查是否过大 |

## 8. 看门狗验证（写进报告更有说服力）

长按 KEY 超过 1.5 s：灯全灭 → 约 0.8 s 后板子自动复位 → 红灯闪一下 → 恢复。

判定：能看到"程序卡死 → 自动重启"这个过程。拍个小视频或记下现象写进 README。

## 9. 收尾

- [ ] 两张 png 放到 `figures/`，取消 README 第十节的两行注释
- [ ] `git add -A && git commit -m "docs: 实测曲线" && git push`
- [ ] 把仓库链接 + 两张图发给老师

## 常见坑

1. **UART 丝印是反的**：4-pin 那个口（丝印 UART2）才是 USART1。
2. **CANH/CANL 接反**：不报错，就是收不到反馈，红灯一直闪。
3. **电机没夹紧**：位置模式 ±60° 来回转，会拖着线跑。
4. **烧录后要按一下复位**：有些下载器 reset 不干净。
5. **改参数忘记重新烧录**：改的是源码，板子里还是老程序。
