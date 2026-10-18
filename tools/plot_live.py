#!/usr/bin/env python3
"""
GM6020 正弦跟随: 串口实时曲线 + 截图

固件每 2ms 上报一行 "ref,fdb" (USART1, 256000, 8N1):
    速度模式: ref / fdb 单位 rpm
    位置模式: ref / fdb 单位 度(相对进入位置模式时的角度)

用法:
    # 先看串口有没有数据(不画图)
    python tools/plot_live.py --port COM5 --print-only

    # 实时曲线, 采集 25 秒后自动存图
    python tools/plot_live.py --port COM5 --mode speed --seconds 25 \
        --out figures/speed_follow.png

    python tools/plot_live.py --port COM5 --mode position --seconds 30 \
        --out figures/position_follow.png

    # 同时把原始数据存成 csv, 方便事后重画
    python tools/plot_live.py --port COM5 --mode speed --csv data/speed.csv

    # 没有硬件时验证脚本本身
    python tools/plot_live.py --demo --mode speed --out figures/demo_speed.png

    # 用之前录的 csv 重画
    python tools/plot_live.py --replay data/speed.csv --mode speed --out figures/x.png

安装依赖:
    pip install pyserial matplotlib
"""

import argparse
import math
import os
import sys
import time


def parse_args():
    p = argparse.ArgumentParser(description="GM6020 sine tracking plotter")
    p.add_argument("--port", help="串口, 例如 COM5")
    p.add_argument("--baud", type=int, default=256000)
    p.add_argument("--mode", choices=["speed", "position"], default="speed",
                   help="决定 y 轴单位和标题")
    p.add_argument("--seconds", type=float, default=25.0, help="采集时长, 0 表示手动结束")
    p.add_argument("--window", type=float, default=12.0, help="绘图窗口显示最近多少秒")
    p.add_argument("--out", help="截图保存路径(png)")
    p.add_argument("--csv", help="同时把数据写成 csv")
    p.add_argument("--replay", help="从 csv 重画, 不连串口")
    p.add_argument("--demo", action="store_true", help="用仿真数据验证脚本")
    p.add_argument("--print-only", action="store_true", help="只打印收到的数据")
    return p.parse_args()


def make_figure(mode):
    import matplotlib
    if not os.environ.get("DISPLAY") and sys.platform.startswith("linux"):
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 4.2), dpi=150)
    ax.set_xlabel("time [s]")
    if mode == "speed":
        ax.set_ylabel("speed [rpm]")
        ax.set_title("GM6020 speed sine tracking")
    else:
        ax.set_ylabel("angle [deg]")
        ax.set_title("GM6020 position sine tracking")
    ax.grid(alpha=0.3)
    (line_ref,) = ax.plot([], [], linewidth=1.6, label="reference")
    (line_fdb,) = ax.plot([], [], linewidth=1.3, linestyle="--", label="feedback")
    ax.legend(loc="upper right")
    return plt, fig, ax, line_ref, line_fdb


def save_figure(fig, out):
    if not out:
        return
    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    fig.tight_layout()
    fig.savefig(out)
    print(f"saved: {out}")


def feed(t, ref, fdb, mode):
    """把 (ref, fdb) 序列画出来; 用于 replay/demo, 不依赖串口。"""
    import argparse as _ap
    args = _ap.Namespace(mode=mode)
    plt, fig, ax, lr, lf = make_figure(mode)

    # 对齐到 0 起点, 只看最后一个完整周期附近
    t0 = t[0] if t else 0.0
    ts = [x - t0 for x in t]
    lr.set_data(ts, ref)
    lf.set_data(ts, fdb)
    if ts:
        ax.set_xlim(0, min(max(ts[-1], 1.0), max(ts[-1], 1.0)))
        lo = min(min(ref), min(fdb))
        hi = max(max(ref), max(fdb))
        pad = max(1.0, 0.12 * (hi - lo))
        ax.set_ylim(lo - pad, hi + pad)
    return plt, fig


def main():
    args = parse_args()

    if args.replay:
        import csv
        t, ref, fdb = [], [], []
        with open(args.replay, newline="") as f:
            for row in csv.reader(f):
                if len(row) < 3:
                    continue
                try:
                    t.append(float(row[0]))
                    ref.append(float(row[1]))
                    fdb.append(float(row[2]))
                except ValueError:
                    continue
        plt, fig = feed(t, ref, fdb, args.mode)
        save_figure(fig, args.out)
        return 0

    if args.demo:
        dt = 0.002
        n = int(25.0 / dt)
        amp = 100.0 if args.mode == "speed" else 60.0
        freq = 0.2 if args.mode == "speed" else 0.1
        t, ref, fdb = [], [], []
        for k in range(n):
            tt = k * dt
            r = amp * math.sin(2 * math.pi * freq * tt) * min(1.0, tt / 1.5)
            lag = 0.06 if args.mode == "speed" else 0.35
            f = amp * math.sin(2 * math.pi * freq * (tt - lag)) * min(1.0, max(0.0, (tt - lag) / 1.5))
            t.append(tt)
            ref.append(r)
            fdb.append(f)
        plt, fig = feed(t, ref, fdb, args.mode)
        save_figure(fig, args.out)
        return 0

    import serial
    from serial.tools import list_ports

    if not args.port:
        ports = [p.device for p in list_ports.comports()]
        print("可用串口:", ", ".join(ports) if ports else "(没有检测到串口)")
        print("用 --port COMx 指定")
        return 2

    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    ser.reset_input_buffer()

    print(f"已连接 {args.port} @ {args.baud}, 等待数据...")

    if args.print_only:
        t0 = time.time()
        try:
            while args.seconds <= 0 or time.time() - t0 < args.seconds:
                raw = ser.readline().decode("ascii", "ignore").strip()
                if raw:
                    print(raw)
        except KeyboardInterrupt:
            pass
        ser.close()
        return 0

    plt, fig, ax, line_ref, line_fdb = make_figure(args.mode)
    plt.ion()
    plt.show(block=False)

    t, ref, fdb = [], [], []
    t0 = None
    csv_f = None
    if args.csv:
        os.makedirs(os.path.dirname(os.path.abspath(args.csv)) or ".", exist_ok=True)
        csv_f = open(args.csv, "w", newline="")
        csv_f.write("t,ref,fdb\n")

    last_draw = 0.0
    t_start = time.time()
    try:
        while True:
            if args.seconds > 0 and time.time() - t_start > args.seconds:
                break
            raw = ser.readline().decode("ascii", "ignore").strip()
            if not raw:
                continue
            parts = raw.split(",")
            if len(parts) != 2:
                continue
            try:
                r, f = float(parts[0]), float(parts[1])
            except ValueError:
                continue

            now = time.time()
            if t0 is None:
                t0 = now
            t.append(now - t0)
            ref.append(r)
            fdb.append(f)
            if csv_f:
                csv_f.write(f"{t[-1]:.4f},{r:.3f},{f:.3f}\n")

            # 滑窗
            while t and t[0] < t[-1] - args.window:
                t.pop(0)
                ref.pop(0)
                fdb.pop(0)

            if now - last_draw > 0.05:
                last_draw = now
                line_ref.set_data(t, ref)
                line_fdb.set_data(t, fdb)
                ax.set_xlim(max(0.0, t[-1] - args.window), max(args.window, t[-1]))
                lo, hi = min(ref + fdb), max(ref + fdb)
                pad = max(1.0, 0.15 * (hi - lo))
                ax.set_ylim(lo - pad, hi + pad)
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
    except KeyboardInterrupt:
        print("手动结束")
    finally:
        ser.close()
        if csv_f:
            csv_f.close()
        save_figure(fig, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
