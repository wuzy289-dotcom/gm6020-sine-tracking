#!/usr/bin/env python3
"""
用 J-Link 直接读内存，把 g_dbg_ref / g_dbg_fdb / g_dbg_u 记成 CSV，再画图。

比 Ozone Timeline 可靠：数据由脚本自己采样，不会出现"丢样本被画成直线"的假象，
而且能拿 CSV 做数值分析（幅值、相位、RMS 误差）。

用法:
    python tools/log_vars.py --seconds 20 --rate 200 --out data/log.csv --plot figures/log.png
    （需要在基址 0x20000414 处依次是 g_dbg_ref, g_dbg_fdb, g_dbg_u, g_dbg_mode）

注意: 运行前要关掉 Ozone，J-Link 一次只能被一个程序占用。
"""

import argparse
import csv
import os
import struct
import sys
import time

JLINK_DLL = r"D:\Program Files\SEGGER\JLink_V976\JLink_x64.dll"   # 必须用 64 位版
BASE = 0x20000414      # g_dbg_ref 的地址, 后面依次是 fdb / u / mode
DEVICE = "STM32F407IG"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--rate", type=float, default=200.0, help="采样频率 Hz")
    ap.add_argument("--out", default="data/log.csv")
    ap.add_argument("--plot", default="")
    ap.add_argument("--base", type=lambda x: int(x, 0), default=BASE)
    args = ap.parse_args()

    import pylink

    try:
        from pylink.library import Library
        jlink = pylink.JLink(lib=Library(JLINK_DLL))
    except Exception:
        jlink = pylink.JLink()
    jlink.open()
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.set_speed(4000)
    jlink.connect(DEVICE)
    print("connected:", jlink.target_connected())

    # 保证目标在跑
    try:
        if jlink.halted():
            jlink.restart()
    except Exception:
        pass

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    period = 1.0 / args.rate
    t0 = time.perf_counter()
    next_t = t0
    rows = []
    try:
        while True:
            now = time.perf_counter()
            if now - t0 >= args.seconds:
                break
            words = jlink.memory_read32(args.base, 4)
            ref, fdb, u, mode = struct.unpack("<ffff", struct.pack("<4I", *words))
            rows.append((now - t0, ref, fdb, u, int(mode)))
            next_t += period
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "ref", "fdb", "u", "mode"])
        w.writerows(rows)
    print(f"wrote {len(rows)} samples -> {args.out}")

    if args.plot and rows:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = [r[0] for r in rows]
        fig, ax = plt.subplots(figsize=(10, 4), dpi=140)
        ax.plot(t, [r[1] for r in rows], label="ref", lw=1.6)
        ax.plot(t, [r[2] for r in rows], label="fdb", lw=1.2, ls="--")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("speed/angle")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.plot)
        print("plot ->", args.plot)

    jlink.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
