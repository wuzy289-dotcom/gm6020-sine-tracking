#!/usr/bin/env python3
"""
离线闭环仿真: 用与固件完全相同的控制律, 验证初值增益、符号和限幅是否合理。

注意: 这是"预期效果"仿真, 用来在没有硬件时先确认参数方向;
      作业要求的两张曲线截图必须来自真实电机(见 tools/plot_live.py)。

用法:
    python tools/sim_control.py            # 生成 figures/sim_speed.png / sim_position.png
"""

import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

DT = 1e-3
T_END = 16.0

# ---- 与 Core/Src/ControlTask.cpp 中的 cfg 保持一致 ----
SPD_AMP_RPM = 100.0
SPD_FREQ_HZ = 0.2
POS_AMP_DEG = 60.0
POS_FREQ_HZ = 0.1
RAMP_IN = 1.5
VOLT_LIMIT = 15000.0
POS_SPD_LIMIT = 20.0
VOLT_PER_RPM = 78.0

SPD_KP, SPD_KI, SPD_KD = 25.0, 150.0, 0.0
POS_KP, POS_KI, POS_KD = 8.0, 0.0, 0.0

# ---- 被控对象(一阶+库仑摩擦), 仅用于验证参数方向 ----
KV_RPM_PER_UNIT = 0.0128      # 25000 -> 24V, 13.33rpm/V
TAU_S = 0.010                 # 机械时间常数
FRICTION_UNIT = 400.0         # 死区/静摩擦折合电压单位

TWO_PI = 2.0 * math.pi


class Pid:
    """与 Core/Src/Pid.cpp 等价的实现。"""

    def __init__(self, kp, ki, kd, out_max, out_min):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_max, self.out_min = out_max, out_min
        self.dt = DT
        self.integ = 0.0
        self.last_err = 0.0

    def reset(self):
        self.integ = 0.0
        self.last_err = 0.0

    def calc(self, ref, fdb):
        err = ref - fdb
        d_raw = (err - self.last_err) / self.dt
        out = self.kp * err + self.integ + self.kd * d_raw
        if out > self.out_max:
            out = self.out_max
        elif out < self.out_min:
            out = self.out_min
        else:
            self.integ += self.ki * err * self.dt
        self.integ = min(max(self.integ, self.out_min), self.out_max)
        self.last_err = err
        return out


class Plant:
    def __init__(self):
        self.rpm = 0.0
        self.theta = 0.0

    def step(self, u):
        drive = 0.0 if abs(u) < FRICTION_UNIT else u - math.copysign(FRICTION_UNIT, u)
        self.rpm += (KV_RPM_PER_UNIT * drive - self.rpm) * DT / TAU_S
        self.theta += self.rpm * TWO_PI / 60.0 * DT


def run_speed():
    pid = Pid(SPD_KP, SPD_KI, SPD_KD, VOLT_LIMIT, -VOLT_LIMIT)
    plant = Plant()
    n = int(T_END / DT)
    t = np.arange(n) * DT
    ref = np.zeros(n)
    fdb = np.zeros(n)

    for k in range(n):
        ramp = min(1.0, t[k] / RAMP_IN)
        w_ref = SPD_AMP_RPM * ramp * math.sin(TWO_PI * SPD_FREQ_HZ * t[k])
        u = VOLT_PER_RPM * w_ref + pid.calc(w_ref, plant.rpm)
        u = min(max(u, -VOLT_LIMIT), VOLT_LIMIT)
        plant.step(u)
        ref[k], fdb[k] = w_ref, plant.rpm

    return t, ref, fdb


def run_position():
    pid_pos = Pid(POS_KP, POS_KI, POS_KD, POS_SPD_LIMIT, -POS_SPD_LIMIT)
    pid_spd = Pid(SPD_KP, SPD_KI, SPD_KD, VOLT_LIMIT, -VOLT_LIMIT)
    plant = Plant()
    amp = math.radians(POS_AMP_DEG)
    w = TWO_PI * POS_FREQ_HZ
    n = int(T_END / DT)
    t = np.arange(n) * DT
    ref = np.zeros(n)
    fdb = np.zeros(n)

    for k in range(n):
        ramp = min(1.0, t[k] / RAMP_IN)
        th_ref = amp * ramp * math.sin(w * t[k])
        w_ref = amp * w * math.cos(w * t[k]) + pid_pos.calc(th_ref, plant.theta)
        w_ref = min(max(w_ref, -POS_SPD_LIMIT), POS_SPD_LIMIT)
        w_ref_rpm = w_ref * 60.0 / TWO_PI
        u = VOLT_PER_RPM * w_ref_rpm + pid_spd.calc(w_ref_rpm, plant.rpm)
        u = min(max(u, -VOLT_LIMIT), VOLT_LIMIT)
        plant.step(u)
        ref[k], fdb[k] = math.degrees(th_ref), math.degrees(plant.theta)

    return t, ref, fdb


def save(t, ref, fdb, title, ylabel, path):
    tail = t > T_END - 1.0 / (SPD_FREQ_HZ if "rpm" in ylabel else POS_FREQ_HZ)
    err = fdb[tail] - ref[tail]
    rms = float(np.sqrt(np.mean(err ** 2)))
    peak = float(np.max(np.abs(err)))

    fig, ax = plt.subplots(figsize=(9, 3.6), dpi=140)
    ax.plot(t, ref, label="reference", linewidth=1.6)
    ax.plot(t, fdb, label="feedback", linewidth=1.2, linestyle="--")
    ax.set_xlabel("time [s]")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    ax.text(0.01, 0.04,
            f"RMS err = {rms:.2f}, peak err = {peak:.2f}  (simulation)",
            transform=ax.transAxes, fontsize=8, color="#555")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    print(f"{os.path.basename(path)}: RMS={rms:.2f}, peak={peak:.2f}")
    return rms, peak


def main():
    os.makedirs("figures", exist_ok=True)
    save(*run_speed(), "Speed tracking (simulation preview)", "speed [rpm]",
         "figures/sim_speed.png")
    save(*run_position(), "Position tracking (simulation preview)", "angle [deg]",
         "figures/sim_position.png")


if __name__ == "__main__":
    main()
