#include "Pid.h"

namespace {
constexpr float kSampleTime = 0.001f;   // 1kHz 控制周期
}

Pid::Pid(float kp, float ki, float kd, float out_max, float out_min)
    : kp_(kp), ki_(ki), kd_(kd),
      out_max_(out_max), out_min_(out_min),
      dt_(kSampleTime), tau_(0.0f),
      integ_(0.0f), last_err_(0.0f), d_filt_(0.0f) {}

void Pid::setParams(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

void Pid::setOutputLimit(float out_max, float out_min) {
    out_max_ = out_max; out_min_ = out_min;
}

void Pid::reset() {
    integ_ = 0.0f;
    last_err_ = 0.0f;
    d_filt_ = 0.0f;
}

float Pid::calc(float ref, float fdb) {
    const float err = ref - fdb;

    // 微分项: 先差分再做一阶低通, 抑制 1kHz 下的量化噪声
    const float d_raw = (err - last_err_) / dt_;
    if (tau_ > 0.0f) {
        const float alpha = dt_ / (tau_ + dt_);
        d_filt_ += alpha * (d_raw - d_filt_);
    } else {
        d_filt_ = d_raw;
    }

    float out = kp_ * err + integ_ + kd_ * d_filt_;

    // 输出限幅 + 条件积分(抗饱和): 只有未饱和时才继续累加积分
    if (out > out_max_) {
        out = out_max_;
    } else if (out < out_min_) {
        out = out_min_;
    } else {
        integ_ += ki_ * err * dt_;
    }

    // 积分项本身也限幅, 防止大偏差长时间存在时积分撑爆
    if (integ_ > out_max_) integ_ = out_max_;
    if (integ_ < out_min_) integ_ = out_min_;

    last_err_ = err;
    return out;
}
