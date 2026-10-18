#ifndef PID_H
#define PID_H

// 位置式 PID 控制器
//
// 离散公式 (对应教材 PID.md):
//   u(k) = Kp*e(k) + Ki*sum(e(j))*T + Kd*(e(k)-e(k-1))/T
// 其中 T 为采样周期(本工程固定 1kHz -> 0.001s)
//
// 在模板基础上做了三处工程化处理:
//   1) 输出限幅 + 条件积分(抗积分饱和): 输出饱和时本拍不累加积分
//   2) 积分项整体限幅, 防止长时间大偏差把积分撑爆
//   3) 微分项一阶低通: 1kHz 下转速/角度量化噪声很大, 直接差分会被放大
class Pid {
public:
    Pid(float kp, float ki, float kd, float out_max, float out_min);
    ~Pid() = default;

    void setParams(float kp, float ki, float kd);
    void setOutputLimit(float out_max, float out_min);
    void setDt(float dt) { dt_ = dt; }                    // 采样周期(s)
    void setDerivativeFilter(float tau) { tau_ = tau; }   // 微分低通时间常数(s), 0=不过滤
    void reset();

    // 计算一次控制量, ref 目标值, fdb 反馈值, 返回限幅后的输出
    float calc(float ref, float fdb);

    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }

private:
    float kp_, ki_, kd_;
    float out_max_, out_min_;
    float dt_;        // 采样周期 T
    float tau_;       // 微分低通时间常数
    float integ_;     // 积分项 Ki*sum(e(j))*T, 已含增益
    float last_err_;  // 上一拍误差 e(k-1)
    float d_filt_;    // 滤波后的微分值 de/dt
};

#endif // PID_H
