/**
  ******************************************************************************
  * @file    ControlTask.cpp
  * @brief   GM6020 正弦跟随作业主逻辑
  *
  *   - TIM6 1kHz 中断里跑速度环 / 位置环, 输出 GM6020 电压给定
  *   - 速度环:  给定 rpm 正弦, PI + 前馈, 输出直接是电压
  *   - 位置环:  给定 rad 正弦, 位置 P(D) -> 速度给定, 串到速度环
  *   - USART1 256000 DMA 上报 ref / fdb 两路曲线(500Hz)
  *   - IWDG 0.8s: 只有在 1kHz 任务心跳正常时主循环才喂狗
  ******************************************************************************
  */

#include "ControlTask.h"

#include "main.h"
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"

#include "Gm6020.h"
#include "Pid.h"

#include <cmath>
#include <cstring>

// ---------------------------------------------------------------------------
// 给调试器(Ozone Timeline / Live Watch)观察的全局量:
// 放全局命名空间 + extern "C" + volatile, 符号名就是 g_dbg_xxx,
// 既不会被优化掉, 在 Ozone 里也能直接按名字搜到。
// ---------------------------------------------------------------------------
extern "C" {
volatile float   g_dbg_ref  = 0.0f;   // 当前给定 (速度: rpm / 位置: 度)
volatile float   g_dbg_fdb  = 0.0f;   // 当前反馈 (同上)
volatile float   g_dbg_u    = 0.0f;   // 当前输出电压给定 (-25000 ~ 25000)
volatile uint8_t g_dbg_mode = 0u;     // 0 = 速度模式, 1 = 位置模式
volatile uint32_t g_dbg_rx_cnt = 0u;  // 收到任意 CAN 帧的累计数(CAN1, 排查用)
volatile uint32_t g_dbg_rx_id  = 0u;  // 最近一帧的标准 ID(CAN1)
volatile uint32_t g_dbg_rx2_cnt = 0u; // CAN2 收到帧的计数(排查用)
volatile uint32_t g_dbg_rx2_id  = 0u; // CAN2 最近一帧的 ID
}

namespace {

constexpr float kPi       = 3.14159265359f;
constexpr float kTwoPi    = 6.28318530718f;
constexpr float kRadToRpm = 60.0f / kTwoPi;
constexpr float kRadToDeg = 180.0f / kPi;

// ---------------------------------------------------------------------------
// 所有可调参数集中在这里, 改完重新编译即可
// ---------------------------------------------------------------------------
namespace cfg {

constexpr uint8_t kMotorId = 1;         // GM6020 拨码开关 ID (1~7)
constexpr float   kLoopDt  = 0.001f;    // TIM6 1kHz

// 速度正弦: ±100 rpm, 0.2Hz (周期 5s)
constexpr float kSpdAmpRpm = 100.0f;
constexpr float kSpdFreqHz = 0.2f;

// 位置正弦: ±60°, 0.1Hz (周期 10s)
constexpr float kPosAmpDeg = 60.0f;
constexpr float kPosFreqHz = 0.1f;

constexpr float kRampInSec   = 1.5f;      // 起振渐变, 避免第一拍就满幅
constexpr float kPosSpdLimit = 20.0f;     // 位置环输出(速度给定)限幅 rad/s

// 这台 GM6020 在 RoboMaster Assistant 里被设成了电流控制模式:
// 电压帧 0x1FF 会被忽略, 必须发电流帧 0x1FE (16384 = 3A)
constexpr bool  kCurrentMode = true;
constexpr float kOutLimit    = 3000.0f;   // 输出限幅, 电流单位(3000 ≈ 0.55A)
// 速度前馈: 空载维持 100rpm 只需约 0.05A, 折合约 3 个电流单位/rpm
constexpr float kCurPerRpm   = 2.5f;
// 速度反馈一阶低通(时间常数 24ms): 电机回传的转速是整数 rpm, 且 GM6020 10 对极
// 在 100rpm 时本身就有 16.7Hz 的齿槽/换相转矩脉动, 直接喂进 PID 会让输出跟着抖
constexpr float kFdbAlpha    = 0.012f;
// 摩擦前馈: 静摩擦主要在低速段, 所以做成随速度衰减
// (|w_ref| >= 50rpm 时只剩高速段的库仑摩擦)
constexpr float kFricLow     = 500.0f;   // 零速附近, 电流单位(≈0.033A)
constexpr float kFricHigh    = 60.0f;   // 高速段(≈0.022A)

// PID 初值(电流模式下整定: 电流→转矩→加速度, 被控对象近似积分环节)
// Kp 给刚度, Ki 用来吃掉摩擦造成的稳态误差; 再大就会去追电机的齿槽纹波
constexpr float kSpdKp = 20.0f, kSpdKi = 2.0f, kSpdKd = 1.0f;
constexpr float kPosKp = 15.0f,  kPosKi = 0.0f,   kPosKd = 0.0f;

constexpr uint32_t kCanTimeoutMs   = 100;   // 超过 100ms 没反馈判定掉线
constexpr uint16_t kTelemetryDiv   = 2;     // 每 2 个控制周期上报一次 = 500Hz
constexpr uint16_t kKeyDebounceMs  = 30;    // 按键消抖
constexpr uint16_t kKeyLongPressMs = 1500;  // 长按触发看门狗演示

} // namespace cfg

// ---------------------------------------------------------------------------
// 对象与状态
// ---------------------------------------------------------------------------
Gm6020 g_motor(cfg::kMotorId);
Pid    g_spd_pid(cfg::kSpdKp, cfg::kSpdKi, cfg::kSpdKd,  cfg::kOutLimit,  -cfg::kOutLimit);
Pid    g_pos_pid(cfg::kPosKp, cfg::kPosKi, cfg::kPosKd,  cfg::kPosSpdLimit, -cfg::kPosSpdLimit);

volatile uint32_t g_task_tick   = 0;      // 1kHz 心跳
volatile uint32_t g_last_rx_ms  = 0;      // 最近一次收到反馈的时刻
volatile bool     g_motor_seen  = false;  // 是否收到过反馈

bool  g_mode_position = false;            // false=速度模式 true=位置模式
float g_t             = 0.0f;             // 进入当前模式后的时间(s)
float g_theta0        = 0.0f;             // 进入位置模式时的角度(rad)
float g_w_filt        = 0.0f;             // 低通后的速度反馈(rpm)
uint16_t g_tele_div    = 0;
uint16_t g_key_hold_ms = 0;
uint8_t  g_tx_idx      = 0;
char     g_tx_buf[2][48];

// ---------------------------------------------------------------------------
// 板载 LED / 按键 (C 型板: PH10 蓝, PH11 绿, PH12 红; PA0 按键, 按下为低)
// ---------------------------------------------------------------------------
inline void Led(bool blue, bool green, bool red) {
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, blue  ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, green ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, red   ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void GpioUserInit(void) {
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Pin   = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &gi);

    gi.Pin  = GPIO_PIN_0;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gi);

    Led(false, false, false);
}

// ---------------------------------------------------------------------------
// CAN: 接收全部标准帧, 1kHz 下发一次电压
// ---------------------------------------------------------------------------
void CanUserInit(void) {
    // 电机没上电时总线上没有第二个节点, 没有任何人给 ACK,
    // 发送会一路 error -> bus-off。CubeMX 默认 AutoBusOff=DISABLE,
    // 一旦 bus-off 就再也回不来(要按复位), 这里改成自动恢复:
    // 电机上电后总线上出现 ACK, 控制器会自己退出 bus-off 恢复正常收发。
    hcan1.Init.AutoBusOff = ENABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();

    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;   // mask 全 0 = 不过滤
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;       // CAN1 用 0~13, CAN2 用 14~27
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) Error_Handler();

    if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    // ---- 诊断用: CAN2 也开一路"只收不发", 看电机到底挂在哪个口 ----
    CAN_FilterTypeDef f2 = {0};
    f2.FilterBank           = 14;                 // CAN2 只能用 14~27
    f2.FilterMode           = CAN_FILTERMODE_IDMASK;
    f2.FilterScale          = CAN_FILTERSCALE_32BIT;
    f2.FilterIdHigh         = 0x0000;
    f2.FilterIdLow          = 0x0000;
    f2.FilterMaskIdHigh     = 0x0000;
    f2.FilterMaskIdLow      = 0x0000;
    f2.FilterFIFOAssignment = CAN_RX_FIFO1;       // 对应 CAN2_RX1 中断
    f2.FilterActivation     = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan2, &f2) != HAL_OK) Error_Handler();
    if (HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

void CanSend(void) {
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) return;   // 邮箱满则跳过本拍

    CAN_TxHeaderTypeDef hdr = {0};
    hdr.StdId = g_motor.txId();
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = 8;

    uint8_t data[8] = {0};
    g_motor.encode(data);

    uint32_t mailbox = 0;
    (void)HAL_CAN_AddTxMessage(&hcan1, &hdr, data, &mailbox);
}

// ---------------------------------------------------------------------------
// 串口上报: "ref,fdb\r\n", 两位小数, 走 USART1 的 DMA 发送
// 自己写定点格式化, 避免 newlib-nano 下 %f 默认不输出
// ---------------------------------------------------------------------------
int AppendFixed(char *dst, float v, int dec) {
    bool neg = (v < 0.0f);
    if (neg) v = -v;
    if (v > 99999.0f) v = 99999.0f;

    uint32_t scale = 1;
    for (int i = 0; i < dec; ++i) scale *= 10u;

    const uint32_t q  = static_cast<uint32_t>(v * static_cast<float>(scale) + 0.5f);
    uint32_t ip = q / scale;
    uint32_t fp = q % scale;
    if (q == 0u) neg = false;   // 避免输出 "-0.00"

    char tmp[24];
    int  n = 0;
    if (neg) tmp[n++] = '-';

    char digits[10];
    int  m = 0;
    do {
        digits[m++] = static_cast<char>('0' + (ip % 10u));
        ip /= 10u;
    } while (ip != 0u && m < 9);
    while (m > 0) tmp[n++] = digits[--m];

    if (dec > 0) {
        tmp[n++] = '.';
        uint32_t d = scale / 10u;
        for (int i = 0; i < dec; ++i) {
            tmp[n++] = static_cast<char>('0' + ((fp / d) % 10u));
            d /= 10u;
        }
    }

    std::memcpy(dst, tmp, static_cast<size_t>(n));
    return n;
}

void TelemetryTick(void) {
    if (++g_tele_div < cfg::kTelemetryDiv) return;
    g_tele_div = 0;

    if (huart1.gState != HAL_UART_STATE_READY) return;   // 上一帧没发完, 丢掉本帧

    char *buf = g_tx_buf[g_tx_idx];
    int   n   = 0;
    n += AppendFixed(buf + n, g_dbg_ref, 2);
    buf[n++] = ',';
    n += AppendFixed(buf + n, g_dbg_fdb, 2);
    buf[n++] = '\r';
    buf[n++] = '\n';

    g_tx_idx ^= 1u;
    (void)HAL_UART_Transmit_DMA(&huart1, reinterpret_cast<uint8_t *>(buf),
                                static_cast<uint16_t>(n));
}

// ---------------------------------------------------------------------------
// 模式切换 / 按键 / 看门狗演示
// ---------------------------------------------------------------------------
void SwitchMode(bool position_mode) {
    g_mode_position = position_mode;
    g_t      = 0.0f;
    g_theta0 = g_motor.angle();    // 位置模式以当前角度为中心, 避免上电跳变
    g_dbg_ref = 0.0f;
    g_w_filt  = 0.0f;
    g_spd_pid.reset();
    g_pos_pid.reset();
}

// 长按 KEY: 先让电机停止输出, 再人为死循环 -> 主循环不再喂狗 -> 0.8s 后复位
void WatchdogDemo(void) {
    g_motor.setVoltage(0);
    CanSend();
    Led(false, false, false);
    __disable_irq();
    while (1) {
    }
}

void HandleKey(void) {
    const bool pressed = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);

    if (pressed) {
        if (g_key_hold_ms < 3000) ++g_key_hold_ms;
    } else {
        if (g_key_hold_ms > cfg::kKeyDebounceMs && g_key_hold_ms < cfg::kKeyLongPressMs) {
            SwitchMode(!g_mode_position);      // 短按切换速度/位置
        }
        g_key_hold_ms = 0;
    }

    if (g_key_hold_ms >= cfg::kKeyLongPressMs) {
        WatchdogDemo();
    }
}

// ---------------------------------------------------------------------------
// 1kHz 主控制
// ---------------------------------------------------------------------------
// 摩擦前馈: 用平滑符号函数表示给定方向, 给定一转正/转负就给对应方向的电流
inline float FrictionFeedforward(float w_ref_rpm) {
    float dir = w_ref_rpm / 1.0f;               // ±1rpm 过渡带: 斜坡而非台阶
    if (dir >  1.0f) dir =  1.0f;
    if (dir < -1.0f) dir = -1.0f;

    float a = (w_ref_rpm < 0.0f ? -w_ref_rpm : w_ref_rpm) / 25.0f;
    if (a > 1.0f) a = 1.0f;

    const float mag = cfg::kFricLow - (cfg::kFricLow - cfg::kFricHigh) * a;
    return mag * dir;
}

// 过零清积分的逻辑已移除（实测会扰动过零那一拍，且 Ki 很小时收益不明显）

void RunControl(void) {
    const uint32_t now    = HAL_GetTick();
    const bool     online = g_motor_seen && ((now - g_last_rx_ms) < cfg::kCanTimeoutMs);

    const float ramp = (g_t < cfg::kRampInSec) ? (g_t / cfg::kRampInSec) : 1.0f;

    float u = 0.0f;

    if (!online) {
        // 掉线保护: 输出清零, 清 PID, 红灯 2Hz 闪
        g_spd_pid.reset();
        g_pos_pid.reset();
        g_dbg_ref = 0.0f;
        g_dbg_fdb = g_mode_position ? (g_motor.angle() - g_theta0) * kRadToDeg
                                    : static_cast<float>(g_motor.velRpm());
        Led(false, false, (now % 500u) < 250u);
    } else if (!g_mode_position) {
        // ---- 速度环: 给定 rpm 正弦, 前馈 + PI, 输出即电流 ----
        g_t += cfg::kLoopDt;
        const float w_ref = cfg::kSpdAmpRpm * ramp * std::sin(kTwoPi * cfg::kSpdFreqHz * g_t);
        g_w_filt += cfg::kFdbAlpha * (static_cast<float>(g_motor.velRpm()) - g_w_filt);
        const float w_fb = g_w_filt;

        u = cfg::kCurPerRpm * w_ref + FrictionFeedforward(w_ref)
          + g_spd_pid.calc(w_ref, w_fb);

        g_dbg_ref = w_ref;
        g_dbg_fdb = w_fb;
        Led(false, true, false);                       // 绿灯常亮 = 速度模式
    } else {
        // ---- 位置环: 给定 rad 正弦, 位置 P(D) -> 速度给定 -> 速度环 ----
        g_t += cfg::kLoopDt;
        const float w_sin  = kTwoPi * cfg::kPosFreqHz;
        const float amp    = cfg::kPosAmpDeg * kPi / 180.0f;
        const float th_ref = g_theta0 + amp * ramp * std::sin(w_sin * g_t);
        const float th_fb  = g_motor.angle();

        // 给定微分前馈: 只靠反馈追正弦会有相位滞后, 前馈把滞后补掉
        float w_ref = amp * w_sin * std::cos(w_sin * g_t)
                    + g_pos_pid.calc(th_ref, th_fb);
        if (w_ref >  cfg::kPosSpdLimit) w_ref =  cfg::kPosSpdLimit;
        if (w_ref < -cfg::kPosSpdLimit) w_ref = -cfg::kPosSpdLimit;

        const float w_ref_rpm = w_ref * kRadToRpm;
        g_w_filt += cfg::kFdbAlpha * (static_cast<float>(g_motor.velRpm()) - g_w_filt);
        const float w_fb_rpm  = g_w_filt;

        u = cfg::kCurPerRpm * w_ref_rpm + FrictionFeedforward(w_ref_rpm)
          + g_spd_pid.calc(w_ref_rpm, w_fb_rpm);

        g_dbg_ref = (th_ref - g_theta0) * kRadToDeg;   // 上报相对角度(度)
        g_dbg_fdb = (th_fb  - g_theta0) * kRadToDeg;
        Led(true, false, false);                       // 蓝灯常亮 = 位置模式
    }

    // 输出总限幅, 再换算成 int16 电流给定
    if (u >  cfg::kOutLimit) u =  cfg::kOutLimit;
    if (u < -cfg::kOutLimit) u = -cfg::kOutLimit;

    g_dbg_u    = u;                                     // 给 Ozone 看控制量
    g_dbg_mode = g_mode_position ? 1u : 0u;
    g_motor.setVoltage(static_cast<int16_t>(u));
}

} // namespace

// ===========================================================================
// 对外接口 (C 链接)
// ===========================================================================
extern "C" {

void ControlTaskInit(void) {
    // 调试时单步暂停不让看门狗复位, 否则断点一停板子就重启
    __HAL_DBGMCU_FREEZE_IWDG();

    // CubeMX 打开了 USART3(DBUS) 和 USART6 的中断, 但本工程只用 USART1。
    // 这两个口没接设备时引脚悬空, 会不停产生溢出/帧错误中断,
    // 它们的中断优先级是 0, 会不停抢占 1kHz 控制任务, 把主循环饿死。
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    HAL_NVIC_DisableIRQ(USART6_IRQn);

    GpioUserInit();
    g_motor.setCurrentMode(cfg::kCurrentMode);   // 电流模式 -> 发 0x1FE
    CanUserInit();

    g_last_rx_ms = HAL_GetTick();
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
}

void ControlTaskLoop(void) {
    static uint32_t s_fed = 0;
    if (g_task_tick != s_fed) {
        s_fed = g_task_tick;
        HAL_IWDG_Refresh(&hiwdg);      // 任务心跳正常才喂狗
    }
}

void MainTask(void) {
    g_task_tick = g_task_tick + 1u;   // C++20 下 ++ 作用于 volatile 已弃用
    HandleKey();
    RunControl();
    CanSend();
    TelemetryTick();
}

void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id != g_motor.rxId()) return;
    g_motor.decode(data);
    g_last_rx_ms = HAL_GetTick();
    g_motor_seen = true;
}

// ---- HAL 弱回调转发 ----
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) MainTask();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance != CAN1) return;

    CAN_RxHeaderTypeDef hdr;
    uint8_t             data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) return;
    g_dbg_rx_cnt = g_dbg_rx_cnt + 1u;   // 先记账, 再看 ID 对不对
    g_dbg_rx_id  = hdr.StdId;
    CanFeedbackCallback(hdr.StdId, data);
}

// CAN2 只做监听(诊断): 不参与控制, 只统计收到的帧和 ID
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef hdr;
    uint8_t             data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &hdr, data) != HAL_OK) return;
    if (hcan->Instance == CAN2) {
        g_dbg_rx2_cnt = g_dbg_rx2_cnt + 1u;
        g_dbg_rx2_id  = hdr.StdId;
    }
}

} // extern "C"
