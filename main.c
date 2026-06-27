#include <rtthread.h>
#include <board.h>
#include <rfp602.h>
#include "oled_test.h"

// 外部驱动函数声明
extern int mpu6050_init(void);
extern void mpu6050_get_gyro(short *gx, short *gy, short *gz);

// ==================== [系统参数与阈值宏定义] ====================
#define SAMPLE_PERIOD_MS            5
#define GYRO_LPF_DIV                4
#define RFP_AVG_WINDOW              10
#define RFP_CALIB_SAMPLES           100  // 极速标定采样数 (0.5秒)
#define FEATURE_WINDOW_SAMPLES      400  // 特征工程滑动窗口大小 (2秒数据)
#define FEATURE_UPDATE_INTERVAL     20   // 特征刷新间隔 (每100ms计算一次)

// 【传感器状态标定阈值】
#define RFP_RELAX_TRIGGER           500  // 放松触发阈值
#define RFP_SQUEEZE_TRIGGER         2200 // 用力紧绷触发阈值
#define RFP_MIN_SPAN                50   // 标定最小差值保护

// 【新增：运动学高级抗噪参数】
#define MIN_VIBRATO_AMPLITUDE       600  // 有效揉弦的最小振幅(大幅提高，无视静息颤抖)
#define MIN_CROSSING_GAP            5    // 施密特触发器的最小时间锁(拒绝高频生理噪点)

// RFP 标定状态机状态定义
enum
{
    RFP_STATE_ADC_ERROR = -1,
    RFP_STATE_WAITING_RELAX = 0,
    RFP_STATE_COLLECTING_RELAX = 1,
    RFP_STATE_WAITING_SQUEEZE = 2,
    RFP_STATE_COLLECTING_SQUEEZE = 3,
    RFP_STATE_READY = 4,
};

// ==================== [供 Python 穿透读取的全局变量] ====================
#define EXPORT_VAR volatile int32_t __attribute__((used))

EXPORT_VAR g_raw_gy = 0;
EXPORT_VAR g_filtered_gy = 0;

EXPORT_VAR g_RFP1_raw = 0;
EXPORT_VAR g_RFP1_filtered = 0;
EXPORT_VAR g_RFP1_tension_pct = 0;
EXPORT_VAR g_RFP1_min = 0;
EXPORT_VAR g_RFP1_max = 4095;
EXPORT_VAR g_RFP1_feature_mean = 0;
EXPORT_VAR g_RFP1_feature_var = 0;

EXPORT_VAR g_RFP2_raw = 0;
EXPORT_VAR g_RFP2_filtered = 0;
EXPORT_VAR g_RFP2_tension_pct = 0;
EXPORT_VAR g_RFP2_min = 0;
EXPORT_VAR g_RFP2_max = 4095;
EXPORT_VAR g_RFP2_feature_mean = 0;
EXPORT_VAR g_RFP2_feature_var = 0;

EXPORT_VAR g_RFP_calib_state = 0;
EXPORT_VAR g_vibrato_freq_x100 = 0;
EXPORT_VAR g_vibrato_amplitude = 0;

// ==================== [滑窗与滤波器静态缓冲区] ====================
static rt_uint32_t s_RFP1_fifo[RFP_AVG_WINDOW] = {0};
static rt_uint32_t s_RFP2_fifo[RFP_AVG_WINDOW] = {0};
static int32_t s_gyro_window[FEATURE_WINDOW_SAMPLES] = {0};
static int32_t s_RFP1_window[FEATURE_WINDOW_SAMPLES] = {0};
static int32_t s_RFP2_window[FEATURE_WINDOW_SAMPLES] = {0};

static int32_t ordered_gyro[FEATURE_WINDOW_SAMPLES];
static int32_t ordered_RFP1[FEATURE_WINDOW_SAMPLES];
static int32_t ordered_RFP2[FEATURE_WINDOW_SAMPLES];

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void update_RFP_statistics(const int32_t *RFP1_window, const int32_t *RFP2_window, int32_t sample_count)
{
    int64_t RFP1_sum = 0, RFP2_sum = 0;
    int64_t RFP1_var_acc = 0, RFP2_var_acc = 0;
    int32_t i;

    for (i = 0; i < sample_count; i++)
    {
        RFP1_sum += RFP1_window[i];
        RFP2_sum += RFP2_window[i];
    }
    g_RFP1_feature_mean = (int32_t)(RFP1_sum / sample_count);
    g_RFP2_feature_mean = (int32_t)(RFP2_sum / sample_count);

    for (i = 0; i < sample_count; i++)
    {
        int32_t RFP1_diff = RFP1_window[i] - g_RFP1_feature_mean;
        int32_t RFP2_diff = RFP2_window[i] - g_RFP2_feature_mean;
        RFP1_var_acc += (int64_t)RFP1_diff * RFP1_diff;
        RFP2_var_acc += (int64_t)RFP2_diff * RFP2_diff;
    }
    g_RFP1_feature_var = (int32_t)(RFP1_var_acc / sample_count);
    g_RFP2_feature_var = (int32_t)(RFP2_var_acc / sample_count);
}

static void update_vibrato_features(const int32_t *gyro_window, int32_t sample_count)
{
    int64_t gyro_sum = 0;
    int32_t gyro_max = gyro_window[0];
    int32_t gyro_min = gyro_window[0];
    int32_t i;

    for (i = 0; i < sample_count; i++)
    {
        gyro_sum += gyro_window[i];
        if (gyro_window[i] > gyro_max) gyro_max = gyro_window[i];
        if (gyro_window[i] < gyro_min) gyro_min = gyro_window[i];
    }

    int32_t gyro_mean = (int32_t)(gyro_sum / sample_count);
    int32_t amplitude = gyro_max - gyro_min;

    int32_t threshold = amplitude / 4;
    if (threshold < 150) threshold = 150; // 提高底层防噪线

    int32_t upward_crossings = 0;
    int32_t first_crossing_idx = -1;
    int32_t last_crossing_idx = -1;

    int8_t current_state = (gyro_window[0] - gyro_mean > 0) ? 1 : -1;
    int32_t last_state_change = -100;

    for (i = 0; i < sample_count; i++)
    {
        int32_t centered = gyro_window[i] - gyro_mean;

        if (current_state == -1 && centered >= threshold)
        {
            // 【时间间隙锁】：两次过零必须间隔至少 MIN_CROSSING_GAP 个点，否则视为噪点丢弃！
            if (i - last_state_change >= MIN_CROSSING_GAP)
            {
                current_state = 1;
                if (first_crossing_idx == -1) first_crossing_idx = i;
                last_crossing_idx = i;
                upward_crossings++;
                last_state_change = i;
            }
        }
        else if (current_state == 1 && centered <= -threshold)
        {
            if (i - last_state_change >= MIN_CROSSING_GAP)
            {
                current_state = -1;
                last_state_change = i;
            }
        }
    }

    g_vibrato_amplitude = amplitude;

    // 【振幅锁】：静置时微小颤动振幅达不到 MIN_VIBRATO_AMPLITUDE (600)，直接归零！
    if (amplitude < MIN_VIBRATO_AMPLITUDE || upward_crossings < 2)
    {
        g_vibrato_freq_x100 = 0;
    }
    else
    {
        int32_t delta_samples = last_crossing_idx - first_crossing_idx;
        if (delta_samples > 0)
        {
            g_vibrato_freq_x100 = ((upward_crossings - 1) * 100000) / (delta_samples * SAMPLE_PERIOD_MS);
        }
        else
        {
            g_vibrato_freq_x100 = 0;
        }
    }
}

static void edge_ai_thread_entry(void *parameter)
{
    short raw_gx, raw_gy, raw_gz;
    int32_t gyro_lpf = 0;
    rt_uint8_t gyro_lpf_initialized = 0;

    int32_t RFP1_fifo_sum = 0, RFP2_fifo_sum = 0;
    int32_t RFP_fifo_count = 0, RFP_fifo_index = 0;

    int64_t relax_sum1 = 0, relax_sum2 = 0;
    int64_t squeeze_sum1 = 0, squeeze_sum2 = 0;
    int32_t calib_count = 0;

    int32_t feature_index = 0;
    int32_t feature_count = 0;
    int32_t feature_tick = 0;

    while (1)
    {
        // 1. 读取 MPU6050
        mpu6050_get_gyro(&raw_gx, &raw_gy, &raw_gz);
        g_raw_gy = (int32_t)raw_gy;

        if (!gyro_lpf_initialized)
        {
            gyro_lpf = (int32_t)raw_gy;
            gyro_lpf_initialized = 1;
        }
        else
        {
            gyro_lpf = gyro_lpf + (((int32_t)raw_gy - gyro_lpf) / GYRO_LPF_DIV);
        }
        g_filtered_gy = gyro_lpf;

        // 2. 读取 RFP 并滤波
        RFP_read_raw();

        int32_t temp_RFP = g_RFP1_raw;
        g_RFP1_raw = g_RFP2_raw;
        g_RFP2_raw = temp_RFP;

        if (g_RFP_calib_state != RFP_STATE_ADC_ERROR)
        {
            if (RFP_fifo_count < RFP_AVG_WINDOW)
            {
                RFP1_fifo_sum += g_RFP1_raw;
                RFP2_fifo_sum += g_RFP2_raw;
                s_RFP1_fifo[RFP_fifo_index] = g_RFP1_raw;
                s_RFP2_fifo[RFP_fifo_index] = g_RFP2_raw;
                RFP_fifo_count++;
            }
            else
            {
                RFP1_fifo_sum -= (int32_t)s_RFP1_fifo[RFP_fifo_index];
                RFP2_fifo_sum -= (int32_t)s_RFP2_fifo[RFP_fifo_index];
                RFP1_fifo_sum += g_RFP1_raw;
                RFP2_fifo_sum += g_RFP2_raw;
                s_RFP1_fifo[RFP_fifo_index] = g_RFP1_raw;
                s_RFP2_fifo[RFP_fifo_index] = g_RFP2_raw;
            }

            RFP_fifo_index = (RFP_fifo_index + 1) % RFP_AVG_WINDOW;
            g_RFP1_filtered = RFP1_fifo_sum / RFP_fifo_count;
            g_RFP2_filtered = RFP2_fifo_sum / RFP_fifo_count;

            switch (g_RFP_calib_state)
            {
                case RFP_STATE_WAITING_RELAX:
                    if ((g_RFP1_filtered > RFP_RELAX_TRIGGER) || (g_RFP2_filtered > RFP_RELAX_TRIGGER))
                    {
                        relax_sum1 = 0; relax_sum2 = 0; calib_count = 0;
                        g_RFP_calib_state = RFP_STATE_COLLECTING_RELAX;
                    }
                    break;

                case RFP_STATE_COLLECTING_RELAX:
                    relax_sum1 += g_RFP1_filtered;
                    relax_sum2 += g_RFP2_filtered;
                    calib_count++;
                    if (calib_count >= RFP_CALIB_SAMPLES)
                    {
                        g_RFP1_min = (int32_t)(relax_sum1 / calib_count);
                        g_RFP2_min = (int32_t)(relax_sum2 / calib_count);
                        calib_count = 0;
                        g_RFP_calib_state = RFP_STATE_WAITING_SQUEEZE;
                    }
                    break;

                case RFP_STATE_WAITING_SQUEEZE:
                    if ((g_RFP1_filtered > RFP_SQUEEZE_TRIGGER) || (g_RFP2_filtered > RFP_SQUEEZE_TRIGGER))
                    {
                        squeeze_sum1 = 0; squeeze_sum2 = 0; calib_count = 0;
                        g_RFP_calib_state = RFP_STATE_COLLECTING_SQUEEZE;
                    }
                    break;

                case RFP_STATE_COLLECTING_SQUEEZE:
                    squeeze_sum1 += g_RFP1_filtered;
                    squeeze_sum2 += g_RFP2_filtered;
                    calib_count++;
                    if (calib_count >= RFP_CALIB_SAMPLES)
                    {
                        g_RFP1_max = (int32_t)(squeeze_sum1 / calib_count);
                        g_RFP2_max = (int32_t)(squeeze_sum2 / calib_count);

                        if (g_RFP1_max <= (g_RFP1_min + RFP_MIN_SPAN)) g_RFP1_max = g_RFP1_min + RFP_MIN_SPAN;
                        if (g_RFP2_max <= (g_RFP2_min + RFP_MIN_SPAN)) g_RFP2_max = g_RFP2_min + RFP_MIN_SPAN;

                        calib_count = 0;
                        g_RFP_calib_state = RFP_STATE_READY;
                    }
                    break;

                case RFP_STATE_READY:
                default:
                    break;
            }

            if (g_RFP_calib_state == RFP_STATE_READY)
            {
                // 【核心修复：动态探底】如果发现当前握力比之前标定的"极小值"还要小，自动降低极小值！
                // 这彻底解决了刚标定完，一恢复正常握力就归 0% 的假死现象。
                if (g_RFP1_filtered < g_RFP1_min) g_RFP1_min = g_RFP1_filtered;
                if (g_RFP2_filtered < g_RFP2_min) g_RFP2_min = g_RFP2_filtered;

                g_RFP1_tension_pct = clamp_i32(((g_RFP1_filtered - g_RFP1_min) * 100) / (g_RFP1_max - g_RFP1_min), 0, 100);
                g_RFP2_tension_pct = clamp_i32(((g_RFP2_filtered - g_RFP2_min) * 100) / (g_RFP2_max - g_RFP2_min), 0, 100);
            }
            else
            {
                g_RFP1_tension_pct = 0;
                g_RFP2_tension_pct = 0;
            }
        }

        // 4. 特征滑动窗装载
        s_gyro_window[feature_index] = g_filtered_gy;
        s_RFP1_window[feature_index] = g_RFP1_tension_pct;
        s_RFP2_window[feature_index] = g_RFP2_tension_pct;

        feature_index = (feature_index + 1) % FEATURE_WINDOW_SAMPLES;
        if (feature_count < FEATURE_WINDOW_SAMPLES)
        {
            feature_count++;
        }

        // 5. 特征定时计算 (主频 100Hz 触发)
        feature_tick++;
        if ((feature_count >= FEATURE_WINDOW_SAMPLES) && (feature_tick >= FEATURE_UPDATE_INTERVAL))
        {
            int32_t i;
            for (i = 0; i < FEATURE_WINDOW_SAMPLES; i++)
            {
                int32_t src = (feature_index + i) % FEATURE_WINDOW_SAMPLES;
                ordered_gyro[i] = s_gyro_window[src];
                ordered_RFP1[i] = s_RFP1_window[src];
                ordered_RFP2[i] = s_RFP2_window[src];
            }
            update_vibrato_features(ordered_gyro, FEATURE_WINDOW_SAMPLES);
            update_RFP_statistics(ordered_RFP1, ordered_RFP2, FEATURE_WINDOW_SAMPLES);
            feature_tick = 0;
        }

        rt_thread_mdelay(SAMPLE_PERIOD_MS);
    }
}

int start_edge_ai_system(void)
{
    rt_thread_t tid = rt_thread_create("edge_ai", edge_ai_thread_entry, RT_NULL, 4096, 15, 10);
    if (tid != RT_NULL) rt_thread_startup(tid);
    return 0;
}

int main(void)
{
    int tick = 0;

    rt_thread_mdelay(1000);

    oled_init();
    mpu6050_init();
    RFP_init();

    start_edge_ai_system();

    oled_clear();

    while (1)
    {
        oled_show_debug_panel(g_raw_gy, g_RFP1_raw, g_RFP2_raw, tick++);
        rt_thread_mdelay(50);
    }
    return 0;
}
