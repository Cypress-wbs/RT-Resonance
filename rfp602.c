#include <rtdevice.h>
#include <board.h>
#include <rfp602.h>
#include <stm32f4xx_hal.h>

// RFP 硬件配置宏
#define RFP_DEV_NAME        "adc1"
#define RFP1_CHANNEL        0
#define RFP2_CHANNEL        1
#define RFP_GPIO_PORT       GPIOA
#define RFP_GPIO_PINS       (GPIO_PIN_0 | GPIO_PIN_1)

static rt_adc_device_t adc_dev = RT_NULL;

// ==================== [ADC 硬件底层引脚复用] ====================
// 当开启 ADC 时，HAL 库会自动回调这个函数来配置 PA0 和 PA1
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = RFP_GPIO_PINS;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; // 必须是模拟输入模式
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(RFP_GPIO_PORT, &GPIO_InitStruct);
    }
}

// ==================== [RFP 初始化] ====================
int RFP_init(void)
{
    adc_dev = (rt_adc_device_t)rt_device_find(RFP_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        g_RFP_calib_state = -1; // 标记 ADC 失败
        return -1;
    }

    // 使能 ADC 通道
    rt_adc_enable(adc_dev, RFP1_CHANNEL);
    rt_adc_enable(adc_dev, RFP2_CHANNEL);
    g_RFP_calib_state = 0; // 标记正常

    return 0;
}

// ==================== [RFP 读取单次原始数据] ====================
void RFP_read_raw(void)
{
    if (adc_dev != RT_NULL && g_RFP_calib_state != -1)
    {
        g_RFP1_raw = (int32_t)rt_adc_read(adc_dev, RFP1_CHANNEL);
        g_RFP2_raw = (int32_t)rt_adc_read(adc_dev, RFP2_CHANNEL);
    }
    else
    {
        // 显影剂：如果 ADC 坏了，读出来是 -500
        g_RFP1_raw = -500;
        g_RFP2_raw = -500;
    }
}
