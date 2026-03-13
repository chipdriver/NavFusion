#include "led.h"
#include "stm32f411xe.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"

#define LED1_GPIO_Port  GPIOA
#define LED1_Pin        GPIO_PIN_1
#define LED2_GPIO_Port  GPIOA
#define LED2_Pin        GPIO_PIN_2

/**
 * @brief LED IO init
 */
void LED_IO_init(void)
{
    GPIO_InitTypeDef    GPIO_InitStruct;
    //1.使能时钟GPIOA
    __HAL_RCC_GPIOA_CLK_ENABLE();

    //2.配置参数
    GPIO_InitStruct.Pin     =   LED1_Pin | LED2_Pin;
    GPIO_InitStruct.Mode    =   GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    =   GPIO_PULLDOWN;
    GPIO_InitStruct.Speed   =   GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);

    //3.默认状态
    LED_AllOff();
}

/**
 * @brief LED wirte
 */
static inline void led_write(GPIO_TypeDef *Port,uint16_t pin,int on)
{
    if(on)
    {
        HAL_GPIO_WritePin(Port,pin,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(Port,pin,GPIO_PIN_SET);
    }
}

/**
 * @brief LED1 ON
 */
void LED1_ON(void)
{
    led_write(LED1_GPIO_Port,LED1_Pin,1);
}

/**
 * @brief LED1 OFF
 */
void LED1_OFF(void)
{
    led_write(LED1_GPIO_Port,LED1_Pin,0);
}

/**
 * @brief LED1 Toggle
 */
void LED1_Toggle(void)
{
    HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
}

/**
 * @brief LED1 ON
 */
void LED2_ON(void)
{
    led_write(LED2_GPIO_Port,LED2_Pin,1);
}

/**
 * @brief LED1 OFF
 */
void LED2_OFF(void)
{
    led_write(LED2_GPIO_Port,LED2_Pin,0);
}

/**
 * @brief LED1 Toggle
 */
void LED2_Toggle(void)
{
    HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
}

/**
 * @brief LED AllOff
 */
void LED_AllOff(void)
{
    LED1_OFF();
    LED2_OFF();
}

/**
 * @brief LED1 闪烁指定次数（阻塞式）
 * @param times   闪烁次数（亮+灭算 1 次）
 * @param on_ms   每次点亮持续时间，单位：ms
 * @param off_ms  每次熄灭持续时间，单位：ms
 * @return None
 * @note  本函数为阻塞式实现：内部使用 HAL_Delay，会占用 CPU。
 *        适合用于启动提示、校准提示等短时间灯语；不建议在实时性要求高的循环中频繁调用。
 *        若你的 LED 为 Active-Low（PA 拉低点亮），LED1_On/Off 内部已处理电平逻辑。
 */
void LED1_Blink(uint8_t times,uint32_t on_ms,uint32_t off_ms)
{
    for(uint8_t i = 0; i < times; i++)
    {
        LED1_ON();
        HAL_Delay(on_ms);
        LED1_OFF();
        HAL_Delay(off_ms);
    }
}

/**
 * @brief LED2 闪烁指定次数（阻塞式）
 * @param times   闪烁次数（亮+灭算 1 次）
 * @param on_ms   每次点亮持续时间，单位：ms
 * @param off_ms  每次熄灭持续时间，单位：ms
 * @return None
 * @note  本函数为阻塞式实现：内部使用 HAL_Delay，会占用 CPU。
 *        适合用于启动提示、校准提示等短时间灯语；不建议在实时性要求高的循环中频繁调用。
 *        若你的 LED 为 Active-Low（PA 拉低点亮），LED2_On/Off 内部已处理电平逻辑。
 */
void LED2_Blink(uint8_t times, uint32_t on_ms, uint32_t off_ms)
{
    for (uint8_t i = 0; i < times; i++)
    {
        LED2_ON();
        HAL_Delay(on_ms);
        LED2_OFF();
        HAL_Delay(off_ms);
    }
}

