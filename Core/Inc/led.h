#ifndef __LED_H__
#define __LED_H__
/* 头文件 */
#include "led.h"
#include "stm32f4xx_hal.h"

/* 函数声明*/
void LED_IO_init(void);

void LED1_ON(void);     // PA1 低电平点亮
void LED1_OFF(void);    // PA1 高电平熄灭
void LED1_Toggle(void);

void LED2_ON(void);     // PA2 低电平点亮
void LED2_OFF(void);    // PA2 高电平熄灭
void LED2_Toggle(void);

void LED_AllOff(void);

void LED1_Blink(uint8_t times, uint32_t on_ms, uint32_t off_ms);
void LED2_Blink(uint8_t times, uint32_t on_ms, uint32_t off_ms);

#endif /* __LED_H__ */