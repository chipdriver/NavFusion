 #ifndef __I2C_H__
 #define __I2C_H__

//头文件
#include "stm32f4xx.h"

//宏定义
#define SCL_H()  (GPIOB->BSRR = (1 << 6))      //PB6置高
#define SCL_L()  (GPIOB->BSRR = (1 << (6 + 16))) //PB6置低
#define SDA_H()  (GPIOB->BSRR = (1 << 7))      //PB7置高
#define SDA_L()  (GPIOB->BSRR = (1 << (7 + 16))) //PB7置低
#define SDA_READ() ( GPIOB->IDR & (1 << 7) ) //读取PB7状态
#define SCL_READ() ( GPIOB->IDR & (1 << 6) ) //读取PB6状态


//函数声明
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t data);
uint8_t I2C_ReadByte(uint8_t send_ack);
int I2C_WriteReg(uint8_t dev7, uint8_t reg, uint8_t val);
uint8_t I2C_ReadReg(uint8_t dev7, uint8_t reg);
uint8_t I2C_WaitAck(void);
void I2C_SendACK(void);
void I2C_NACK(void);


 #endif /* __I2C_H__ */