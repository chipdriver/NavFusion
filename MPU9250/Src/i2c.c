/**
 ******************************************************************************
 * @file    i2c.c
 * @brief   软件模拟 I2C 协议实现（GPIO 位操作）
 * @note    适用于 STM32 平台，支持标准 I2C 通信协议
 ******************************************************************************
 */

#include "i2c.h"

/*==============================================================================
 *                           私有函数声明
 *============================================================================*/
static void    I2C_Delay(void);
static void    I2C_SendBit(uint8_t bit);
static uint8_t I2C_ReadBit(void);

/*==============================================================================
 *                           私有函数实现
 *============================================================================*/

/**
 * @brief  简单延时函数（用于 I2C 时序控制）
 * @param  None
 * @return None
 * @note   延时时间约为 10us（100MHz 主频下），可根据实际需求调整
 */
static void I2C_Delay(void)
{
    for (volatile int i = 0; i < 100; i++);
}

/*==============================================================================
 *                           公共函数实现
 *============================================================================*/

/**
 * @brief  产生 I2C 起始条件（START）
 * @param  None
 * @return None
 * @note   I2C 协议：当 SCL 为高电平时，SDA 从高到低跳变 → 产生 START 条件
 */
void I2C_Start(void)
{
    // 1. 设置初始状态
    SDA_H();      // SDA 置高（确保初始状态）
    SCL_H();      // SCL 置高（准备产生 START）
    I2C_Delay();  // 延时，保持稳定状态

    // 2. 产生 START 条件
    SDA_L();      // SDA 拉低 → 产生 START 条件！
    I2C_Delay();  // 延时，保持 START 条件

    // 3. 准备传输数据
    SCL_L();      // SCL 拉低，准备传输数据
}

/**
 * @brief  产生 I2C 停止条件（STOP）
 * @param  None
 * @return None
 * @note   I2C 协议：当 SCL 为高电平时，SDA 从低到高跳变 → 产生 STOP 条件
 */
void I2C_Stop(void)
{
    // 1. 准备停止条件
    SCL_L();      // SCL 拉低，准备停止条件
    SDA_L();      // SDA 拉低，保持数据有效
    I2C_Delay();  // 延时，保持稳定状态

    // 2. 产生 STOP 条件
    SCL_H();      // SCL 置高，产生 STOP 条件
    I2C_Delay();  // 延时，确保 SCL 稳定
    SDA_H();      // SDA 置高，释放总线

    // 3. 延时
    I2C_Delay();  // 延时，确保 STOP 条件被识别
}

/**
 * @brief  发送 1 个数据位（主机驱动 SDA）
 * @param  bit  要发送的位（0 或 1）
 * @return None
 * @note   协议要求：
 *         - 数据在 SCL 高电平期间保持稳定，供接收方采样读取
 *         - 在 SCL 低电平期间切换 SDA，准备下一个数据位
 */
static void I2C_SendBit(uint8_t bit)
{
    if (bit)
        SDA_H();
    else
        SDA_L();

    I2C_Delay();  // 等待数据稳定
    SCL_H();      // 拉高时钟，对方开始采样
    I2C_Delay();  // 保持时钟高电平，确保数据被采样
    SCL_L();      // 拉低时钟，准备发送下一个 bit
}

/**
 * @brief 接收 1 个数据位（从机驱动 SDA）
 * @param  None
 * @return 0/1
 * @note 协议要求：主机在 SCL 高电平期间采样 SDA；SDA 在该期间保持稳定
 **/
static uint8_t I2C_ReadBit(void) 
{
    SDA_H();    // 释放 SDA，总线由从机驱动
    I2C_Delay();  // 等待总线稳定
    SCL_H();    // 拉高时钟，准备采样数据
    I2C_Delay();  // 在 SCL 高电平期间采样
    uint8_t bit = SDA_READ() ? 1 : 0; // 读取 SDA 状态
    SCL_L();    // 拉低时钟，准备下一个 bit
    return bit;
}


/**
 * @brief  发送 1 个字节数据（MSB→LSB）
 * @param  data  要发送的数据
 * @return None
 * @note   协议要求：每发送 8 个数据位后，接收方在第 9 个 SCL 周期产生 ACK/NACK
 */
void I2C_SendByte(uint8_t data)
{
    for (int i = 7; i >= 0; i--)
        I2C_SendBit((data >> i) & 0x01);
}

/**
 * @brief 读取 1 字节（MSB→LSB）
 * @param send_ack 0=读完后发送 ACK；1=读完后发送 NACK
 * @return 读取到的数据
 * @note 协议要求：接收方在每个 SCL 高电平期间采样数据，8 位后在第 9 个周期回 ACK/NACK。
 */
uint8_t I2C_ReadByte(uint8_t send_ack)
{
    uint8_t data = 0;
    SDA_H();    // 释放 SDA，总线由从机驱动

    for(int i = 0;i < 8; i++)
    {
        data <<= 1;
        data |= I2C_ReadBit();  // 读取 SDA 状态，组装数据
        
    }

    if(send_ack == 0)
        I2C_SendACK();  // 读完后发送 ACK
    else
        I2C_NACK();     // 读完后发送 NACK
    return data;
}

/**
 * @brief  写寄存器（单字节）: 起始 → 设备写地址 → 寄存器地址 → 数据 → 停止
 * @param  dev7  7位设备地址（未包含R/W位）
 * @param  reg   寄存器地址
 * @param  val   要写入的数据
 * @return 0=成功，非0=失败
 * @note   协议要求：地址帧和数据帧后均需检查从机 ACK。
 */
int I2C_WriteReg(uint8_t dev7, uint8_t reg, uint8_t val)
{
    I2C_Start();                   // 产生起始条件
    I2C_SendByte((dev7 << 1) | 0); // 发送设备地址+写位
    if(I2C_WaitAck()) { I2C_Stop();return -1;} // 检查 ACK

    I2C_SendByte(reg);
    if(I2C_WaitAck()) { I2C_Stop();return -2;} // 检查 ACK

    I2C_SendByte(val);
    if(I2C_WaitAck()) { I2C_Stop();return -3;} // 检查 ACK

    I2C_Stop();
    return 0;
}

/**
 * @brief  读寄存器（单字节）: 起始 → 设备写地址 → 寄存器地址 → 重起始 → 设备读地址 → 读数据(NACK) → 停止
 * @param  dev7  7位设备地址（未包含R/W位）
 * @param  reg   寄存器地址
 * @return 读取到的 1 字节（失败返回 0xFF）
 * @note   协议要求：随机读需要“重起始（Repeated START）”，最后一个字节后返回 NACK 再 STOP。
 */
uint8_t I2C_ReadReg(uint8_t dev7, uint8_t reg)
{
    uint8_t val;

    I2C_Start();
    I2C_SendByte((uint8_t)(dev7 << 1) | 0U);         /* 写方向 / write direction */
    if (I2C_WaitAck()) { I2C_Stop(); return 0xFF; }

    I2C_SendByte(reg);
    if (I2C_WaitAck()) { I2C_Stop(); return 0xFF; }

    /* 重起始进入读 / repeated START for read */
    I2C_Start();
    I2C_SendByte((uint8_t)(dev7 << 1) | 1U);         /* 读方向 / read direction */
    if (I2C_WaitAck()) { I2C_Stop(); return 0xFF; }

    val = I2C_ReadByte(1);                            /* 读1字节并NACK / read one byte then NACK */
    I2C_Stop();

    return val;
}

/**
 * @brief  等待从机应答（ACK）
 * @param  None
 * @return 0=收到ACK，1=未收到ACK
 * @note   协议要求：发送完 8 位后，发送方释放 SDA；接收方在第 9 个 SCL 高电平期间拉低 SDA 表示 ACK
 */
uint8_t I2C_WaitAck(void)
{
    SDA_H();    // 释放 SDA，总线由从机驱动
    I2C_Delay();  // 等待总线稳定
    SCL_H();    // 拉高时钟，准备采样 ACK
    I2C_Delay();  // 在 SCL 高电平期间采样
    uint8_t ack = (SDA_READ() ? 1 : 0);
    SCL_L();    // 拉低时钟，准备下一个 bit
    return ack;
}

/**
 * @brief 主机发送ACK（继续读取的场景）
 * @note 协议要求：在第 9 个时钟期间，发送方将 SDA 拉低表示 ACK。
 */
void I2C_SendACK(void)
{  
    SDA_L();    //主机拉低SDA，表示ACK
    I2C_Delay();  //等待数据稳定
    SCL_H();    //拉高时钟，发送ACK
    I2C_Delay();  //保持时钟高电平，确保ACK被识别
    SCL_L();    //拉低时钟，结束ACK，准备下一个操作
    SDA_H();    //释放SDA
}



/**
 * @brief 主机发送NACK(读最后一个字节后)
 * @note 协议要求：在第 9 个时钟期间，发送方释放 SDA（保持高）表示 NACK。
 */
void I2C_NACK(void)
{
    SDA_H();    // 主机保持SDA高电平，表示NACK
    I2C_Delay();  // 等待数据稳定
    SCL_H();    // 拉高时钟，发送NACK
    I2C_Delay();  // 保持时钟高电平，确保NACK被识别
    SCL_L();    // 拉低时钟，结束NACK，准备下一个操作
    I2C_Delay();  // 等待总线稳定
}









