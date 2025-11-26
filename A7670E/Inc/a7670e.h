/**
 ******************************************************************************
 * @file    a7670e.h
 * @brief   SIMCom A7670E 4G/GNSS 模块驱动头文件
 * @note    支持 AT 指令、网络初始化、GNSS 定位功能
 ******************************************************************************
 */

#ifndef A7670E_H
#define A7670E_H

/*==============================================================================
 *                           头文件包含
 *============================================================================*/
#include "stm32f4xx_hal.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
/*==============================================================================
 *                           宏定义
 *============================================================================*/
#define AT_RX_BUF_SIZE 512  // 接收缓冲区大小

/*==============================================================================
 *                           外部变量声明
 *============================================================================*/
extern double lat_wgs, lon_wgs;  // WGS-84 坐标
/*==============================================================================
 *                           函数声明
 *============================================================================*/

/* AT 指令操作函数 */
uint8_t AT_SendAndWait(const char *cmd, const char *expect, uint32_t timeout_ms);               // 发送AT指令并等待响应
uint8_t AT_SendFormatAndWait(const char *expect, uint32_t timeout_ms, const char *format, ...); // 发送格式化AT指令并等待响应

/* UART 操作函数 */
void AT_UART_Start_IT(void);    //使能UART接收中断

/* 网络与 GNSS 初始化函数 */
uint8_t AT_Network_Init(void);  //初始化网络
uint8_t AT_GNSS_Init(void);     //初始化GNSS模块
void AT_Getlocation_Init(void); //获取位置信息初始化

/* GNSS 数据获取函数 */
uint8_t AT_GNSS_GetLocation(void); //获取位置信息

//void wgs84_to_gcj02(double lat, double lon, double *out_lat, double *out_lon);  // WGS-84 转 GCJ-02 坐标转换

#endif // A7670E_H