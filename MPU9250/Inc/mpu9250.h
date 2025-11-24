#ifndef __MPU9250_H__
#define __MPU9250_H__

/*========================头文件 ========================*/
#include "i2c.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "gpio.h"
#include <math.h> 
/*======================== 宏定义 ======================*/
//存放 MPU9250 原始数据
typedef struct 
{
    int16_t accel_x;  // 加速度计 X 轴原始值（-32768 ~ +32767）
    int16_t accel_y;  // 加速度计 Y 轴原始值
    int16_t accel_z;  // 加速度计 Z 轴原始值
    int16_t gyro_x;   // 陀螺仪 X 轴原始值
    int16_t gyro_y;   // 陀螺仪 Y 轴原始值
    int16_t gyro_z;   // 陀螺仪 Z 轴原始值
    int16_t temp;     // 温度传感器原始值
} MPU9250_raw_Data;

//存放转换后的物理量
typedef struct 
{
    float accel_x_g;    // 加速度计 X 轴物理量（单位：g，重力加速度）
    float accel_y_g;    // 加速度计 Y 轴物理量（单位：g）
    float accel_z_g;    // 加速度计 Z 轴物理量（单位：g）
    float gyro_x_dps;   // 陀螺仪 X 轴物理量（单位：°/s，度每秒）
    float gyro_y_dps;   // 陀螺仪 Y 轴物理量（单位：°/s）
    float gyro_z_dps;   // 陀螺仪 Z 轴物理量（单位：°/s）
    float temp_c;       // 温度物理量（单位：°C，摄氏度）
} MPU9250_Physical_Data;  

// 磁力计原始数据
typedef struct
{
    int16_t mag_x;   // X 轴原始值
    int16_t mag_y;   // Y 轴原始值
    int16_t mag_z;   // Z 轴原始值
} AK8963_raw_Data;

// 磁力计物理量（单位：μT）
typedef struct
{
    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;
} AK8963_Physical_Data;

/*====================== 姿态角相关定义 ======================*/

/**
 * @brief 欧拉角结构体（单位：弧度）
 * 
 * 说明：
 *   - roll  : 绕 X 轴旋转（机体前进方向）
 *   - pitch : 绕 Y 轴旋转（机体右侧方向）
 *   - yaw   : 绕 Z 轴旋转（机体向下方向，NED）
 */
typedef struct
{
    float roll;   // 横滚角（rad）
    float pitch;  // 俯仰角（rad）
    float yaw;    // 航向角（rad）
} EulerAngle_t;

/**
 * @brief 由加速度计 + 磁力计解算得到的姿态角（单位：弧度）
 * 
 * 说明：
 *   - roll/pitch 由加速度计确定（静止或低动态更可靠）
 *   - yaw        由磁力计（倾斜补偿后）确定
 *   - 后续 Mahony/Madgwick 融合时，可将其作为“测量值”使用
 */
extern EulerAngle_t g_euler_acc_mag;

/**
 * @brief 使用加速度计 + 磁力计进行姿态解算（A+M → Roll/Pitch/Yaw）
 *
 * @param imu  已转换为物理量的六轴数据（单位：g / °/s），
 *             注意：此处只会用到 accel_x_g/y_g/z_g
 * @param mag  已做硬铁/软铁校准后的磁力计物理量（单位：μT）
 *
 * @note  输入的 imu/mag 为“模块坐标系”的物理量，
 *        函数内部会自动转换到标准机体系（X 前、Y 右、Z 下）。
 */
void MPU9250_ComputeEuler_FromAccMag(const MPU9250_Physical_Data *imu,
                                     const AK8963_Physical_Data *mag);

/**
 * @brief 获取当前姿态角（单位：度）
 * @param roll_deg  输出 roll 角（度）
 * @param pitch_deg 输出 pitch 角（度）
 * @param yaw_deg   输出 yaw 角（度）
 */
void MPU9250_GetEulerDeg(float *roll_deg, float *pitch_deg, float *yaw_deg);



extern float g_gyro_bias_dps[3]; //陀螺仪偏置（单位：°/s）
extern float g_accel_bias_g[3]; // 加速度计零偏（单位：g）
extern float g_mag_offset[3]; // 磁力计硬铁偏置（单位：μT）
extern float g_mag_scale[3]; // 磁力计软铁缩放（单位：比例因子）
#define MPU9250_I2C_ADDR7        0x68U  // AD0 接 GND -> I2C 7 bit 地址为0b11010000 = 0x68
#define AK8963_I2C_ADDR7         0x0CU  // AK8963 地磁计 I2C 7 bit 地址为0b00001100 = 0x0C
#define MPU9250_REG_WHO_AM_I     0x75U  // WHO_AM_I 寄存器地址 = 117(dec) = 0x75
/*======================== 函数声明 ========================*/
int MPU9250_9Axis_Init(void); //初始化MPU9250九轴传感器
int MPU9250_Read_9Axis(MPU9250_raw_Data *mpu_raw, 
                       MPU9250_Physical_Data *mpu_phys,
                       AK8963_raw_Data *ak_raw,
                       AK8963_Physical_Data *ak_phys); //读取 MPU9250 九轴传感器的完整数据
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms); //校准陀螺仪零偏       
void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms); //校准加速度计零偏  
void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms); //磁力计硬铁 + 软铁校准           
#endif /* __MPU9250_H__ */
