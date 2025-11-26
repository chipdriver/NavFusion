#ifndef __MPU9250_H__
#define __MPU9250_H__

/*======================== 头文件 ========================*/
#include "i2c.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "gpio.h"
#include <math.h> 

/*======================== 硬件配置宏定义 ========================*/
#define MPU9250_I2C_ADDR7        0x68U  // AD0 接 GND -> I2C 7位地址 0x68
#define AK8963_I2C_ADDR7         0x0CU  // AK8963 磁力计 I2C 7位地址 0x0C
#define MPU9250_REG_WHO_AM_I     0x75U  // WHO_AM_I 寄存器地址

/*======================== 数据结构定义 ========================*/

/**
 * @brief MPU9250 原始数据结构体
 * @note 存储从寄存器直接读取的16位原始值
 */
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

/**
 * @brief MPU9250 物理量数据结构体
 * @note 经过量程转换后的实际物理量
 */
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

/**
 * @brief AK8963 磁力计原始数据结构体
 */
typedef struct
{
    int16_t mag_x;   // X 轴原始值
    int16_t mag_y;   // Y 轴原始值
    int16_t mag_z;   // Z 轴原始值
} AK8963_raw_Data;

/**
 * @brief AK8963 磁力计物理量数据结构体
 */
typedef struct
{
    float mag_x_ut;  // X 轴磁感应强度（单位：μT）
    float mag_y_ut;  // Y 轴磁感应强度（单位：μT）
    float mag_z_ut;  // Z 轴磁感应强度（单位：μT）
} AK8963_Physical_Data;

/*======================== 姿态表示结构体 ========================*/

/**
 * @brief 欧拉角结构体（单位：弧度）
 * 
 * 坐标系定义（NED - 北东地坐标系）：
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
 * @brief 四元数结构体（Mahony 内部状态）
 */
typedef struct
{
    float q0;  // w (实部)
    float q1;  // x (虚部i)
    float q2;  // y (虚部j)
    float q3;  // z (虚部k)
} Quaternion_t;

/*======================== 全局变量声明 ========================*/

// 校准参数
extern float g_gyro_bias_dps[3];  // 陀螺仪偏置（单位：°/s）
extern float g_accel_bias_g[3];   // 加速度计零偏（单位：g）
extern float g_mag_offset[3];     // 磁力计硬铁偏置（单位：μT）
extern float g_mag_scale[3];      // 磁力计软铁缩放（单位：比例因子）

// 姿态角输出
extern EulerAngle_t g_euler_acc_mag;  // 由加速度计+磁力计解算的姿态角（单位：弧度）
extern EulerAngle_t g_euler_fused;    // Mahony融合后的姿态角（单位：弧度）

// 经纬度（GCJ-02）
extern double lat_gcj, lon_gcj;  
/*======================== 基础功能函数 ========================*/

/**
 * @brief 初始化MPU9250九轴传感器
 * @return 0: 成功, 其他值: 错误码
 */
int MPU9250_9Axis_Init(void);

/**
 * @brief 读取MPU9250九轴传感器的完整数据
 * @param mpu_raw  MPU9250原始数据输出
 * @param mpu_phys MPU9250物理量数据输出
 * @param ak_raw   AK8963原始数据输出
 * @param ak_phys  AK8963物理量数据输出
 * @return 0: 成功, 其他值: 错误码
 */
int MPU9250_Read_9Axis(MPU9250_raw_Data *mpu_raw, 
                       MPU9250_Physical_Data *mpu_phys,
                       AK8963_raw_Data *ak_raw,
                       AK8963_Physical_Data *ak_phys);

/*======================== 校准功能函数 ========================*/

/**
 * @brief 校准陀螺仪零偏（需要保持静止）
 * @param samples   采样次数
 * @param delay_ms  每次采样间隔（ms）
 */
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms);

/**
 * @brief 校准加速度计零偏（需要保持静止）
 * @param samples   采样次数
 * @param delay_ms  每次采样间隔（ms）
 */
void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms);

/**
 * @brief 磁力计硬铁+软铁校准（需要旋转传感器）
 * @param samples   采样次数
 * @param delay_ms  每次采样间隔（ms）
 * @note 校准过程中需要将传感器在各个方向旋转，远离电子设备
 */
void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms);

/**
 * @brief 一键校准所有传感器（陀螺仪+加速度计+磁力计）
 * @note 该函数会调用上述三个校准函数，使用默认参数
 */
void MPU9250_CalibrateAll(void);

/*======================== 姿态解算功能 ========================*/

/**
 * @brief 使用加速度计+磁力计进行姿态解算（A+M → Roll/Pitch/Yaw）
 * @param imu  已转换为物理量的六轴数据（单位：g / °/s）
 * @param mag  已做硬铁/软铁校准后的磁力计物理量（单位：μT）
 * @note  输入的 imu/mag 为"模块坐标系"的物理量，函数内部会自动转换到标准机体系（X前、Y右、Z下）
 */
void MPU9250_ComputeEuler_FromAccMag(const MPU9250_Physical_Data *imu,
                                     const AK8963_Physical_Data *mag);

/**
 * @brief 获取当前A+M解算的姿态角（单位：度）
 * @param roll_deg  输出 roll 角（度）
 * @param pitch_deg 输出 pitch 角（度）
 * @param yaw_deg   输出 yaw 角（度）
 */
void MPU9250_GetEulerDeg(float *roll_deg, float *pitch_deg, float *yaw_deg);

/*======================== Mahony姿态融合功能 ========================*/

/**
 * @brief Mahony算法参数初始化
 * @param kp 比例项，越大越信任A/M（收敛快但可能更抖）
 * @param ki 积分项，消除长时间漂移（一般先设0）
 */
void MPU9250_MahonyInit(float kp, float ki);

/**
 * @brief Mahony九轴融合更新（每帧调用）
 * @param imu 六轴物理量（g / dps）
 * @param mag 磁力计物理量（uT，已校准）
 * @param dt  采样周期（秒）
 */
void MPU9250_MahonyUpdate(const MPU9250_Physical_Data *imu,
                          const AK8963_Physical_Data *mag,
                          float dt);

/**
 * @brief Mahony六轴融合更新（无磁力计时使用）
 * @param imu 六轴物理量（g / dps）
 * @param dt  采样周期（秒）
 */
void MPU9250_MahonyUpdateIMU(const MPU9250_Physical_Data *imu, float dt);

/**
 * @brief 获取Mahony融合后的姿态角（单位：度）
 * @param roll_deg  输出 roll 角（度）
 * @param pitch_deg 输出 pitch 角（度）
 * @param yaw_deg   输出 yaw 角（度）
 */
void MPU9250_GetEulerFusedDeg(float *roll_deg, float *pitch_deg, float *yaw_deg);

#endif /* __MPU9250_H__ */