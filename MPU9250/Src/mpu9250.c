#include "mpu9250.h"
#include "i2c.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal.h"


/*======================== 全局变量定义 ========================*/

// ========== AK8963磁力计相关参数 ==========
uint8_t g_ak8963_asa[3];                                    // 存放原始ASA值
float   g_ak8963_sensitivity[3];                            // 存放换算后的灵敏度调整系数

// ========== 传感器校准参数 ==========
float   g_gyro_bias_dps[3]  = {0.0f, 0.0f, 0.0f};         // 陀螺仪零偏（单位：°/s）
float   g_accel_bias_g[3]   = {0.0f, 0.0f, 0.0f};         // 加速度计零偏（单位：g）
float   g_mag_offset[3]     = {0.0f, 0.0f, 0.0f};         // 磁力计硬铁偏置（单位：μT）
float   g_mag_scale[3]      = {1.0f, 1.0f, 1.0f};         // 磁力计软铁缩放（单位：比例因子）

// ========== 姿态解算输出 ==========
EulerAngle_t g_euler_acc_mag = {0.0f, 0.0f, 0.0f};        // 由加速度计+磁力计解算的姿态角（单位：弧度）

/*========================================================================初始化MPU9250======================================================================================*/

/*-----------------------------------六轴的初始化所需函数---------------------------------------*/

/**
 * @brief 1.软复位
 * @param None
 * @return None
 */
void MPU9250_SoftReset(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x6BU); //读取寄存器0x6B的值

    val &= ~(1 << 7); //清除bit7（DEVICE_RESET位）
    
    val |= (1 << 7); //设置bit7（DEVICE_RESET位）为1，触发软复位
    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x6BU, val); //向PWR_MGMT_1寄存器写入新的值，触发软复位
    HAL_Delay(100); //等待100ms，确保复位完成
}


/**
 * @brief 2.将唤醒以及设置时钟源
 * @param None
 * @return None
 */
void mpu_set_clock_to_auto(void)
{
    /*1.读取当前寄存器的值*/
    uint8_t val =  I2C_ReadReg(MPU9250_I2C_ADDR7, 0x6BU);
    
    /*2.确保SLEEP = 0*/
    val &= ~(1 << 6); //bit6 SLEEP 清零 → 保证不睡眠

    /*选择 X 轴陀螺仪 PLL 作为时钟源 */
    val = (val & ~(0x7)) | 0x1; // 清除 CLKSEL 位，然后设置为 001

    /*4.写回寄存器0x6B */
    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x6BU, val);
}

/**
 * @brief 3.使能六轴传感器
 * @param None
 * @return None
 */
void mpu_enable_six_axis(void)
{
    /* 1.读取当前寄存器PWR_MGMT_2的值*/
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x6CU); //读取寄存器0x6C的值

    /* 2.清除bit5~0（六个DIS位） -> 全轴启用*/
    val &= ~(0x3FU); //清除bit5~0（六个DIS位）

    /* 3.写回寄存器0x6C */
    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x6CU, val);
}

/**
 * @brief 4.配置陀螺仪的DLPF
 * @param None
 * @return None
 */
void mpu_set_dlpf_cfg_3(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x1AU); //读取寄存器0x1A的值

    val = (val & ~(0x07)) | 0x03; // 清除 DLPF_CFG 位，然后设置为 011（配置DLPF为3）

    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x1AU, val);
}

/**
 * @brief 5.配置陀螺仪以及加速度计的采样率
 * @param None
 * @return None
 */
void mpu_set_sample_rate_200hz(void)
{
    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x19U, 0x04U); //设置采样率寄存器SMPLRT_DIV为4，采样率=1000/(1+4)=200Hz
}

/**
 * @brief 6.配置加速度计的DLPF
 * @param None
 * @return None
 */
void mpu_set_accel_dlpf(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x1DU);  // ACCEL_CONFIG2

    val &= ~0x07;    // 清除 A_DLPFCFG[2:0]
    val |= 0x03;     // 设置 A_DLPFCFG = 3 → 加速度计带宽 41Hz

    val &= ~(1 << 3); // accel_fchoice_b = 0（启用滤波器）
                      // 一般默认就是 0，但写上更保险

    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x1DU, val);
}

/**
 * @brief 7.配置陀螺仪的量程
 * @param None
 * @return None
 */
void mpu_set_gyro_config(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x1BU); //读取寄存器0x1B的值
    val &= ~0x03;        // Fchoice_b = 00（bit1:0 清 0）
    val &= ~(3 << 3);    // 清量程选择位
    val |=  (2 << 3);    // GYRO_FS_SEL = 10 → ±1000 dps
    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x1BU, val);
}

/**
 * @brief 8.配置加速度计的量程
 * @param None
 * @return None
 */
void mpu_set_accel_range(void)
{
    uint8_t val =  I2C_ReadReg(MPU9250_I2C_ADDR7, 0X1CU);
     
    val &= ~(3 << 3);    // 清量程选择位
    val |=  (2 << 3);    // ACCEL_FS_SEL = 10 → ±8g
    I2C_WriteReg(MPU9250_I2C_ADDR7, 0X1CU, val);
}

/**
 * @brief 初始化 MPU9250 六轴传感器（加速度计 + 陀螺仪）
 * @return  0：成功
 *         -1：初始化失败
 */
int MPU9250_6Axis_Init(void)
{
    MPU9250_SoftReset();             // 1. 软复位
    HAL_Delay(100);
    
    mpu_set_clock_to_auto();         // 2. 配置时钟源
    mpu_enable_six_axis();           // 3. 使能六轴传感器
    mpu_set_dlpf_cfg_3();           // 4. 配置陀螺仪DLPF
    mpu_set_sample_rate_200hz();     // 5. 设置采样率
    mpu_set_gyro_config();          // 6. 配置陀螺仪量程
    mpu_set_accel_range();          // 7. 配置加速度计量程
    mpu_set_accel_dlpf();           // 8. 配置加速度计DLPF
    
    return 0;  // 初始化成功
}

/*-----------------------------------磁力计的初始化所需函数---------------------------------------*/

/**
 * @brief 1.配置AK8963为MCU可直接访问的方式
 * @param None
 * @return None
 */
void mpu_set_ak8963_by_mcu(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0X6AU); //读取寄存器0x6A的值

    val &= ~(1 << 5);
    I2C_WriteReg(MPU9250_I2C_ADDR7, 0X6AU, val); //向寄存器0x6A写入新的值

    val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0X37U); //读取寄存器0x37的值

    val |= (1 << 1);
    I2C_WriteReg(MPU9250_I2C_ADDR7, 0X37U, val); //向寄存器0x37写入新的值
}

/**
 * @brief 2.检查AK8963设备ID是否为0X48
 * @param None
 * @return 0 ：设备ID正确，非0：设备ID错误
 */
int AK8963_CheckDeviceID(void)
{
    uint8_t device_id = I2C_ReadReg(AK8963_I2C_ADDR7, 0x00U); //读取AK8963的WIA寄存器

    if(device_id == 0x48U)
        return 0; //设备ID正确
    else
        return -1; //设备ID错误
}

/**
 * @brief 3.使AK8963进入Power-down mode
 * @param None
 * @return None
 */
void AK8963_EnterPowerDownMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, 0x0AU, 0x00U);

    HAL_Delay(10); //等待10ms，确保进入模式完成
}

/**
 * @brief 4.进入Fuse ROM mode
 * @param None
 * @return None
 */
void AK8963_EnterFuseROMMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, 0x0AU, 0x0FU);

    HAL_Delay(10); //等待10ms，确保进入模式完成
}

/**
 * @brief 5.进入连续测量模式
 * @param None
 * @return None
 */
void AK8963_EnterContinuousMeasurementMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, 0x0AU, 0x16U);
    HAL_Delay(10); //等待10ms，确保进入模式完成
}

/**
 * @brief 6.调整灵敏度补偿系数（ASA）
 * @param None
 * @return None
 */
void AK8963_AdjustSensitivity(void)
{
    //1.读取Fuse ROM模式下的ASAX、ASAY、ASAZ的原始值
    g_ak8963_asa[0] = I2C_ReadReg(AK8963_I2C_ADDR7, 0x10U); //ASAX
    g_ak8963_asa[1] = I2C_ReadReg(AK8963_I2C_ADDR7, 0x11U); //ASAY
    g_ak8963_asa[2] = I2C_ReadReg(AK8963_I2C_ADDR7, 0x12U); //ASAZ

    //2.根据原始值计算灵敏度调整系数
    g_ak8963_sensitivity[0] = (float)((g_ak8963_asa[0] - 128) * 0.5f) / 128 + 1.0f;
    g_ak8963_sensitivity[1] = (float)((g_ak8963_asa[1] - 128) * 0.5f) / 128 + 1.0f;
    g_ak8963_sensitivity[2] = (float)((g_ak8963_asa[2] - 128) * 0.5f) / 128 + 1.0f;
}




/**
 * @brief 初始化 AK8963 磁力计
 * @param None
 * @return None
 * @note 初始化流程：
 *       1. 配置 MPU9250，使 AK8963 可被 MCU 直接访问（旁路模式）
 *       2. 检查 AK8963 设备 ID（应为 0x48）
 *       3. 进入 Power-down 模式（必须先关闭才能切换模式）
 *       4. 进入 Fuse ROM 模式（读取工厂校准数据）
 *       5. 读取灵敏度调整值（ASA：Sensitivity Adjustment）
 *          - ASAX、ASAY、ASAZ 寄存器（0x10~0x12）
 *          - 计算灵敏度补偿系数：Hadj = H × ((ASA-128)×0.5/128 + 1)
 *       6. 再次进入 Power-down 模式（退出 Fuse ROM 模式）
 *       7. 进入连续测量模式 2（16-bit 输出，100Hz 采样率）
 *          - 模式设置：CNTL1 = 0x16 (0001 0110)
 *          - bit[4]: 16-bit 输出
 *          - bit[3:0]: 0110 = 连续测量模式 2
 */
int AK8963_Init(void)
{
    mpu_set_ak8963_by_mcu(); // 配置 AK8963 为 MCU 可直接访问的方式

    if (AK8963_CheckDeviceID() != 0) // 检查 AK8963 设备 ID 是否正确
    {
        // 设备 ID 错误，处理错误
        return -1;
    }

    AK8963_EnterPowerDownMode(); // 进入 Power-down 模式

    AK8963_EnterFuseROMMode(); // 进入 Fuse ROM 模式

    AK8963_AdjustSensitivity(); // 调整灵敏度补偿系数（ASA）

    AK8963_EnterPowerDownMode(); // 进入 Power-down 模式

    AK8963_EnterContinuousMeasurementMode(); // 进入连续测量模式

    return 0; // 初始化成功

}

/**
 * @brief 初始化 MPU9250 九轴传感器（六轴 + 磁力计）
 * @return  0：成功
 *         -1：AK8963 设备 ID 错误
 *         -2：I2C 通信失败
 */
int MPU9250_9Axis_Init(void)
{
    // ========== 步骤 1：初始化 I2C GPIO ==========
    I2C_GPIO_Init();
    HAL_Delay(100);  // 等待 MPU9250 上电稳定
    
    // ========== 步骤 2：初始化六轴传感器 ==========
    MPU9250_6Axis_Init();
    
    // ========== 步骤 3：初始化磁力计（AK8963）==========
    AK8963_Init();
    // 注意：校准应该在 main.c 中手动调用，不在初始化函数内部自动执行
    // MPU9250_CalibrateGyro(500, 10); // 已移到 main.c
    // MPU9250_CalibrateAccel(300, 5); // 已移到 main.c
    
    return 0;  // 初始化成功
}

/*========================================================================读取MPU9250数据======================================================================================*/

/**
 * @brief 读取 MPU9250 的 16 位寄存器值
 * @param reg 寄存器地址
 * @return 读取到的 16 位有符号整数
 */
static int16_t mpu_read_word(uint8_t reg)
{
    uint8_t high_byte = I2C_ReadReg(MPU9250_I2C_ADDR7, reg);
    uint8_t low_byte = I2C_ReadReg(MPU9250_I2C_ADDR7, reg + 1);
    return (int16_t)((high_byte << 8) | low_byte);
}
/**
 * @brief 读取 MPU9250 六轴传感器的原始数据
 * @param None
 * @return None
 */
void MPU9250_Read_six_Axis(MPU9250_raw_Data * raw)
{
    //------------加速度--------------
    raw->accel_x = mpu_read_word(0x3B);
    raw->accel_y = mpu_read_word(0x3D);
    raw->accel_z = mpu_read_word(0x3F);
    //------------陀螺仪----------------
    raw->gyro_x = mpu_read_word(0x43);
    raw->gyro_y = mpu_read_word(0x45);
    raw->gyro_z = mpu_read_word(0x47);

    //------------温度----------------
    raw->temp   = mpu_read_word(0x41);
}

/**
 * @brief 将读取的MPU9250原始数据转换成物理量
 * @param raw 指向原始数据结构体的指针
 * @param None
 */
void MPU9250_ConvertToPhysical(const MPU9250_raw_Data * raw, MPU9250_Physical_Data * physical)
{
    // 加速度转换：单位 g
    physical->accel_x_g = (float)raw->accel_x / 4096.0f - g_accel_bias_g[0]; // ±8g 对应 4096 LSB/g
    physical->accel_y_g = (float)raw->accel_y / 4096.0f - g_accel_bias_g[1];
    physical->accel_z_g = (float)raw->accel_z / 4096.0f - g_accel_bias_g[2];

    // 陀螺仪转换：单位 °/s
    physical->gyro_x_dps = (float)raw->gyro_x / 32.8f - g_gyro_bias_dps[0]; // ±1000°/s 对应 32.8 LSB/(°/s)
    physical->gyro_y_dps = (float)raw->gyro_y / 32.8f - g_gyro_bias_dps[1];
    physical->gyro_z_dps = (float)raw->gyro_z / 32.8f - g_gyro_bias_dps[2];

    // 温度转换：单位 ℃
    physical->temp_c = ((float)raw->temp / 333.87f) + 21.0f; // 温度公式
}

/**
 * @brief 检查DRDY是否准备好数据
 * @param None
 * @return 1：数据准备好  0：数据未准备好
 */
int AK8963_CheckDataReady(void)
{
    uint8_t status = I2C_ReadReg(AK8963_I2C_ADDR7, 0x02U); //读取AK8963的ST1寄存器

    if(status == 0xFFU)
    {
        // I2C读失败，建议返回 -1 向上层报告,避免 I2C 出错时误判为“未准备好”(W2C底层代码中，读失败会返回 0xFF)
        return -1;
    }

    if(status & 0x01U)
        return 1; //数据准备好
    else
        return 0; //数据未准备好
}

/**
 * @brief 读取AK8963的16位寄存器值
 * @param reg 寄存器地址
 * @return 读取到的16位有符号整数
 */
static int16_t ak8963_read_word(uint8_t reg)
{
    uint8_t low  = I2C_ReadReg(AK8963_I2C_ADDR7, reg);     // HXL / HYL / HZL
    uint8_t high = I2C_ReadReg(AK8963_I2C_ADDR7, reg + 1); // HXH / HYH / HZH
    return (int16_t)((high << 8) | low); // Little-endian
}

/**
 * @brief  读取 AK8963 磁力计的原始数据（完整一次：DRDY 检查 + XYZ + ST2）
 * @param  raw 指向 AK8963_raw_Data 结构体的指针
 * @return  0：成功
 *         -1：数据未准备好（DRDY=0）
 *         -2：I2C 读失败
 *         -3：ST2.HOFL=1，数据溢出
 */
int AK8963_Read_Axis(AK8963_raw_Data *raw)
{
    int drdy = AK8963_CheckDataReady();   // 只调用一次

    if (drdy == 1)
    {
        // 1. 读取 X/Y/Z（三轴原始数据）
        raw->mag_x = ak8963_read_word(0x03U);   // HXL/HXH
        raw->mag_y = ak8963_read_word(0x05U);   // HYL/HYH
        raw->mag_z = ak8963_read_word(0x07U);   // HZL/HZH

        // 2. 读取 ST2 寄存器，清除 DRDY / DOR / HOFL 等标志
        uint8_t st2 = I2C_ReadReg(AK8963_I2C_ADDR7, 0x09U);
        if (st2 == 0xFFU)
        {
            // I2C 读失败
            return -2;
        }

        // 3. 检查 HOFL（bit3），是否发生溢出
        if (st2 & 0x08U)
        {
            // 数据溢出，本次数据无效
            return -3;
        }

        return 0;   // 一切正常，返回成功
    }
    else if (drdy == 0)
    {
        // 数据未准备好（DRDY=0）
        return -1;
    }
    else
    {
        // AK8963_CheckDataReady 内部 I2C 读 ST1 失败
        return -2;
    }
}


/**
 * @brief 对磁力计数据补偿计算且转换为物理量（单位：微特斯拉 uT）
 * @param raw 指向 AK8963_raw_Data 结构体的指针
 * @param physical 指向 AK8963_Physical_Data 结构体的指针
 */
void AK8963_Calibrate(const AK8963_raw_Data *raw, AK8963_Physical_Data *physical)
{
    // 1) 原始 μT
    float mx = (float)raw->mag_x * g_ak8963_sensitivity[0] * 0.15f;
    float my = (float)raw->mag_y * g_ak8963_sensitivity[1] * 0.15f;
    float mz = (float)raw->mag_z * g_ak8963_sensitivity[2] * 0.15f;

    // 2) 硬铁 offset
    mx -= g_mag_offset[0];
    my -= g_mag_offset[1];
    mz -= g_mag_offset[2];

    // 3) 软铁 scale
    mx *= g_mag_scale[0];
    my *= g_mag_scale[1];
    mz *= g_mag_scale[2];

    // 最终输出
    physical->mag_x_ut = mx;
    physical->mag_y_ut = my;
    physical->mag_z_ut = mz;
}

/**
 * @brief 读取 MPU9250 九轴传感器的完整数据（加速度 + 陀螺仪 + 温度 + 磁力计）
 * @param mpu_raw 指向 MPU9250 原始数据结构体的指针
 * @param mpu_phys 指向 MPU9250 物理量数据结构体的指针
 * @param ak_raw 指向 AK8963 原始数据结构体的指针
 * @param ak_phys 指向 AK8963 物理量数据结构体的指针
 * @return  0：成功读取所有数据
 *         -1：磁力计数据未准备好（DRDY=0）
 *         -2：磁力计 I2C 读取失败
 *         -3：磁力计数据溢出（HOFL=1）
 * 
 * @note 使用示例：
 *       MPU9250_raw_Data mpu_raw;
 *       MPU9250_Physical_Data mpu_phys;
 *       AK8963_raw_Data ak_raw;
 *       AK8963_Physical_Data ak_phys;
 *       
 *       int ret = MPU9250_Read_9Axis(&mpu_raw, &mpu_phys, &ak_raw, &ak_phys);
 *       if (ret == 0) {
 *           // 数据读取成功，使用 mpu_phys 和 ak_phys 中的物理量
 *           printf("Accel: %.2f, %.2f, %.2f g\n", 
 *                  mpu_phys.accel_x_g, mpu_phys.accel_y_g, mpu_phys.accel_z_g);
 *           printf("Mag: %.1f, %.1f, %.1f µT\n",
 *                  ak_phys.mag_x_ut, ak_phys.mag_y_ut, ak_phys.mag_z_ut);
 *       }
 */
int MPU9250_Read_9Axis(MPU9250_raw_Data *mpu_raw, 
                       MPU9250_Physical_Data *mpu_phys,
                       AK8963_raw_Data *ak_raw,
                       AK8963_Physical_Data *ak_phys)
{
    // ========== 1. 读取六轴数据（加速度 + 陀螺仪 + 温度）==========
    MPU9250_Read_six_Axis(mpu_raw);
    
    // ========== 2. 转换为物理量 ==========
    MPU9250_ConvertToPhysical(mpu_raw, mpu_phys);
    
    // ========== 3. 读取磁力计原始数据 ==========
    int ret = AK8963_Read_Axis(ak_raw);
    if (ret != 0)
    {
        // 磁力计读取失败，返回错误码
        // ret = -1: 数据未准备好
        // ret = -2: I2C 读取失败
        // ret = -3: 数据溢出
        return ret;
    }
    
    // ========== 4. 磁力计数据补偿并转换为物理量 ==========
    AK8963_Calibrate(ak_raw, ak_phys);
    
    return 0;  // 成功
}

/*=========================================================================校准================================================================================*/

/**
 * @brief 设置陀螺仪零偏
 * @param samples 校准采样次数
 * @return delay_ms 每次采样间隔时间（单位：毫秒）
 * @return None
 * @note 调用该函数时，MPU9250必须保持静止
 */
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms)
{
    MPU9250_raw_Data raw;
    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;

    for(uint16_t i = 0; i < samples; i++)
    {
        MPU9250_Read_six_Axis(&raw); //读取六轴原始数据

        gyro_x_sum += raw.gyro_x;
        gyro_y_sum += raw.gyro_y;
        gyro_z_sum += raw.gyro_z;

        HAL_Delay(delay_ms); //等待指定时间
    }

    //求平均值
    float gyro_x_avg = (float)gyro_x_sum / (float)samples;
    float gyro_y_avg = (float)gyro_y_sum / (float)samples;
    float gyro_z_avg = (float)gyro_z_sum / (float)samples;

    //因为设置的量程是 ±1000°/s，对应 32.8 LSB/(°/s)
    const float lsb_per_dps = 32.8f;

    g_gyro_bias_dps[0] = gyro_x_avg / lsb_per_dps; //计算陀螺仪X轴零偏（单位：°/s）
    g_gyro_bias_dps[1] = gyro_y_avg / lsb_per_dps; //计算陀螺仪Y轴零偏（单位：°/s）
    g_gyro_bias_dps[2] = gyro_z_avg / lsb_per_dps; //计算陀螺仪Z轴零偏（单位：°/s）

}

/**
 * @brief  简单加速度计偏置校准（静止水平放置）
 * @param  samples   采样次数（建议 200~500）
 * @param  delay_ms  采样间隔（ms）
 * @note   调用该函数时，MPU9250 必须保持水平静止，Z 轴朝上
 */
void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms)
{
    MPU9250_raw_Data raw;
    float ax_sum = 0.0f;
    float ay_sum = 0.0f;
    float az_sum = 0.0f;

    for (uint16_t i = 0; i < samples; i++)
    {
        // 读取六轴原始数据
        MPU9250_Read_six_Axis(&raw);

        // 你的量程是 ±8g → 4096 LSB/g
        ax_sum += (float)raw.accel_x / 4096.0f;
        ay_sum += (float)raw.accel_y / 4096.0f;
        az_sum += (float)raw.accel_z / 4096.0f;

        HAL_Delay(delay_ms);
    }

    // 求平均值（单位：g）
    float ax_avg = ax_sum / (float)samples;
    float ay_avg = ay_sum / (float)samples;
    float az_avg = az_sum / (float)samples;

    // 理论值为 (0, 0, +1g)，求出偏置值
    g_accel_bias_g[0] = ax_avg - 0.0f;  // X 轴偏置
    g_accel_bias_g[1] = ay_avg - 0.0f;  // Y 轴偏置
    g_accel_bias_g[2] = az_avg - 1.0f;  // Z 轴相对 1g 的偏置
}

/**
 * @brief 读取一帧磁力计数据，并输出为 μT（已做 ASA 补偿）
 * @param mag_out 指向 AK8963_Physical_Data 的指针
 * @return 0 成功，其他失败
 */
int AK8963_Read_Mag_UT(AK8963_Physical_Data *mag_out)
{
    AK8963_raw_Data raw;

    int ret = AK8963_Read_Axis(&raw);
    if (ret != 0)
        return ret;

    // 调用已有的补偿转换函数
    AK8963_Calibrate(&raw, mag_out);

    return 0;
}

/**
 * @brief  磁力计硬铁 + 软铁校准
 * @note   采集 samples 次数据，期间需要手动旋转模块（尽量覆盖所有方向）
 */
void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms)
{
    AK8963_Physical_Data mag;

    float min_x = 99999, min_y = 99999, min_z = 99999;
    float max_x = -99999, max_y = -99999, max_z = -99999;

    for (uint16_t i = 0; i < samples; i++)
    {
        // 读取补偿后的 μT 单位磁力计数据
        AK8963_Read_Mag_UT(&mag);

        if (mag.mag_x_ut < min_x) min_x = mag.mag_x_ut;
        if (mag.mag_x_ut > max_x) max_x = mag.mag_x_ut;

        if (mag.mag_y_ut < min_y) min_y = mag.mag_y_ut;
        if (mag.mag_y_ut > max_y) max_y = mag.mag_y_ut;

        if (mag.mag_z_ut < min_z) min_z = mag.mag_z_ut;
        if (mag.mag_z_ut > max_z) max_z = mag.mag_z_ut;

        HAL_Delay(delay_ms);
    }

    // ----------- 硬铁校准（偏置 Offset）-----------
    g_mag_offset[0] = (min_x + max_x) / 2.0f;
    g_mag_offset[1] = (min_y + max_y) / 2.0f;
    g_mag_offset[2] = (min_z + max_z) / 2.0f;

    // 半径
    float radius_x = (max_x - min_x) / 2.0f;
    float radius_y = (max_y - min_y) / 2.0f;
    float radius_z = (max_z - min_z) / 2.0f;

    // 平均半径
    float radius_avg = (radius_x + radius_y + radius_z) / 3.0f;

    // ----------- 软铁校准（缩放 Scale）-----------
    g_mag_scale[0] = radius_avg / radius_x;
    g_mag_scale[1] = radius_avg / radius_y;
    g_mag_scale[2] = radius_avg / radius_z;
}

/*=================================================================================姿态解算====================================================================================================*/
/**
 * @brief 使用加速度计 + 磁力计进行姿态解算
 *
 *  输入：
 *    - imu：MPU9250_ConvertToPhysical() 输出的加速度物理量（单位：g）
 *    - mag：AK8963_Calibrate()   输出的磁力计物理量（单位：μT）
 *
 *  约定：
 *    - 当前 imu/mag 仍然是“模块坐标系”的数据（即板子丝印方向）：
 *        模块 +X：朝机头
 *        模块 +Y：朝机体左侧
 *        模块 +Z：朝上
 *
 *    - 本函数内部会将其转换到“标准机体系”：
 *        机体系 X：前
 *        机体系 Y：右
 *        机体系 Z：下   （NED）
 *
 *  输出：
 *    - 更新全局变量 g_euler_acc_mag（单位：弧度）
 *      其中：
 *        roll  —— 横滚角（绕 X 轴）
 *        pitch —— 俯仰角（绕 Y 轴）
 *        yaw   —— 航向角（绕 Z 轴，指北）
 */
void MPU9250_ComputeEuler_FromAccMag(const MPU9250_Physical_Data *imu,
                                     const AK8963_Physical_Data *mag)
{
    /*=================== 1. 模块坐标系 → 机体系（X 前, Y 右, Z 下） ===================*/
    // 根据之前分析：模块 +X=机体 +X，模块 +Y=机体左侧，模块 +Z=机体上方
    // 故坐标变换为：
    //   X_body =  X_module
    //   Y_body = -Y_module
    //   Z_body = -Z_module

    float ax =  imu->accel_x_g;
    float ay = -imu->accel_y_g;
    float az =  imu->accel_z_g;

    float mx =  mag->mag_x_ut;
    float my = -mag->mag_y_ut;
    float mz = -mag->mag_z_ut;

    /*=================== 2. 归一化加速度向量 ===================*/
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f)
    {
        // 模长过小，说明当前数据异常（例如自由落体），本次不更新姿态
        return;
    }
    ax /= norm;
    ay /= norm;
    az /= norm;

    /*=================== 3. 用加速度计算 Roll / Pitch ===================*/
    // 公式：
    //   roll  = atan2( Ay, Az )
    //   pitch = atan2( -Ax, sqrt(Ay^2 + Az^2) )
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    /*=================== 4. 归一化磁力计向量 ===================*/
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm < 1e-6f)
    {
        // 磁场模长异常（可能环境磁干扰），只更新 roll/pitch，不更新 yaw
        g_euler_acc_mag.roll  = roll;
        g_euler_acc_mag.pitch = pitch;
        // 保持原 yaw 不变
        return;
    }
    mx /= norm;
    my /= norm;
    mz /= norm;

    /*=================== 5. 对磁力计做倾斜补偿，得到水平面分量 ===================*/
    // 根据当前的 roll / pitch，将地磁向量“旋转回水平状态”
    float sinRoll  = sinf(roll);
    float cosRoll  = cosf(roll);
    float sinPitch = sinf(pitch);
    float cosPitch = cosf(pitch);

    // 先补偿 pitch，再补偿 roll，求出水平面的磁场分量 mx2 / my2
    float mx2 = mx * cosPitch + mz * sinPitch;
    float my2 = mx * sinRoll * sinPitch
              + my * cosRoll
              - mz * sinRoll * cosPitch;

    /*=================== 6. 根据水平面磁场方向计算 Yaw ===================*/
    // NED 坐标系常用写法：yaw = atan2(-My, Mx)
    float yaw = atan2f(-my2, mx2);

    /*=================== 7. 更新全局姿态角（单位：弧度） ===================*/
    g_euler_acc_mag.roll  = roll;
    g_euler_acc_mag.pitch = pitch;
    g_euler_acc_mag.yaw   = yaw;
}

/**
 * @brief 获取当前姿态角（单位：度）
 *
 * @param roll_deg  输出横滚角（deg）
 * @param pitch_deg 输出俯仰角（deg）
 * @param yaw_deg   输出航向角（deg）
 *
 * @note  方便上位机/串口打印使用，内部直接从 g_euler_acc_mag 转换。
 */
void MPU9250_GetEulerDeg(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float rad2deg = 57.295779513f; // 180 / pi

    if (roll_deg)
        *roll_deg  = g_euler_acc_mag.roll  * rad2deg;

    if (pitch_deg)
        *pitch_deg = g_euler_acc_mag.pitch * rad2deg;

    if (yaw_deg)
        *yaw_deg   = g_euler_acc_mag.yaw   * rad2deg;
}

/*=================================================================================姿态融合====================================================================================================*/

// 融合后的欧拉角（rad）
EulerAngle_t g_euler_fused = {0.0f, 0.0f, 0.0f};

// 当前四元数状态（初始为单位四元数）
static Quaternion_t g_q = {1.0f, 0.0f, 0.0f, 0.0f};

// 误差积分项（积分反馈用）
static float g_exInt = 0.0f, g_eyInt = 0.0f, g_ezInt = 0.0f;

// Mahony 参数
static float g_kp = 2.0f;
static float g_ki = 0.0f;

// 静态零偏抑制参数
static float g_acc_norm_prev = 1.0f;  // 上一次加速度模长

void MPU9250_MahonyInit(float kp, float ki)
{
    g_kp = kp;
    g_ki = ki;

    // 重置四元数与积分项
    g_q.q0 = 1.0f; g_q.q1 = g_q.q2 = g_q.q3 = 0.0f;
    g_exInt = g_eyInt = g_ezInt = 0.0f;

    g_euler_fused.roll = g_euler_fused.pitch = g_euler_fused.yaw = 0.0f;
    g_acc_norm_prev = 1.0f;
}

/**
 * @brief Mahony 融合更新
 */
void MPU9250_MahonyUpdate(const MPU9250_Physical_Data *imu,
                          const AK8963_Physical_Data *mag,
                          float dt)
{
    /*---------- 1) 统一到机体系（X前 Y右 Z下） ----------*/
    // 按你已验证通过的转换（与 A+M 解算保持一致）
    float ax =  imu->accel_x_g;
    float ay = -imu->accel_y_g;
    float az =  imu->accel_z_g;

    // 陀螺同轴同方向：先按同样规则转换
    const float deg2rad = 0.01745329252f;
    float gx =  imu->gyro_x_dps * deg2rad;
    float gy = -imu->gyro_y_dps * deg2rad;
    float gz =  imu->gyro_z_dps * deg2rad;

    // 磁力计保持与你 yaw 解算一致的符号（Y/Z 取反）
    float mx =  mag->mag_x_ut;
    float my = -mag->mag_y_ut;
    float mz = -mag->mag_z_ut;

    /*---------- 2) 归一化 accel / mag（保留原始模长用于门控）---------- */
    float norm_acc = sqrtf(ax*ax + ay*ay + az*az);
    if (norm_acc < 1e-6f) return;  // 加速度异常（自由落体等）
    
    float norm_mag = sqrtf(mx*mx + my*my + mz*mz);
    if (norm_mag < 1e-6f) return;  // 磁场异常
    
    /*---------- 2.1) 动态自适应 Kp：根据加速度状态调整 ----------*/
    float acc_diff = fabsf(norm_acc - g_acc_norm_prev);
    float acc_error = fabsf(norm_acc - 1.0f);
    
    float kp_dynamic = g_kp;  // 默认使用基础 Kp
    
    // 三级自适应策略
    if (acc_error < 0.05f && acc_diff < 0.02f) {
        // 完全静止：强力修正（地面待机）
        kp_dynamic = g_kp * 10.0f;
    } 
    else if (acc_error < 0.15f && acc_diff < 0.1f) {
        // 小幅运动：中等修正（悬停/慢速飞行）
        kp_dynamic = g_kp * 3.0f;
    }
    else if (acc_error > 0.3f || acc_diff > 0.3f) {
        // 剧烈加速：降低加速度计权重（避免误修正）
        kp_dynamic = g_kp * 0.5f;
    }
    // 其他情况保持基础 Kp（正常飞行）
    
    g_acc_norm_prev = norm_acc;  // 更新上次模长
    
    // 归一化
    ax/=norm_acc; ay/=norm_acc; az/=norm_acc;
    mx/=norm_mag; my/=norm_mag; mz/=norm_mag;

    /*---------- 3) 取当前四元数 ----------*/
    float q0=g_q.q0, q1=g_q.q1, q2=g_q.q2, q3=g_q.q3;

    /*---------- 4) 预测重力方向 v ----------*/
    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /*---------- 5) 预测地磁方向 w ----------*/
    float hx = 2.0f*mx*(0.5f - q2*q2 - q3*q3)
             + 2.0f*my*(q1*q2 - q0*q3)
             + 2.0f*mz*(q1*q3 + q0*q2);
    float hy = 2.0f*mx*(q1*q2 + q0*q3)
             + 2.0f*my*(0.5f - q1*q1 - q3*q3)
             + 2.0f*mz*(q2*q3 - q0*q1);
    float hz = 2.0f*mx*(q1*q3 - q0*q2)
             + 2.0f*my*(q2*q3 + q0*q1)
             + 2.0f*mz*(0.5f - q1*q1 - q2*q2);

    float bx = sqrtf(hx*hx + hy*hy);
    float bz = hz;

    float wx = 2.0f*bx*(0.5f - q2*q2 - q3*q3)
             + 2.0f*bz*(q1*q3 - q0*q2);
    float wy = 2.0f*bx*(q1*q2 - q0*q3)
             + 2.0f*bz*(q0*q1 + q2*q3);
    float wz = 2.0f*bx*(q0*q2 + q1*q3)
             + 2.0f*bz*(0.5f - q1*q1 - q2*q2);

    /*---------- 6) 误差 e = 观测 × 预测 ----------*/
    // 重力误差（总是使用）
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    // 地磁误差（带门控：仅当磁场强度合理时才使用）
    float mag_norm = sqrtf(mx*mx + my*my + mz*mz);
    const float MAG_MIN = 0.25f;  // 地磁场合理范围 25~65 μT（归一化后 0.25~0.65）
    const float MAG_MAX = 0.65f;
    
    if (mag_norm > MAG_MIN && mag_norm < MAG_MAX) {
        // 磁场强度合理，使用磁力计修正
        ex += (my*wz - mz*wy);
        ey += (mz*wx - mx*wz);
        ez += (mx*wy - my*wx);
    }
    // 否则，只使用加速度计修正（不使用磁力计）

    /*---------- 7) PI 反馈修正陀螺（带 Anti-windup）---------- */
    if (g_ki > 0.0f)
    {
        // 积分项累加
        g_exInt += ex * g_ki * dt;
        g_eyInt += ey * g_ki * dt;
        g_ezInt += ez * g_ki * dt;

        // Anti-windup：限制积分项幅度（防止积分饱和）
        const float INT_LIM = 0.1f;  // 积分限幅 ±0.1 rad/s
        g_exInt = fminf(fmaxf(g_exInt, -INT_LIM), INT_LIM);
        g_eyInt = fminf(fmaxf(g_eyInt, -INT_LIM), INT_LIM);
        g_ezInt = fminf(fmaxf(g_ezInt, -INT_LIM), INT_LIM);

        gx += g_exInt;
        gy += g_eyInt;
        gz += g_ezInt;
    }

    gx += kp_dynamic * ex;
    gy += kp_dynamic * ey;
    gz += kp_dynamic * ez;

    /*---------- 8) 四元数积分更新 ----------*/
    float halfDt = 0.5f * dt;

    q0 += (-q1*gx - q2*gy - q3*gz) * halfDt;
    q1 += ( q0*gx + q2*gz - q3*gy) * halfDt;
    q2 += ( q0*gy - q1*gz + q3*gx) * halfDt;
    q3 += ( q0*gz + q1*gy - q2*gx) * halfDt;

    float norm_q = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q < 1e-6f) return;

    g_q.q0=q0/norm_q; g_q.q1=q1/norm_q; g_q.q2=q2/norm_q; g_q.q3=q3/norm_q;

    /*---------- 9) 四元数 → 欧拉角 ----------*/
    q0=g_q.q0; q1=g_q.q1; q2=g_q.q2; q3=g_q.q3;

    g_euler_fused.roll  = atan2f(2*(q0*q1 + q2*q3),
                                 1 - 2*(q1*q1 + q2*q2));
    g_euler_fused.pitch = asinf (2*(q0*q2 - q3*q1));
    g_euler_fused.yaw   = atan2f(2*(q0*q3 + q1*q2),
                                 1 - 2*(q2*q2 + q3*q3));
}

void MPU9250_MahonyUpdateIMU(const MPU9250_Physical_Data *imu, float dt)
{
    /*---------- 1) 统一到机体系（X前 Y右 Z下） ----------*/
    float ax =  imu->accel_x_g;
    float ay = -imu->accel_y_g;
    float az =  imu->accel_z_g;

    const float deg2rad = 0.01745329252f;
    float gx =  imu->gyro_x_dps * deg2rad;
    float gy = -imu->gyro_y_dps * deg2rad;
    float gz =  imu->gyro_z_dps * deg2rad;

    /*---------- 2) 归一化 accel ----------*/
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    ax/=norm; ay/=norm; az/=norm;

    /*---------- 3) 当前四元数 ----------*/
    float q0=g_q.q0, q1=g_q.q1, q2=g_q.q2, q3=g_q.q3;

    /*---------- 4) 预测重力方向 v ----------*/
    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /*---------- 5) 重力误差 e = a × v ----------*/
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    /*---------- 6) PI 反馈修正陀螺 ----------*/
    if (g_ki > 0.0f)
    {
        g_exInt += ex * g_ki * dt;
        g_eyInt += ey * g_ki * dt;
        g_ezInt += ez * g_ki * dt;

        gx += g_exInt;
        gy += g_eyInt;
        gz += g_ezInt;
    }

    gx += g_kp * ex;
    gy += g_kp * ey;
    gz += g_kp * ez;

    /*---------- 7) 四元数积分更新 ----------*/
    float halfDt = 0.5f * dt;

    q0 += (-q1*gx - q2*gy - q3*gz) * halfDt;
    q1 += ( q0*gx + q2*gz - q3*gy) * halfDt;
    q2 += ( q0*gy - q1*gz + q3*gx) * halfDt;
    q3 += ( q0*gz + q1*gy - q2*gx) * halfDt;

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm < 1e-6f) return;

    g_q.q0=q0/norm; g_q.q1=q1/norm; g_q.q2=q2/norm; g_q.q3=q3/norm;

    /*---------- 8) 四元数 → 欧拉角 ----------*/
    q0=g_q.q0; q1=g_q.q1; q2=g_q.q2; q3=g_q.q3;

    g_euler_fused.roll  = atan2f(2*(q0*q1 + q2*q3),
                                 1 - 2*(q1*q1 + q2*q2));
    g_euler_fused.pitch = asinf (2*(q0*q2 - q3*q1));
    g_euler_fused.yaw   = atan2f(2*(q0*q3 + q1*q2),
                                 1 - 2*(q2*q2 + q3*q3));
}

void MPU9250_GetEulerFusedDeg(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float rad2deg = 57.295779513f;
    if (roll_deg)  *roll_deg  = g_euler_fused.roll  * rad2deg;
    if (pitch_deg) *pitch_deg = g_euler_fused.pitch * rad2deg; 
    if (yaw_deg)   *yaw_deg   = g_euler_fused.yaw   * rad2deg;
}
