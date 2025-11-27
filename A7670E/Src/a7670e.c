/**
 ******************************************************************************
 * @file    a7670e.c
 * @brief   SIMCom A7670E 4G/GNSS 模块驱动实现
 * @note    支持 AT 指令、网络初始化、GNSS 定位功能
 ******************************************************************************
 */

#include "a7670e.h"

/*==============================================================================
 *                           私有函数声明
 *============================================================================*/
uint16_t AT_ReadAllToBuffer_Timeout(uint32_t window_ms,
                                    uint32_t idle_ms,
                                    uint8_t  flush);

static float nmea_to_deg(const char *nmea, const char *hemi);  // NMEA 格式转换为十进制度数

/*==============================================================================
 *                           全局变量定义
 *============================================================================*/

// 定义环形缓冲区（USART1 IRQ 写入）
char AT_RingBuffer[AT_RX_BUF_SIZE];      // 接收缓冲区
static volatile uint16_t AT_RxWrite = 0; // 写指针
static volatile uint16_t AT_RxRead  = 0; // 读指针

// 定义线性拼接缓冲（用于 strstr / strtok 解析）
char     AT_rx_buffer[AT_RX_BUF_SIZE]; // 线性接收缓冲区
static uint16_t AT_rx_index = 0;              // 线性缓冲区索引

// 可在外部查看是否截断
volatile uint8_t AT_rx_truncated = 0;

/* 经纬度（GCJ-02） */
double lat_gcj, lon_gcj;

// 经纬度（WGS-84）
double lat_wgs, lon_wgs;

//GNSS结构体
GNSS_Data_t gnss_data = { 0 };

/*==============================================================================
 *                           UART 底层操作（IRQ + RingBuffer）
 *============================================================================*/

/**
 * @brief 启动UART接收中断
 * @param None
 * @return None
 */
void AT_UART_Start_IT(void)
{
    // 1) 使能接收中断
    USART1->CR1 |= USART_CR1_RXNEIE;

    // 2) NVIC：设置优先级并使能
    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_EnableIRQ(USART1_IRQn); // 使能USART1中断
}


/**
 * @brief USART1中断处理函数(核心环形缓冲逻辑)
 * @param None
 * @return None
 */
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE) // 收到数据
    {
        char c = USART1->DR;       // 读取接收到的数据（会自动清除RXNE标志位）

        AT_RingBuffer[AT_RxWrite] = c;                  // 存入环形缓冲区
        AT_RxWrite = (AT_RxWrite + 1) % AT_RX_BUF_SIZE; // 更新写指针
    }
}


/**
 * @brief 从环形缓冲区读取一个字节
 * @param None
 * @return 读取到的字节，若无数据则返回 -1
 */
int AT_RingBuffer_ReadByte(char *out)
{
    if (AT_RxRead == AT_RxWrite)
    {
        return -1; // 缓冲区为空
    }

    *out = AT_RingBuffer[AT_RxRead];
    AT_RxRead = (AT_RxRead + 1) % AT_RX_BUF_SIZE; // 更新读指针
    return 1;                                     // 成功读取一个字节
}


/*==============================================================================
 *                           AT 核心收发逻辑
 *============================================================================*/

/**
 * @brief 发送AT指令并等待指定响应
 * @param cmd AT指令（不需要加\r\n)
 * @param expect 期望的响应字符串
 * @param timeout_ms: 超时时间（毫秒）
 * @return 0 成功找到期望响应
 * @return 1 超时或未找到期望响应
 * @return 2 参数错误
 */
uint8_t AT_SendAndWait(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    // 参数检查
    if (cmd == NULL || expect == NULL || timeout_ms == 0)
    {
        return 2; // 参数错误
    }

    uint32_t start_time;   // 获取当前时间戳的变量
    char     c;
    char     AT_buffer[256];

    memset(AT_buffer, 0, sizeof(AT_buffer));
    snprintf(AT_buffer, sizeof(AT_buffer), "%s\r\n", cmd); // 构造完整AT指令

    // 清空环形缓冲区
    __disable_irq();            // 清空时关闭中断，防止再清空时写入新数据
    AT_RxRead = AT_RxWrite = 0; // 重置读写指针
    __enable_irq();             // 重新开启中断

    // 清空UART的接收寄存器
    volatile uint32_t d;
    while (USART1->SR & USART_SR_RXNE) { d = USART1->DR; }
    (void)d; // C语言的写法：我故意不使用变量 d，别给我编译器警告

    // 发送命令
    for (int i = 0; AT_buffer[i] != '\0'; i++)
    {
        while (!(USART1->SR & USART_SR_TXE)); // 等待发送缓冲区空
        USART1->DR = AT_buffer[i];           // 发送一个字节
    }

    start_time = HAL_GetTick(); // 获取当前时间戳

    memset(AT_rx_buffer, 0, sizeof(AT_rx_buffer)); // 清空接收缓冲区
    AT_rx_index = 0;                               // 重置线性缓冲区索引

    // 等待响应
    while (HAL_GetTick() - start_time < timeout_ms) // 判断是否超时
    {
        if (AT_RingBuffer_ReadByte(&c) == 1)        // 存在新数据
        {
            if (AT_rx_index < sizeof(AT_rx_buffer) - 1) // 防止溢出
            {
                AT_rx_buffer[AT_rx_index++] = c;   // 存入线性缓冲区
                AT_rx_buffer[AT_rx_index]   = '\0';// 保持字符串结束符

                if (strstr(AT_rx_buffer, expect) != NULL) // 检查是否包含期望响应
                {
                    printf_uart6("%s\n", AT_rx_buffer);     // 打印接收到的完整响应（调试用）
                    return 0;                             // 找到期望响应
                }
            }
        }
    }
    return 1; // 超时或未找到期望响应
}


/**
 * @brief 发送格式化AT指令并等待指定响应
 * @param expect 期望的响应字符串
 * @param timeout_ms 超时时间（毫秒）
 * @param format 格式化字符串（类似printf）
 * @param ... 可变参数列表
 * @return 0 成功找到期望响应
 * @return 1 超时或未找到期望响应
 * @return 2 参数错误
 */
uint8_t AT_SendFormatAndWait(const char *expect,
                             uint32_t timeout_ms,
                             const char *format, ...)
{
    va_list args;            // 声明可变参数列表
    char    cmd_buffer[256]; // 格式化后的指令缓冲区

    // 参数检查
    if (format == NULL || expect == NULL || timeout_ms == 0)
    {
        return 2;
    }

    // 开始处理可变参数
    va_start(args, format); // 告诉编译器“从 format 之后的那个参数开始取”。 -------开始

    // 将格式化字符串和可变参数组合成完整指令
    vsnprintf(cmd_buffer, sizeof(cmd_buffer), format, args); // 像 sprintf，但多一个 size，防止溢出。

    // 结束可变参数处理                                  -------结束
    va_end(args);

    // 调用基础AT指令函数
    return AT_SendAndWait(cmd_buffer, expect, timeout_ms);
}


/*==============================================================================
 *                           网络功能
 *============================================================================*/

/**
 * @brief 初始化网络
 * @param None
 * @return 0 成功初始化网络
 * @return 1 初始化网络失败
 */
uint8_t AT_Network_Init(void)
{
    // 1.接入APN
    if (0 == AT_SendAndWait("AT+CGDCONT=1,\"IP\",\"ctnet\"", "OK", 2000)) // 接入电信APN，ctnet是电信
    {
        printf_uart6("APN connection is normal. \r\n");
    }
    else
    {
        printf_uart6("APN connection failed. \r\n");
        return 1; // 初始化网络失败
    }

    //2.打开详细错误（否则只显示 ERROR，看不到 +CME ERROR: xxx）
    printf_uart1("AT+CMEE=2\r\n");//调试专用
    AT_ReadAllToBuffer_Timeout(2000, 120, 1);//调试专用
    printf_uart6("%s", AT_rx_buffer);//调试专用

    //3.建立数据连接
    printf_uart1("AT+NETOPEN\r\n");
    AT_ReadAllToBuffer_Timeout(10000, 200, 1);
    printf_uart6("%s", AT_rx_buffer);//调试专用

    return 0; // 成功初始化网络
}


/*==============================================================================
 *                           GNSS 功能
 *============================================================================*/

/**
 * @brief GNSS初始化
 * @param None
 * @return None
 */
uint8_t AT_GNSS_Init(void)
{
    // 开启GNSS电源
    printf_uart1("AT+CGNSSPWR=1\r\n");
    AT_ReadAllToBuffer_Timeout(2000, 120, 1);
    printf_uart6("%s", AT_rx_buffer);//调试专用

    // 等待30秒让GNSS模块定位
    HAL_Delay(30000); // 30秒

    // 开启AGPS辅助定位
    AT_SendAndWait("AT+CAGPS", "OK", 2000);

    return 0;
}


/**
 * @brief 关闭GNSS定位功能
 * @param None
 * @return 0 成功关闭GNSS
 * @return 1 关闭GNSS失败
 */
uint8_t AT_GNSS_PowerOff(void)
{
    // 关闭GNSS
    return AT_SendAndWait("AT+CGNSPWR=0", "OK", 1000);
}


/**
 * @brief 获取一次 GNSS 定位信息并解析（整数打印版，带行过滤）
 * @return 1：解析成功；0：未定位或格式无效
 */
uint8_t AT_GNSS_GetLocation(void)
{
    // 1. 发送命令并把应答读到 AT_rx_buffer 里
    printf_uart1("AT+CGPSINFO\r\n");
    AT_ReadAllToBuffer_Timeout(3000, 500, 1);
    printf_uart6("RAW: %s\r\n", AT_rx_buffer); // 调试用

    // 2. 在 AT_rx_buffer 里找到 "+CGPSINFO:" 那一行
    char *start = strstr(AT_rx_buffer, "+CGPSINFO:");
    if (!start)
    {
        printf_uart6("No +CGPSINFO found.\r\n");
        return 0;
    }

    // 把这一行拷贝出来（到回车/换行为止）
    char line[160];
    int  i = 0;

    while (i < (int)sizeof(line) - 1 && start[i] != '\r' && start[i] != '\n' && start[i] != '\0')
    {
        line[i] = start[i];
        i++;
    }
    line[i] = '\0';

    printf_uart6("LINE: %s\r\n", line); // 只包含一行 +CGPSINFO: ...

    // 3. 跳过前缀 "+CGPSINFO:"
    char *p = strstr(line, "+CGPSINFO:");
    if (!p)
    {
        printf_uart6("LINE has no +CGPSINFO: ???\r\n");
        return 0;
    }
    p += strlen("+CGPSINFO:");

    // 跳过前面的空格
    while (*p == ' ' || *p == '\t') p++;

    // 4. 用逗号分割字段
    char buf[160];
    strncpy(buf, p, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *field[10] = {0};
    int   count     = 0;

    char *token = strtok(buf, ",");
    while (token && count < 10)
    {
        field[count++] = token;
        token = strtok(NULL, ",");
    }

    if (count < 4)
    {
        printf_uart6("Fields < 4. count=%d\r\n", count);
        return 0;
    }

    // 调试：把每个字段打出来看一眼
    for (int k = 0; k < count; k++)
    {
        printf_uart6("field[%d] = '%s'\r\n", k, field[k]);
    }

    gnss_data.latitude = nmea_to_deg(field[0], field[1]); // 纬度
    gnss_data.longitude = nmea_to_deg(field[2], field[3]); // 经度
    gnss_data.altitude = field[6] ? (float)atof(field[6]) : 0.0f; // 海拔
    gnss_data.speed_knots = field[7] ? (float)atof(field[7]) * 0.514444f : 0.0f; // 速度（节 -> 米/秒）


    // wgs84_to_gcj02(lat_wgs, lon_wgs, &lat_gcj, &lon_gcj);

    // printf_uart6("WGS84 : %.6f, %.6f\r\n", lat_wgs, lon_wgs);
    // printf_uart6("GCJ-02: %.6f, %.6f\r\n", lat_gcj, lon_gcj);

    // field[0]    纬度（DDMM.MMMM）
    // field[1]    南北半球 N/S
    // field[2]    经度（DDDMM.MMMM）
    // field[3]    东西半球 E/W
    // field[4]    日期（DDMMYY）
    // field[5]    时间（UTC, HHMMSS）
    // field[6]    海拔（米）
    // field[7]    速度（节 knots）
    // field[8]    航向角（度）
    
    return 1;
}


/*==============================================================================
 *                           对外初始化入口
 *============================================================================*/

/**
 * @brief 获取位置信息初始化
 * @param None
 * @return None
 * @note 在程序开始时调用一次即可
 */
void AT_Getlocation_Init(void)
{
    AT_UART_Start_IT(); // 开启USART1中断

    // 1. 初始化 4G（APN + NETOPEN）
    AT_Network_Init();

    // 2. 打开 GNSS（你当前代码自动 delay 30 秒，不判断结果）
    AT_GNSS_Init();
}


/*==============================================================================
 *                           通用工具函数
 *============================================================================*/

/**
 * @brief 在给定窗口内把环形缓冲搬到线性缓冲；遇到 idle_ms 空闲即提前结束
 * @param window_ms 总窗口超时：整个过程的最大等待时间，到了就结束。
 * @param idle_ms   空闲判定时长（如 100）：如果已经收到过数据了，但连续这么久都没有新数据，就认为“这一帧结束了”，提前退出。
 * @param flush     1=结束后清空环形缓冲；0=保留未读数据
 * @return 搬运到线性缓冲的字节数
 */
uint16_t AT_ReadAllToBuffer_Timeout(uint32_t window_ms,
                                    uint32_t idle_ms,
                                    uint8_t  flush)
{
    char     c;
    uint16_t count   = 0;
    uint32_t start   = HAL_GetTick();
    uint32_t last_rx = start;

    AT_rx_truncated = 0;
    memset(AT_rx_buffer, 0, sizeof(AT_rx_buffer));
    AT_rx_index = 0;

    while (HAL_GetTick() - start < window_ms)
    {
        if (AT_RingBuffer_ReadByte(&c) == 1)
        {
            last_rx = HAL_GetTick();

            // 线性缓冲写入（留1字节给 '\0'）
            if (AT_rx_index < sizeof(AT_rx_buffer) - 1)
            {
                AT_rx_buffer[AT_rx_index++] = c;
                count++;
            }
            else
            {
                // 线性缓冲已满：标记截断，但仍把环形缓冲读空，避免卡住
                AT_rx_truncated = 1;
            }
        }
        else
        {
            // 没新数据：若已接收且空闲超过 idle_ms 就认为结束
            if (count > 0 && (HAL_GetTick() - last_rx) > idle_ms) break;
        }
    }

    AT_rx_buffer[AT_rx_index] = '\0';

    if (flush)
    {
        __disable_irq();
        AT_RxRead = AT_RxWrite = 0;
        __enable_irq();
    }

    return count;
}
/*==============================================================================
 *                           坐标系转换函数
 *============================================================================*/

/**
 * @brief  NMEA 度分格式坐标 -> 十进制度坐标
 * @param  nmea  NMEA 坐标字符串
 *              - 纬度格式：DDMM.MMMM  (例如 "3112.3456" 表示 31度12.3456分)
 *              - 经度格式：DDDMM.MMMM (例如 "12130.1234" 表示 121度30.1234分)
 * @param  hemi  半球标识字符串："N"/"S" 或 "E"/"W"
 * @return 十进制度坐标（decimal degrees）
 *         - 北纬/东经为正
 *         - 南纬/西经为负
 *
 * 计算方法：
 *   1) 从 NMEA 中取出 “度” = 整数部分(v/100)
 *   2) “分” = v - 度*100
 *   3) 十进制度 = 度 + 分/60
 */
static float nmea_to_deg(const char *nmea, const char *hemi)
{
    /* 参数保护：空指针或空字符串直接返回 0 */
    if (!nmea || !hemi || nmea[0] == '\0') return 0.0;

    /* 例： "3112.3456" -> 3112.3456 */
    float v = atof(nmea);

    /* 取 “度” 部分：3112.3456 /100 = 31.xx -> 31 */
    int deg = (int)(v / 100);

    /* 取 “分” 部分：3112.3456 - 31*100 = 12.3456 */
    float min = v - deg * 100.0;

    /* 十进制度 = 度 + 分/60 */
    float dec = deg + min / 60.0;

    /* 南纬(S) / 西经(W) 取负 */
    if (hemi[0] == 'S' || hemi[0] == 'W') dec = -dec;

    return dec;
}

// /* 圆周率 */
// static const double PI = 3.14159265358979323846;
// /* 地球长半轴（GCJ-02 使用的参考椭球参数） */
// static const double A  = 6378245.0;
// /* 地球偏心率平方 */
// static const double EE = 0.00669342162296594323;

// /**
//  * @brief 判断坐标是否在中国境内
//  * @param lat 纬度（十进制度）
//  * @param lon 经度（十进制度）
//  * @return 1=不在中国，0=在中国
//  *
//  * GCJ-02 只对中国境内做加偏移处理。
//  * 超出边界则直接返回原始 WGS84。
//  */
// static int out_of_china(double lat, double lon)
// {
//     return (lon < 72.004 || lon > 137.8347 ||
//             lat < 0.8293 || lat > 55.8271);
// }

// /**
//  * @brief 计算纬度方向的偏移量（内部用）
//  * @param x 经度差 (lon - 105.0)
//  * @param y 纬度差 (lat - 35.0)
//  * @return 纬度偏移（单位：约等于米级映射到角度前的中间值）
//  *
//  * 这是 GCJ-02 的核心扰动模型之一，不建议修改。
//  */
// static double transform_lat(double x, double y)
// {
//     double ret = -100.0 + 2.0*x + 3.0*y + 0.2*y*y
//                  + 0.1*x*y + 0.2*sqrt(fabs(x));
//     ret += (20.0*sin(6.0*x*PI) + 20.0*sin(2.0*x*PI)) * 2.0/3.0;
//     ret += (20.0*sin(y*PI) + 40.0*sin(y/3.0*PI)) * 2.0/3.0;
//     ret += (160.0*sin(y/12.0*PI) + 320.0*sin(y*PI/30.0)) * 2.0/3.0;
//     return ret;
// }

// /**
//  * @brief 计算经度方向的偏移量（内部用）
//  * @param x 经度差 (lon - 105.0)
//  * @param y 纬度差 (lat - 35.0)
//  * @return 经度偏移（单位：约等于米级映射到角度前的中间值）
//  *
//  * 这是 GCJ-02 的核心扰动模型之一，不建议修改。
//  */
// static double transform_lon(double x, double y)
// {
//     double ret = 300.0 + x + 2.0*y + 0.1*x*x
//                  + 0.1*x*y + 0.1*sqrt(fabs(x));
//     ret += (20.0*sin(6.0*x*PI) + 20.0*sin(2.0*x*PI)) * 2.0/3.0;
//     ret += (20.0*sin(x*PI) + 40.0*sin(x/3.0*PI)) * 2.0/3.0;
//     ret += (150.0*sin(x/12.0*PI) + 300.0*sin(x/30.0*PI)) * 2.0/3.0;
//     return ret;
// }

// /**
//  * @brief 将 WGS-84 十进制度经纬度转换为 GCJ-02 十进制度经纬度
//  * @param lat      输入：WGS84 纬度（十进制度）
//  * @param lon      输入：WGS84 经度（十进制度）
//  * @param out_lat  输出：GCJ-02 纬度（十进制度）
//  * @param out_lon  输出：GCJ-02 经度（十进制度）
//  *
//  * 使用方式：
//  *  - 传入十进制度 WGS84
//  *  - 函数内部判定是否中国境内
//  *  - 返回 GCJ-02（常用于高德/腾讯/国内地图）
//  */
// void wgs84_to_gcj02(double lat, double lon, double *out_lat, double *out_lon)
// {
//     if (!out_lat || !out_lon) return;

//     /* 中国境外不偏移 */
//     if (out_of_china(lat, lon))
//     {
//         *out_lat = lat;
//         *out_lon = lon;
//         return;
//     }

//     /* 计算中间偏移值 */
//     double dLat = transform_lat(lon - 105.0, lat - 35.0);
//     double dLon = transform_lon(lon - 105.0, lat - 35.0);

//     /* 椭球体与纬度相关的修正 */
//     double radLat = lat / 180.0 * PI;
//     double sinRad = sin(radLat);
//     double magic  = 1.0 - EE * sinRad * sinRad;
//     double sqrtMagic = sqrt(magic);

//     /* 将偏移量从“米级扰动”换算成角度偏移 */
//     dLat = (dLat * 180.0) / ((A * (1.0 - EE)) / (magic * sqrtMagic) * PI);
//     dLon = (dLon * 180.0) / (A / sqrtMagic * cos(radLat) * PI);

//     /* 得到 GCJ-02 坐标 */
//     *out_lat = lat + dLat;
//     *out_lon = lon + dLon;
// }

