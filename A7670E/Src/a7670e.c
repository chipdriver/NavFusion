#include "a7670e.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal_uart.h"

uint16_t AT_ReadAllToBuffer_Timeout(uint32_t window_ms, uint32_t idle_ms, uint8_t flush);


volatile uint32_t usart1_isr_cnt = 0; //中断计数器

//定义环形缓冲区
char AT_RingBuffer[AT_RX_BUF_SIZE];  //接收缓冲区

static volatile uint16_t AT_RxWrite = 0;    //写指针
static volatile uint16_t AT_RxRead = 0;     //读指针

//定义线性拼接缓冲（用于strstr）
static char AT_rx_buffer[AT_RX_BUF_SIZE];   //定义线性接收缓冲区
static uint16_t AT_rx_index = 0;            //线性缓冲区索引

/**
 * @brief 启动UART接收中断
 * @param None
 * @return None
 */
void AT_UART_Start_IT(void)
{
    // 1) 使能外设 + 接收 + 接收中断
    USART1->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;

    // 2) NVIC：设置优先级并使能
    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_EnableIRQ(USART1_IRQn); //使能USART1中断
}


/**
 * @brief USART1中断处理函数(核心环形缓冲逻辑)
 * @param None
 * @return None
 */
void USART1_IRQHandler(void)
{
    
    if(USART1 -> SR & USART_SR_RXNE)//收到数据
    {
        char c = USART1 -> DR; //读取接收到的数据（会自动清除RXNE标志位）

        AT_RingBuffer[AT_RxWrite] = c; //存入环形缓冲区
        AT_RxWrite = (AT_RxWrite + 1) % AT_RX_BUF_SIZE; //更新写指针
    }
}

/**
 * @brief 从环形缓冲区读取一个字节
 * @param None
 * @return 读取到的字节，若无数据则返回 -1
 */
int AT_RingBuffer_ReadByte(char *out)
{   
    if(AT_RxRead == AT_RxWrite)
    {
        return -1; //缓冲区为空
    }

    *out = AT_RingBuffer[AT_RxRead];
    AT_RxRead = (AT_RxRead + 1) % AT_RX_BUF_SIZE; //更新读指针
    return 1; //成功读取一个字节
}


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
    //参数检查
    if(cmd == NULL || expect == NULL || timeout_ms == 0)
    {
        return 2; //参数错误
    }
    uint32_t start_time; //获取当前时间戳的变量
    char c;
    char AT_buffer[256];

    memset(AT_buffer,0,sizeof(AT_buffer));
    snprintf(AT_buffer,sizeof(AT_buffer), "%s\r\n", cmd); //构造完整AT指令

    //清空环形缓冲区
    __disable_irq(); //清空时关闭中断，防止再清空时写入新数据
    AT_RxRead = AT_RxWrite = 0; //重置读写指针
    __enable_irq(); //重新开启中断

    //清空UART的接收寄存器
    volatile uint32_t d;
    while (USART1->SR & USART_SR_RXNE) { d = USART1->DR; }
    (void)d;//C语言的写法：我故意不使用变量 d，别给我编译器警告

    //发送命令
    for(int i =0; AT_buffer[i] != '\0'; i++)
    {   
        while(!(USART1->SR & USART_SR_TXE)); //等待发送缓冲区空
        USART1->DR = AT_buffer[i]; //发送一个字节
    }

    start_time = HAL_GetTick(); //获取当前时间戳

    memset(AT_rx_buffer,0,sizeof(AT_rx_buffer)); //清空接收缓冲区
    AT_rx_index = 0; //重置线性缓冲区索引

    //等待响应
    while(HAL_GetTick() - start_time < timeout_ms)   //判断是否超时
    {
        if(AT_RingBuffer_ReadByte(&c) == 1)  //存在新数据
        {
            if(AT_rx_index <sizeof(AT_rx_buffer) - 1)  //防止溢出
            {
                AT_rx_buffer[AT_rx_index++] = c; //存入线性缓冲区
                AT_rx_buffer[AT_rx_index] = '\0'; //保持字符串结束符

                if(strstr(AT_rx_buffer, expect) != NULL) //检查是否包含期望响应
                {
                    printf_uart6("%s", AT_rx_buffer); //打印接收到的完整响应（调试用）
                    return 0; //找到期望响应
                }
            }
        }
    }
    return 1; //超时或未找到期望响应
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
uint8_t AT_SendFormatAndWait(const char *expect, uint32_t timeout_ms, const char *format, ...)
{
    va_list args;                    // 声明可变参数列表
    char cmd_buffer[256];            // 格式化后的指令缓冲区
    
    // 参数检查
    if (format == NULL || expect == NULL || timeout_ms == 0) {
        return 2;
    }
    
    // 开始处理可变参数
    va_start(args, format);//告诉编译器“从 format 之后的那个参数开始取”。   -------开始
    
    // 将格式化字符串和可变参数组合成完整指令
    vsnprintf(cmd_buffer, sizeof(cmd_buffer), format, args);//像 sprintf，但多一个 size，防止溢出。
    
    // 结束可变参数处理                         -------结束
    va_end(args);
    
    // 调用基础AT指令函数
    return AT_SendAndWait(cmd_buffer, expect, timeout_ms);
}


/**
 * @brief 初始化网络
 * @param None
 * @return 0 成功初始化网络
 * @return 1 初始化网络失败
 */
uint8_t AT_Network_Init(void)
{
    //接入APN
    if( 0 == AT_SendAndWait("AT+CGDCONT=1,\"IP\",\"ctnet\"", "OK", 2000)) //接入电信APN，ctnet是电信
    {
        printf_uart6("APN connection is normal. \r\n");
    }
    else
    {
        printf_uart6("APN connection failed. \r\n");
        return 1; //初始化网络失败
    }  

    // 打开详细错误（否则只显示 ERROR，看不到 +CME ERROR: xxx）
    printf_uart1("AT+CMEE=2\r\n");
    AT_ReadAllToBuffer_Timeout(2000, 120, 1);  printf_uart6("%s", AT_rx_buffer);

    //建立数据连接
    printf_uart1("AT+NETOPEN\r\n");
    AT_ReadAllToBuffer_Timeout(10000, 200, 1); printf_uart6("%s", AT_rx_buffer);

    return 0; //成功初始化网络
}

/**
 * @brief GNSS初始化
 * @param None
 * @return None
 */
uint8_t AT_GNSS_Init(void)
{
    //开启GNSS电源
    printf_uart1("AT+CGNSSPWR=1\r\n");
    AT_ReadAllToBuffer_Timeout(2000, 120, 1);   printf_uart6("%s", AT_rx_buffer);

    // 等待30秒让GNSS模块定位
    HAL_Delay(30000); //30秒

    //开启AGPS辅助定位
    AT_SendAndWait("AT+CAGPS", "OK", 2000);
}


/**
 * @brief 关闭GNSS定位功能
 * @param None
 * @return 0 成功关闭GNSS
 * @return 1 关闭GNSS失败
 */
uint8_t AT_GNSS_PowerOff(void)
{
    //关闭GNSS
    return AT_SendAndWait("AT+CGNSPWR=0", "OK", 1000);
}


/**
 * @brief 获取一次 GNSS 定位信息并解析（整数打印版）
 * @return 1：解析成功；0：未定位或格式无效
 */
/**
 * @brief 获取一次 GNSS 定位信息并解析（整数打印版，带行过滤）
 * @return 1：解析成功；0：未定位或格式无效
 */
uint8_t AT_GNSS_GetLocation(void)
{
    // 1. 发送命令并把应答读到 AT_rx_buffer 里
    printf_uart1("AT+CGPSINFO\r\n");
    // 窗口放宽一点，避免被拆成多段
    AT_ReadAllToBuffer_Timeout(3000, 500, 1);

    printf_uart6("RAW: %s\r\n", AT_rx_buffer);

    // 2. 在 AT_rx_buffer 里找到 "+CGPSINFO:" 那一行
    char *start = strstr(AT_rx_buffer, "+CGPSINFO:");
    if (!start) {
        printf_uart6("No +CGPSINFO found.\r\n");
        return 0;
    }

    // 把这一行拷贝出来（到回车/换行为止）
    char line[160];
    int i = 0;
    while (i < (int)sizeof(line) - 1 &&
           start[i] != '\r' && start[i] != '\n' &&
           start[i] != '\0')
    {
        line[i] = start[i];
        i++;
    }
    line[i] = '\0';

    printf_uart6("LINE: %s\r\n", line);  // 只包含一行 +CGPSINFO: ...

    // 3. 跳过前缀 "+CGPSINFO:"
    char *p = strstr(line, "+CGPSINFO:");
    if (!p) {
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
    int count = 0;

    char *token = strtok(buf, ",");
    while (token && count < 10) {
        field[count++] = token;
        token = strtok(NULL, ",");
    }

    if (count < 4) {
        printf_uart6("Fields < 4. count=%d\r\n", count);
        return 0;
    }

    // 调试：把每个字段打出来看一眼
    for (int k = 0; k < count; k++) {
        printf_uart6("field[%d] = '%s'\r\n", k, field[k]);
    }

    // 纬度或经度为空 => 未定位成功（,,,,,,,, 这种）
    if (field[0][0] == '\0' || field[2][0] == '\0') {
        printf_uart6("Not fixed.\r\n");
        return 0;
    }

    // 5. 解析为 double（内部用 double 运算）
    double lat_raw = atof(field[0]);              // 3616.87580
    int    lat_d   = (int)(lat_raw / 100);
    double lat_m   = lat_raw - lat_d * 100;
    double lat     = lat_d + lat_m / 60.0;

    if (field[1][0] == 'S') lat = -lat;

    double lon_raw = atof(field[2]);              // 12015.78464
    int    lon_d   = (int)(lon_raw / 100);
    double lon_m   = lon_raw - lon_d * 100;
    double lon     = lon_d + lon_m / 60.0;

    if (field[3][0] == 'W') lon = -lon;

    double alt = 0.0;
    if (count > 6 && field[6][0] != '\0') {
        alt = atof(field[6]);                     // 米
    }

    double speed_knots = 0.0;
    if (count > 7 && field[7][0] != '\0') {
        speed_knots = atof(field[7]);             // 节
    }
    double speed_kmh = speed_knots * 1.852;       // 转 km/h

    double course = 0.0;
    if (count > 8 && field[8][0] != '\0') {
        course = atof(field[8]);                  // 度
    }

    // 6. 整数打印（不会用到 %f）
    int32_t lat_i    = (int32_t)(lat * 1000000);  // 1e-6 度
    int32_t lon_i    = (int32_t)(lon * 1000000);
    int32_t alt_i    = (int32_t)(alt * 10);       // 0.1 m
    int32_t speed_i  = (int32_t)(speed_kmh * 10); // 0.1 km/h
    int32_t course_i = (int32_t)(course * 10);    // 0.1 deg

    printf_uart6("Lat_i    = %ld (×1e-6 deg)\r\n", lat_i);
    printf_uart6("Lon_i    = %ld (×1e-6 deg)\r\n", lon_i);
    printf_uart6("Alt_i    = %ld (×0.1 m)\r\n",   alt_i);
    printf_uart6("Speed_i  = %ld (×0.1 km/h)\r\n", speed_i);
    printf_uart6("Course_i = %ld (×0.1 deg)\r\n", course_i);

    return 1;
}





/**
 * @brief 获取位置信息初始化
 * @param None
 * @return None
 */
void AT_Getlocation_Init(void)
{
    AT_UART_Start_IT(); //开启USART1中断

    // 1. 初始化 4G（APN + NETOPEN）
    AT_Network_Init();

    // 2. 打开 GNSS（你当前代码自动 delay 30 秒，不判断结果）
    AT_GNSS_Init();


}

// 可在外部查看是否截断
volatile uint8_t AT_rx_truncated = 0;

/**
 * @brief 在给定窗口内把环形缓冲搬到线性缓冲；遇到 idle_ms 空闲即提前结束
 * @param window_ms 总窗口超时：整个过程的最大等待时间，到了就结束。
 * @param idle_ms   空闲判定时长（如 100）：如果已经收到过数据了，但连续这么久都没有新数据，就认为“这一帧结束了”，提前退出。
 * @param flush     1=结束后清空环形缓冲；0=保留未读数据
 * @return 搬运到线性缓冲的字节数
 */
uint16_t AT_ReadAllToBuffer_Timeout(uint32_t window_ms, uint32_t idle_ms, uint8_t flush)
{
    char c;
    uint16_t count = 0;
    uint32_t start = HAL_GetTick();
    uint32_t last_rx = start;

    AT_rx_truncated = 0;
    memset(AT_rx_buffer, 0, sizeof(AT_rx_buffer));
    AT_rx_index = 0;

    while (HAL_GetTick() - start < window_ms) {
        if (AT_RingBuffer_ReadByte(&c) == 1) {
            last_rx = HAL_GetTick();

            // 线性缓冲写入（留1字节给 '\0'）
            if (AT_rx_index < sizeof(AT_rx_buffer) - 1) {
                AT_rx_buffer[AT_rx_index++] = c;
                count++;
            } else {
                // 线性缓冲已满：标记截断，但仍把环形缓冲读空，避免卡住
                AT_rx_truncated = 1;
            }
        } else {
            // 没新数据：若已接收且空闲超过 idle_ms 就认为结束
            if (count > 0 && (HAL_GetTick() - last_rx) > idle_ms) break;
        }
    }

    AT_rx_buffer[AT_rx_index] = '\0';

    if (flush) {
        __disable_irq();
        AT_RxRead = AT_RxWrite = 0;
        __enable_irq();
    }
    return count;
}
