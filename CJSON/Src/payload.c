#include "payload.h"
#include "cJSON.h"
#include "a7670e.h"
#include <string.h>
#include <stdio.h>

/*******************************************************
 *                    内部函数声明
 * *****************************************************/
static inline void AT_ClearRing(void);
static inline void UART1_ClearRXNE(void);
static int AT_WaitForPromptOrUrc(const char *prompt, uint32_t total_ms);
int MQTT_ResetAndReconnect(void);   
/*******************************************************
 *                    全局互斥锁
 * *****************************************************/
volatile int mqtt_publishing = 0; // 互斥锁：publish 窗口期禁止其它 AT


/*******************************************************
 * @brief 组装 WGS84 + 姿态 的 JSON payload
 * @return malloc 出来的 JSON 字符串，调用者用完必须 free()
 *****************************************************/
char* BuildPayload_WGS84_Attitude(
    const char *tid,
    const char *bid,
    long long timestamp_ms,
    const char *gateway,
    double lat_wgs,
    double lon_wgs,
    double height_m,
    double hspeed_knots,
    double roll_deg,
    double pitch_deg,
    double yaw_deg,
    int battery_pct,
    int gps_num
)
{
    cJSON *root = cJSON_CreateObject();
    if(!root) return NULL;

    cJSON_AddStringToObject(root, "tid", tid ? tid : "");
    cJSON_AddStringToObject(root, "bid", bid ? bid : "");
    cJSON_AddNumberToObject(root, "timestamp", (double)timestamp_ms);
    cJSON_AddStringToObject(root, "gateway", gateway ? gateway : "");

    cJSON *data = cJSON_CreateObject();
    if(!data) { cJSON_Delete(root); return NULL; }
    cJSON_AddItemToObject(root, "data", data);

    cJSON_AddNumberToObject(data,"latitude",lat_wgs);
    cJSON_AddNumberToObject(data,"longitude",lon_wgs);
    cJSON_AddNumberToObject(data, "height", height_m);
    cJSON_AddNumberToObject(data, "horizontal_speed", hspeed_knots);
    cJSON_AddNumberToObject(data, "attitude_roll",  roll_deg);
    cJSON_AddNumberToObject(data, "attitude_pitch", pitch_deg);
    cJSON_AddNumberToObject(data, "attitude_head",  yaw_deg);

    cJSON_AddNumberToObject(data, "mode_code", 0);
    cJSON_AddNumberToObject(data, "vertical_speed", 0);
    cJSON_AddNumberToObject(data, "home_longitude", 0);
    cJSON_AddNumberToObject(data, "home_latitude", 0);

    cJSON *battery = cJSON_CreateObject();
    if(!battery) { cJSON_Delete(root); return NULL; }
    cJSON_AddItemToObject(data, "battery", battery);
    cJSON_AddNumberToObject(battery, "percentage", (battery_pct >= 0) ? battery_pct : 0);

    if (gps_num >= 0)
    {
        cJSON *pos = cJSON_CreateObject();
        if(pos){
            cJSON_AddItemToObject(data, "position_state", pos);
            cJSON_AddNumberToObject(pos, "gps_number", gps_num);
        }
    }

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return payload;
}


/*******************************************************
 * @brief 启动 MQTT 并连接
 * @return 0 成功，非0失败
 *****************************************************/
int MQTT_InitAndConnect_raw(void)
{
    printf_uart6("\r\n========== MQTT_InitAndConnect_raw ==========\r\n");

    // 0) 关回显，减噪（可选但强烈建议）
    AT_ClearRing(); UART1_ClearRXNE();
    printf_uart1("ATE0\r\n");
    AT_ReadAllToBuffer_Timeout(1000, 150, 1);

    // 1) CMQTTSTART 启动 MQTT 客户端服务
    AT_ClearRing(); UART1_ClearRXNE();
    printf_uart1("AT+CMQTTSTART\r\n");
    AT_ReadAllToBuffer_Timeout(3000, 200, 1);
    printf_uart6("%s", AT_rx_buffer);

    if (strstr(AT_rx_buffer, "OK") == NULL &&
        strstr(AT_rx_buffer, "+CMQTTSTART: 0") == NULL)
    {
        printf_uart6("[WARN] CMQTTSTART not OK, maybe already started.\r\n");
    }
    HAL_Delay(300);

    // 2) ACCQ client
    AT_ClearRing(); UART1_ClearRXNE();
    printf_uart1("AT+CMQTTACCQ=0,\"" MQTT_DEVICE_ID "\",0\r\n");
    AT_ReadAllToBuffer_Timeout(3000, 200, 1);
    printf_uart6("%s", AT_rx_buffer);
    if (strstr(AT_rx_buffer, "OK") == NULL)
    {
        printf_uart6("[ERR] CMQTTACCQ failed.\r\n");
        return -2;
    }
    HAL_Delay(300);

    // 3) CFG argtopic
    AT_ClearRing(); UART1_ClearRXNE();
    printf_uart1("AT+CMQTTCFG=\"argtopic\",0,1,1\r\n");
    AT_ReadAllToBuffer_Timeout(2000, 150, 1);
    printf_uart6("%s", AT_rx_buffer);
    if (strstr(AT_rx_buffer, "OK") == NULL)
    {
        printf_uart6("[ERR] CMQTTCFG failed.\r\n");
        return -3;
    }
    HAL_Delay(300);

    // 4) CONNECT
    AT_ClearRing(); UART1_ClearRXNE();
    printf_uart1("AT+CMQTTCONNECT=0,\"tcp://47.104.246.138:1883\",60,1,\"tiantong\",\"yuandu@2022##\"\r\n");

    // CONNECT 可能有 URC 延迟/穿插，给足时间
    AT_ReadAllToBuffer_Timeout(12000, 400, 1);
    printf_uart6("%s", AT_rx_buffer);

    if (strstr(AT_rx_buffer, "+CMQTTCONNECT: 0,0") == NULL)
    {
        printf_uart6("[ERR] CMQTTCONNECT failed.\r\n");
        return -4;
    }

    printf_uart6("[OK] MQTT CONNECTED.\r\n");
    printf_uart6("============================================\r\n");
    return 0;
}


/*******************************************************
 * @brief 通过 CMQTTPUB 发布一条 JSON payload（原始风格）
 * @return 0 成功，-1 参数错误，-2 未收到'>'提示符，-3 发布失败
 *
 * 关键增强：
 *  - PUB 前/看到 > 后硬清 ring
 *  - 没看到 > 时做 resync，不立刻放锁，避免后续 AT 被吞成 payload
 *  - payload 统一阻塞 TX + 等 TC
 *****************************************************/
int MQTT_Publish_raw(const char *topic, const char *payload)
{
    mqtt_publishing = 1;

    if (!topic || !payload)
    {
        mqtt_publishing = 0;
        return -1;
    }

    int len = (int)strlen(payload);
    char cmd[256];

    snprintf(cmd, sizeof(cmd),
             "AT+CMQTTPUB=0,\"%s\",0,%d\r\n", topic, len);

    printf_uart6("\r\n========== MQTT_Publish_raw ==========\r\n");
    printf_uart6("[TOPIC] %s\r\n", topic);
    printf_uart6("[LEN  ] %d\r\n", len);
    printf_uart6("[PAYLD] %s\r\n", payload);

    /* ---------- 1) PUB 命令前硬清 ring + 清 RXNE ---------- */
    AT_ClearRing();
    UART1_ClearRXNE();

    printf_uart1("%s", cmd);

    /* ---------- 2) 等待 '>' 或直接等待发布结果 ---------- */
    uint32_t t0 = HAL_GetTick();
    int got_prompt = 0;   // 是否看到 '>'
    int got_result = 0;   // 是否直接看到 +CMQTTPUB:0,0
    int got_error  = 0;

    AT_rx_buffer[0] = 0;

    while ((HAL_GetTick() - t0) < 10000) // 累计等 10s
    {
        // 小步读取，避免 200ms 空闲提前结束错过 prompt/URC
        AT_ReadAllToBuffer_Timeout(500, 120, 0);

        if (strchr(AT_rx_buffer, '>') != NULL)
        {
            got_prompt = 1;
            break;  // 进入数据窗口，继续发 payload
        }

        // ★ 关键：有时模块不吐 '>'，但会直接给结果 URC
        if (strstr(AT_rx_buffer, "+CMQTTPUB: 0,0") != NULL)
        {
            got_result = 1;
            break;  // 认为已经成功 publish，不再发 payload
        }

        if (strstr(AT_rx_buffer, "ERROR") != NULL ||
            strstr(AT_rx_buffer, "+CMQTTPUB: 0,") != NULL) // 非0错误码也算失败
        {
            got_error = 1;
            break;
        }
    }

    if (got_result)
    {
        printf_uart6("[OK] PUB SUCCESS (no prompt path).\r\n");
        printf_uart6("======================================\r\n");
        mqtt_publishing = 0;
        return 0;
    }

    if (!got_prompt)
    {
        printf_uart6("[ERR] No '>' prompt, true fail.\r\n");

        // 真失败才复位重连
        MQTT_ResetAndReconnect();

        mqtt_publishing = 0;
        return -2;
    }

    /* ---------- 3) 进入数据窗口前再硬清一次 ---------- */
    AT_ClearRing();
    UART1_ClearRXNE();

    /* ---------- 4) payload 阻塞一口气发 + 等 TC ---------- */
    HAL_UART_Transmit(&huart1, (uint8_t *)payload, len, 5000);
    while(!(USART1->SR & USART_SR_TC));  // 最后一个字节也出线

    /* ---------- 5) 等发布结果 ---------- */
    AT_ReadAllToBuffer_Timeout(8000, 500, 1);
    printf_uart6("%s", AT_rx_buffer);

    if (strstr(AT_rx_buffer, "+CMQTTPUB: 0,0") == NULL)
    {
        printf_uart6("[ERR] CMQTTPUB failed.\r\n");
        mqtt_publishing = 0;
        return -3;
    }

    printf_uart6("[OK] PUB SUCCESS.\r\n");
    printf_uart6("======================================\r\n");

    mqtt_publishing = 0;
    return 0;
}


/*******************************************************
 * @brief 清空环形缓冲区（硬清）
 *****************************************************/
static inline void AT_ClearRing(void)
{
    __disable_irq();
    AT_RxRead = 0;
    AT_RxWrite = 0;
    __enable_irq();
}

/*******************************************************
 * @brief 清 RXNE（把 DR 里残留字节读空）
 *****************************************************/
static inline void UART1_ClearRXNE(void)
{
    volatile uint32_t tmp;
    while (USART1->SR & USART_SR_RXNE)
    {
        tmp = USART1->DR;
        (void)tmp;
    }
}

/*******************************************************
 * @brief 在 total_ms 内等待 prompt（比如 ">"）
 *        期间允许 URC/回显穿插，不会因为 200ms 空闲就提前错过
 * @return 0 找到 prompt；非0 超时
 *****************************************************/
static int AT_WaitForPromptOrUrc(const char *prompt, uint32_t total_ms)
{
    uint32_t t0 = HAL_GetTick();
    AT_rx_buffer[0] = 0;

    while ((HAL_GetTick() - t0) < total_ms)
    {
        // 每次小步读一段，避免错过迟到 '>'
        AT_ReadAllToBuffer_Timeout(400, 80, 0);

        if (strstr(AT_rx_buffer, prompt) != NULL)
        {
            return 0;
        }
    }
    return -1;
}

/**
 * @brief 真失败时才调用：释放/停止 MQTT 客户端并重连
 * @return 0 重连成功，非0失败
 */
int MQTT_ResetAndReconnect(void)
{
    printf_uart6("\r\n========== MQTT_ResetAndReconnect ==========\r\n");

    // 先尽量把残留 URC 吃干净
    AT_ReadAllToBuffer_Timeout(1500, 300, 1);

    // 1) DISC 断开
    AT_ClearRing(); UART1_ClearRXNE();
    printf_uart1("AT+CMQTTDISC=0,60\r\n");
    AT_ReadAllToBuffer_Timeout(6000, 300, 1);
    printf_uart6("%s", AT_rx_buffer);

    // 2) REL 释放 client（关键！不 REL 会导致 ACCQ failed）
    AT_ClearRing(); UART1_ClearRXNE();
    printf_uart1("AT+CMQTTREL=0\r\n");
    AT_ReadAllToBuffer_Timeout(3000, 200, 1);
    printf_uart6("%s", AT_rx_buffer);

    // 3) STOP 停服务
    AT_ClearRing(); UART1_ClearRXNE();
    printf_uart1("AT+CMQTTSTOP\r\n");
    AT_ReadAllToBuffer_Timeout(3000, 200, 1);
    printf_uart6("%s", AT_rx_buffer);

    HAL_Delay(300);

    // 4) 重新 START->ACCQ->CONNECT
    if (MQTT_InitAndConnect_raw() != 0)
    {
        printf_uart6("[ERR] MQTT reinit failed.\r\n");
        return -1;
    }

    printf_uart6("[OK] MQTT reconnected.\r\n");
    printf_uart6("============================================\r\n");
    return 0;
}