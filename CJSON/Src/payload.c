#include "payload.h"
#include "cJSON.h"

/**
 * @brief 组装 WGS84 + 姿态 的 JSON payload
 * @param tid           设备 TID（字符串，必填）
 * @param bid           业务 BID（字符串，必填）
 * @param timestamp_ms  时间戳（毫秒，必填）
 * @param gateway       网关/设备标识（字符串，必填）
 * @param lat_wgs       WGS84 纬度（十进制度）
 * @param lon_wgs       WGS84 经度（十进制度）
 * @param height_m      高度/海拔（米）
 * @param hspeed_knots  水平速度（knots，地面速度）
 * @param roll_deg      横滚角 roll（度）
 * @param pitch_deg     俯仰角 pitch（度）
 * @param yaw_deg       航向/偏航 yaw/head（度）
 * @param battery_pct   电量百分比（0~100），没有就传 -1
 * @param gps_num       卫星数，没有就传 -1
 * @return malloc 出来的 JSON 字符串，调用者用完必须 free()
 */
char* BuildPayload_WGS84_Attitude(
                                    const char *tid,           // 设备唯一标识符，只读参数
                                    const char *bid,           // 业务标识符，只读参数
                                    long long timestamp_ms,    // 数据采集时间戳（毫秒级）
                                    const char *gateway,       // 网关设备标识，只读参数
                                    double lat_wgs,            // WGS84坐标系纬度（度）
                                    double lon_wgs,            // WGS84坐标系经度（度）
                                    double height_m,           // 海拔高度（米）
                                    double hspeed_knots,       // 水平速度（海里/小时）
                                    double roll_deg,           // 横滚角（绕X轴旋转，度）
                                    double pitch_deg,          // 俯仰角（绕Y轴旋转，度）
                                    double yaw_deg,            // 偏航角（绕Z轴旋转，度）
                                    int battery_pct,           // 电池电量百分比（0-100%）
                                    int gps_num                // GPS可见卫星数量
)
{
    cJSON *root = cJSON_CreateObject();
    if(!root) return NULL;
    
    // --- 外层 ---
    cJSON_AddStringToObject(root, "tid", tid ? tid : "");      // 添加设备TID，NULL保护
    cJSON_AddStringToObject(root, "bid", bid ? bid : "");      // 添加业务BID，NULL保护
    cJSON_AddNumberToObject(root, "timestamp", (double)timestamp_ms); // 添加时间戳，转为double类型
    cJSON_AddStringToObject(root, "gateway", gateway ? gateway : ""); // 添加网关标识，NULL保护

    // --- data 子对象 ---
    cJSON *data = cJSON_CreateObject();
    if(!data) return NULL;
    cJSON_AddItemToObject(root, "data", data);

        cJSON_AddNumberToObject(data,"latitude",lat_wgs);          // 纬度
        cJSON_AddNumberToObject(data,"longitude",lon_wgs);         // 经度
        cJSON_AddNumberToObject(data, "height",    height_m);
        cJSON_AddNumberToObject(data, "horizontal_speed", hspeed_knots);
        cJSON_AddNumberToObject(data, "attitude_roll",  roll_deg);
        cJSON_AddNumberToObject(data, "attitude_pitch", pitch_deg);
        cJSON_AddNumberToObject(data, "attitude_head",  yaw_deg);

        // --- data 里协议必填但现在没有的：先填 0 占位 ---
        cJSON_AddNumberToObject(data, "mode_code", 0);
        cJSON_AddNumberToObject(data, "vertical_speed", 0);
        cJSON_AddNumberToObject(data, "home_longitude", 0);
        cJSON_AddNumberToObject(data, "home_latitude", 0);
    
        //battery
        cJSON *battery = cJSON_CreateObject();
        if(!battery) return NULL;
        cJSON_AddItemToObject(data, "battery", battery);
            if(battery_pct >= 0)
                cJSON_AddNumberToObject(battery, "percentage", battery_pct);
            else
                cJSON_AddNumberToObject(battery, "percentage", 0); // 占位0
        
        //gps
        if (gps_num >= 0) 
        {
        cJSON *pos = cJSON_CreateObject();
        cJSON_AddItemToObject(data, "position_state", pos);
        cJSON_AddNumberToObject(pos, "gps_number", gps_num);
        }

    // 生成紧凑 JSON（malloc 内存）
    char *payload = cJSON_PrintUnformatted(root);

    // 释放 cJSON 树
    cJSON_Delete(root);

    // payload 由调用者 free
    return payload;

}