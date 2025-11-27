#ifndef __PAYLOAD_H__
#define __PAYLOAD_H__

#include "cJSON.h"     // cJSON 库

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
);
int MQTT_InitAndConnect_raw(void);
int MQTT_Publish_raw(const char *topic, const char *payload);
#endif /* __PAYLOAD_H__ */