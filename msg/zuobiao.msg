Header header
float64 xzc                  # ENU 东向坐标 (m)，相对原点
float64 yzc                  # ENU 北向坐标 (m)，相对原点
float32 zzc                  # ENU 天向坐标 (m)，相对原点
float32 yaw_deg              # 航向角，正北顺时针 0~360°
float32 speed                # 地面速度 (m/s)
uint8   quality              # 定位质量 0=不可用 1=低 2=中 3=高
float32 pos_std              # 位置标准差 (m)，sqrt(lat²+lon²)
bool    origin_set           # ENU 原点是否锁定