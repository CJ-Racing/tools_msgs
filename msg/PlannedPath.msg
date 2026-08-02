Header header                        # 时间戳 + 坐标系 (world)
geometry_msgs/Point[] points         # 路径点 (等距, 默认 0.1m)
float32[] curvatures                 # 各点曲率 1/m (左转正, 右转负)
float32[] velocities                 # 各点目标速度 m/s

# 三数组一一对应且等长, 打包发布保证 control 取到同帧的路径+速度。
# 分开发 /planned_path 与 /planned_velocity 时, control 按索引取速度
# 无法保证同帧, 路径更新而速度滞后一帧就会错位。
