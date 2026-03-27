#!/usr/bin/env python3
import math


def yaw_deg_to_rad(yaw_deg):
    """航向角转换：正北顺时针（A23 chcnav 格式）→ 数学弧度
    Args:
        yaw_deg: 正北顺时针 0-360°
        
    Returns:
        yaw_rad: 数学弧度（东向逆时针）
    """
    math_angle_deg = 90.0 - yaw_deg
    if math_angle_deg > 180:
        math_angle_deg -= 360
    elif math_angle_deg < -180:
        math_angle_deg += 360
    return math.radians(math_angle_deg)


def vehicle_to_world(cone_veh_x, cone_veh_y, veh_world_x, veh_world_y, veh_yaw_rad):
    """车辆坐标系 → 世界坐标系（ENU）
    
    Args:
        cone_veh_x, cone_veh_y: 锥桶在车辆坐标系中的位置（米）
        veh_world_x, veh_world_y: 车辆在世界坐标系中的位置（米）
        veh_yaw_rad: 车辆航向角（数学弧度）
        
    Returns:
        (cone_world_x, cone_world_y): 锥桶世界坐标（米）
    """
    cos_yaw = math.cos(veh_yaw_rad)
    sin_yaw = math.sin(veh_yaw_rad)

    cone_world_x = veh_world_x + cone_veh_x * cos_yaw - cone_veh_y * sin_yaw
    cone_world_y = veh_world_y + cone_veh_x * sin_yaw + cone_veh_y * cos_yaw

    return cone_world_x, cone_world_y


def world_to_vehicle(cone_world_x, cone_world_y, veh_world_x, veh_world_y, veh_yaw_rad):
    """世界坐标系 → 车辆坐标系（world_cone.py 未实现，A26 新增）
    
    Args:
        cone_world_x, cone_world_y: 锥桶世界坐标（米）
        veh_world_x, veh_world_y: 车辆世界坐标（米）
        veh_yaw_rad: 车辆航向角（数学弧度）
        
    Returns:
        (cone_veh_x, cone_veh_y): 锥桶车辆坐标（米）
    """
    dx = cone_world_x - veh_world_x
    dy = cone_world_y - veh_world_y

    cos_yaw = math.cos(-veh_yaw_rad)
    sin_yaw = math.sin(-veh_yaw_rad)

    cone_veh_x = dx * cos_yaw - dy * sin_yaw
    cone_veh_y = dx * sin_yaw + dy * cos_yaw

    return cone_veh_x, cone_veh_y


def batch_vehicle_to_world(cones_veh, veh_world_x, veh_world_y, veh_yaw_rad):
    """批量转换
    
    Args:
        cones_veh: [(x, y, color), ...] 车辆坐标系锥桶列表
        veh_world_x, veh_world_y: 车辆世界坐标
        veh_yaw_rad: 车辆航向角（数学弧度）
        
    Returns:
        [(x, y, color), ...] 世界坐标系锥桶列表
    """
    result = []
    for veh_x, veh_y, color in cones_veh:
        wx, wy = vehicle_to_world(veh_x, veh_y, veh_world_x, veh_world_y, veh_yaw_rad)
        result.append((wx, wy, color))
    return result


# 使用示例
if __name__ == '__main__':
    # 模拟 A23 chcnav 导航数据
    # 车辆在世界坐标系位置（ENU）
    veh_x = 100.0   # 东向 100m
    veh_y = 200.0   # 北向 200m
    yaw_deg = 45.0  # 正北顺时针 45°（东偏北）

    # 转换航向角
    yaw_rad = yaw_deg_to_rad(yaw_deg)
    print(f"航向角：{yaw_deg}°(顺时针) → {math.degrees(yaw_rad):.2f}°(数学弧度)")

    # 模拟锥桶在车辆坐标系中的位置（车前方 5m，右侧 1m）
    cones_veh = [
        (5.0,  1.0, "blue_cone"),
        (5.0, -1.0, "yellow_cone"),
        (8.0,  1.0, "blue_cone"),
        (8.0, -1.0, "yellow_cone"),
    ]

    # 批量转换到世界坐标系
    cones_world = batch_vehicle_to_world(cones_veh, veh_x, veh_y, yaw_rad)

    print("\n锥桶坐标转换结果：")
    print(f"{'车辆坐标 (x, y)':<25} {'世界坐标 (x, y)':<25} 颜色")
    print("-" * 70)
    for (vx, vy, color), (wx, wy, _) in zip(cones_veh, cones_world):
        print(f"({vx:.1f}, {vy:.1f}){'':<17} ({wx:.2f}, {wy:.2f}){'':<12} {color}")

    # 反向验证
    print("\n反向转换验证：")
    for wx, wy, color in cones_world:
        vx, vy = world_to_vehicle(wx, wy, veh_x, veh_y, yaw_rad)
        print(f"世界({wx:.2f}, {wy:.2f}) → 车辆({vx:.2f}, {vy:.2f}) ✓")
