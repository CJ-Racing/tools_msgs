"""
传感器坐标系 -> 车体坐标系 
调用方在外参标定时将坐标轴对齐算入 R
"""

import numpy as np


def build_rotation_matrix(yaw: float, pitch: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """构建旋转矩阵（ZYX 欧拉角，单位：弧度）

    只做 RPY -> 旋转矩阵，不预设任何坐标系轴方向。
    调用方在外参标定时将轴对齐（如相机 X右Y下Z前 -> 车体 X前Y左Z上）算进外参。

    Args:
        yaw:   绕 Z 轴旋转角（弧度）
        pitch: 绕 Y 轴旋转角（弧度），默认 0
        roll:  绕 X 轴旋转角（弧度），默认 0

    Returns:
        3x3 旋转矩阵 (np.ndarray, float64)
    """
    cy, sy = np.cos(yaw),   np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll),  np.sin(roll)

    R_z = np.array([
        [ cy, -sy,  0],
        [ sy,  cy,  0],
        [  0,   0,  1]
    ], dtype=np.float64)

    R_y = np.array([
        [ cp,  0,  sp],
        [  0,  1,   0],
        [-sp,  0,  cp]
    ], dtype=np.float64)

    R_x = np.array([
        [1,   0,   0],
        [0,  cr, -sr],
        [0,  sr,  cr]
    ], dtype=np.float64)

    return R_z @ R_y @ R_x


def sensor_to_vehicle(point: np.ndarray, R: np.ndarray, T: np.ndarray) -> np.ndarray:
    """将传感器坐标系下的单个 3D 点变换到车体坐标系

    Args:
        point: shape (3,)，传感器坐标系下的 3D 点
        R:     3x3 旋转矩阵，传感器到车体的旋转
        T:     shape (3,)，传感器在车体坐标系中的安装位置

    Returns:
        shape (3,)，车体坐标系 3D 点
    """
    return R @ point + T


def batch_sensor_to_vehicle(points: np.ndarray, R: np.ndarray, T: np.ndarray) -> np.ndarray:
    """将传感器坐标系下的集群 3D 点变换到车体坐标系

    Args:
        points: shape (N, 3)，传感器坐标系下的 N 个 3D 点
        R:      3x3 旋转矩阵，传感器到车体的旋转
        T:      shape (3,)，传感器在车体坐标系中的安装位置

    Returns:
        shape (N, 3)，车体坐标系 3D 点
    """
    return (R @ points.T).T + T


def monocular_depth(pixel_height: float, fy: float, real_height: float = 0.3) -> float:
    """单目用 基于目标高度先验估算单目深度

    Args:
        pixel_height: 检测框像素高度（px）
        fy:           相机纵向焦距（px）
        real_height:  目标实际高度（米），默认 0.3（锥桶高度）

    Returns:
        估算深度 Z（米）
    """
    return fy * real_height / pixel_height


def pixel_to_cam_3d(u: float, v: float, depth: float, K: np.ndarray) -> np.ndarray:
    """像素坐标 + 深度 -> 相机坐标系 3D 点，相机使用

    Args:
        u, v:  去畸变后的像素坐标
        depth: 深度 Z（米）
        K:     3x3 相机内参矩阵

    Returns:
        shape (3,)，相机坐标系 3D 点 [X, Y, Z]
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    return np.array([   (u - cx) * depth / fx,
                        (v - cy) * depth / fy,
                        depth
                                ], dtype=np.float64)
