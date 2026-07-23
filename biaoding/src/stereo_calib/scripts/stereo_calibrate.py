#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目鱼眼相机联合标定 (cv2.fisheye + stereoCalibrate)

输入: images/left_*.png + right_*.png (由 capture_stereo_pairs.py 采集)
输出: output/stereo_calib.npz, output/stereo_calib.yaml

用法:
    python3 stereo_calibrate.py --pattern 8 6 --square 0.070
    python3 stereo_calibrate.py --pattern 11 8 --square 0.030 --image-dir ./new_images
"""
import os
import re
import sys
import glob
import argparse
import numpy as np
import cv2


def parse_args():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(
        description="Fisheye stereo calibration via cv2.fisheye.stereoCalibrate"
    )
    ap.add_argument(
        "--pattern", type=int, nargs=2, default=[8, 6],
        metavar=("COLS", "ROWS"),
        help="棋盘格内角点数 (cols rows). 默认 8 6 (对应方格 9×7)"
    )
    ap.add_argument(
        "--square", type=float, default=0.070,
        help="棋盘格方格边长(米). 默认 0.070 = 70mm"
    )
    ap.add_argument(
        "--image-dir", type=str, default=os.path.join(here, "images"),
        help="图像对所在目录"
    )
    ap.add_argument(
        "--output-dir", type=str, default=os.path.join(here, "output"),
        help="标定结果输出目录"
    )
    ap.add_argument(
        "--show", action="store_true",
        help="显示每张图的角点检测结果"
    )
    return ap.parse_args()


def build_object_points(pattern, square_size):
    """构造标定板在其自身坐标系下的 3D 角点 (fisheye要求 Nx1x3)"""
    objp = np.zeros((1, pattern[0] * pattern[1], 3), np.float64)
    objp[0, :, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp


def find_corners_all(image_dir, pattern, show=False):
    """遍历所有图像对, 提取角点 (返回 fisheye 所需 Nx1x2 格式)"""
    left_files = sorted(glob.glob(os.path.join(image_dir, "left_*.png")))
    if not left_files:
        # 尝试 jpg 格式
        left_files = sorted(glob.glob(os.path.join(image_dir, "left_*.jpg")))
    if not left_files:
        raise RuntimeError(f"未找到图像: {image_dir}/left_*.png 或 left_*.jpg")

    subpix_criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001
    )

    imgpts_l = []
    imgpts_r = []
    used_left_files = []
    img_size = None

    for lf in left_files:
        # 匹配对应的右图
        rf = lf.replace(os.sep + "left_", os.sep + "right_")
        if not os.path.isfile(rf):
            print(f"  [SKIP] 缺少右图: {rf}")
            continue

        img_l = cv2.imread(lf)
        img_r = cv2.imread(rf)
        if img_l is None or img_r is None:
            print(f"  [SKIP] 读取失败: {lf}")
            continue

        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

        if img_size is None:
            img_size = (gray_l.shape[1], gray_l.shape[0])  # (w, h)

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ok_l, corners_l = cv2.findChessboardCorners(gray_l, tuple(pattern), flags)
        ok_r, corners_r = cv2.findChessboardCorners(gray_r, tuple(pattern), flags)

        if ok_l and ok_r:
            corners_l = cv2.cornerSubPix(
                gray_l, corners_l, (11, 11), (-1, -1), subpix_criteria
            )
            corners_r = cv2.cornerSubPix(
                gray_r, corners_r, (11, 11), (-1, -1), subpix_criteria
            )
            # fisheye 要求 1xNx2 float64 格式
            imgpts_l.append(corners_l.reshape(1, -1, 2).astype(np.float64))
            imgpts_r.append(corners_r.reshape(1, -1, 2).astype(np.float64))
            used_left_files.append(lf)
            print(f"  [OK]   {os.path.basename(lf)}")

            if show:
                vis_l = img_l.copy()
                vis_r = img_r.copy()
                cv2.drawChessboardCorners(vis_l, tuple(pattern), corners_l, True)
                cv2.drawChessboardCorners(vis_r, tuple(pattern), corners_r, True)
                scale = 0.5
                vis_l = cv2.resize(vis_l, (0, 0), fx=scale, fy=scale)
                vis_r = cv2.resize(vis_r, (0, 0), fx=scale, fy=scale)
                cv2.imshow("corners", cv2.hconcat([vis_l, vis_r]))
                cv2.waitKey(300)
        else:
            print(f"  [SKIP] 角点提取失败: {os.path.basename(lf)}")

    if show:
        cv2.destroyAllWindows()

    return imgpts_l, imgpts_r, used_left_files, img_size


def skew(v):
    """向量 v 的反对称矩阵 [v]_x"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ], dtype=np.float64)


def compute_fundamental(K_left, K_right, R, T):
    """从 K, R, T 计算基础矩阵 F: x_r^T @ F @ x_l = 0"""
    T_flat = T.flatten()
    T_x = skew(T_flat)
    E = T_x @ R  # Essential matrix
    F = np.linalg.inv(K_right).T @ E @ np.linalg.inv(K_left)
    # 归一化使 ||F||_F = 1
    F = F / np.linalg.norm(F)
    return E, F


def save_yaml(path, data):
    """保存为人类可读的 YAML 格式 (OpenCV FileStorage)"""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    for k, v in data.items():
        if isinstance(v, (int, float)):
            fs.write(k, v)
        elif isinstance(v, np.ndarray):
            fs.write(k, v)
        else:
            fs.write(k, v)
    fs.release()


def main():
    args = parse_args()
    pattern = tuple(args.pattern)

    print("=" * 60)
    print("双目鱼眼联合标定 (cv2.fisheye)")
    print(f"  图像目录: {args.image_dir}")
    print(f"  输出目录: {args.output_dir}")
    print(f"  棋盘格:   {pattern[0]}x{pattern[1]} 内角点, {args.square*1000:.1f}mm")
    print("=" * 60)

    # 1. 角点提取
    print("\n[1/3] 遍历图像对, 提取角点...")
    imgpts_l, imgpts_r, used_files, img_size = find_corners_all(
        args.image_dir, pattern, show=args.show
    )
    n = len(imgpts_l)
    print(f"\n  有效图像对: {n} (建议 >= 20)")
    if n < 10:
        print("  ❌ 有效图像对过少, 请重新采集")
        sys.exit(1)
    print(f"  图像尺寸:   {img_size[0]} x {img_size[1]}")

    # 2. 鱼眼单目标定 (带坏图剔除)
    objp = build_object_points(pattern, args.square)
    objpts = [objp for _ in range(n)]

    print("\n[2/3] 鱼眼单目标定 (自动剔除病态图像)...")

    # 策略: 用 CALIB_CHECK_COND 先尝试, 逐个剔除失败的图像对
    def fisheye_calibrate_robust(objpts_list, imgpts_list, img_size, label=""):
        """Iteratively remove ill-conditioned images until calibration succeeds"""
        K = np.zeros((3, 3), dtype=np.float64)
        D = np.zeros((4, 1), dtype=np.float64)
        good_indices = list(range(len(objpts_list)))
        calib_criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6
        )

        max_remove = min(10, len(good_indices) // 3)  # 最多剔除 1/3
        removed = []

        for attempt in range(max_remove + 1):
            obj_sub = [objpts_list[i] for i in good_indices]
            img_sub = [imgpts_list[i] for i in good_indices]
            K = np.zeros((3, 3), dtype=np.float64)
            D = np.zeros((4, 1), dtype=np.float64)
            try:
                flags = (
                    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC +
                    cv2.fisheye.CALIB_CHECK_COND +
                    cv2.fisheye.CALIB_FIX_SKEW
                )
                ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                    obj_sub, img_sub, img_size, K, D,
                    flags=flags, criteria=calib_criteria
                )
                return ret, K, D, good_indices, removed
            except cv2.error as e:
                msg = str(e)
                m = re.search(r'input array (\d+)', msg)
                if m:
                    bad_local_idx = int(m.group(1))
                    bad_global_idx = good_indices[bad_local_idx]
                    good_indices.remove(bad_global_idx)
                    removed.append(bad_global_idx)
                    print(f"    [{label}] 剔除病态图像 #{bad_global_idx}")
                else:
                    # 无法解析, 回退到无检查模式
                    break

        # 回退: 去掉 CALIB_CHECK_COND
        print(f"    [{label}] 回退到无条件数检查模式")
        obj_sub = [objpts_list[i] for i in good_indices]
        img_sub = [imgpts_list[i] for i in good_indices]
        K = np.zeros((3, 3), dtype=np.float64)
        D = np.zeros((4, 1), dtype=np.float64)
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
        ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            obj_sub, img_sub, img_size, K, D,
            flags=flags, criteria=calib_criteria
        )
        return ret, K, D, good_indices, removed

    ret_l, K_left, D_left, good_l, removed_l = fisheye_calibrate_robust(
        objpts, imgpts_l, img_size, "左"
    )
    ret_r, K_right, D_right, good_r, removed_r = fisheye_calibrate_robust(
        objpts, imgpts_r, img_size, "右"
    )

    # 取左右都通过的图像对交集
    good_both = sorted(set(good_l) & set(good_r))
    removed_all = sorted(set(removed_l) | set(removed_r))
    if removed_all:
        print(f"\n  剔除的图像对索引: {removed_all}")
    print(f"  用于双目标定的有效图像对: {len(good_both)} (原{n}对)")
    print(f"  左相机重投影误差: {ret_l:.4f} px")
    print(f"  右相机重投影误差: {ret_r:.4f} px")

    if len(good_both) < 10:
        print("  ❌ 有效图像对不足 10, 请重新采集")
        sys.exit(1)

    # 用交集进入双目标定
    objpts_clean = [objpts[i] for i in good_both]
    imgpts_l_clean = [imgpts_l[i] for i in good_both]
    imgpts_r_clean = [imgpts_r[i] for i in good_both]
    n_clean = len(good_both)

    # 3. 双目联合标定: 鱼眼去畸变→归一化坐标→标准 stereoCalibrate
    # (规避 OpenCV 4.2 fisheye.stereoCalibrate 的 abs_max assertion bug)
    print("\n[3/3] 双目联合标定 (去畸变 + 标准stereoCalibrate)...")
    imgpts_l_norm = []
    imgpts_r_norm = []
    for i in range(n_clean):
        pts_l = imgpts_l_clean[i].reshape(-1, 1, 2).astype(np.float64)
        pts_r = imgpts_r_clean[i].reshape(-1, 1, 2).astype(np.float64)
        undist_l = cv2.fisheye.undistortPoints(pts_l, K_left, D_left)
        undist_r = cv2.fisheye.undistortPoints(pts_r, K_right, D_right)
        imgpts_l_norm.append(undist_l.astype(np.float64))
        imgpts_r_norm.append(undist_r.astype(np.float64))

    # OpenCV 4.2 stereoCalibrate 要求 Point3f/Point2f (float32)
    objpts_std = [o.reshape(-1, 1, 3).astype(np.float32) for o in objpts_clean]
    imgpts_l_norm = [p.reshape(-1, 1, 2).astype(np.float32) for p in imgpts_l_norm]
    imgpts_r_norm = [p.reshape(-1, 1, 2).astype(np.float32) for p in imgpts_r_norm]

    K_eye = np.eye(3, dtype=np.float64)
    D_zero = np.zeros((5, 1), dtype=np.float64)

    stereo_criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 1e-7
    )
    stereo_flags = cv2.CALIB_FIX_INTRINSIC

    ret, _, _, _, _, R, T, E_std, F_std = cv2.stereoCalibrate(
        objpts_std, imgpts_l_norm, imgpts_r_norm,
        K_eye, D_zero, K_eye, D_zero,
        img_size, criteria=stereo_criteria, flags=stereo_flags
    )
    avg_fx = 0.5 * (K_left[0, 0] + K_right[0, 0])
    ret_px = ret * avg_fx
    print(f"  双目标定成功! 使用 {n_clean} 对图像")
    print(f"  归一化坐标 RMS: {ret:.6f} (≈{ret_px:.2f} px)")

    E, F = compute_fundamental(K_left, K_right, R, T)

    baseline = float(np.linalg.norm(T))
    cos_theta = float(np.clip((np.trace(R) - 1) / 2, -1, 1))
    axis_angle_deg = float(np.degrees(np.arccos(cos_theta)))

    print("\n" + "=" * 60)
    print("标定结果")
    print("=" * 60)
    print(f"  双目重投影误差: {ret:.4f} px   (<0.5 很好, <1.0 可用, >1.5 重采)")
    print(f"  基线长度:       {baseline:.4f} m")
    print(f"  光轴夹角:       {axis_angle_deg:.2f}°  (八字安装期望 ≈60-70°)")
    print(f"\n  K_left:\n{K_left}")
    print(f"\n  D_left (4系数):  {D_left.flatten()}")
    print(f"\n  K_right:\n{K_right}")
    print(f"\n  D_right (4系数): {D_right.flatten()}")
    print(f"\n  R_stereo (左→右旋转):\n{R}")
    print(f"\n  T_stereo (左→右平移): {T.flatten()}")

    # 合理性检查
    print("\n" + "-" * 60)
    print("合理性检查:")
    if ret > 1.5:
        print(f"  ⚠️  重投影误差 {ret:.4f} px 偏高, 建议重新采集更高质量的图像对")
    else:
        print(f"  ✓ 重投影误差 {ret:.4f} px 在合理范围")

    if baseline < 0.05 or baseline > 1.0:
        print(f"  ⚠️  基线 {baseline:.4f} m 看起来不太合理 (期望 0.1~0.3m)")
    else:
        print(f"  ✓ 基线 {baseline:.4f} m 合理")

    if axis_angle_deg < 40 or axis_angle_deg > 90:
        print(f"  ⚠️  光轴夹角 {axis_angle_deg:.2f}° 偏离预期 (八字安装期望 60-70°)")
    else:
        print(f"  ✓ 光轴夹角 {axis_angle_deg:.2f}° 合理")

    # ===== 保存 =====
    os.makedirs(args.output_dir, exist_ok=True)

    npz_path = os.path.join(args.output_dir, "stereo_calib.npz")
    np.savez(
        npz_path,
        K_left=K_left, D_left=D_left,
        K_right=K_right, D_right=D_right,
        R_stereo=R, T_stereo=T,
        E=E, F=F,
        image_size=np.array(img_size),
        reprojection_error=float(ret),
        baseline=baseline,
        axis_angle_deg=axis_angle_deg,
        n_pairs=n
    )
    print(f"\n\u2705 NumPy 格式已保存: {npz_path}")

    yaml_path = os.path.join(args.output_dir, "stereo_calib.yaml")
    save_yaml(yaml_path, {
        "image_width": img_size[0],
        "image_height": img_size[1],
        "K_left": K_left,
        "D_left": D_left,
        "K_right": K_right,
        "D_right": D_right,
        "R_stereo": R,
        "T_stereo": T,
        "baseline_m": baseline,
        "axis_angle_deg": axis_angle_deg,
        "reprojection_error_px": float(ret),
        "n_pairs": n,
    })
    print(f"\u2705 YAML 格式已保存: {yaml_path}")


if __name__ == "__main__":
    main()
