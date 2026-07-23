#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目标定结果验证工具

读取 stereo_calib.npz, 对图像对做立体校正, 拼接并画水平线检查极线对齐.

用法:
    python3 verify_calibration.py
    python3 verify_calibration.py --index 5 --save
"""
import os
import argparse
import numpy as np
import cv2


def parse_args():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="Stereo calibration verifier")
    ap.add_argument(
        "--calib", type=str,
        default=os.path.join(here, "output", "stereo_calib.npz"),
        help="标定结果 npz 文件路径"
    )
    ap.add_argument(
        "--image-dir", type=str,
        default=os.path.join(here, "images"),
        help="待验证图像目录"
    )
    ap.add_argument(
        "--index", type=int, default=0,
        help="验证用图像对序号, 默认 0 号"
    )
    ap.add_argument(
        "--save", action="store_true",
        help="保存校正拼接图到 output/rectified_<index>.png"
    )
    return ap.parse_args()


def main():
    args = parse_args()

    if not os.path.isfile(args.calib):
        print(f"❌ 未找到标定文件: {args.calib}")
        print("   请先运行 stereo_calibrate.py")
        return

    calib = np.load(args.calib)
    K_left = calib["K_left"]
    D_left = calib["D_left"]
    K_right = calib["K_right"]
    D_right = calib["D_right"]
    R = calib["R_stereo"]
    T = calib["T_stereo"]
    img_size = tuple(int(x) for x in calib["image_size"])  # (w, h)

    print("=" * 60)
    print("标定结果摘要")
    print("=" * 60)
    print(f"  图像尺寸:       {img_size}")
    print(f"  基线长度:       {float(calib['baseline']):.4f} m")
    print(f"  光轴夹角:       {float(calib['axis_angle_deg']):.2f}°")
    print(f"  重投影误差:     {float(calib['reprojection_error']):.4f} px")
    print(f"  参与标定图像:   {int(calib['n_pairs'])} 对")

    # 加载验证图像
    left_path = os.path.join(args.image_dir, f"left_{args.index:03d}.png")
    right_path = os.path.join(args.image_dir, f"right_{args.index:03d}.png")
    if not (os.path.isfile(left_path) and os.path.isfile(right_path)):
        print(f"\n❌ 未找到验证图: {left_path}")
        return

    img_l = cv2.imread(left_path)
    img_r = cv2.imread(right_path)

    # 立体校正
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_left, D_left, K_right, D_right, img_size,
        R, T, alpha=0
    )
    map_lx, map_ly = cv2.initUndistortRectifyMap(
        K_left, D_left, R1, P1, img_size, cv2.CV_32FC1
    )
    map_rx, map_ry = cv2.initUndistortRectifyMap(
        K_right, D_right, R2, P2, img_size, cv2.CV_32FC1
    )

    rect_l = cv2.remap(img_l, map_lx, map_ly, cv2.INTER_LINEAR)
    rect_r = cv2.remap(img_r, map_rx, map_ry, cv2.INTER_LINEAR)

    # 拼接 + 画水平极线
    combined = cv2.hconcat([rect_l, rect_r])
    h, w = rect_l.shape[:2]
    step = h // 20
    for y in range(step, h, step):
        cv2.line(combined, (0, y), (w * 2, y), (0, 255, 0), 1)

    # 缩放显示
    scale = min(1.0, 1600.0 / combined.shape[1])
    disp = cv2.resize(combined, (0, 0), fx=scale, fy=scale)

    print(f"\n校正后左右图已拼接, 同一物体应落在同一水平线上.")
    print("  按任意键关闭...")
    cv2.imshow(f"rectified pair #{args.index:03d}", disp)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    if args.save:
        out_dir = os.path.dirname(args.calib)
        out_path = os.path.join(out_dir, f"rectified_{args.index:03d}.png")
        cv2.imwrite(out_path, combined)
        print(f"✅ 校正拼接图已保存: {out_path}")


if __name__ == "__main__":
    main()
