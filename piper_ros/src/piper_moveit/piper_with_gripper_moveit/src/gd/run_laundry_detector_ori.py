#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
통합 세탁기/바구니 감지 스크립트
 - interactive  : 키보드(d/q)로 탐지·종료
 - batch(--json): 1회 탐지 후 JSON 결과 출력, 2 초 시각화, 프로세스 종료
author : Seung-yeon Cheon
date   : 2025-08-04
"""

import argparse, time, json, sys
import numpy as np
import cv2, pyrealsense2 as rs, open3d as o3d
import torch, torchvision.transforms as T
from PIL import Image


# ────────────────── 내부 모듈 ────────────────── #
from detector.rgb_classifier  import RGBOnlyClassifier
from detector.utils           import (get_current_pointcloud, load_and_crop_pcd,
                                      align_icp, to_open3d_pcd)
from detector.zscore_detector import (detect_anomalies_adaptive,
                                      cluster_and_get_targets,
                                      visualize_pointcloud_with_garments_open3d)

# ────────────────── 모드별 설정 ────────────────── #
CFG = {
    "wm": {  # 세탁기
        "ref_pcd"      : "reference/washer.pcd",
        "z_crop"       : (0.3, 0.95),
        "z_range"      : (0.47, 0.95),
        "need_clf"     : True,
        "eps"          : 0.037, "min_samples": 30
    },
    "basket": {  # 바구니
        "ref_pcd"      : "reference/basket.pcd",
        "z_crop"       : (0.0, 1.0),
        "z_range"      : (0.10, 0.60),
        "need_clf"     : False,
        "eps"          : 0.037, "min_samples": 30
    }
}

# ────────────────── RGB 분류기 ────────────────── #
def load_rgb_model(weight="models/rgb_only_best.pth"):
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model  = RGBOnlyClassifier().to(device)
    model.load_state_dict(torch.load(weight, map_location=device))
    model.eval()
    tf = T.Compose([
        T.Resize((224, 224)), T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406],
                    [0.229, 0.224, 0.225])
    ])
    return model, tf, device

def predict_rgb(model, tf, device, frame):
    img  = np.asanyarray(frame.get_data())
    x    = tf(Image.fromarray(img[..., ::-1])).unsqueeze(0).to(device)
    pred = model(x).argmax(1).item()   # 1: CLOTH / 0: EMPTY
    return pred, img

# ────────────────── 단일 탐지 함수 ────────────────── #
def detect_once(pipeline, cfg, ref_pts, aabb=None, rgb_pack=None):
    """한 번의 포인트클라우드 감지 실행 → centers(list[np.ndarray]), success(bool)"""
    # (A) RGB 분류기로 빈 드럼이면 바로 종료
    if cfg["need_clf"]:
        model, tf, device = rgb_pack
        color_frame = pipeline.wait_for_frames().get_color_frame()
        pred, _ = predict_rgb(model, tf, device, color_frame)
        if pred == 0:
            return [], False

    # (B) 현재 포인트클라우드 획득
    curr_pts = get_current_pointcloud(pipeline)
    curr_pcd = to_open3d_pcd(curr_pts)

    # (C) AABB crop(세탁기)
    if aabb is not None:
        curr_pcd = curr_pcd.crop(aabb)

    # (D) ICP 정합
    aligned_pts, _ = align_icp(np.asarray(curr_pcd.points), ref_pts)

    # (E) Down-sample & z-clip
    pcd_down = to_open3d_pcd(aligned_pts).voxel_down_sample(0.01)
    pts_down = np.asarray(pcd_down.points)
    zmin, zmax = cfg["z_range"]
    final_pts  = pts_down[(pts_down[:, 2] >= zmin) & (pts_down[:, 2] <= zmax)]

    # (F) Adaptive anomaly + 클러스터
    mask, _, _ = detect_anomalies_adaptive(final_pts, ref_pts)
    centers, clusters, filtered = cluster_and_get_targets(
        final_pts, mask,
        eps=cfg["eps"], min_samples=cfg["min_samples"])

    # (G) 시각화
    visualize_pointcloud_with_garments_open3d(
        ref_pts, final_pts, final_pts[mask],
        centers, filtered, clusters)

    return centers, bool(centers)

# ────────────────── 메인 함수 ────────────────── #
def run(mode: str, json_out: bool = False):
    cfg = CFG[mode]
    print(f"[INFO] mode = {mode}")

    # (1) Reference PCD 로드
    if mode == "wm":
        ref_pcd = load_and_crop_pcd(cfg["ref_pcd"], *cfg["z_crop"])
    else:
        ref_pcd = o3d.io.read_point_cloud(cfg["ref_pcd"])
    ref_pts = np.asarray(ref_pcd.points)

    aabb = None
    if mode == "wm":
        aabb         = ref_pcd.get_axis_aligned_bounding_box()
        mb, M        = aabb.get_min_bound(), aabb.get_max_bound()
        mb[2]        = cfg["z_range"][0]
        aabb         = o3d.geometry.AxisAlignedBoundingBox(mb, M)

    # (2) RealSense 스트림
    pipeline = rs.pipeline()
    rs_cfg   = rs.config()
    rs_cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(rs_cfg)

    # (3) RGB 분류기
    rgb_pack = None
    if cfg["need_clf"]:
        rgb_pack = load_rgb_model()


    try:
        # ── batch 모드 : 바로 1회 탐지 후 종료 ── #
        if json_out:
            centers, success = detect_once(pipeline, cfg, ref_pts, aabb, rgb_pack)

            # JSON 결과 출력
            out = json.dumps({
                "success": success,
                "centers": [c.tolist() for c in centers] if centers else []
            })
            print(out)
            sys.stdout.flush()

            # 2 초 시각화 유지
            time.sleep(2.0)
            return

        # ── interactive 모드 ── #
        label, col = "Press 'd' to detect", (0, 0, 0)
        while True:
            frames = pipeline.wait_for_frames()
            color  = frames.get_color_frame()
            color_img = np.asanyarray(color.get_data())
            cv2.putText(color_img, label, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, col, 3)
            cv2.imshow("Laundry RGB Stream", color_img)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            if key != ord('d'):
                continue

            centers, success = detect_once(pipeline, cfg, ref_pts, aabb, rgb_pack)
            if success:
                label, col = f"CLOTH", (0, 0, 255)
            else:
                label, col = "EMPTY", (0, 255, 0)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# ────────────────── CLI ────────────────── #
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["wm", "basket"], required=True,
                        help="'wm' : 세탁기 모드 / 'basket' : 바구니 모드")
    parser.add_argument("--json", action="store_true",
                        help="1회 탐지 후 JSON 결과만 출력하고 종료")
    args = parser.parse_args()
    run(args.mode, args.json)
