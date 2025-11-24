#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rule-based RGB segmentation via per-pixel difference (reference vs. current).
- 참조(배경) 이미지와 현재 이미지를 픽셀 단위로 비교해 룰베이스로 세그먼트.
- RGB L1, LAB ΔE, HSV 변화량(ΔH/ΔS/ΔV)을 조합한 규칙으로 마스크 생성.
- 모폴로지/소영역 제거로 클린업.
- 마스크, 오버레이(단색/블렌드), 분리(seg_rgb) 저장.
- 콘솔로 JSON 요약(영역 수, bbox, 중심, 디버그 통계) 출력. 필요 시 파일로 저장 가능.

사용 예:
python rgb_diff_segment.py --ref path/to/ref.jpg --cur path/to/cur.jpg --outdir out \
    --thr_de 15 --thr_rgb 60 --thr_h 10 --thr_s 20 --thr_v 20 --s_min 30 \
    --open 3 --close 5 --min_area 150 --overlay_mode blend --solid_color 0 255 255 --save_json

주의:
- OpenCV는 기본 BGR 로드. 변환은 내부에서 처리.
- Hue 임계치(thr_h)는 OpenCV 기준 [0..179].
- ΔE는 CIE76(OpenCV LAB)로 계산(변화 감지엔 충분).
"""
import os
import json
import argparse
from typing import Tuple, Dict, Any

import cv2
import numpy as np


def _ensure_3c(img: np.ndarray) -> np.ndarray:
    if img is None:
        raise ValueError("Image is None")
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.shape[2] == 4:
        return img[:, :, :3]
    return img


def _match_size(src: np.ndarray, target_shape: Tuple[int, int, int]) -> np.ndarray:
    th, tw = target_shape[:2]
    if src.shape[0] == th and src.shape[1] == tw:
        return src
    return cv2.resize(src, (tw, th), interpolation=cv2.INTER_LINEAR)


def delta_e_cie76_lab(bgr1: np.ndarray, bgr2: np.ndarray) -> np.ndarray:
    lab1 = cv2.cvtColor(bgr1, cv2.COLOR_BGR2LAB).astype(np.float32)
    lab2 = cv2.cvtColor(bgr2, cv2.COLOR_BGR2LAB).astype(np.float32)
    diff = lab1 - lab2
    de = np.sqrt(np.sum(diff * diff, axis=2))
    return de


def hsv_diffs(bgr1: np.ndarray, bgr2: np.ndarray):
    hsv1 = cv2.cvtColor(bgr1, cv2.COLOR_BGR2HSV)
    hsv2 = cv2.cvtColor(bgr2, cv2.COLOR_BGR2HSV)
    h1, s1, v1 = cv2.split(hsv1)
    h2, s2, v2 = cv2.split(hsv2)
    dh = cv2.absdiff(h1, h2).astype(np.float32)
    # Hue wrap-around (OpenCV hue ∈ [0,179])
    dh = np.minimum(dh, 180.0 - dh)
    ds = cv2.absdiff(s1, s2).astype(np.float32)
    dv = cv2.absdiff(v1, v2).astype(np.float32)
    s_max = np.maximum(s1.astype(np.float32), s2.astype(np.float32))
    v_max = np.maximum(v1.astype(np.float32), v2.astype(np.float32))
    return dh, ds, dv, s_max, v_max


def rgb_l1(bgr1: np.ndarray, bgr2: np.ndarray) -> np.ndarray:
    d = cv2.absdiff(bgr1, bgr2).astype(np.float32)
    return d[:, :, 0] + d[:, :, 1] + d[:, :, 2]


def make_kernel(ksize: int):
    if ksize is None or ksize <= 0:
        return None
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))


def remove_small_components(mask: np.ndarray, min_area: int) -> np.ndarray:
    if min_area is None or min_area <= 1:
        return mask
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    keep = np.zeros_like(mask, dtype=np.uint8)
    for lbl in range(1, num_labels):
        area = stats[lbl, cv2.CC_STAT_AREA]
        if area >= min_area:
            keep[labels == lbl] = 255
    return keep


def rule_based_mask(
    ref_bgr: np.ndarray,
    cur_bgr: np.ndarray,
    thr_de: float = 15.0,
    thr_rgb: float = 60.0,
    thr_h: float = 10.0,
    thr_s: float = 20.0,
    thr_v: float = 20.0,
    s_min: float = 30.0,
) -> Dict[str, Any]:
    de = delta_e_cie76_lab(cur_bgr, ref_bgr)
    l1 = rgb_l1(cur_bgr, ref_bgr)
    dh, ds, dv, s_max, _ = hsv_diffs(cur_bgr, ref_bgr)

    mask_de = (de > thr_de)
    mask_rgb = (l1 > thr_rgb)
    mask_h = (dh > thr_h) & (s_max > s_min)
    mask_sv = (ds > thr_s) | (dv > thr_v)

    # 최종 규칙:
    #  - LAB 변화 큼 OR
    #  - RGB L1 큼 + Value 변화 큼 OR
    #  - Hue 점프(채도 충분) + (Sat or Val 변화)
    mask = (mask_de | (mask_rgb & (dv > thr_v)) | (mask_h & mask_sv)).astype(np.uint8) * 255

    debug = {
        "deltaE_mean": float(de.mean()),
        "rgbL1_mean": float(l1.mean()),
        "dh_mean": float(dh.mean()),
        "ds_mean": float(ds.mean()),
        "dv_mean": float(dv.mean()),
    }
    return {"mask": mask, "debug": debug}


def colorize_and_overlay(cur_bgr: np.ndarray, mask: np.ndarray, solid_color: Tuple[int, int, int], mode: str):
    mask_bool = mask > 0
    seg_solid = np.zeros_like(cur_bgr)
    seg_solid[mask_bool] = np.array(solid_color, dtype=np.uint8)

    if mode == "solid":
        overlay = seg_solid
    elif mode == "blend":
        overlay = cur_bgr.copy()
        overlay[mask_bool] = (0.6 * cur_bgr[mask_bool] + 0.4 * seg_solid[mask_bool]).astype(np.uint8)
    else:
        raise ValueError("overlay_mode must be 'solid' or 'blend'")
    seg_rgb = np.zeros_like(cur_bgr)
    seg_rgb[mask_bool] = cur_bgr[mask_bool]
    return seg_solid, overlay, seg_rgb


def parse_args():
    ap = argparse.ArgumentParser(description="Rule-based per-pixel RGB segmentation (diff-based)")
    ap.add_argument("--ref", required=True, help="Reference/background image path")
    ap.add_argument("--cur", required=True, help="Current image path")
    ap.add_argument("--outdir", default="out", help="Output directory")
    ap.add_argument("--thr_de", type=float, default=15.0, help="LAB ΔE threshold")
    ap.add_argument("--thr_rgb", type=float, default=60.0, help="RGB L1 threshold")
    ap.add_argument("--thr_h", type=float, default=10.0, help="Hue delta threshold (0..179)")
    ap.add_argument("--thr_s", type=float, default=20.0, help="Saturation delta threshold")
    ap.add_argument("--thr_v", type=float, default=20.0, help="Value delta threshold")
    ap.add_argument("--s_min", type=float, default=30.0, help="Min saturation to trust hue")
    ap.add_argument("--bilateral", action="store_true", help="Apply bilateral filter before differencing")
    ap.add_argument("--bil_d", type=int, default=5, help="Bilateral diameter")
    ap.add_argument("--bil_sigma_color", type=float, default=50.0, help="Bilateral sigmaColor")
    ap.add_argument("--bil_sigma_space", type=float, default=7.0, help="Bilateral sigmaSpace")
    ap.add_argument("--open", type=int, default=3, help="Opening kernel size (0 to skip)")
    ap.add_argument("--close", type=int, default=5, help="Closing kernel size (0 to skip)")
    ap.add_argument("--min_area", type=int, default=150, help="Remove components smaller than this area")
    ap.add_argument("--overlay_mode", choices=["solid", "blend"], default="blend", help="Overlay rendering mode")
    ap.add_argument("--solid_color", type=int, nargs=3, default=[0, 255, 255], help="BGR color for solid fill/overlay")
    ap.add_argument("--save_json", action="store_true", help="Save JSON summary")
    return ap.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    ref = cv2.imread(args.ref, cv2.IMREAD_COLOR)
    cur = cv2.imread(args.cur, cv2.IMREAD_COLOR)
    if ref is None:
        raise FileNotFoundError(f"Failed to read ref image: {args.ref}")
    if cur is None:
        raise FileNotFoundError(f"Failed to read cur image: {args.cur}")

    ref = _ensure_3c(ref)
    cur = _ensure_3c(cur)
    cur = _match_size(cur, ref.shape)

    if args.bilateral:
        ref = cv2.bilateralFilter(ref, d=args.bil_d, sigmaColor=args.bil_sigma_color, sigmaSpace=args.bil_sigma_space)
        cur = cv2.bilateralFilter(cur, d=args.bil_d, sigmaColor=args.bil_sigma_color, sigmaSpace=args.bil_sigma_space)

    # Rule-based mask
    rb = rule_based_mask(
        ref_bgr=ref, cur_bgr=cur,
        thr_de=args.thr_de, thr_rgb=args.thr_rgb,
        thr_h=args.thr_h, thr_s=args.thr_s, thr_v=args.thr_v,
        s_min=args.s_min
    )
    mask = rb["mask"]

    # Morphology
    if args.open and args.open > 0:
        k = make_kernel(args.open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    if args.close and args.close > 0:
        k = make_kernel(args.close)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    # Remove small components
    mask = remove_small_components(mask, args.min_area)

    # Visuals
    seg_solid, overlay, seg_rgb = colorize_and_overlay(cur, mask, tuple(args.solid_color), args.overlay_mode)

    base_cur = os.path.splitext(os.path.basename(args.cur))[0]
    out_mask = os.path.join(args.outdir, f"{base_cur}_mask.png")
    out_overlay = os.path.join(args.outdir, f"{base_cur}_overlay_{args.overlay_mode}.png")
    out_seg = os.path.join(args.outdir, f"{base_cur}_seg_rgb.png")
    cv2.imwrite(out_mask, mask)
    cv2.imwrite(out_overlay, overlay)
    cv2.imwrite(out_seg, seg_rgb)

    # JSON summary (optional save + always print compact line)
    info = {
        "success": bool(np.count_nonzero(mask) > 0),
        "regions": 0,
        "debug": rb["debug"],
        "outputs": {
            "mask": out_mask,
            "overlay": out_overlay,
            "seg_rgb": out_seg
        }
    }
    # Count regions after cleanup
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    info["regions"] = int(max(0, num_labels - 1))
    info["bboxes"] = []
    info["centroids"] = []
    for lbl in range(1, num_labels):
        x, y, w, h, area = stats[lbl, 0], stats[lbl, 1], stats[lbl, 2], stats[lbl, 3], stats[lbl, 4]
        cx, cy = centroids[lbl]
        info["bboxes"].append([int(x), int(y), int(w), int(h)])
        info["centroids"].append([float(cx), float(cy)])
    if args.save_json:
        out_json = os.path.join(args.outdir, f"{base_cur}_summary.json")
        with open(out_json, "w", encoding="utf-8") as f:
            json.dump(info, f, ensure_ascii=False, indent=2)

    # Print single-line JSON for easy piping
    print(json.dumps(info, ensure_ascii=False))


if __name__ == "__main__":
    main()