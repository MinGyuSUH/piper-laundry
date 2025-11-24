# garment_detector/main/run_basket_adaptive.py

import open3d as o3d
import numpy as np
import cv2
import pyrealsense2 as rs
from detector.utils import get_current_pointcloud, to_open3d_pcd, align_icp
from detector.zscore_detector import (
    detect_anomalies_adaptive,
    cluster_and_get_targets,
    visualize_pointcloud_with_garments_open3d
)

# 바구니 내부 z축 필터 범위 (환경에 맞게 조정)
MIN_Z = 0.10   # 예: 바구니 바닥 높이
MAX_Z = 0.60   # 예: 바구니 테두리 높이

def main():
    # 1) Reference PCD 로드
    ref_pcd = o3d.io.read_point_cloud("reference/basket.pcd")
    pts_all = np.asarray(ref_pcd.points)
    # Z축 값이 1m 이하인 점만 골라냄
    pts_filtered = pts_all[pts_all[:,2] <= 1.0]
    # 필터링된 포인트로 PCD 업데이트
    ref_pcd.points = o3d.utility.Vector3dVector(pts_filtered)
    ref_pts = pts_filtered
    # 2) RealSense 파이프라인 설정
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth,  640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color,  640, 480, rs.format.bgr8, 30)
    pipeline.start(cfg)

    print("[INFO] 'd' 키로 감지, 'q' 키로 종료")

    label = "Press 'd' to detect"
    label_color = (0, 0, 0)

    try:
        while True:
            # --- RGB 디스플레이 + 상태 표시 ---
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_img = np.asanyarray(color_frame.get_data())
            cv2.putText(color_img, label, (20,40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, label_color, 3)
            cv2.imshow("Basket RGB Stream", color_img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            # --- 이상점 탐지 트리거 ---
            elif key == ord('d'):
                print("[INFO] adaptive change detection 시작...")
                # 3) 현재 PointCloud 획득
                curr_pts = get_current_pointcloud(pipeline)
                curr_pcd = to_open3d_pcd(curr_pts)

                # 4) ICP 얼라인
                aligned_pts, _ = align_icp(np.asarray(curr_pcd.points), ref_pts)

                # 5) Voxel 다운샘플 + Z 필터
                pcd_down = to_open3d_pcd(aligned_pts).voxel_down_sample(voxel_size=0.01)
                pts_down = np.asarray(pcd_down.points)
                final_pts = pts_down[
                    (pts_down[:,2] >= MIN_Z) &
                    (pts_down[:,2] <= MAX_Z)
                ]

                # 6) adaptive threshold 기반 이상점(mask) 계산
                mask, dists, thresholds = detect_anomalies_adaptive(
                    final_pts, ref_pts, k=6, lam=2.0
                )

                # 7) 클러스터링 및 타겟 추출
                centers, clusters, filtered = cluster_and_get_targets(
                    final_pts, mask, eps=0.037, min_samples=30
                )

                # 8) 결과 출력
                if centers:
                    print(f" - 탐지된 클러스터 수: {len(centers)}")
                    for i, c in enumerate(centers):
                        print(f"   Grasp {i}: {np.round(c,3)}")
                    found = True
                else:
                    print(" - 의류 클러스터 미발견")
                    found = False

                # 9) 시각화
                visualize_pointcloud_with_garments_open3d(
                    ref_pts, final_pts, final_pts[mask],
                    centers, filtered, clusters
                )

                # 10) UI 상태 업데이트
                if found:
                    label = f"CLOTH {len(clusters)} clusters"
                    label_color = (0, 0, 255)
                else:
                    label = "EMPTY"
                    label_color = (0, 255, 0)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
