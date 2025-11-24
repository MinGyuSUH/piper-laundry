# garment_detector/main/run_basket.py
import open3d as o3d
import pyrealsense2 as rs
import numpy as np
import cv2
from detector.icp_detector import ICPBasedLaundryDetector
from detector.utils import get_current_pointcloud, to_open3d_pcd
import json

def main():
    REF_PCD = "reference/basket.pcd"
    MASK_JSON = "reference/basket_mask.json"
    MODE = "basket"

    detector = ICPBasedLaundryDetector(REF_PCD, MASK_JSON, MODE)

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(cfg)

    print("[INFO] 'd' to detect, 'q' to quit")

    # 🔸 초기 문구
    label = "Press 'd' to detect"
    label_color = (0, 0, 0)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            color_img = np.asanyarray(color.get_data())

            # 🔸 직전 상태 유지된 문구 표시
            cv2.putText(color_img, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, label_color, 3)
            cv2.imshow("Basket RGB Stream", color_img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('d'):
                print("[INFO] Detecting laundry in basket...")
                pcd = get_current_pointcloud(pipeline)
                pcd = to_open3d_pcd(pcd)
                found, num_of_new_voxels = detector.detect(pcd, visualize=True)

                # 🔸 상태 갱신 (유지됨)
                if found:
                    label = f"CLOTH {num_of_new_voxels}/75"
                    label_color = (0, 0, 255)
                else:
                    label = f"EMPTY {num_of_new_voxels}/75"
                    label_color = (0, 255, 0)

                print(">> Result:", label)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
