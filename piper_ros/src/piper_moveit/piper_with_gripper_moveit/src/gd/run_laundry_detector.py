# run_laundry_detector_rgb_cluster.py
# 기존 run_laundry_detector.py + 클러스터 내부 RGB 색상 기반 재분리 통합

import argparse, time, cv2, numpy as np, pyrealsense2 as rs, open3d as o3d
from PIL import Image
import torch, torchvision.transforms as T
from sklearn.cluster import KMeans, DBSCAN

from detector.rgb_classifier   import RGBOnlyClassifier
from detector.utils            import (get_current_pointcloud, load_and_crop_pcd,
                                       align_icp, to_open3d_pcd)
from detector.zscore_detector  import (detect_anomalies_adaptive,
                                       cluster_and_get_targets,
                                       visualize_pointcloud_with_garments_open3d)

# --------------------------------------------------------------------------- #
# 모드 설정                                                                  #
# --------------------------------------------------------------------------- #
CFG = {
    "wm": {
        "ref_pcd"      : "reference/washer.pcd",
        "z_crop"       : (0.3, 0.99),
        "z_range"      : (0.47, 0.99),
        "need_clf"     : True,
        "eps"          : 0.037, "min_samples": 30
    },
    "basket": {
        "ref_pcd"      : "reference/basket.pcd",
        "z_crop"       : (0.0, 1.0),
        "z_range"      : (0.10, 0.60),
        "need_clf"     : False,
        "eps"          : 0.037, "min_samples": 30
    }
}

# --------------------------------------------------------------------------- #
# 도우미 함수                                                                #
# --------------------------------------------------------------------------- #
def load_rgb_model(weight="models/rgb_only_best.pth"):
    model = RGBOnlyClassifier().to("cuda" if torch.cuda.is_available() else "cpu")
    model.load_state_dict(torch.load(weight, map_location="cpu"))
    model.eval()
    tf = T.Compose([
        T.Resize((224, 224)), T.ToTensor(),
        T.Normalize([0.485,0.456,0.406], [0.229,0.224,0.225])
    ])
    return model, tf

def predict_rgb(model, tf, pipeline):
    frames = pipeline.wait_for_frames()
    color  = frames.get_color_frame()
    if not color: return None, None
    img  = np.asanyarray(color.get_data())
    x    = tf(Image.fromarray(img[..., ::-1])).unsqueeze(0).to(
           "cuda" if torch.cuda.is_available() else "cpu")
    pred = model(x).argmax(1).item()
    return pred, img

def get_intrinsics(pipeline):
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    return rs.video_stream_profile(depth.profile).get_intrinsics()

def assign_rgb_to_points(points, color_image, intr):
    xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
    us = (xs * intr.fx / zs + intr.ppx).astype(int)
    vs = (ys * intr.fy / zs + intr.ppy).astype(int)
    us = np.clip(us, 0, color_image.shape[1] - 1)
    vs = np.clip(vs, 0, color_image.shape[0] - 1)
    return color_image[vs, us]

def split_cluster_by_color(cluster_points, cluster_colors, k=2):
    km = KMeans(n_clusters=k, random_state=42).fit(cluster_colors)
    labels = km.labels_
    subclusters = [cluster_points[labels == i] for i in np.unique(labels)]
    return subclusters

def split_cluster_by_dbscan_color(cluster_points, cluster_colors, eps=10, min_samples=10):
    # Lab 공간으로 변환
    lab = cv2.cvtColor(np.uint8([cluster_colors]), cv2.COLOR_RGB2LAB)[0]
    ab = lab[:, 1:]  # 밝기(L) 제외

    # DBSCAN으로 클러스터링
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(ab)
    labels = db.labels_

    subclusters = []
    for label in set(labels):
        if label == -1:  # 잡음
            continue
        subclusters.append(cluster_points[labels == label])
    return subclusters

# --------------------------------------------------------------------------- #
# 메인 파이프라인                                                            #
# --------------------------------------------------------------------------- #
def run(mode:str):
    cfg = CFG[mode]
    print(f"[INFO] mode = {mode}")

    if mode == "wm":
        ref_pcd = load_and_crop_pcd(cfg["ref_pcd"], *cfg["z_crop"])
    else:
        ref_pcd = o3d.io.read_point_cloud(cfg["ref_pcd"])
        pts     = np.asarray(ref_pcd.points)
        ref_pcd.points = o3d.utility.Vector3dVector(pts[pts[:,2] <= cfg["z_crop"][1]])
    ref_pts = np.asarray(ref_pcd.points)

    if mode == "wm":
        aabb = ref_pcd.get_axis_aligned_bounding_box()
        mb, M = aabb.get_min_bound(), aabb.get_max_bound()
        mb[2] = cfg["z_range"][0]
        aabb = o3d.geometry.AxisAlignedBoundingBox(mb, M)

    pipeline = rs.pipeline()
    rs_cfg = rs.config()
    rs_cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(rs_cfg)

    if cfg["need_clf"]:
        clf, tf = load_rgb_model()
    else:
        clf, tf = None, None

    label, label_col = "Press 'd' to detect", (0,0,0)
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color  = frames.get_color_frame()
            color_img = np.asanyarray(color.get_data())
            cv2.putText(color_img, label, (20,40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, label_col, 3)
            cv2.imshow("Laundry RGB Stream", color_img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            if key == ord('d'):
                if cfg["need_clf"]:
                    pred, _ = predict_rgb(clf, tf, pipeline)
                    if pred == 0:
                        print("[INFO] RGB 분류 결과 EMPTY → 탐지 스킵")
                        label = "EMPTY"
                        continue

                print("[INFO] Adaptive change detection…")
                curr_pts = get_current_pointcloud(pipeline)
                curr_pcd = to_open3d_pcd(curr_pts)

                if mode == "wm":
                    curr_pcd = curr_pcd.crop(aabb)

                aligned_pts, _ = align_icp(np.asarray(curr_pcd.points), ref_pts)
                pcd_down = to_open3d_pcd(aligned_pts).voxel_down_sample(0.01)
                pts_down = np.asarray(pcd_down.points)
                zmin, zmax = cfg["z_range"]
                final_pts = pts_down[(pts_down[:,2] >= zmin) & (pts_down[:,2] <= zmax)]

                mask, _, _ = detect_anomalies_adaptive(final_pts, ref_pts)
                centers, clusters, filtered = cluster_and_get_targets(
                    final_pts, mask,
                    eps=cfg["eps"], min_samples=cfg["min_samples"])

                if not clusters:
                    label, label_col = "EMPTY", (0,255,0)
                    print(" - 의류 클러스터 미발견")
                    continue

                

                # --- [추가] 색상 기반 클러스터 분리 --- #
                intr = get_intrinsics(pipeline)
                refined_clusters = []
                for cl in clusters:
                    if len(cl) < 10: continue
                    cl_colors = assign_rgb_to_points(cl, color_img, intr)
                    subgroups = split_cluster_by_color(cl, cl_colors, k=2)
                    refined_clusters.extend(subgroups)
                new_centers = [np.mean(cl, axis=0) for cl in refined_clusters]

                label, label_col = f"CLOTH", (0,0,255)
                print(f" - 탐지된 클러스터 수: {len(new_centers)}")
                for i,c in enumerate(new_centers):
                    print(f"   grasp {i}: {np.round(c,3)}")
                visualize_pointcloud_with_garments_open3d(
                    ref_pts, final_pts, final_pts[mask],
                    new_centers, filtered, clusters)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["wm","basket"], required=True,
                        help="'wm': 세탁기 모드, 'basket': 바구니 모드")
    args = parser.parse_args()
    run(args.mode)
