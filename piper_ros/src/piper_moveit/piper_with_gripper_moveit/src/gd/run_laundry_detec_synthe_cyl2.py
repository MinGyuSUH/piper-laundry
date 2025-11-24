import argparse, time, cv2, numpy as np, pyrealsense2 as rs, open3d as o3d
from PIL import Image
import torch, torchvision.transforms as T
import sys, json, time

from detector.rgb_classifier   import RGBOnlyClassifier
from detector.utils            import (get_current_pointcloud, load_and_crop_pcd,
                                       align_icp, to_open3d_pcd, split_clusters_by_color_lab)
from detector.zscore_detector  import (detect_anomalies_adaptive,
                                       cluster_and_get_targets,
                                       visualize_pointcloud_with_garments_open3d, split_colorful_clusters)
# wm global var
Z_MIN, Z_MAX = 0.46, 0.9
CYL_RADIUS = 0.29
FILTER_RADIUS = 0.30

CFG = {
    "wm": {
        "ref_pcd"      : "reference/washer.pcd",
        "z_crop"       : (0.6, 1.2),
        "z_range"      : (Z_MIN, Z_MAX),
        "need_clf"     : True,
        "eps"          : 0.037, "min_samples": 30
    },
    "basket": {
        "ref_pcd"      : "reference/basket.pcd",
        "z_crop"       : (0.0, 1.0),
        "z_range"      : (0.10, 0.60),
        "need_clf"     : False,
        "eps"          : 0.037, "min_samples": 30
    },
    "failure": {
        "ref_pcd": "reference/failure_empty.pcd",     # ✅ 필터링된 PCD 사용
        "z_crop": (0.0, 0.5),                         # ✅ 필요 시 z 필터
        "z_range": (0.0, 0.5),
        "need_clf": False,
        "eps": 0.037, "min_samples": 30
    }

}


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


def estimate_center_from_pcd(pcd, z_thresh=0.4):
    pts = np.asarray(pcd.points)
    mask = (pts[:, 2] > z_thresh) & (pts[:, 2] < 1.0)
    pts_filtered = pts[mask]
    pcd_z = o3d.geometry.PointCloud()
    pcd_z.points = o3d.utility.Vector3dVector(pts_filtered)
    pcd_z, _ = pcd_z.remove_radius_outlier(nb_points=10, radius=0.02)
    pcd_z, _ = pcd_z.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
    center = np.mean(np.asarray(pcd_z.points), axis=0)
    return center


def create_cylinder_shell(center_xy, radius, z_min, z_max, radial_res=300, height_res=100, color=[1, 0, 0]):
    cx, cy = center_xy
    zs = np.linspace(z_min, z_max, height_res)
    thetas = np.linspace(0, 2 * np.pi, radial_res)
    points = [[cx + radius * np.cos(t), cy + radius * np.sin(t), z] for z in zs for t in thetas]
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    pc.paint_uniform_color(color)
    return pc

def create_z_axis_cylinder(radius, z_min, z_max, radial_res=200, height_res=150, color=[1, 0, 0]):
    """
    원점 기준 Z축 방향 실린더 생성 (기준축이 z축임)
    """
    zs = np.linspace(z_min, z_max, height_res)
    thetas = np.linspace(0, 2 * np.pi, radial_res)
    points = [[radius * np.cos(t), radius * np.sin(t), z] for z in zs for t in thetas]
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    pc.paint_uniform_color(color)
    return pc

def filter_points_inside_radius(points, center_xy, radius):
    x, y = points[:, 0], points[:, 1]
    cx, cy = center_xy
    dist = np.sqrt((x - cx)**2 + (y - cy)**2)
    return dist <= radius


def run_icp_custom(curr_pcd, ref_pcd):
    ref_pts = np.asarray(ref_pcd.points)
    curr_pts = np.asarray(curr_pcd.points)

    ref_mask = ref_pts[:,1] < 0.0
    curr_mask = curr_pts[:,1] < 0.0

    ref_half = o3d.geometry.PointCloud()
    curr_half = o3d.geometry.PointCloud()
    ref_half.points = o3d.utility.Vector3dVector(ref_pts[ref_mask])
    curr_half.points = o3d.utility.Vector3dVector(curr_pts[curr_mask])

    reg = o3d.pipelines.registration.registration_icp(
        curr_half.voxel_down_sample(0.005),
        ref_half.voxel_down_sample(0.005),
        0.05, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
    )
    curr_pcd.transform(reg.transformation)
    return curr_pcd, reg.transformation

def run_icp_y_negative_only(curr_pcd, ref_pcd):
    """
    curr_pcd와 ref_pcd의 y<0 부분만 추출해 ICP 정합을 수행함
    """
    ref_pts = np.asarray(ref_pcd.points)
    curr_pts = np.asarray(curr_pcd.points)

    ref_mask = ref_pts[:, 1] < 0.0
    curr_mask = curr_pts[:, 1] < 0.0

    ref_half = o3d.geometry.PointCloud()
    curr_half = o3d.geometry.PointCloud()
    ref_half.points = o3d.utility.Vector3dVector(ref_pts[ref_mask])
    curr_half.points = o3d.utility.Vector3dVector(curr_pts[curr_mask])

    reg = o3d.pipelines.registration.registration_icp(
        curr_half.voxel_down_sample(0.005),
        ref_half.voxel_down_sample(0.005),
        0.05, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
    )
    curr_pcd.transform(reg.transformation)
    return curr_pcd, reg.transformation


def rotate_cluster(cluster, center, angle_deg):
    angle_rad = np.radians(angle_deg)
    rot_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad),  np.cos(angle_rad)]
    ])
    translated = cluster[:, :2] - center[:2]
    rotated = translated @ rot_matrix.T + center[:2]
    return np.hstack([rotated, cluster[:, 2:]])

def camera_to_ee(p_cam: np.ndarray):
    R = np.array([
        [0, -1, 0],
        [1,  0, 0],
        [0,  0, 1]
    ])
    T = np.array([0.1, 0, -0.1])
    return R @ p_cam + T 


def run(mode: str, json_output: bool):
    cfg = CFG[mode]
    print(f"[INFO] mode = {mode}")

    pipeline = rs.pipeline()
    rs_cfg   = rs.config()
    rs_cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(rs_cfg)

    clf, tf = (load_rgb_model() if cfg["need_clf"] else (None, None))
    label, label_col = "Press 'd' to detect", (0, 0, 0)

    try:
        while True:
            s_time = time.time()
            frames = pipeline.wait_for_frames()
            color  = frames.get_color_frame()
            color_img = np.asanyarray(color.get_data())
            cv2.putText(color_img, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, label_col, 3)
            cv2.imshow("Laundry RGB Stream", color_img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break

            if key == ord('d') or json_output:
                if cfg["need_clf"]:
                    pred, _ = predict_rgb(clf, tf, pipeline)
                    if pred == 0:
                        print("[INFO] RGB 분류 결과 EMPTY → 탐지 스킵")

                print("[INFO] Adaptive change detection…")
                if mode == "wm":
                    curr_pts, color_image, intr = get_current_pointcloud(pipeline, (Z_MIN, Z_MAX))
                    curr_pcd = to_open3d_pcd(curr_pts)
                else:
                    curr_pts, color_image, intr = get_current_pointcloud(pipeline, (0.0, 1.0))
                    curr_pcd = to_open3d_pcd(curr_pts)
                    
                    
                if mode == "wm":
                    center = estimate_center_from_pcd(curr_pcd)
                    #cx, cy = center[0], center[1]
                    #print(f"center x,y : {cx}, {cy}")
                    ref_pcd = create_z_axis_cylinder(
                        #center_xy=(cx, cy),
                        radius=CYL_RADIUS,
                        z_min=Z_MIN,
                        z_max=Z_MAX,
                        color=[1, 0, 0]
                    )
                    ref_pts = np.asarray(ref_pcd.points)
                    
                    aligned_pcd, _ = run_icp_y_negative_only(curr_pcd, ref_pcd)

                    down = aligned_pcd.voxel_down_sample(0.01)
                    pts_down = np.asarray(down.points)
                    zmin, zmax = cfg["z_range"]
                    z_mask = (pts_down[:, 2] >= zmin) & (pts_down[:, 2] <= zmax)
                    pts_clipped = pts_down[z_mask]

                    # 반지름 필터링
                    radius_mask = filter_points_inside_radius(pts_clipped, (0, 0), FILTER_RADIUS)
                    pts_roi = pts_clipped[radius_mask]  # (N, 3)

                    # Adaptive threshold 기반 이상치 필터링
                    ab_mask, _, _ = detect_anomalies_adaptive(pts_roi, ref_pts)
                    pts_roi = pts_roi[ab_mask]  
                    # 날개 탐지
                    _, _, wings = cluster_and_get_targets(pts_roi, np.ones(len(pts_roi), dtype=bool), eps=cfg["eps"], min_samples=cfg["min_samples"])
                    rotated_wings = []
                    for wing in wings:
                        rotated_wings.extend([
                            rotate_cluster(wing, (0,0), 120),
                            rotate_cluster(wing, (0,0), 240)
                        ])

                    # 날개 근처 제거 마스크 적용
                    if rotated_wings or wings:
                        all_wing_pts = np.vstack(rotated_wings + wings)
                        print("wing detected")
                    else:
                        all_wing_pts = np.empty((0, 3)) 
                        print("no detected wings")
                        
                    def is_close_to_wing(pt):
                        diffs = all_wing_pts - pt  # shape: (N, 3)
                        xy_dists = np.linalg.norm(diffs[:, :2], axis=1)  # XY 평면 거리
                        z_dists = np.abs(diffs[:, 2])                    # Z축 거리
                        close_mask = (xy_dists < 0.02) & (z_dists < 0.15)
                        return np.any(close_mask)

                    mask_keep = np.array([not is_close_to_wing(p) for p in pts_roi])
                    final_pts = pts_roi[mask_keep]
                    
                    
                elif mode == "basket":
                    ref_pcd = o3d.io.read_point_cloud(cfg["ref_pcd"])
                    pts = np.asarray(ref_pcd.points)
                    ref_pcd.points = o3d.utility.Vector3dVector(pts[pts[:, 2] <= cfg["z_crop"][1]])
                    ref_pts = np.asarray(ref_pcd.points)

                    aligned_pts, _ = align_icp(np.asarray(curr_pcd.points), ref_pts)
                    pcd_down = to_open3d_pcd(aligned_pts).voxel_down_sample(0.01)
                    pts_down = np.asarray(pcd_down.points)
                    zmin, zmax = cfg["z_range"]
                    final_pts = pts_down[(pts_down[:, 2] >= zmin) & (pts_down[:, 2] <= zmax)]

                elif mode == "failure":
                    ref_pcd = o3d.io.read_point_cloud(cfg["ref_pcd"])
                    pts = np.asarray(ref_pcd.points)
                    ref_pcd.points = o3d.utility.Vector3dVector(pts[pts[:, 2] <= cfg["z_crop"][1]])
                    ref_pts = np.asarray(ref_pcd.points)

                    # 현재 프레임 포인트클라우드 수집
                    curr_pts, color_image, intr = get_current_pointcloud(pipeline, depth_crop=cfg["z_range"])
                    curr_pcd = to_open3d_pcd(curr_pts)

                    # ICP 정합
                    aligned_pts, _ = align_icp(np.asarray(curr_pcd.points), ref_pts)

                    # 다운샘플 & z 필터링
                    pcd_down = to_open3d_pcd(aligned_pts).voxel_down_sample(0.01)
                    pts_down = np.asarray(pcd_down.points)
                    final_pts = pts_down[(pts_down[:, 2] >= cfg["z_range"][0]) & (pts_down[:, 2] <= cfg["z_range"][1])]

                    

                mask, _, _ = detect_anomalies_adaptive(final_pts, ref_pts)
                centers, clusters, filtered = cluster_and_get_targets(final_pts, mask, eps=cfg["eps"], min_samples=cfg["min_samples"])
                clusters = split_colorful_clusters(clusters, color_image, intr, method="mean_shift", color_thresh=1)
                
                centers = [np.mean(c, axis=0) for c in clusters]

                if clusters and len(clusters) > 0:
                    print("do color lab")
                    same_clusters, diff_clusters = split_clusters_by_color_lab(
                        clusters, final_pts, color_image, intr
                    )
                else:
                    same_clusters, diff_clusters = [], []
                    
                if centers:
                    print(f" - 탐지된 클러스터 수: {len(centers)}")
                    for i, c in enumerate(centers):
                        print(f"   grasp {i}: {np.round(c, 3)}")
                    label, label_col = f"CLOTH {len(centers)}", (0, 0, 255)
                else:
                    print(" - 의류 클러스터 미발견")
                    label, label_col = "EMPTY", (0, 255, 0)

                if json_output:
                    print(f"before cali : {centers}")
                    if mode == "basket":
                        
                        new_center = []
                        for c  in centers:
                            
                            if c[0] < 0.1 and c[1] < 0.1:
                                new_center.append(c)
                        centers = new_center
                    centers = [camera_to_ee(np.array(c)) for c in centers]
                    print(f"after cali {centers}")
                    
                    
                    result_json = {
                        "success": len(centers) > 0,
                        "centers": [c.tolist() for c in centers]
                    }
                    print(json.dumps(result_json))
                    return
                e_time = time.time()
                print(f"detect_takes : {e_time-s_time}")
                visualize_pointcloud_with_garments_open3d(
                    ref_pts, pts_down, final_pts[mask],
                    centers, filtered, same_clusters + diff_clusters)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["wm", "basket", "failure"], required=True)
    parser.add_argument("--json", action="store_true", help="JSON 포맷으로 결과 출력")
    args = parser.parse_args()
    run(args.mode, args.json)
