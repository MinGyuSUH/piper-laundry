import argparse, time, cv2, numpy as np, pyrealsense2 as rs, open3d as o3d
from PIL import Image
import torch, torchvision.transforms as T

from detector.rgb_classifier   import RGBOnlyClassifier
from detector.utils            import (get_current_pointcloud, load_and_crop_pcd,
                                       align_icp, to_open3d_pcd)
from detector.zscore_detector  import (detect_anomalies_adaptive,
                                       cluster_and_get_targets,
                                       visualize_pointcloud_with_garments_open3d)

Z_MIN, Z_MAX = 0.43, 0.90
CYL_RADIUS = 0.30
FILTER_RADIUS = 0.30

CFG = {
    "wm": {
        "ref_pcd"      : "reference/washer.pcd",
        "z_crop"       : (0.3, 0.99),
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
    mask = (pts[:, 2] > z_thresh) & (pts[:, 2] < 1.19)
    pts_filtered = pts[mask]
    pcd_z = o3d.geometry.PointCloud()
    pcd_z.points = o3d.utility.Vector3dVector(pts_filtered)
    pcd_z, _ = pcd_z.remove_radius_outlier(nb_points=10, radius=0.02)
    pcd_z, _ = pcd_z.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
    center = np.mean(np.asarray(pcd_z.points), axis=0)
    return center

def estimate_cylinder_axis(pcd):
    pts = np.asarray(pcd.points)
    if len(pts) < 3:
        raise ValueError("Point cloud is too small.")

    # 중심화
    centered = pts - pts.mean(axis=0)

    # (N, 3)의 공분산 행렬에 대해 SVD
    cov = centered.T @ centered / (centered.shape[0] - 1)
    _, _, Vt = np.linalg.svd(cov)

    # 주축 (가장 큰 singular vector 방향)
    axis = Vt[0]  # 또는 Vt[-1] → 사용 목적에 따라
    return axis / np.linalg.norm(axis)


def align_z_to_axis(points, axis):
    z = np.array([0, 0, 1])
    axis = axis / np.linalg.norm(axis)
    v = np.cross(z, axis)
    c = np.dot(z, axis)
    s = np.linalg.norm(v)
    if s == 0:
        return points
    vx = np.array([[    0, -v[2],  v[1]],
                   [ v[2],     0, -v[0]],
                   [-v[1],  v[0],     0]])
    R = np.eye(3) + vx + vx @ vx * ((1 - c)/(s**2))
    return points @ R.T

def create_rotated_cylinder(center, axis, radius, z_min, z_max, radial_res=200, height_res=150, color=[1, 0, 0]):
    zs = np.linspace(z_min, z_max, height_res)
    thetas = np.linspace(0, 2*np.pi, radial_res)
    grid = np.array([[radius * np.cos(t), radius * np.sin(t), z] for z in zs for t in thetas])
    aligned = align_z_to_axis(grid, axis)
    aligned += center
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(aligned)
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

def rotate_cluster(cluster, center, angle_deg):
    angle_rad = np.radians(angle_deg)
    rot_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad),  np.cos(angle_rad)]
    ])
    translated = cluster[:, :2] - center[:2]
    rotated = translated @ rot_matrix.T + center[:2]
    return np.hstack([rotated, cluster[:, 2:]])

def run(mode:str):
    cfg = CFG[mode]
    print(f"[INFO] mode = {mode}")

    pipeline = rs.pipeline()
    rs_cfg   = rs.config()
    rs_cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    rs_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(rs_cfg)

    clf, tf = (load_rgb_model() if cfg["need_clf"] else (None, None))
    label, label_col = "Press 'd' to detect", (0,0,0)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color  = frames.get_color_frame()
            color_img = np.asanyarray(color.get_data())
            cv2.putText(color_img, label, (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, label_col, 3)
            cv2.imshow("Laundry RGB Stream", color_img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break

            if key == ord('d'):
                if cfg["need_clf"]:
                    pred, _ = predict_rgb(clf, tf, pipeline)
                    if pred == 0:
                        print("[INFO] RGB 분류 결과 EMPTY → 탐지 스킵")

                print("[INFO] Adaptive change detection…")

                curr_pts = get_current_pointcloud(pipeline)
                curr_pcd = to_open3d_pcd(curr_pts)

                if mode == "wm":
                    center = estimate_center_from_pcd(curr_pcd)
                    axis = estimate_cylinder_axis(curr_pcd)
                    ref_pcd = create_rotated_cylinder(center, axis, CYL_RADIUS, Z_MIN, Z_MAX)
                    ref_pts = np.asarray(ref_pcd.points)
                    aligned_pcd, _ = run_icp_custom(curr_pcd, ref_pcd)
                    down = aligned_pcd.voxel_down_sample(0.01)
                    pts_down = np.asarray(down.points)
                    zmin, zmax = cfg["z_range"]
                    z_mask = (pts_down[:, 2] >= zmin) & (pts_down[:, 2] <= zmax)
                    pts_clipped = pts_down[z_mask]
                    radius_mask = filter_points_inside_radius(pts_clipped, center[:2], FILTER_RADIUS)
                    final_pts = pts_clipped[radius_mask]

                elif mode == "basket":
                    ref_pcd = o3d.io.read_point_cloud(cfg["ref_pcd"])
                    pts = np.asarray(ref_pcd.points)
                    ref_pcd.points = o3d.utility.Vector3dVector(pts[pts[:,2] <= cfg["z_crop"][1]])
                    ref_pts = np.asarray(ref_pcd.points)
                    aligned_pts, _ = align_icp(np.asarray(curr_pcd.points), ref_pts)
                    pcd_down = to_open3d_pcd(aligned_pts).voxel_down_sample(0.01)
                    pts_down = np.asarray(pcd_down.points)
                    zmin, zmax = cfg["z_range"]
                    final_pts = pts_down[(pts_down[:,2] >= zmin) & (pts_down[:,2] <= zmax)]

                mask, _, _ = detect_anomalies_adaptive(final_pts, ref_pts)
                centers, clusters, filtered = cluster_and_get_targets(final_pts, mask, eps=cfg["eps"], min_samples=cfg["min_samples"])

                if mode == "wm" and filtered:
                    rotated_filtered = []
                    for f in filtered:
                        rotated_1 = rotate_cluster(f, center, 120)
                        rotated_2 = rotate_cluster(f, center, 240)
                        rotated_filtered.extend([rotated_1, rotated_2])
                    filtered.extend(rotated_filtered)

                if centers:
                    print(f" - 탐지된 클러스터 수: {len(centers)}")
                    for i,c in enumerate(centers):
                        print(f"   grasp {i}: {np.round(c,3)}")
                    label, label_col = f"CLOTH {len(centers)}", (0,0,255)
                else:
                    print(" - 의류 클러스터 미발견")
                    label, label_col = "EMPTY", (0,255,0)

                visualize_pointcloud_with_garments_open3d(
                    ref_pts, final_pts, final_pts[mask],
                    centers, filtered, clusters)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["wm","basket"], required=True)
    args = parser.parse_args()
    run(args.mode)
    