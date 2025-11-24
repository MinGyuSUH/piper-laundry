import open3d as o3d
import numpy as np
from detector.zscore_detector import (
    detect_anomalies_adaptive,
    cluster_and_get_targets,
    visualize_pointcloud_with_garments_open3d
)

Z_MIN, Z_MAX = 0.47, 0.95
CYL_RADIUS = 0.30  # 마스크용 기준 반지름
FILTER_RADIUS = 0.29  # 필터링 반지름

def filter_outliers_and_get_center(pcd, method="statistical", z_thresh=0.4, verbose=True):
    pts = np.asarray(pcd.points)
    z_mask = (pts[:, 2] > z_thresh) & (pts[:, 2] < 1.19)
    pts_filtered = pts[z_mask]
    pcd_z = o3d.geometry.PointCloud()
    pcd_z.points = o3d.utility.Vector3dVector(pts_filtered)

    if method == "statistical":
        pcd_filtered, _ = pcd_z.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
    elif method == "radius":
        pcd_filtered, _ = pcd_z.remove_radius_outlier(nb_points=10, radius=0.02)
    else:
        raise ValueError("지원하지 않는 방식입니다.")

    center = np.mean(np.asarray(pcd_filtered.points), axis=0)
    if verbose:
        print(f"[INFO] 중심점: {center}")
    return pcd_filtered, center

def create_cylinder_shell(center_xy, radius, z_min, z_max, radial_res=120, height_res=100, color=[1, 0, 0]):
    cx, cy = center_xy
    zs = np.linspace(z_min, z_max, height_res)
    thetas = np.linspace(0, 2 * np.pi, radial_res)
    points = [[cx + radius * np.cos(t), cy + radius * np.sin(t), z] for z in zs for t in thetas]
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    pc.paint_uniform_color(color)
    return pc

def run_icp(source_pcd, target_pcd, threshold=0.05, max_iter=100):
    source_temp = source_pcd.voxel_down_sample(0.005)
    target_temp = target_pcd.voxel_down_sample(0.005)
    reg = o3d.pipelines.registration.registration_icp(
        source_temp, target_temp, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
    )
    aligned = source_pcd.transform(reg.transformation)
    return aligned, reg.transformation

def filter_points_inside_radius(points, center_xy, radius):
    x, y = points[:, 0], points[:, 1]
    cx, cy = center_xy
    dist = np.sqrt((x - cx)**2 + (y - cy)**2)
    return dist <= radius

def main():
    pcd_path = "/home/cheon/Documents/garment_decision/init/roi/dataset/wm/empty/pc/14.pcd"
    pcd = o3d.io.read_point_cloud(pcd_path)

    # 1. 이상점 제거 + 중심 추정
    pcd_filtered, _ = filter_outliers_and_get_center(pcd, method="radius", z_thresh=0.45)
    pcd_filtered, center = filter_outliers_and_get_center(pcd_filtered, method="statistical", z_thresh=0.4)
    cx, cy = center[0], center[1]

    # 2. 겉면 실린더 생성
    ref_cylinder = create_cylinder_shell((cx, cy), CYL_RADIUS, Z_MIN, Z_MAX)

    # 3. ICP 정합
    aligned_pcd, T = run_icp(pcd_filtered, ref_cylinder)

    # 4. z 필터링 + 반지름 필터링
    down = aligned_pcd.voxel_down_sample(0.01)
    pts_down = np.asarray(down.points)
    z_mask = (pts_down[:, 2] >= Z_MIN) & (pts_down[:, 2] <= Z_MAX)
    pts_clipped = pts_down[z_mask]

    radius_mask = filter_points_inside_radius(pts_clipped, (cx, cy), FILTER_RADIUS)
    final_pts = pts_clipped[radius_mask]

    # 5. adaptive threshold 적용
    ref_pts = np.asarray(ref_cylinder.points)
    mask, dists, thresholds = detect_anomalies_adaptive(final_pts, ref_pts, k=6, lam=2.0)

    # 6. 클러스터링 + 시각화
    centers, clusters, filtered = cluster_and_get_targets(final_pts, mask, eps=0.037, min_samples=30)
    print(f"[INFO] 탐지된 클러스터 수: {len(centers)}")
    for i, c in enumerate(centers):
        print(f" - Grasp {i}: {np.round(c, 3)}")

    visualize_pointcloud_with_garments_open3d(ref_pts, final_pts, final_pts[mask], centers, filtered, clusters)

if __name__ == "__main__":
    main()
