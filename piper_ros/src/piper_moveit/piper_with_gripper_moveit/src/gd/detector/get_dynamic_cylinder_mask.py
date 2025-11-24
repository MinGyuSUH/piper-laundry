import open3d as o3d
import numpy as np

Z_MIN, Z_MAX = 0.47, 0.95

def filter_outliers(pcd: o3d.geometry.PointCloud,
                    method: str = "statistical",
                    z_thresh: float = 0.4):
    """
    이상점 제거 및 z 값 필터링
    """
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
        raise ValueError("지원하지 않는 제거 방식입니다.")

    return pcd_filtered


def create_center_axis(center, z_min=Z_MIN, z_max=Z_MAX, color=[0, 0, 0]):
    cx, cy, _ = center
    start = [cx, cy, z_min]
    end = [cx, cy, z_max]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector([start, end])
    line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
    line_set.colors = o3d.utility.Vector3dVector([color])
    return line_set


def create_cylinder_pointcloud(center_xy, radius, z_min=Z_MIN, z_max=Z_MAX,
                               resolution=300, height_resolution=200, color=[1, 0, 0]):
    cx, cy = center_xy
    zs = np.linspace(z_min, z_max, height_resolution)
    thetas = np.linspace(0, 2 * np.pi, resolution)
    points = []
    for z in zs:
        for theta in thetas:
            x = cx + radius * np.cos(theta)
            y = cy + radius * np.sin(theta)
            points.append([x, y, z])
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    pc.paint_uniform_color(color)
    return pc


def run_icp(source_pcd, target_pcd, threshold=0.05, max_iter=100):
    source_temp = source_pcd.voxel_down_sample(voxel_size=0.005)
    target_temp = target_pcd.voxel_down_sample(voxel_size=0.005)
    reg = o3d.pipelines.registration.registration_icp(
        source_temp, target_temp, threshold,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
    )
    aligned = source_pcd.transform(reg.transformation)
    return aligned, reg.transformation


def filter_points_inside_radius(points, center_xy, radius):
    x, y = points[:, 0], points[:, 1]
    cx, cy = center_xy
    dist = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    return dist <= radius


def get_dynamic_cylinder_mask_from_pcd(pcd,
                                       radius=0.29,
                                       z_range=(Z_MIN, Z_MAX)):
    """
    현재 PCD에서 중심축과 cylinder mask를 생성하고 적용하는 함수

    Args:
        pcd: open3d.geometry.PointCloud
        radius: cylinder 반지름
        z_range: (z_min, z_max)

    Returns:
        mask: 정합된 cylinder 내부 boolean mask
        center: 정합된 중심 (x, y, z)
        axis: 정합된 중심축 LineSet (시각화용)
        pcd_filtered: 이상점 제거 + z 필터링된 PointCloud
    """
    # 1. 이상점 제거
    pcd_r = filter_outliers(pcd, method="radius", z_thresh=z_range[0])
    pcd_filtered = filter_outliers(pcd_r, method="statistical", z_thresh=z_range[0])
    pts = np.asarray(pcd_filtered.points)

    # 2. 중심 추정
    center = np.mean(pts, axis=0)
    cx, cy = center[0], center[1]

    # 3. 기준 cylinder 생성
    cylinder_pc = create_cylinder_pointcloud((cx, cy), radius,
                                             z_min=z_range[0], z_max=z_range[1], color=[1, 0, 0])

    
    # 4. ICP 정합
    aligned_cylinder, T = run_icp(cylinder_pc, pcd_filtered)

    # 5. 정합된 중심 계산
    center_before = np.array([cx, cy, (z_range[0] + z_range[1]) / 2, 1.0])
    center_after = T @ center_before
    cx_aligned, cy_aligned = center_after[:2]

    # 6. 반지름 기준 마스크 생성
    mask = filter_points_inside_radius(pts, (cx_aligned, cy_aligned), radius)

    # 7. 중심축 생성
    axis = create_center_axis([cx_aligned, cy_aligned, 0],
                              z_min=z_range[0], z_max=z_range[1],
                              color=[0, 1, 0])

    return mask, center_after[:3], axis, pcd_filtered, np.asarray(aligned_cylinder.points)
