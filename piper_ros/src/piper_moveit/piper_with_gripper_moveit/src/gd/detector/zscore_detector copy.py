# detector/zscore_detector.py
import numpy as np
from sklearn.neighbors import KDTree
from sklearn.cluster import DBSCAN, MeanShift
import open3d as o3d
import cv2 
def detect_anomalies_adaptive(curr_points, ref_points, k=6, lam=2.0):
    tree = KDTree(curr_points)
    dists_knn, _ = tree.query(curr_points, k=k+1)
    avg_dists = np.mean(dists_knn[:, 1:], axis=1)
    rk = np.max(dists_knn[:, 1:], axis=1)
    density = k / (np.pi * rk ** 2 + 1e-6)

    log_density = np.log10(density + 1e-6)
    norm_log_density = log_density / (np.max(log_density) + 1e-6)
    thresholds = (lam - norm_log_density) * avg_dists

    ref_tree = KDTree(ref_points)
    dists_ref, _ = ref_tree.query(curr_points, k=1)
    dists_ref = dists_ref.flatten()
    
    mask = dists_ref > thresholds
    return mask, dists_ref, thresholds

def cluster_and_get_targets(points, mask, eps=0.037, min_samples=30):
    target_pts = points[mask]
    if len(target_pts) == 0:
        return [], [], []

    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(target_pts)
    labels = clustering.labels_
    clusters, centers, filtered = [], [], []

    for label in set(labels):
        if label == -1: 
            continue
        

        cluster = target_pts[labels == label]
        # if cluster.mean(axis=0)[1] < -0.15:
        #     print(f" - [Filtered] y={cluster.mean(axis=0)[1]:.3f} < -0.15 → 제외")
        #     continue
        
        # if cluster.mean(axis=0)[1] < -0.15:
        #     print(f" - [Filtered] y={cluster.mean(axis=0)[1]:.3f} < -0.15 → 제외")
        #     continue
        
        centered = cluster - cluster.mean(axis=0)
        _, S, Vt = np.linalg.svd(centered)
        ratio = S[0] / (S[1] + 1e-6)
        cos_angle = np.abs(np.dot(Vt[0], np.array([0, 0, 1])))
        principal_dir = Vt[0]
        #print(f"number of cluster : ")
        
        if cos_angle > 0.8 and cluster.mean(axis=0)[1] < -0.05:
            print(f" - cos_angle {cos_angle}")
            filtered.append(cluster)
            continue
        
        
        # debug 
        # print(f" - Principal Direction: {np.round(principal_dir, 3)}")
        # print(f" - PCA ratio: {ratio:.2f}")
        

        
        clusters.append(cluster)
        centers.append(np.mean(cluster, axis=0))
        
    return centers, clusters, filtered


def visualize_pointcloud_with_garments_open3d(ref_pts, curr_pts, garment_pts, centers=None, filtered_clusters=None, clusters=None):
    def make_pcd(points, color):
        if not isinstance(points, np.ndarray):
            points = np.array(points)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color(color)
        return pcd

    vis_objs = []

    # 1. ref_pts (항상 표시)
    if len(ref_pts) > 0:
        vis_objs.append(make_pcd(ref_pts, [0, 0, 1]))  # 파란색

    # 2. curr_pts 중에서 garment, cluster, filtered_cluster 에 포함되지 않은 점들만 회색으로
    cluster_points = np.vstack(clusters) if clusters else np.empty((0, 3))
    filtered_points = np.vstack(filtered_clusters) if filtered_clusters else np.empty((0, 3))

    garment_set = set(map(tuple, garment_pts)) if len(garment_pts) > 0 else set()
    cluster_set = set(map(tuple, cluster_points)) if len(cluster_points) > 0 else set()
    filtered_set = set(map(tuple, filtered_points)) if len(filtered_points) > 0 else set()

    exclude_set = garment_set.union(cluster_set).union(filtered_set)
    curr_filtered = np.array([pt for pt in curr_pts if tuple(pt) not in exclude_set])
    if len(curr_filtered) > 0:
        vis_objs.append(make_pcd(curr_filtered, [0.5, 0.5, 0.5]))  # 회색

    # 3. garment_pts 중 cluster/filtered 에 포함되지 않은 점들만 빨간색
    cluster_filtered_set = cluster_set.union(filtered_set)
    unique_garments = np.array([pt for pt in garment_pts if tuple(pt) not in cluster_filtered_set])
    if len(unique_garments) > 0:
        vis_objs.append(make_pcd(unique_garments, [1, 0, 0]))  # 빨간색

    # 4. 필터링된 날개 
    if filtered_clusters:
        #len(filtered_clusters)
        for fc in filtered_clusters:
            vis_objs.append(make_pcd(fc, [0, 1, 0]))  # 초록색

    # 5. 클러스터별 
    cluster_colors = [
        [0.4, 0.6, 1.0],   # 하늘색
        [1.0, 0.5, 0.0],   # 주황
        [1.0, 0.0, 1.0],   # 마젠타
        [0.9, 0.6, 0.2],   # 살구색
        [0.0, 1.0, 1.0],   # 시안
        [0.8, 0.2, 0.5],   # 핑크
        [0.6, 0.4, 0.8],   # 보라
        [0.3, 0.3, 0.7],    # 네이비
        [0.2, 0.7, 0.2],   # 연녹색
        [0.6, 0.3, 0.0],   # 브라운
    ]

    if clusters:
        for i, cluster in enumerate(clusters):
            color = cluster_colors[i % len(cluster_colors)]
            vis_objs.append(make_pcd(cluster, color))

    # 6. 중심점
    if centers:
        for center in centers:
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            sphere.translate(center)
            sphere.paint_uniform_color([0, 0, 0])  # 검은색
            vis_objs.append(sphere)
    # 축 추가 
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.2, origin=[0,0,0]
    )
    vis_objs.append(axes)
    # o3d.visualization.draw_geometries(vis_objs)   
    vis = o3d.visualization.Visualizer()
    
    vis.create_window(window_name="PointCloud View", width=640, height=480, left=300, top=100)
    for obj in vis_objs:
        vis.add_geometry(obj)
    vis.run()
    vis.destroy_window()
    
    
def split_colorful_clusters(clusters, color_image, intr, method="mean_shift", color_thresh=15):
    """
    기존 cluster 리스트에 대해, 내부 색상 분산이 크면 색상 기반으로 다시 분할

    Args:
        clusters: List[np.ndarray] - 각 클러스터 (N_i, 3)
        color_image: (H, W, 3) RGB
        intr: RealSense intrinsics
        method: "kmeans" or "mean_shift"
        color_thresh: ΔE threshold (기준 이상이면 재분리)

    Returns:
        new_clusters: 분할된 전체 클러스터 리스트
    """
    new_clusters = []

    for cluster in clusters:
        # --- 1. 포인트 색상 추출 (RGB → Lab)
        xs, ys, zs = cluster[:, 0], cluster[:, 1], cluster[:, 2]
        us = (xs * intr.fx / zs + intr.ppx).astype(int)
        vs = (ys * intr.fy / zs + intr.ppy).astype(int)
        us = np.clip(us, 0, color_image.shape[1] - 1)
        vs = np.clip(vs, 0, color_image.shape[0] - 1)

        rgb = color_image[vs, us].astype(np.uint8)
        lab = cv2.cvtColor(rgb.reshape(-1, 1, 3), cv2.COLOR_RGB2LAB).reshape(-1, 3)

        # --- 2. 색상 분산 (a,b만 사용)
        ab = lab[:, 1:3]
        mean_ab = ab.mean(axis=0)
        dists = np.linalg.norm(ab - mean_ab, axis=1)
        spread = np.std(dists)

        # --- 3. 기준보다 크면 재분리 수행
        if spread > color_thresh:
            if method == "kmeans":
                from sklearn.cluster import KMeans
                km = KMeans(n_clusters=2, n_init="auto").fit(ab)
                for i in range(2):
                    new_clusters.append(cluster[km.labels_ == i])
            elif method == "mean_shift":
                from sklearn.cluster import MeanShift
                ms = MeanShift(bandwidth=10.0).fit(ab)
                labels = ms.labels_
                for lbl in np.unique(labels):
                    new_clusters.append(cluster[labels == lbl])
        else:
            new_clusters.append(cluster)
    return new_clusters
