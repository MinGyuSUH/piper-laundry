# detector/icp_detector.py
import open3d as o3d
import numpy as np
import json
from scipy.spatial import cKDTree

class ICPBasedLaundryDetector:
    def __init__(self, ref_pcd_path: str, mask_path: str, mode: str):
        self.mode = mode
        self.voxel_size = 0.01

        # Load reference point cloud
        self.ref_pcd = o3d.io.read_point_cloud(ref_pcd_path)
        self.ref_pcd_down = self.ref_pcd.voxel_down_sample(self.voxel_size)
        self.ref_pcd_down.estimate_normals()

        # Load mask
        with open(mask_path, 'r') as f:
            data = json.load(f)
        self.origin = np.array(data["origin"])
        #set filter
        ref_mask = set(tuple(v) for v in data["mask"])
        self.ref_mask = self.expand_mask(ref_mask, margin=1)
        
        # center, 보정, outlier 제거. # basket
        filtered_ref_mask = set()
        for idx in self.ref_mask:
            center = (np.array(idx, dtype=np.float32) + 0.5) * self.voxel_size + self.origin
            if center[1] > -0.17:  # y 값이 -0.17 이상인 경우만 유지
                filtered_ref_mask.add(idx)
        self.ref_mask = filtered_ref_mask
        
        self.ref_centers = np.array([
        (np.array(idx, dtype=np.float32) + 0.5) * self.voxel_size + self.origin
        for idx in self.ref_mask
        ])
        self.ref_aabb = o3d.geometry.AxisAlignedBoundingBox(
            self.ref_centers.min(axis=0),
            self.ref_centers.max(axis=0)
    )

    def expand_mask(self, mask, margin=1):
        expanded = set()
        for x, y, z in mask:
            for dx in range(-margin, margin+1):
                for dy in range(-margin, margin+1):
                    for dz in range(-margin, margin+1):
                        expanded.add((x+dx, y+dy, z+dz))
        return expanded

    def align_icp(self, pcd):
        pcd_down = pcd.voxel_down_sample(self.voxel_size)
        pcd_down.estimate_normals()
        result = o3d.pipelines.registration.registration_icp(
            pcd_down, self.ref_pcd_down, 0.05, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        return pcd.transform(result.transformation)

    def detect(self, current_pcd, visualize=False):
        aligned = self.align_icp(current_pcd)
        aligned = aligned.crop(self.ref_aabb)
        vg = o3d.geometry.VoxelGrid.create_from_point_cloud(aligned, voxel_size=self.voxel_size)

        tree = cKDTree(self.ref_centers)
        new_voxels = []
        for v in vg.get_voxels():
            center = vg.get_voxel_center_coordinate(v.grid_index)
            dist, _ = tree.query(center)
            if dist > 0.015:
                new_voxels.append(center)
        
        if visualize:
            #print("[DEBUG] 빨간 점들 (new_voxel 중심 좌표):")
            # for pt in new_voxels:
            #     print(np.round(pt, 4))
            self.visualize_detection(aligned, new_voxels)

        return len(new_voxels) > 75, len(new_voxels)

    def visualize_detection(self, aligned, new_voxel_pts):
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="PointCloud View", width=640, height=480, left=300, top=100)

        # New voxel
        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(new_voxel_pts)
        new_pcd.paint_uniform_color([1.0, 0.0, 0.0])

        ref_mk = o3d.geometry.PointCloud()
        ref_mk.points = o3d.utility.Vector3dVector(self.ref_mask)
        ref_mk.paint_uniform_color([0,0,1])
        
        
        # Aligned
        aligned.paint_uniform_color([0.2, 0.8, 0.2])

        # AABB (as line set)
        aabb_lineset = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(self.ref_aabb)
        aabb_lineset.paint_uniform_color([0.0, 0.5, 1.0])
        
        vis.add_geometry(aligned)
        vis.add_geometry(new_pcd)
        # vis.add_geometry(aabb_lineset)
        #vis.add_geometry(ref_mk)
        vis.run()
        vis.destroy_window()  
