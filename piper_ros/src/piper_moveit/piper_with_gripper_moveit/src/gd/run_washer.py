# garment_detector/main/run_washer.py
import torch, time, cv2, numpy as np
import pyrealsense2 as rs
from PIL import Image
import torchvision.transforms as T
from detector.rgb_classifier import RGBOnlyClassifier
from detector.zscore_detector import *
from detector.utils import get_current_pointcloud, load_and_crop_pcd, align_icp, to_open3d_pcd

MIN_Z = 0.47


def get_pointcloud_and_color(pipeline):
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    color = frames.get_color_frame()
    if not depth or not color:
        return None, None, None, None

    intr = rs.video_stream_profile(depth.profile).get_intrinsics()
    pc = rs.pointcloud()
    points = pc.calculate(depth)
    vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
    vtx = vtx[np.isfinite(vtx[:, 2])]
    color_image = np.asanyarray(color.get_data())
    return vtx, color_image, intr, depth


def load_rgb_model(weight_path='models/rgb_only_best.pth'):
    model = RGBOnlyClassifier().to("cuda" if torch.cuda.is_available() else "cpu")
    model.load_state_dict(torch.load(weight_path, map_location="cpu"))
    model.eval()
    return model

def predict_from_rgb(model, tf, pipeline):
    frames = pipeline.wait_for_frames()
    color = frames.get_color_frame()
    if not color:
        return None, None
    img = np.asanyarray(color.get_data())
    img_pil = Image.fromarray(img[..., ::-1])
    x = tf(img_pil).unsqueeze(0).to("cuda" if torch.cuda.is_available() else "cpu")
    pred = model(x).argmax(1).item()
    return pred, img

def main():
    # [1] 세팅
    model = load_rgb_model()
    tf = T.Compose([
        T.Resize((224, 224)),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])
    ref_pcd = load_and_crop_pcd("reference/washer.pcd", zmin=0.3, zmax=0.99)
    ref_pts = np.asarray(ref_pcd.points)
    ref_pcd = o3d.geometry.PointCloud()
    ref_pcd.points = o3d.utility.Vector3dVector(ref_pts)
    
    
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    ## ---------- aabb ----------
    aabb = ref_pcd.get_axis_aligned_bounding_box()
    min_bound = aabb.get_min_bound()
    max_bound = aabb.get_max_bound()
    min_bound[2] = MIN_Z 
    aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

    # crop 후 ICP 정합
    
    ## ---------------------------
    print("[INFO] 'd' to detect, 'q' to quit")

    try:
        while True:
            pred, img = predict_from_rgb(model, tf, pipeline)
            if img is not None:
                label = "CLOTH" if pred == 1 else "EMPTY"
                color = (0, 0, 255) if pred == 1 else (0, 255, 0)
                cv2.putText(img, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
                cv2.imshow("Washer RGB Classification", img)

            key = cv2.waitKey(1) #& 0xFF
            if key == ord('q'):
                break
            elif key == ord('d'): #and pred == 1:
                
                print("[INFO] CLOTH detected, running change detection...")
                #curr_pts = get_current_pointcloud(pipeline, depth_crop=(0.3, 0.96))
                #aabb crop
                curr_pts, color_image, intr, _ = get_pointcloud_and_color(pipeline)
                curr_pcd = to_open3d_pcd(curr_pts)
                cropped = curr_pcd.crop(aabb)
                cropped = np.asarray(cropped.points) 
                
                aligned_pts, _ = align_icp(cropped, ref_pts)
                
                pcd_down =  to_open3d_pcd(aligned_pts).voxel_down_sample(voxel_size = 0.01)
                down_pts = np.asarray(pcd_down.points)
                final_pts = down_pts[(down_pts[:, 2] >= MIN_Z) & (down_pts[:, 2] <= 0.99)]
                
                
                mask, dist, thres = detect_anomalies_adaptive(final_pts, ref_pts)
                centers, clusters, filtered = cluster_and_get_targets(final_pts, mask)

                # # (2) 포인트별 색상 매핑
                # xs, ys, zs = final_pts[:,0], final_pts[:,1], final_pts[:,2]
                # us = ( xs*intr.fx/zs + intr.ppx ).astype(int)
                # vs = ( ys*intr.fy/zs + intr.ppy ).astype(int)
                # us = np.clip(us, 0, color_image.shape[1]-1)
                # vs = np.clip(vs, 0, color_image.shape[0]-1)
                # pt_colors = color_image[vs, us]

                # # (3) 클러스터별 평균 색상 계산
                # mean_colors = []
                # idx = 0
                # for cl in clusters:
                #     L = len(cl)
                #     mean_colors.append(pt_colors[idx:idx+L].mean(axis=0))
                #     idx += L

                # # (4) Lab 공간으로 변환 후 거리 계산
                # lab = cv2.cvtColor(np.uint8([mean_colors]), cv2.COLOR_RGB2LAB)[0]
                # ref = lab[0]
                # dists = np.linalg.norm(lab - ref, axis=1)

                # # (5) 색상이 다른 클러스터 분리 (ΔE > 15)
                # sep = [clusters[i] for i in np.where(dists>15)[0]]
                # same = [clusters[i] for i in np.where(dists<=15)[0]]

                # # (6) 시각화—다른 색으로 강조
                # visualize_pointcloud_with_garments_open3d(
                #     ref_pts, final_pts, final_pts[mask],
                #     centers, filtered, same   # 기존 시각화에는 same만 넘기고
                # )
                # # 다른 색 클러스터는 직접 o3d.visualization 에 추가!

                
                print(f" - Found {len(centers)} garment clusters")
                for i, c in enumerate(centers):
                    print(f"   Grasp {i}: {np.round(c, 3)}")
                visualize_pointcloud_with_garments_open3d(ref_pts, final_pts, final_pts[mask], centers, filtered, clusters)
                
                
                
                
            else: 
                continue
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()