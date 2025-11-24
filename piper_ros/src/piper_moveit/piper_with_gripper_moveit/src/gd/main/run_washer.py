# garment_detector/main/run_washer.py
import torch, time, cv2, numpy as np
import pyrealsense2 as rs
from PIL import Image
import torchvision.transforms as T
from detector.rgb_classifier import RGBOnlyClassifier
from detector.zscore_detector import *
from detector.utils import get_current_pointcloud, load_and_crop_pcd, align_icp

MIN_Z = 0.47


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

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    ## ---------- aabb ----------
    aabb = ref_pts.get_axis_aligned_bounding_box()
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

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('d') and pred == 1:
                
                print("[INFO] CLOTH detected, running change detection...")
                curr_pts = get_current_pointcloud(pipeline, depth_crop=(0.4, 1.0))
                aligned_pts, _ = align_icp(curr_pts, ref_pts)
                cropped = aligned_pts.crop(aabb)
                
                mask, dist, thres = detect_anomalies_adaptive(cropped, ref_pts)
                centers, clusters, filtered = cluster_and_get_targets(aligned_pts, mask)
                print(f" - Found {len(centers)} garment clusters")
                for i, c in enumerate(centers):
                    print(f"   Grasp {i}: {np.round(c, 3)}")
                visualize_pointcloud_with_garments_open3d(ref_pts, aligned_pts, aligned_pts[mask], centers, filtered, clusters)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()