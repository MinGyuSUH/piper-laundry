# Garment Detector

3D 의류 탐지 시스템 (세탁기 / 바구니 내부) — RGB 분류 + ICP 정합 + 밀도 기반 Adaptive Change Detection

## 🔧 구성

```
├── main/
│   ├── run_washer.py        # washer 모드 (RGB → Change Detection)
│   └── run_basket.py        # basket 모드 (ICP + Mask)
│
├── detector/
│   ├── rgb_classifier.py    # RGB ResNet18 분류기 (학습/추론)
│   ├── icp_detector.py      # ICP 기반 마스크 비교
│   ├── zscore_detector.py   # Adaptive threshold + 밀도 기반
│   └── utils.py             # 공통 유틸 (정합, PCD 처리 등)
│
├── reference/               # 기준 point cloud
│   ├── washer.pcd
│   └── basket.pcd
│
├── configs/                
│   ├── washer_mask.json
│   └── basket_mask.json
│
├── models/
│   └── rgb_only_best.pth    # 학습된 RGB classifier 모델
```

---

## 🚀 실행 방법

### 1. Basket 모드 (ICP + mask)

```bash
python main/run_basket.py
```

* `d` 키 입력 시 ICP 정합 후 마스크 기반 비교
* `q` 키로 종료

### 2. Washer 모드 (RGB → Adaptive Detection)

```bash
python main/run_washer.py
```

* RGB classifier로 cloth 여부 판단
* 감지되면 ICP + adaptive threshold + clustering 실행

---