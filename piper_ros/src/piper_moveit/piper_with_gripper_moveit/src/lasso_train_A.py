#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#### A @ pred + B

import numpy as np
import pandas as pd
from sklearn.linear_model import LassoCV
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline
from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error
from sklearn.model_selection import train_test_split

# ===== 설정 =====
csv_path = "./piper_torque_log.csv"  # 로그 경로
test_ratio = 0.2
alphas = np.logspace(-4, 2, 30)                # Lasso 후보 알파
max_iter = 20000
random_state = 42
# =================

# 데이터 로드
df = pd.read_csv(csv_path)
pred_cols = [f"tau_pred{i}" for i in range(1,7)]       # X: 예측 토크 6개
real_cols = ["tau_real3","tau_real4","tau_real5"]      # Y: 실제 3,4,5
# real_cols = [f"tau_real{i}" for i in range(1,7)]      # Y: 실제 3,4,5
df = df.replace([np.inf, -np.inf], np.nan).dropna(subset=pred_cols+real_cols)

X = df[pred_cols].to_numpy(float)   # (N,6)
Y = df[real_cols].to_numpy(float)   # (N,3)

# 학습/테스트 분리
Xtr, Xte, Ytr, Yte = train_test_split(
    X, Y, test_size=test_ratio, random_state=random_state, shuffle=True
)

# LassoCV (각 출력축 독립 회귀)
H_list, b_list, a_list = [], [], []
for j in range(Y.shape[1]):
    pipe = make_pipeline(
        StandardScaler(with_mean=True, with_std=True),
        LassoCV(alphas=alphas, max_iter=max_iter, random_state=random_state)
    )
    pipe.fit(Xtr, Ytr[:, j])

    lass  = pipe.named_steps['lassocv']
    scl   = pipe.named_steps['standardscaler']

    w_std = lass.coef_         # (6,)  표준화된 X 기준 가중치
    b_std = lass.intercept_    #       표준화된 X 기준 절편
    mu, sig = scl.mean_, scl.scale_

    # 원스케일 환산:
    # y = ( (x - mu)/sig )·w_std + b_std
    #   = x·(w_std/sig) + (b_std - mu·(w_std/sig))
    W = w_std / sig
    b = b_std - np.dot(mu, W)

    H_list.append(W)
    b_list.append(b)
    a_list.append(lass.alpha_)

H = np.vstack(H_list)      # (3,6)
b = np.array(b_list)       # (3,)

# 평가
Yhat = Xte @ H.T + b
r2   = r2_score(Yte, Yhat, multioutput='raw_values')
mae  = np.mean(np.abs(Yte - Yhat), axis=0)
rmse = np.sqrt(np.mean((Yte - Yhat)**2, axis=0))

# 출력
np.set_printoptions(precision=6, suppress=True)
print("\n==============================")
print("🧩 Lasso Calibration Matrix (H) [3x6]")
print(H)
print("\n📊 Bias vector (b) [3]")
print(b)
print("\n🔧 Selected alpha per-axis [j3, j4, j5]")
print(np.array(a_list))

print("\n--- Evaluation (test set) ---")
names = ["joint3","joint4","joint5"]
for i, nm in enumerate(names):
    print(f"{nm}:  R2={r2[i]:.4f}  MAE={mae[i]:.6f}  RMSE={rmse[i]:.6f}")
print("==============================\n")
