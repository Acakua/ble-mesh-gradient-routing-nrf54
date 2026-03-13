"""
LightGBM Routing Cost Predictor
================================
Target: Routing_Cost (pre-computed ground truth in CSV)

Formula (computed in Gateway_main.py per-link):
  hop_cost         = Grad x 10.0
  signal_cost      = max(0, -RSSI - 70) x 2.5       [free zone t\u1edbi -70 dBm]
  reliability_cost = Drop_Rate(%)^1.5               [per-node]
  battery_cost     = 200 if Pin<20%, (60-Pin)*1.5 if Pin<60%, else 0
  stability_cost   = 200 x exp(-Link_UP / 300)      [per-link decay t=300s]
  Routing_Cost     = sum of above
"""

import pandas as pd
import numpy as np
import glob
import os
import warnings
warnings.filterwarnings('ignore')

import lightgbm as lgb
from sklearn.model_selection import train_test_split, KFold
from sklearn.metrics import mean_absolute_error, r2_score, mean_squared_error

# ==========================================
# CONFIG
# ==========================================
CSV_PATTERN   = r'C:\Users\admin\Documents\Work\GRADIENT_SRV\ml_training\wsn_training_data_ready.csv'
# Native LightGBM format: cross-platform (Windows train -> Linux ARM Pi inference)
MODEL_OUTPUT  = r'C:\Users\admin\Documents\Work\GRADIENT_SRV\ml_training\routing_cost_model.txt'

FEATURE_COLS = [
    'Grad',
    'RSSI',
    'Link_UP(s)',
    'Drop_Count',
    'Fwd_Count',
    'Drop_Rate(%)',
    'Pin(%)',
    'Neighbor_Count',    # derived: number of neighbors reported
    'Link_Stability',    # derived: log1p(Link_UP)
    'Load_Per_Neighbor', # derived: Fwd_Count / Neighbor_Count
]
TARGET_COL = 'Routing_Cost'


# ==========================================
# LOAD
# ==========================================
def load_data(pattern):
    files = glob.glob(pattern)
    if not files:
        raise FileNotFoundError(f"No CSV files found: {pattern}")
    dfs = []
    for f in files:
        df = pd.read_csv(f)
        df['session'] = os.path.basename(f)
        dfs.append(df)
    df = pd.concat(dfs, ignore_index=True)
    for col in ['Origin', 'Parent', 'Neighbor']:
        df[col] = df[col].astype(str).str.replace('="', '').str.replace('"', '')
    df['Timestamp'] = pd.to_datetime(df['Timestamp'])
    print(f"[Load] {len(df)} rows from {len(files)} session(s)")
    return df


# ==========================================
# DERIVED FEATURES
# ==========================================
def add_derived_features(df):
    # Neighbor_Count: stateless - count per (Timestamp, Origin) cycle
    # Matches Gateway real-time: neighbor_count = len(data["neighbors"])
    df['Neighbor_Count'] = df.groupby(['Timestamp', 'Origin'])['Neighbor'].transform('count')
    df['Link_Stability']    = np.log1p(df['Link_UP(s)'])
    df['Load_Per_Neighbor'] = df['Fwd_Count'] / (df['Neighbor_Count'] + 1)
    return df


# ==========================================
# TRAIN
# ==========================================
def train(df):
    # Filter relay storm outliers
    df = df[df['Fwd_Count'] <= 200].copy()
    print(f"[Train] {len(df)} rows after outlier filter (Fwd_Count <= 200)")

    X = df[FEATURE_COLS]
    y = df[TARGET_COL]

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    params = dict(
        objective='regression',
        metric='rmse',
        boosting_type='gbdt',
        n_estimators=150,        # Pi-optimized: 150 trees sufficient for this feature space
        learning_rate=0.08,      # Slightly higher to compensate fewer trees
        max_depth=4,             # Shallow: Pi predicts ~4x faster than depth=6
        num_leaves=15,           # Matches max_depth=4
        min_child_samples=20,
        subsample=0.8,
        colsample_bytree=0.8,
        random_state=42,
        n_jobs=-1,
        verbose=-1,
    )

    print("[Train] Training LightGBM Regressor...")
    model = lgb.LGBMRegressor(**params)
    model.fit(
        X_train, y_train,
        eval_set=[(X_test, y_test)],
        callbacks=[
            lgb.early_stopping(50, verbose=False),
            lgb.log_evaluation(100)
        ]
    )

    y_pred = model.predict(X_test)
    mae  = mean_absolute_error(y_test, y_pred)
    rmse = np.sqrt(mean_squared_error(y_test, y_pred))
    r2   = r2_score(y_test, y_pred)

    print(f"\n=== EVALUATION (20% test set) ===")
    print(f"  MAE   (avg error):     {mae:.4f} cost units")
    print(f"  RMSE:                  {rmse:.4f}")
    print(f"  R2    (1.0 = perfect): {r2:.4f}")
    print(f"  Mean Routing_Cost:     {y_test.mean():.2f}")
    print(f"  Relative error:        {mae / y_test.mean() * 100:.1f}%")

    # 5-fold CV
    print(f"\n[CV] 5-fold Cross Validation...")
    kf    = KFold(n_splits=5, shuffle=True, random_state=42)
    r2_cv = []
    for fold, (tr_idx, va_idx) in enumerate(kf.split(X)):
        m = lgb.LGBMRegressor(**params)
        m.fit(
            X.iloc[tr_idx], y.iloc[tr_idx],
            eval_set=[(X.iloc[va_idx], y.iloc[va_idx])],
            callbacks=[lgb.early_stopping(50, verbose=False),
                       lgb.log_evaluation(-1)]
        )
        score = r2_score(y.iloc[va_idx], m.predict(X.iloc[va_idx]))
        r2_cv.append(score)
        print(f"  Fold {fold+1}: R2 = {score:.4f}")
    print(f"  CV Mean R2: {np.mean(r2_cv):.4f} +/- {np.std(r2_cv):.4f}")

    # Feature importance
    print(f"\n=== FEATURE IMPORTANCE (gain) ===")
    fi = pd.DataFrame({
        'Feature':    FEATURE_COLS,
        'Importance': model.feature_importances_
    }).sort_values('Importance', ascending=False)
    print(fi.to_string(index=False))

    return model


# ==========================================
# DEMO INFERENCE
# ==========================================
def demo_predict(model):
    print("\n=== DEMO INFERENCE ===")
    samples = pd.DataFrame([
        # Good link
        {'Grad': 1, 'RSSI': -50, 'Link_UP(s)': 7200, 'Drop_Count': 0,
         'Fwd_Count': 30, 'Drop_Rate(%)': 0.0, 'Pin(%)': 90.0,
         'Neighbor_Count': 5, 'Link_Stability': np.log1p(7200), 'Load_Per_Neighbor': 6.0},
        # Bad link: weak signal, many drops, low battery
        {'Grad': 3, 'RSSI': -78, 'Link_UP(s)': 300, 'Drop_Count': 15,
         'Fwd_Count': 80, 'Drop_Rate(%)': 15.8, 'Pin(%)': 18.0,
         'Neighbor_Count': 2, 'Link_Stability': np.log1p(300), 'Load_Per_Neighbor': 40.0},
    ])
    costs = model.predict(samples[FEATURE_COLS])
    print(f"  Good link (Grad=1, RSSI=-50, Drop=0%,    Pin=90%): cost = {costs[0]:.2f}")
    print(f"  Bad link  (Grad=3, RSSI=-78, Drop=15.8%, Pin=18%): cost = {costs[1]:.2f}")
    print(f"  -> Dijkstra will prefer the link with lower cost.")


# ==========================================
# MAIN
# ==========================================
if __name__ == '__main__':
    print("=" * 50)
    print("  LightGBM Routing Cost Trainer")
    print("=" * 50)

    # Check if Routing_Cost column exists
    sample_file = glob.glob(CSV_PATTERN)
    if not sample_file:
        print(f"[ERROR] No CSV files found matching: {CSV_PATTERN}")
        exit(1)

    sample_cols = pd.read_csv(sample_file[0], nrows=1).columns.tolist()
    if TARGET_COL not in sample_cols:
        print(f"[ERROR] Column '{TARGET_COL}' not found in CSV.")
        print("  Columns found:", sample_cols)
        print("  -> Run Gateway_main.py to collect new data with Routing_Cost column.")
        exit(1)

    df = load_data(CSV_PATTERN)
    df = add_derived_features(df)

    print(f"\n[Data] {TARGET_COL} distribution:")
    print(f"  min  = {df[TARGET_COL].min():.2f}")
    print(f"  max  = {df[TARGET_COL].max():.2f}")
    print(f"  mean = {df[TARGET_COL].mean():.2f}")
    print(f"  Danger zone (cost > 100): {(df[TARGET_COL] > 100).sum()} rows "
          f"({(df[TARGET_COL] > 100).mean() * 100:.1f}%)")

    model = train(df)

    # Save as native LightGBM text format: cross-platform, no pickle
    # Compatible: Windows x86_64 train -> Linux ARM Pi inference
    model.booster_.save_model(MODEL_OUTPUT)
    # Also save feature list alongside the model
    feat_file = MODEL_OUTPUT.replace('.txt', '_features.txt')
    with open(feat_file, 'w') as f:
        f.write('\n'.join(FEATURE_COLS))
    print(f"\n[Export] Model saved: {MODEL_OUTPUT}")
    print(f"[Export] Feature list: {feat_file}")

    demo_predict(model)
    print("\nDone!")
