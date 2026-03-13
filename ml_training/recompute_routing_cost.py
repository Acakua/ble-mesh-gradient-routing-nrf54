"""
recompute_routing_cost.py
=========================
Tái tạo cột Routing_Cost trong CSV huấn luyện theo công thức per-link mới.

Công thức mới (per-neighbor):
  hop_cost       = Grad × 10
  signal_cost    = max(0, -RSSI - 70) × 2.5       [free zone tới -70 dBm]
  reliability    = Drop_Rate^1.5                    [per-node]
  battery_cost   = 200 if Pin<20%, (60-Pin)*1.5 if Pin<60%, else 0
  stability_cost = 200 × exp(-Link_UP / 300)        [per-link, τ=300s=5phút]
                   Link mới <1phút  : ~196pt
                   Link 5 phút      :  ~90pt
                   Link 30 phút     :   ~3pt
                   Link 1 giờ+      :   ~0pt
  Routing_Cost = sum of above
"""

import pandas as pd
import numpy as np
import math
import os

INPUT_CSV  = r'C:\Users\admin\Documents\Work\GRADIENT_SRV\ml_training\wsn_training_data_ready.csv'
OUTPUT_CSV = r'C:\Users\admin\Documents\Work\GRADIENT_SRV\ml_training\wsn_training_data_ready.csv'


def compute_routing_cost_per_link(row):
    grad         = row['Grad']
    nb_rssi      = row['RSSI']
    nb_link_up_s = row['Link_UP(s)']
    drop_rate    = row['Drop_Rate(%)']
    pin          = row['Pin(%)']

    hop_cost         = grad * 10.0
    signal_cost      = max(0.0, (-nb_rssi - 70.0)) * 2.5
    reliability_cost = pow(drop_rate, 1.5) if drop_rate > 0 else 0.0

    if pin < 20.0:
        battery_cost = 200.0
    elif pin < 60.0:
        battery_cost = (60.0 - pin) * 1.5
    else:
        battery_cost = 0.0

    stability_cost = 200.0 * math.exp(-nb_link_up_s / 300.0)

    return round(hop_cost + signal_cost + reliability_cost + battery_cost + stability_cost, 2)


def main():
    print(f"[Load] Đọc: {INPUT_CSV}")
    df = pd.read_csv(INPUT_CSV)

    for col in ['Origin', 'Parent', 'Neighbor']:
        df[col] = df[col].astype(str).str.replace('="', '').str.replace('"', '')

    print(f"[Load] {len(df)} hàng, columns: {list(df.columns)}")

    # ----------------------------------------------------------------
    # Kiểm tra synthetic samples — chúng không có Link_UP thực
    # Với synthetic rows ta vẫn tính lại bình thường (Link_UP do augment)
    # ----------------------------------------------------------------
    print("[Compute] Tính lại Routing_Cost theo công thức per-link mới...")
    df['Routing_Cost'] = df.apply(compute_routing_cost_per_link, axis=1)

    # ----------------------------------------------------------------
    # Thống kê
    # ----------------------------------------------------------------
    print(f"\n=== Phân phối Routing_Cost mới ===")
    print(f"  min  = {df['Routing_Cost'].min():.2f}")
    print(f"  max  = {df['Routing_Cost'].max():.2f}")
    print(f"  mean = {df['Routing_Cost'].mean():.2f}")
    print(f"  std  = {df['Routing_Cost'].std():.2f}")
    print(f"  Danger zone (>100): {(df['Routing_Cost'] > 100).sum()} hàng "
          f"({(df['Routing_Cost'] > 100).mean()*100:.1f}%)")

    # Kiểm tra có đa dạng không (routing cost khác nhau trong cùng 1 node)
    same_cost_check = df.groupby(['Timestamp', 'Origin'])['Routing_Cost'].nunique()
    avg_unique = same_cost_check.mean()
    print(f"\n  Avg unique Routing_Cost per (Timestamp, Origin): {avg_unique:.2f}")
    print(f"  (Expected > 1.0 — xác nhận cost khác nhau giữa các neighbor)")

    # ----------------------------------------------------------------
    # Save
    # ----------------------------------------------------------------
    df.to_csv(OUTPUT_CSV, index=False)
    print(f"\n[Done] Đã lưu lại: {OUTPUT_CSV}")
    print(f"  Columns: {list(df.columns)}")

    # Preview vài hàng của cùng 1 node để xác nhận
    print("\n=== Preview: 10 hàng đầu của node 0005 ===")
    sample = df[df['Origin'] == '0005'].head(10)[
        ['Timestamp', 'Origin', 'Neighbor', 'RSSI', 'Link_UP(s)', 'Drop_Rate(%)', 'Pin(%)', 'Routing_Cost']
    ]
    print(sample.to_string(index=False))


if __name__ == '__main__':
    main()
