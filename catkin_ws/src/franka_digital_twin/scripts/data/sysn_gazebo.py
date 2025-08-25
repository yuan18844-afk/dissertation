#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, math, numpy as np, pandas as pd

REAL_PATH = "/home/yuan8868/real_moveit_20250801_094114.csv"
OUT_PATH  = "/home/yuan8868/gazebo_moveit_synth_ep01.csv"

TARGET_AVG_MAE = 0.0126877    
TARGET_LAT_AVG_MS = 8.647213   
TARGET_LAT_STD_MS = 4.253148   
TARGET_LAT_MAX_MS = 20.0        

JOINTS = [f"joint{i+1}" for i in range(7)]

def load_csv(path):
    df = pd.read_csv(path)
    df.columns = ["time"] + JOINTS
    return df.sort_values("time").dropna(subset=["time"]).reset_index(drop=True)

def merge_nearest(a, b, a_label, b_label):
    return pd.merge_asof(
        a.sort_values("time"), b.sort_values("time"),
        on="time", direction="nearest",
        suffixes=(f"_{a_label}", f"_{b_label}")
    )

def compute_avg_mae(df_aligned, test_label, ref_label):
    maes = []
    for j in JOINTS:
        err = df_aligned[f"{j}_{test_label}"] - df_aligned[f"{j}_{ref_label}"]
        maes.append(np.mean(np.abs(err)))
    return float(np.mean(maes))

def synthesize_latencies(n, mean_ms, std_ms, max_ms):
    rng = np.random.default_rng(42)
    samples = []
    while len(samples) < n:
        batch = rng.normal(loc=mean_ms, scale=std_ms, size=n*2)
        batch = batch[(batch >= 0) & (batch <= max_ms)]
        samples.extend(batch.tolist())
    lat = np.array(samples[:n])
    cur_std = lat.std(ddof=0)
    if cur_std > 1e-12:
        lat = (lat - lat.mean()) * (std_ms / cur_std) + lat.mean()
    lat = np.clip(lat, 0.0, max_ms)
    lat = lat + (mean_ms - lat.mean())
    return np.clip(lat, 0.0, max_ms)

def main():
    real_df = load_csv(REAL_PATH)
    n = len(real_df)


    lat_ms = synthesize_latencies(n, TARGET_LAT_AVG_MS, TARGET_LAT_STD_MS, TARGET_LAT_MAX_MS)
    time_shifted = real_df["time"].to_numpy() + lat_ms/1000.0


    base_sigma = TARGET_AVG_MAE / math.sqrt(2.0 / math.pi)
    rng = np.random.default_rng(123)
    gazebo_df = pd.DataFrame({"time": time_shifted})
    for j in JOINTS:
        noise = rng.normal(0.0, base_sigma, size=n)
        gazebo_df[j] = real_df[j].to_numpy() + noise


    aligned = merge_nearest(gazebo_df, real_df, "gazebo", "real")
    achieved_mae = compute_avg_mae(aligned, "gazebo", "real")
    scale = TARGET_AVG_MAE / achieved_mae if achieved_mae > 0 else 1.0
    for j in JOINTS:
        gazebo_df[j] = real_df[j] + (gazebo_df[j] - real_df[j]) * scale


    aligned = merge_nearest(gazebo_df, real_df, "gazebo", "real")
    achieved_mae = compute_avg_mae(aligned, "gazebo", "real")


    merged_t = pd.merge_asof(
        gazebo_df.rename(columns={"time":"time_test"}).sort_values("time_test"),
        real_df.rename(columns={"time":"time_ref"}).sort_values("time_ref"),
        left_on="time_test", right_on="time_ref", direction="nearest"
    )
    lat_ms_actual = (merged_t["time_test"] - merged_t["time_ref"]).abs()*1000.0


    print("Synthesized Gazebo vs Real — targets vs achieved")
    print(f"Target Avg MAE (rad): {TARGET_AVG_MAE:.6f} | Achieved: {achieved_mae:.6f}")
    print(f"Target Avg Lat (ms) : {TARGET_LAT_AVG_MS:.3f} | Achieved: {lat_ms_actual.mean():.3f}")
    print(f"Target Std Lat (ms) : {TARGET_LAT_STD_MS:.3f} | Achieved: {lat_ms_actual.std(ddof=1):.3f}")
    print(f"Target Max Lat (ms) : {TARGET_LAT_MAX_MS:.3f} | Achieved: {lat_ms_actual.max():.3f}")
    print(f"Rows: {n}")

    gazebo_df.to_csv(OUT_PATH, index=False)
    print(f"Saved: {OUT_PATH}")

if __name__ == "__main__":
    main()

