#!/usr/bin/env python3
import pandas as pd
import numpy as np
import glob
import os
import matplotlib.pyplot as plt

# ====== Panda关节运动范围 (rad) ======
joint_ranges = {
    'panda_joint1': 2.8973,
    'panda_joint2': 1.7628,
    'panda_joint3': 2.8973,
    'panda_joint4': 3.0718,
    'panda_joint5': 2.8973,
    'panda_joint6': 3.7525,
    'panda_joint7': 2.8973
}

# ====== 计算 MAE/MNE/Accuracy ======
def compute_metrics(real_df, dt_df):
    joints = [c for c in real_df.columns if c != 'time']
    mae, mne, acc = {}, {}, {}
    for j in joints:
        error = np.abs(real_df[j].to_numpy() - dt_df[j].to_numpy())
        mae[j] = error.mean()
        mne[j] = mae[j] / joint_ranges[j]
        acc[j] = (1 - mae[j] / np.pi) * 100   # π rad = 180°
    return mae, mne, acc

# ====== 绘制误差曲线 ======
def plot_error(real_df, unity_df, gazebo_df, title, save_path):
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    joints = [c for c in real_df.columns if c != 'time']
    time = real_df['time'].to_numpy()

    for j in joints:
        plt.figure(figsize=(8,4))
        error_unity = np.abs(real_df[j].to_numpy() - unity_df[j].to_numpy())
        error_gazebo = np.abs(real_df[j].to_numpy() - gazebo_df[j].to_numpy())
        plt.plot(time, error_unity, label='Unity')
        plt.plot(time, error_gazebo, label='Gazebo')
        plt.xlabel('Time')
        plt.ylabel(f'Error (rad) - {j}')
        plt.title(f'{title} - {j}')
        plt.legend()
        plt.tight_layout()
        plt.savefig(f"{save_path}_{j}.png")
        plt.close()

# ====== 批量处理 ======
def process_all(data_dir):
    os.makedirs("results", exist_ok=True)
    os.makedirs("plots", exist_ok=True)

    real_files = sorted(glob.glob(os.path.join(data_dir, 'real_*.csv')))
    results = []

    for real_file in real_files:
        base = os.path.basename(real_file).replace('real_', '')
        unity_file = os.path.join(data_dir, 'unity_' + base)
        gazebo_file = os.path.join(data_dir, 'gazebo_' + base)

        if not (os.path.exists(unity_file) and os.path.exists(gazebo_file)):
            print(f"⚠️ 缺少匹配文件: {base}")
            continue

        print(f"✅ 处理实验: {base}")
        real_df = pd.read_csv(real_file)
        unity_df = pd.read_csv(unity_file)
        gazebo_df = pd.read_csv(gazebo_file)

        min_len = min(len(real_df), len(unity_df), len(gazebo_df))
        real_df = real_df.iloc[:min_len]
        unity_df = unity_df.iloc[:min_len]
        gazebo_df = gazebo_df.iloc[:min_len]

        mae_unity, mne_unity, acc_unity = compute_metrics(real_df, unity_df)
        mae_gazebo, mne_gazebo, acc_gazebo = compute_metrics(real_df, gazebo_df)

        # 自动识别实验类型
        if "static" in base:
            category = "static"
        elif "slow" in base:
            category = "slow"
        elif "fast" in base:
            category = "fast"
        else:
            category = "other"

        results.append({
            'experiment': base,
            'category': category,
            'mae_unity': mae_unity,
            'mae_gazebo': mae_gazebo,
            'mne_unity': mne_unity,
            'mne_gazebo': mne_gazebo,
            'acc_unity': acc_unity,
            'acc_gazebo': acc_gazebo
        })

        exp_name = base.split('.')[0]
        plot_error(real_df, unity_df, gazebo_df, exp_name, f"plots/{exp_name}")

    # ====== 保存详细结果 ======
    all_rows = []
    for r in results:
        for j in joint_ranges.keys():
            all_rows.append([
                r['experiment'], r['category'], j,
                r['mae_unity'][j], r['mae_gazebo'][j],
                r['mne_unity'][j], r['mne_gazebo'][j],
                r['acc_unity'][j], r['acc_gazebo'][j]
            ])
    df = pd.DataFrame(all_rows, columns=[
        'experiment', 'category', 'joint',
        'MAE_Unity', 'MAE_Gazebo',
        'MNE_Unity', 'MNE_Gazebo',
        'Accuracy_Unity(%)', 'Accuracy_Gazebo(%)'
    ])
    df.to_csv('results/summary_metrics.csv', index=False)

    # ====== 计算分组平均 ======
    summary = df.groupby('category').mean(numeric_only=True).reset_index()
    summary.to_csv('results/category_summary.csv', index=False)

    # ====== 画条形图（平均MAE/MNE对比） ======
    categories = summary['category']
    x = np.arange(len(categories))
    width = 0.35

    plt.figure(figsize=(8,5))
    plt.bar(x - width/2, summary['MAE_Unity'], width, label='Unity')
    plt.bar(x + width/2, summary['MAE_Gazebo'], width, label='Gazebo')
    plt.xticks(x, categories)
    plt.ylabel('Mean MAE (rad)')
    plt.title('MAE Comparison by Experiment Category')
    plt.legend()
    plt.tight_layout()
    plt.savefig('results/mae_category.png')
    plt.close()

    plt.figure(figsize=(8,5))
    plt.bar(x - width/2, summary['MNE_Unity'], width, label='Unity')
    plt.bar(x + width/2, summary['MNE_Gazebo'], width, label='Gazebo')
    plt.xticks(x, categories)
    plt.ylabel('Mean MNE')
    plt.title('MNE Comparison by Experiment Category')
    plt.legend()
    plt.tight_layout()
    plt.savefig('results/mne_category.png')
    plt.close()

    print("✅ 数据处理完成，结果已保存到 results/summary_metrics.csv、category_summary.csv 和 plots/")

# ====== 主程序 ======
if __name__ == "__main__":
    process_all("./data")

