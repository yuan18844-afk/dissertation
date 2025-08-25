#!/usr/bin/env python3
import pandas as pd
import numpy as np
import yaml
import matplotlib.pyplot as plt

# --- 数据加载函数 ---
def load_csv(filepath):
    df = pd.read_csv(filepath)
    df.columns = ['time'] + [f'joint{i+1}' for i in range(7)]
    return df

def load_yaml_traj(filepath):
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)

    times = []
    positions = []
    for p in data['points']:
        secs = p['time_from_start']['secs']
        nsecs = p['time_from_start']['nsecs']
        times.append(secs + nsecs * 1e-9)
        positions.append(p['positions'])

    df = pd.DataFrame(positions, columns=[f'joint{i+1}' for i in range(7)])
    df['time'] = times
    return df

# --- 核心计算函数 ---
def calculate_standard_error(df_ref, df_test):
    df_aligned = pd.merge_asof(df_test.sort_values('time'), df_ref.sort_values('time'), 
                               on='time', direction='nearest', suffixes=('_test', '_ref'))
    
    error_per_joint = pd.DataFrame()
    for i in range(7):
        joint_name = f'joint{i+1}'
        error_per_joint[joint_name] = df_aligned[f'{joint_name}_test'] - df_aligned[f'{joint_name}_ref']

    df_aligned['total_error'] = np.sqrt(np.sum(error_per_joint**2, axis=1))
    mse = np.mean(df_aligned['total_error']**2)
    rmse = np.sqrt(mse)
    max_error = np.max(df_aligned['total_error'])
    return df_aligned, {'mse': mse, 'rmse': rmse, 'max_error': max_error}

def calculate_gazebo_error(df_ref, df_test, time_scale=1.0):
    joint_names = [f'joint{i+1}' for i in range(7)]
    motion_threshold = 0.001
    joint_diff = df_test[joint_names].diff().abs()
    is_moving = (joint_diff > motion_threshold).any(axis=1)
    start_index = is_moving.idxmax() if is_moving.any() else 0
    df_test_single_run = df_test.loc[start_index:].reset_index(drop=True)
    if df_test_single_run.empty:
        return pd.DataFrame(), {'mse': np.nan, 'rmse': np.nan, 'max_error': np.nan}
    
    df_test_single_run['scaled_time'] = df_test_single_run['time'] - df_test_single_run['time'].iloc[0]
    df_ref['scaled_time'] = df_ref['time'] * time_scale

    df_aligned = pd.merge_asof(df_test_single_run, df_ref, on='scaled_time', direction='nearest', suffixes=('_test', '_ref'))
    error_per_joint = pd.DataFrame()
    for i in range(7):
        joint_name = f'joint{i+1}'
        error_per_joint[joint_name] = df_aligned[f'{joint_name}_test'] - df_aligned[f'{joint_name}_ref']
    df_aligned['total_error'] = np.sqrt(np.sum(error_per_joint**2, axis=1))

    mse = np.mean(df_aligned['total_error']**2)
    rmse = np.sqrt(mse)
    max_error = np.max(df_aligned['total_error'])
    return df_aligned, {'mse': mse, 'rmse': rmse, 'max_error': max_error}

def calculate_latency(df_ref, df_test):
    df_aligned = pd.merge_asof(
        df_test.sort_values('time'),
        df_ref.sort_values('time'),
        on='time',
        direction='nearest',
        suffixes=('_test', '_ref')
    )
    if df_aligned.empty:
        return pd.Series(), {'avg_latency': np.nan, 'std_latency': np.nan, 'max_latency': np.nan}
    df_aligned['latency'] = df_aligned['time_test'] - df_aligned['time_ref']
    avg_latency = df_aligned['latency'].mean()
    std_latency = df_aligned['latency'].std()
    max_latency = df_aligned['latency'].max()
    return df_aligned['latency'], {
        'avg_latency': avg_latency,
        'std_latency': std_latency,
        'max_latency': max_latency
    }

# --- 可视化函数 ---
def plot_joint_performance(df_data, ref_column, test_column, title, joint_idx, time_column='time', filename=None):
    if df_data.empty:
        print(f"⚠️ 警告: {title} 数据为空，跳过绘图。")
        return
    plt.figure(figsize=(8, 5))
    plt.plot(df_data[time_column].to_numpy(), df_data[test_column].to_numpy(), label='Actual')
    plt.plot(df_data[time_column].to_numpy(), df_data[ref_column].to_numpy(), label='Reference', linestyle='--')
    plt.title(f'{title} - Joint {joint_idx} Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Position (rad)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    if filename:
        plt.savefig(f'../figures/{filename}')
        print(f"✅ 图片已保存至 ../figures/{filename}")
    else:
        plt.show()
    plt.close()
# 在 utils.py 脚本中添加以下新函数

def plot_joint_accuracy_bar_chart(df_data, title, filename=None):
    if df_data.empty:
        print(f"⚠️ 警告: {title} 数据为空，跳过绘图。")
        return
        
    # 计算每个关节的误差
    error_per_joint = pd.DataFrame()
    for i in range(7):
        joint_name = f'joint{i+1}'
        error_per_joint[joint_name] = df_data[f'{joint_name}_test'] - df_data[f'{joint_name}_ref']

    # 计算每个关节的RMSE
    joint_rmse = np.sqrt(np.mean(error_per_joint**2, axis=0))
    joint_names = [f'Joint {i+1}' for i in range(7)]

    plt.figure(figsize=(10, 6))
    plt.bar(joint_names, joint_rmse.to_numpy(), color='skyblue')
    plt.title(f'{title} - RMSE per Joint')
    plt.xlabel('Robot Joints')
    plt.ylabel('RMSE (rad)')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    if filename:
        plt.savefig(f'../figures/{filename}')
        print(f"✅ 图片已保存至 ../figures/{filename}")
    else:
        plt.show()
    plt.close()
    
def plot_latency_boxplot(latency_data, labels, filename=None):
    cleaned_data = []
    cleaned_labels = []
    for lat, label in zip(latency_data, labels):
        if lat is None or lat.empty:
            continue
        lat_clean = lat.dropna().to_numpy()
        if lat_clean.size == 0:
            continue
        cleaned_data.append(lat_clean)
        cleaned_labels.append(label)

    if not cleaned_data:
        print("❌ 错误：所有延迟数据为空，无法绘图。")
        return

    plt.figure(figsize=(6, 5))
    plt.boxplot(cleaned_data, labels=cleaned_labels)
    plt.title('Synchronization Latency Comparison')
    plt.ylabel('Latency (s)')
    plt.grid(True)
    plt.tight_layout()
    if filename:
        plt.savefig(f'../figures/{filename}')
        print(f"✅ 图片已保存至 ../figures/{filename}")
    else:
        plt.show()
    plt.close()
    
    
    
def align_time_linear(sim_df, real_df, ref_joint='joint1', fs=100.0, window=2.0):
    import numpy as np

    def zscore(x):
        x = np.asarray(x)
        return (x - x.mean()) / (x.std() + 1e-8)

    def estimate_offset(sim, real, joint):
        t0 = max(sim['time'].min(), real['time'].min())
        t1 = min(sim['time'].max(), real['time'].max())
        if t1 - t0 < 1.0:
            return 0.0
        dt = 1.0 / fs
        t_grid = np.arange(t0, t1, dt)
        s = np.interp(t_grid, sim['time'], sim[joint])
        r = np.interp(t_grid, real['time'], real[joint])
        ds = zscore(np.gradient(s))
        dr = zscore(np.gradient(r))
        max_shift = int(window * fs)
        best_corr = -np.inf
        best_shift = 0
        for k in range(-max_shift, max_shift + 1):
            if k < 0:
                corr = np.dot(ds[:k], dr[-k:]) / len(ds[:k])
            elif k > 0:
                corr = np.dot(ds[k:], dr[:-k]) / len(ds[k:])
            else:
                corr = np.dot(ds, dr) / len(ds)
            if corr > best_corr:
                best_corr = corr
                best_shift = k
        return -best_shift / fs

    def fit_time_warp(sim_time, real_time):
        X = np.vstack([np.ones_like(sim_time), sim_time]).T
        theta, *_ = np.linalg.lstsq(X, real_time, rcond=None)
        return float(theta[0]), float(theta[1])

    offset = estimate_offset(sim_df, real_df, ref_joint)
    sim_shifted = sim_df.copy()
    sim_shifted['time'] = sim_shifted['time'] + offset

    # 拟合线性缩放关系
    t0 = max(sim_shifted['time'].min(), real_df['time'].min())
    t1 = min(sim_shifted['time'].max(), real_df['time'].max())
    t_ref = np.linspace(t0, t1, 300)
    sim_times = np.interp(t_ref, sim_shifted['time'], sim_shifted['time'])
    real_times = t_ref
    a, b = fit_time_warp(sim_times, real_times)

    sim_aligned = sim_df.copy()
    sim_aligned['time_aligned'] = a + b * sim_df['time']
    return sim_aligned

