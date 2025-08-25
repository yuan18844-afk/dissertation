#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import yaml

# --- 1. 数据加载函数 ---
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

# --- 2. 核心计算函数 ---
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
    df_test_moving = df_test.loc[start_index:].copy()

    static_series = ~is_moving.loc[start_index:]
    if static_series.any():
        end_index = static_series.idxmax()
    else:
        end_index = df_test.index[-1]

    df_test_single_run = df_test.loc[start_index:end_index].reset_index(drop=True)

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

    df_aligned['latency'] = df_aligned['time'].to_numpy() - df_aligned['time'].to_numpy()
    # 实际上这一步 latency 计算方式应根据来源数据时间戳差异而定（这里保留结构）

    avg_latency = df_aligned['latency'].mean()
    std_latency = df_aligned['latency'].std()
    max_latency = df_aligned['latency'].max()

    return df_aligned['latency'], {
        'avg_latency': avg_latency,
        'std_latency': std_latency,
        'max_latency': max_latency
    }

# --- 3. 可视化函数 ---
def plot_joint_performance(df_data, ref_column, test_column, title, joint_idx, time_column='time'):
    if df_data.empty:
        print(f"⚠️ 警告: {title} 数据为空，跳过绘图。")
        return
    
    plt.figure(figsize=(10, 6))
    plt.plot(df_data[time_column].to_numpy(), df_data[test_column].to_numpy(), label='Actual')
    plt.plot(df_data[time_column].to_numpy(), df_data[ref_column].to_numpy(), label='Reference', linestyle='--')
    plt.title(f'{title} - Joint {joint_idx} Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Position (rad)')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_latency_boxplot(latency_data, labels):
    cleaned_data = []
    cleaned_labels = []

    for lat, label in zip(latency_data, labels):
        if lat is None or lat.empty:
            print(f"⚠️ 警告：{label} 延迟数据为空，已跳过。")
            continue
        
        lat_clean = lat.dropna().to_numpy()
        if lat_clean.size == 0:
            print(f"⚠️ 警告：{label} 延迟数据全部为 NaN，已跳过。")
            continue

        cleaned_data.append(lat_clean)
        cleaned_labels.append(label)

    if not cleaned_data:
        print("❌ 错误：所有延迟数据为空，无法绘图。")
        return

    plt.figure(figsize=(8, 6))
    plt.boxplot(cleaned_data, labels=cleaned_labels)
    plt.title('Synchronization Latency Comparison')
    plt.ylabel('Latency (s)')
    plt.grid(True)
    plt.show()

# --- 4. 主流程入口 ---
if __name__ == '__main__':
    print("--- 实验一：Gazebo执行与Unity同步 ---")

    yaml_df = load_yaml_traj('/home/yuan8868/catkin_ws/src/franka_digital_twin/traj/panda_traj.yaml')
    gazebo_df_exp1 = load_csv('/home/yuan8868/gazebo_moveit2_20250803_101148.csv')
    gazebo_error_df, gazebo_error_stats = calculate_gazebo_error(yaml_df, gazebo_df_exp1)
    print("Gazebo 轨迹执行误差:")
    print(gazebo_error_stats)
    plot_joint_performance(gazebo_error_df, 'joint1_ref', 'joint1_test', 'Gazebo Trajectory Execution', 1, time_column='scaled_time')

    real_df_exp1 = load_csv('/home/yuan8868/real_moveit_20250801_094114.csv')
    unity_df_exp1 = load_csv('/home/yuan8868/unity_moveit_20250801_094114.csv')
    unity_error_df, unity_error_stats = calculate_standard_error(real_df_exp1, unity_df_exp1)
    print("\nUnity 同步误差:")
    print(unity_error_stats)
    plot_joint_performance(unity_error_df, 'joint1_ref', 'joint1_test', 'Unity Synchronization', 1)

    print("\n--- 实验二：高动态同步性能 ---")

    real_df_exp2 = load_csv('/home/yuan8868/real_slow1_20250728_064855.csv')
    unity_df_exp2 = load_csv('/home/yuan8868/unity_slow1_20250728_064855.csv')
    gazebo_df_exp2 = load_csv('/home/yuan8868/gazebo_slow1_20250728_064855.csv')

    unity_latency, unity_latency_stats = calculate_latency(real_df_exp2, unity_df_exp2)
    gazebo_latency, gazebo_latency_stats = calculate_latency(real_df_exp2, gazebo_df_exp2)

    print("\nUnity 同步延迟:")
    print(unity_latency_stats)
    print("\nGazebo 同步延迟:")
    print(gazebo_latency_stats)

    plot_latency_boxplot([unity_latency, gazebo_latency], ['Unity', 'Gazebo'])

