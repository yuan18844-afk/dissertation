% ========== CONFIG ==========
unity_path  = 'C:\Users\14595\Desktop\data\unity_moveit_20250801_094114.csv';
real_path   = 'C:\Users\14595\Desktop\data\real_moveit_20250801_094114.csv';
gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_moveit_synth_ep01.csv';
%real_path   = 'C:\Users\14595\Desktop\data\unity_slow3_20250728_065324.csv';
%unity_path  = 'C:\Users\14595\Desktop\data\real_slow3_20250728_065324.csv';
%gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_slow3_20250728_065324.csv';

%real_path   = 'C:\Users\14595\Desktop\data\real_static1_20250728_062551.csv';
%unity_path  = 'C:\Users\14595\Desktop\data\unity_static1_20250728_062551.csv';
%gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_static1_20250728_062551.csv';




output_excel = '1joint_error_metrics.xlsx';  % Excel 文件名


real_data   = readmatrix(real_path);
unity_data  = readmatrix(unity_path);
gazebo_data = readmatrix(gazebo_path);

time        = real_data(:, 1);
real_all    = real_data(:, 2:8);  % 关节 1~7

unity_time  = unity_data(:, 1);
unity_raw   = unity_data(:, 2:8);

gazebo_time = gazebo_data(:, 1);
gazebo_raw  = gazebo_data(:, 2:8);


[t_uni_u, idx_u] = unique(unity_time, 'stable');
unity_interp = interp1(t_uni_u, unity_raw(idx_u, :), time, 'linear', 'extrap');

[t_uni_g, idx_g] = unique(gazebo_time, 'stable');
gazebo_interp = interp1(t_uni_g, gazebo_raw(idx_g, :), time, 'linear', 'extrap');


compute_metrics = @(gt, pred) deal(...
    mean(abs(gt - pred)), ...       % MAE
    mean((gt - pred).^2), ...       % MSE
    sqrt(mean((gt - pred).^2)), ... % RMSE
    max(abs(gt - pred)) ...         % Max Error
);


n_joints = size(real_all, 2);

MAE  = zeros(n_joints, 2);   % Unity, Gazebo
MSE  = zeros(n_joints, 2);
RMSE = zeros(n_joints, 2);
MAXE = zeros(n_joints, 2);

for i = 1:n_joints
    gt = real_all(:, i);
    pred_u = unity_interp(:, i);
    pred_g = gazebo_interp(:, i);

    [MAE(i,1), MSE(i,1), RMSE(i,1), MAXE(i,1)] = compute_metrics(gt, pred_u);
    [MAE(i,2), MSE(i,2), RMSE(i,2), MAXE(i,2)] = compute_metrics(gt, pred_g);
end


MAE(end+1,:)  = mean(MAE, 1);
MSE(end+1,:)  = mean(MSE, 1);
RMSE(end+1,:) = mean(RMSE, 1);
MAXE(end+1,:) = mean(MAXE, 1);


row_names = [arrayfun(@(i) sprintf('Joint %d', i), 1:n_joints, 'UniformOutput', false), {'Avg'}];
col_names = {'Unity - Real', 'Gazebo - Real'};



header = {'Joint', 'MAE (Unity)', 'MAE (Gazebo)', ...
                  'MSE (Unity)', 'MSE (Gazebo)', ...
                  'RMSE (Unity)', 'RMSE (Gazebo)', ...
                  'Max Error (Unity)', 'Max Error (Gazebo)'};

writecell(header, output_excel, 'Sheet', 1, 'Range', 'A1');



writecell(row_names', output_excel, 'Sheet', 1, 'Range', 'A2');



metrics_all = [MAE, MSE, RMSE, MAXE];   

writematrix(metrics_all, output_excel, 'Sheet', 1, 'Range', 'B2');

fprintf("成功导出所有指标（Unity + Gazebo）到 Excel：%s\n", output_excel);

