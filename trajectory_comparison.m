% ========== CONFIG ==========
real_path   = 'C:\Users\14595\Desktop\data\real_moveit_20250801_094114.csv';
unity_path  = 'C:\Users\14595\Desktop\data\unity_moveit_20250801_094114.csv';
gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_moveit_synth_ep01.csv';
%real_path   = 'C:\Users\14595\Desktop\data\unity_slow3_20250728_065324.csv';
%unity_path  = 'C:\Users\14595\Desktop\data\real_slow3_20250728_065324.csv';
%gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_slow3_20250728_065324.csv';

%real_path   = 'C:\Users\14595\Desktop\data\real_static1_20250728_062551.csv';
%unity_path  = 'C:\Users\14595\Desktop\data\unity_static1_20250728_062551.csv';
%gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_static1_20250728_062551.csv';


joint_id = 1;  
save_plot = true;
output_dir = 'plots';
dpi = 600;


real_data   = readmatrix(real_path);
unity_data  = readmatrix(unity_path);
gazebo_data = readmatrix(gazebo_path);


time        = real_data(:, 1);
real_joint  = real_data(:, joint_id + 1);

unity_time  = unity_data(:, 1);
unity_raw   = unity_data(:, joint_id + 1);

gazebo_time = gazebo_data(:, 1);
gazebo_raw  = gazebo_data(:, joint_id + 1);


[unity_time_unique, ia] = unique(unity_time, 'stable');
unity_raw_unique = unity_raw(ia);
unity_joint = interp1(unity_time_unique, unity_raw_unique, time, 'linear', 'extrap');

[gazebo_time_unique, ib] = unique(gazebo_time, 'stable');
gazebo_raw_unique = gazebo_raw(ib);
gazebo_joint = interp1(gazebo_time_unique, gazebo_raw_unique, time, 'linear', 'extrap');

window = 5;
real_joint   = movmean(real_joint, window);
unity_joint  = movmean(unity_joint, window);
gazebo_joint = movmean(gazebo_joint, window);


gazebo_joint = smoothdata(gazebo_joint, 'movmedian', 3);


% ========== TRAJECTORY OVERLAY ==========
figure('Position', [100, 100, 900, 300]);
hold on;

plot(time, real_joint,   'Color', [1.0 0.4 0.7], 'LineWidth', 2.0); % 粉色
plot(time, unity_joint,  'Color', [1.0 0.5 0.0], 'LineWidth', 1.5); % 橙色
plot(time, gazebo_joint, 'Color', [0.0 0.6 0.0], 'LineWidth', 1.5); % 绿色

xlabel('Time (s)', 'FontSize', 12);
ylabel('Joint Position (rad)', 'FontSize', 12);
title(sprintf('Trajectory Comparison — Joint %d', joint_id), 'FontSize', 13);
legend({'Reference (Real)', 'Unity (Actual)', 'Gazebo (Actual)'}, 'Location', 'eastoutside');
grid on;
ylim([-1, 1]);
set(gca, 'FontSize', 11);

if save_plot
    if ~exist(output_dir, 'dir'); mkdir(output_dir); end
    filename = sprintf('%s/joint%d_trajectory_overlay.png', output_dir, joint_id);
    print(filename, '-dpng', sprintf('-r%d', dpi));
    fprintf("✅ Saved trajectory plot: %s\n", filename);
end

% ========== ERROR ANALYSIS ==========
unity_err  = unity_joint  - real_joint;
gazebo_err = gazebo_joint - real_joint;


err_max = max(abs([unity_err; gazebo_err]));
err_ylim = [-1 1] * max(0.2, round(err_max * 1.2, 2));

% ==========  PLOT ==========
figure('Position', [100, 100, 800, 300]);
hold on;

plot(time, unity_err,  'Color', [1.0 0.5 0.0], 'DisplayName', 'Unity - Real');
plot(time, gazebo_err, 'Color', [0.0 0.6 0.0], 'DisplayName', 'Gazebo - Real');

xlabel('Time (s)', 'FontSize', 12);
ylabel('Joint Error (rad)', 'FontSize', 12);
title(sprintf('Joint Error over Time — Joint %d', joint_id), 'FontSize', 13);
legend('Location', 'eastoutside');
ylim(err_ylim);
grid on;
set(gca, 'FontSize', 11);

if save_plot
    filename = sprintf('%s/joint%d_error_plot.png', output_dir, joint_id);
    print(filename, '-dpng', sprintf('-r%d', dpi));
    fprintf("✅ Saved error plot: %s\n", filename);
end


figure;
boxplot([unity_err, gazebo_err], 'Labels', {'Unity - Real', 'Gazebo - Real'});
ylabel('Joint Error (rad)', 'FontSize', 12);
title(sprintf('Error Distribution — Joint %d', joint_id), 'FontSize', 13);
ylim(err_ylim);
set(gca, 'FontSize', 11);
grid on;

if save_plot
    filename = sprintf('%s/joint%d_error_boxplot.png', output_dir, joint_id);
    print(filename, '-dpng', sprintf('-r%d', dpi));
    fprintf(" Saved boxplot: %s\n", filename);
end
