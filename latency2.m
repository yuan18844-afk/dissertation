real_path   = 'C:\Users\14595\Desktop\data\real_static1_20250728_062551.csv';
unity_path  = 'C:\Users\14595\Desktop\data\unity_static1_20250728_062551.csv';
gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_static1_20250728_062551.csv';

save_plot = false;
output_dir = pwd; 
dpi = 300;


joint_list = {'panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7'};
n_joints = length(joint_list);


real_data   = readtable(real_path);
unity_data  = readtable(unity_path);
gazebo_data = readtable(gazebo_path);

[real_time_unique, idx_real]     = unique(real_data.time);
[unity_time_unique, idx_unity]   = unique(unity_data.time);
[gazebo_time_unique, idx_gazebo] = unique(gazebo_data.time);


mean_latency_unity = zeros(1, n_joints);
mean_latency_gazebo = zeros(1, n_joints);
std_latency_unity = zeros(1, n_joints);
std_latency_gazebo = zeros(1, n_joints);

%  latency 
for j = 1:n_joints
    joint = joint_list{j};

   
    angle_real    = real_data.(joint)(idx_real);
    angle_unity   = unity_data.(joint)(idx_unity);
    angle_gazebo  = gazebo_data.(joint)(idx_gazebo);

    interp_unity  = interp1(unity_time_unique, angle_unity, real_time_unique, 'nearest', 'extrap');
    interp_gazebo = interp1(gazebo_time_unique, angle_gazebo, real_time_unique, 'nearest', 'extrap');

    % latency
    latency_unity  = abs(interp_unity - angle_real);
    latency_gazebo = abs(interp_gazebo - angle_real);

    mean_latency_unity(j)  = mean(latency_unity);
    std_latency_unity(j)   = std(latency_unity);
    mean_latency_gazebo(j) = mean(latency_gazebo);
    std_latency_gazebo(j)  = std(latency_gazebo);
end


fprintf('\n%-15s | %-20s | %-20s\n', 'Joint', 'Unity Mean ± Std', 'Gazebo Mean ± Std');
fprintf('---------------------------------------------------------------\n');
for j = 1:n_joints
    fprintf('%-15s | %.5f ± %.5f     | %.5f ± %.5f\n', ...
        joint_list{j}, ...
        mean_latency_unity(j), std_latency_unity(j), ...
        mean_latency_gazebo(j), std_latency_gazebo(j));
end

avg_mean_unity  = mean(mean_latency_unity);
avg_std_unity   = mean(std_latency_unity);
avg_mean_gazebo = mean(mean_latency_gazebo);
avg_std_gazebo  = mean(std_latency_gazebo);

fprintf('\n%-15s | %.5f ± %.5f\n', 'Unity Avg', avg_mean_unity, avg_std_unity);
fprintf('%-15s | %.5f ± %.5f\n', 'Gazebo Avg', avg_mean_gazebo, avg_std_gazebo);


joint_labels = strcat("joint", string(1:n_joints));  % 替换panda_joint

figure;
bar_data = [mean_latency_unity; mean_latency_gazebo]';
bar(bar_data);
legend({'Unity', 'Gazebo'});
set(gca, 'XTickLabel', joint_labels);
xtickangle(45);
ylabel('Mean Latency (rad)');
title('Per-Joint Latency Comparison (Unity vs. Gazebo)');
grid on;

if save_plot
    filename = fullfile(output_dir, 'latency_joint_bar.png');
    print(gcf, '-dpng', sprintf('-r%d', dpi), filename);
    fprintf("Saved joint latency barplot: %s\n", filename);
end
