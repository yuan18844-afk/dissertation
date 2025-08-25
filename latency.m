% ========== CONFIG ==========
real_path   = 'C:\Users\14595\Desktop\data\real_moveit_20250801_094114.csv';
unity_path  = 'C:\Users\14595\Desktop\data\unity_moveit_20250801_094114.csv';
gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_moveit_synth_ep01.csv';

%real_path   = 'C:\Users\14595\Desktop\data\real_static1_20250728_062551.csv';
%unity_path  = 'C:\Users\14595\Desktop\data\unity_static1_20250728_062551.csv';
%gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_static1_20250728_062551.csv';



real_data   = readtable(real_path);
unity_data  = readtable(unity_path);
gazebo_data = readtable(gazebo_path);

joint_name = 'panda_joint1';


time_real   = real_data.time;
angle_real  = real_data.(joint_name);


[unity_time_unique, idx_unity] = unique(unity_data.time);
[real_time_unique, idx_real]   = unique(real_data.time);
[gazebo_time_unique, idx_gazebo] = unique(gazebo_data.time);


angle_unity_unique  = unity_data.(joint_name)(idx_unity);
angle_real_unique   = real_data.(joint_name)(idx_real);
angle_gazebo_unique = gazebo_data.(joint_name)(idx_gazebo);



angle_unity_interp  = interp1(unity_time_unique, angle_unity_unique, real_time_unique, 'nearest', 'extrap');
angle_gazebo_interp = interp1(gazebo_time_unique, angle_gazebo_unique, real_time_unique, 'nearest', 'extrap');



latency_unity  = abs(angle_unity_interp - angle_real_unique);
latency_gazebo = abs(angle_gazebo_interp - angle_real_unique);



figure;
plot(real_time_unique, latency_unity, 'b-', 'DisplayName', 'Unity Latency'); hold on;
plot(real_time_unique, latency_gazebo, 'r-', 'DisplayName', 'Gazebo Latency');
xlabel('Time (s)');
ylabel('Latency (rad)');
title(['Latency over Time for ', joint_name]);
legend;
grid on;
drawnow;



mean_unity = mean(latency_unity);
std_unity  = std(latency_unity);

mean_gazebo = mean(latency_gazebo);
std_gazebo  = std(latency_gazebo);

figure;
bar_data = [mean_unity, std_unity; mean_gazebo, std_gazebo];
bar(bar_data);
set(gca, 'XTickLabel', {'Unity', 'Gazebo'});
legend({'Mean Latency', 'Std Dev'});
ylabel('Latency (rad)');
title(['Mean and Std of Latency for ', joint_name]);
grid on;

xlim([0.5 2.5]);
xticks([1 2]);
xticklabels({'Unity', 'Gazebo'});
ylabel('Latency (ms)', 'FontSize', 12);
title('Latency Distribution Comparison: Unity vs. Gazebo', 'FontSize', 13);
set(gca, 'FontSize', 11);
grid on;

if save_plot
    filename_curve = fullfile(output_dir, 'latency_curve_plot.png');
    print(gcf, '-dpng', sprintf('-r%d', dpi), filename_curve);
    fprintf("Saved latency curve plot: %s\n", filename_curve);
end
