function robot_accuracy_comparison_final()

    
    fprintf('=== 机械臂数据准确性对比工具 (最终版) ===\n');
    fprintf('功能: 对比真实机械臂、Gazebo虚拟机械臂和Unity虚拟机械臂的关节数据准确性\n\n');
    

    [real_data, gazebo_data, unity_data] = load_robot_data_final();
    
    if isempty(real_data) || isempty(gazebo_data) || isempty(unity_data)
        fprintf('数据加载失败，请检查文件路径\n');
        return;
    end
    

    perform_accuracy_analysis_final(real_data, gazebo_data, unity_data);
end

function [real_data, gazebo_data, unity_data] = load_robot_data_final()

    fprintf('请选择数据文件:\n');
    

    [real_file, real_path] = uigetfile('*.csv', '选择真实机械臂数据文件');
    if real_file == 0
        real_data = [];
        gazebo_data = [];
        unity_data = [];
        return;
    end
    real_filepath = fullfile(real_path, real_file);
    

    [gazebo_file, gazebo_path] = uigetfile('*.csv', '选择Gazebo虚拟机械臂数据文件');
    if gazebo_file == 0
        real_data = [];
        gazebo_data = [];
        unity_data = [];
        return;
    end
    gazebo_filepath = fullfile(gazebo_path, gazebo_file);
    

    [unity_file, unity_path] = uigetfile('*.csv', '选择Unity虚拟机械臂数据文件');
    if unity_file == 0
        real_data = [];
        gazebo_data = [];
        unity_data = [];
        return;
    end
    unity_filepath = fullfile(unity_path, unity_file);
    
    try

        real_data = readtable(real_filepath);
        gazebo_data = readtable(gazebo_filepath);
        unity_data = readtable(unity_filepath);
        
        fprintf('数据加载成功:\n');
        fprintf('真实机械臂: %s (行数: %d)\n', real_file, height(real_data));
        fprintf('Gazebo虚拟机械臂: %s (行数: %d)\n', gazebo_file, height(gazebo_data));
        fprintf('Unity虚拟机械臂: %s (行数: %d)\n', unity_file, height(unity_data));
        

        fprintf('\n列名信息:\n');
        fprintf('真实机械臂列名: %s\n', strjoin(real_data.Properties.VariableNames, ', '));
        fprintf('Gazebo列名: %s\n', strjoin(gazebo_data.Properties.VariableNames, ', '));
        fprintf('Unity列名: %s\n', strjoin(unity_data.Properties.VariableNames, ', '));
        
    catch ME
        fprintf('读取文件时出错: %s\n', ME.message);
        real_data = [];
        gazebo_data = [];
        unity_data = [];
    end
end

function perform_accuracy_analysis_final(real_data, gazebo_data, unity_data)

    fprintf('\n=== 开始准确性分析 ===\n');
    

    [joint_mapping, joint_cols] = create_joint_mapping(real_data, gazebo_data, unity_data);
    if isempty(joint_mapping)
        fprintf('无法创建关节映射，退出分析\n');
        return;
    end
    
    fprintf('找到 %d 个关节映射: %s\n', length(joint_cols), strjoin(joint_cols, ', '));
    

    [synced_real, synced_gazebo, synced_unity] = synchronize_data_final(real_data, gazebo_data, unity_data);
    

    accuracy_results = calculate_accuracy_metrics_final(synced_real, synced_gazebo, synced_unity, joint_mapping);
    

    display_accuracy_results_final(accuracy_results, joint_cols);
    

    create_accuracy_visualizations_final(synced_real, synced_gazebo, synced_unity, joint_mapping, joint_cols);
    

    generate_accuracy_report_final(accuracy_results, joint_cols);
end

function [joint_mapping, joint_cols] = create_joint_mapping(real_data, gazebo_data, unity_data)

    real_cols = real_data.Properties.VariableNames;
    gazebo_cols = gazebo_data.Properties.VariableNames;
    unity_cols = unity_data.Properties.VariableNames;
    

    real_joints = {};
    gazebo_joints = {};
    unity_joints = {};
    

    for i = 1:length(real_cols)
        col = real_cols{i};
        if startsWith(col, 'joint') && length(col) > 5
            real_joints{end+1} = col;
        end
    end
    

    for i = 1:length(gazebo_cols)
        col = gazebo_cols{i};
        if contains(col, 'joint') && contains(col, 'panda')
            gazebo_joints{end+1} = col;
        end
    end
    

    for i = 1:length(unity_cols)
        col = unity_cols{i};
        if contains(col, 'joint') && contains(col, 'panda')
            unity_joints{end+1} = col;
        end
    end
    
    fprintf('找到关节列:\n');
    fprintf('真实机械臂: %s\n', strjoin(real_joints, ', '));
    fprintf('Gazebo: %s\n', strjoin(gazebo_joints, ', '));
    fprintf('Unity: %s\n', strjoin(unity_joints, ', '));
    
 
    joint_mapping = struct();
    joint_cols = {};
    

    real_nums = zeros(1, length(real_joints));
    for i = 1:length(real_joints)
        num_str = regexp(real_joints{i}, '\d+', 'match');
        if ~isempty(num_str)
            real_nums(i) = str2double(num_str{1});
        end
    end
    [~, real_sort_idx] = sort(real_nums);
    real_joints = real_joints(real_sort_idx);
    
    gazebo_nums = zeros(1, length(gazebo_joints));
    for i = 1:length(gazebo_joints)
        num_str = regexp(gazebo_joints{i}, '\d+', 'match');
        if ~isempty(num_str)
            gazebo_nums(i) = str2double(num_str{1});
        end
    end
    [~, gazebo_sort_idx] = sort(gazebo_nums);
    gazebo_joints = gazebo_joints(gazebo_sort_idx);
    
    unity_nums = zeros(1, length(unity_joints));
    for i = 1:length(unity_joints)
        num_str = regexp(unity_joints{i}, '\d+', 'match');
        if ~isempty(num_str)
            unity_nums(i) = str2double(num_str{1});
        end
    end
    [~, unity_sort_idx] = sort(unity_nums);
    unity_joints = unity_joints(unity_sort_idx);
    

    min_joints = min([length(real_joints), length(gazebo_joints), length(unity_joints)]);
    
    for i = 1:min_joints
        joint_name = sprintf('joint%d', i);
        joint_mapping.(joint_name).real = real_joints{i};
        joint_mapping.(joint_name).gazebo = gazebo_joints{i};
        joint_mapping.(joint_name).unity = unity_joints{i};
        joint_cols{end+1} = joint_name;
    end
    
    fprintf('创建了 %d 个关节映射\n', length(joint_cols));
end

function [synced_real, synced_gazebo, synced_unity] = synchronize_data_final(real_data, gazebo_data, unity_data)

    
    fprintf('正在同步数据...\n');
    

    time_col = 'time';
    

    if ~ismember(time_col, real_data.Properties.VariableNames) || ...
       ~ismember(time_col, gazebo_data.Properties.VariableNames) || ...
       ~ismember(time_col, unity_data.Properties.VariableNames)
        fprintf('警告: 某些数据缺少时间列，使用索引同步\n');

        min_len = min([height(real_data), height(gazebo_data), height(unity_data)]);
        synced_real = real_data(1:min_len, :);
        synced_gazebo = gazebo_data(1:min_len, :);
        synced_unity = unity_data(1:min_len, :);
        return;
    end
    

    real_times = real_data.(time_col);
    gazebo_times = gazebo_data.(time_col);
    unity_times = unity_data.(time_col);
    

    start_time = max([min(real_times), min(gazebo_times), min(unity_times)]);
    end_time = min([max(real_times), max(gazebo_times), max(unity_times)]);
    

    real_idx = real_times >= start_time & real_times <= end_time;
    gazebo_idx = gazebo_times >= start_time & gazebo_times <= end_time;
    unity_idx = unity_times >= start_time & unity_times <= end_time;
    
    synced_real = real_data(real_idx, :);
    synced_gazebo = gazebo_data(gazebo_idx, :);
    synced_unity = unity_data(unity_idx, :);
    
    fprintf('同步完成: 真实机械臂 %d 点, Gazebo %d 点, Unity %d 点\n', ...
        height(synced_real), height(synced_gazebo), height(synced_unity));
end

function accuracy_results = calculate_accuracy_metrics_final(real_data, gazebo_data, unity_data, joint_mapping)

    
    accuracy_results = struct();
    joint_names = fieldnames(joint_mapping);
    
    for i = 1:length(joint_names)
        joint = joint_names{i};
        mapping = joint_mapping.(joint);
        
        try

            real_joint = real_data.(mapping.real);
            gazebo_joint = gazebo_data.(mapping.gazebo);
            unity_joint = unity_data.(mapping.unity);
            

            min_len = min([length(real_joint), length(gazebo_joint), length(unity_joint)]);
            real_joint = real_joint(1:min_len);
            gazebo_joint = gazebo_joint(1:min_len);
            unity_joint = unity_joint(1:min_len);
            

            gazebo_error = real_joint - gazebo_joint;
            

            unity_error = real_joint - unity_joint;
            
  
            gazebo_unity_error = gazebo_joint - unity_joint;
            

            accuracy_results.(joint).gazebo_vs_real = struct();
            accuracy_results.(joint).unity_vs_real = struct();
            accuracy_results.(joint).gazebo_vs_unity = struct();
            

            accuracy_results.(joint).gazebo_vs_real.mean_error = mean(gazebo_error, 'omitnan');
            accuracy_results.(joint).gazebo_vs_real.std_error = std(gazebo_error, 'omitnan');
            accuracy_results.(joint).gazebo_vs_real.max_error = max(abs(gazebo_error), [], 'omitnan');
            accuracy_results.(joint).gazebo_vs_real.rmse = sqrt(mean(gazebo_error.^2, 'omitnan'));
            accuracy_results.(joint).gazebo_vs_real.mae = mean(abs(gazebo_error), 'omitnan');
            

            accuracy_results.(joint).unity_vs_real.mean_error = mean(unity_error, 'omitnan');
            accuracy_results.(joint).unity_vs_real.std_error = std(unity_error, 'omitnan');
            accuracy_results.(joint).unity_vs_real.max_error = max(abs(unity_error), [], 'omitnan');
            accuracy_results.(joint).unity_vs_real.rmse = sqrt(mean(unity_error.^2, 'omitnan'));
            accuracy_results.(joint).unity_vs_real.mae = mean(abs(unity_error), 'omitnan');
            

            accuracy_results.(joint).gazebo_vs_unity.mean_error = mean(gazebo_unity_error, 'omitnan');
            accuracy_results.(joint).gazebo_vs_unity.std_error = std(gazebo_unity_error, 'omitnan');
            accuracy_results.(joint).gazebo_vs_unity.max_error = max(abs(gazebo_unity_error), [], 'omitnan');
            accuracy_results.(joint).gazebo_vs_unity.rmse = sqrt(mean(gazebo_unity_error.^2, 'omitnan'));
            accuracy_results.(joint).gazebo_vs_unity.mae = mean(abs(gazebo_unity_error), 'omitnan');
            
        catch ME
            fprintf('处理关节 %s 时出错: %s\n', joint, ME.message);
            continue;
        end
    end
end

function display_accuracy_results_final(accuracy_results, joint_cols)

    
    fprintf('\n=== 准确性分析结果 ===\n');
    
    for i = 1:length(joint_cols)
        joint = joint_cols{i};
        if isfield(accuracy_results, joint)
            fprintf('\n%s:\n', joint);
            fprintf('  Gazebo vs 真实机械臂:\n');
            fprintf('    平均误差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_real.mean_error);
            fprintf('    标准差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_real.std_error);
            fprintf('    最大误差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_real.max_error);
            fprintf('    RMSE: %.6f rad\n', accuracy_results.(joint).gazebo_vs_real.rmse);
            fprintf('    MAE: %.6f rad\n', accuracy_results.(joint).gazebo_vs_real.mae);
            
            fprintf('  Unity vs 真实机械臂:\n');
            fprintf('    平均误差: %.6f rad\n', accuracy_results.(joint).unity_vs_real.mean_error);
            fprintf('    标准差: %.6f rad\n', accuracy_results.(joint).unity_vs_real.std_error);
            fprintf('    最大误差: %.6f rad\n', accuracy_results.(joint).unity_vs_real.max_error);
            fprintf('    RMSE: %.6f rad\n', accuracy_results.(joint).unity_vs_real.rmse);
            fprintf('    MAE: %.6f rad\n', accuracy_results.(joint).unity_vs_real.mae);
            
            fprintf('  Gazebo vs Unity:\n');
            fprintf('    平均误差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_unity.mean_error);
            fprintf('    标准差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_unity.std_error);
            fprintf('    最大误差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_unity.max_error);
            fprintf('    RMSE: %.6f rad\n', accuracy_results.(joint).gazebo_vs_unity.rmse);
            fprintf('    MAE: %.6f rad\n', accuracy_results.(joint).gazebo_vs_unity.mae);
        end
    end
end

function create_accuracy_visualizations_final(real_data, gazebo_data, unity_data, joint_mapping, joint_cols)

    
    fprintf('\n正在生成可视化图表...\n');
    

    figure('Name', '机械臂数据准确性对比', 'Position', [100, 100, 1400, 1000]);
    
    for i = 1:length(joint_cols)
        joint = joint_cols{i};
        mapping = joint_mapping.(joint);
        
        try

            real_joint = real_data.(mapping.real);
            gazebo_joint = gazebo_data.(mapping.gazebo);
            unity_joint = unity_data.(mapping.unity);
            

            min_len = min([length(real_joint), length(gazebo_joint), length(unity_joint)]);
            real_joint = real_joint(1:min_len);
            gazebo_joint = gazebo_joint(1:min_len);
            unity_joint = unity_joint(1:min_len);
            

            gazebo_error = real_joint - gazebo_joint;
            unity_error = real_joint - unity_joint;
            

            subplot(length(joint_cols), 3, (i-1)*3 + 1);
            plot(1:min_len, real_joint, 'b-', 'LineWidth', 2, 'DisplayName', '真实机械臂');
            hold on;
            plot(1:min_len, gazebo_joint, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Gazebo');
            plot(1:min_len, unity_joint, 'g:', 'LineWidth', 1.5, 'DisplayName', 'Unity');
            title(sprintf('%s - 时间序列对比', joint));
            xlabel('数据点');
            ylabel('关节角度 (rad)');
            legend;
            grid on;
            

            subplot(length(joint_cols), 3, (i-1)*3 + 2);
            histogram(gazebo_error, 'FaceColor', 'red', 'FaceAlpha', 0.7, 'DisplayName', 'Gazebo误差');
            hold on;
            histogram(unity_error, 'FaceColor', 'green', 'FaceAlpha', 0.7, 'DisplayName', 'Unity误差');
            title(sprintf('%s - 误差分布', joint));
            xlabel('误差 (rad)');
            ylabel('频次');
            legend;
            grid on;
            

            subplot(length(joint_cols), 3, (i-1)*3 + 3);
            plot(1:min_len, gazebo_error, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Gazebo误差');
            hold on;
            plot(1:min_len, unity_error, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Unity误差');
            title(sprintf('%s - 误差时间序列', joint));
            xlabel('数据点');
            ylabel('误差 (rad)');
            legend;
            grid on;
            
        catch ME
            fprintf('创建关节 %s 的可视化时出错: %s\n', joint, ME.message);
        end
    end
    
    sgtitle('机械臂数据准确性对比分析', 'FontSize', 16, 'FontWeight', 'bold');
    
  
    create_summary_charts_final(real_data, gazebo_data, unity_data, joint_mapping, joint_cols);
end

function create_summary_charts_final(real_data, gazebo_data, unity_data, joint_mapping, joint_cols)

    
    figure('Name', '准确性汇总分析', 'Position', [200, 200, 1200, 800]);

    gazebo_rmse = zeros(1, length(joint_cols));
    unity_rmse = zeros(1, length(joint_cols));
    
    for i = 1:length(joint_cols)
        joint = joint_cols{i};
        mapping = joint_mapping.(joint);
        
        try
            real_joint = real_data.(mapping.real);
            gazebo_joint = gazebo_data.(mapping.gazebo);
            unity_joint = unity_data.(mapping.unity);
            
            min_len = min([length(real_joint), length(gazebo_joint), length(unity_joint)]);
            real_joint = real_joint(1:min_len);
            gazebo_joint = gazebo_joint(1:min_len);
            unity_joint = unity_joint(1:min_len);
            
            gazebo_error = real_joint - gazebo_joint;
            unity_error = real_joint - unity_joint;
            
            gazebo_rmse(i) = sqrt(mean(gazebo_error.^2, 'omitnan'));
            unity_rmse(i) = sqrt(mean(unity_error.^2, 'omitnan'));
            
        catch ME
            fprintf('计算关节 %s 的RMSE时出错: %s\n', joint, ME.message);
            gazebo_rmse(i) = NaN;
            unity_rmse(i) = NaN;
        end
    end
    

    subplot(2, 2, 1);
    bar_data = [gazebo_rmse; unity_rmse]';
    bar(bar_data);
    title('各关节RMSE对比');
    xlabel('关节');
    ylabel('RMSE (rad)');
    legend('Gazebo', 'Unity');
    set(gca, 'XTickLabel', joint_cols);
    grid on;
    

    subplot(2, 2, 2);
    gazebo_mae = zeros(1, length(joint_cols));
    unity_mae = zeros(1, length(joint_cols));
    
    for i = 1:length(joint_cols)
        joint = joint_cols{i};
        mapping = joint_mapping.(joint);
        
        try
            real_joint = real_data.(mapping.real);
            gazebo_joint = gazebo_data.(mapping.gazebo);
            unity_joint = unity_data.(mapping.unity);
            
            min_len = min([length(real_joint), length(gazebo_joint), length(unity_joint)]);
            real_joint = real_joint(1:min_len);
            gazebo_joint = gazebo_joint(1:min_len);
            unity_joint = unity_joint(1:min_len);
            
            gazebo_error = real_joint - gazebo_joint;
            unity_error = real_joint - unity_joint;
            
            gazebo_mae(i) = mean(abs(gazebo_error), 'omitnan');
            unity_mae(i) = mean(abs(unity_error), 'omitnan');
            
        catch ME
            fprintf('计算关节 %s 的MAE时出错: %s\n', joint, ME.message);
            gazebo_mae(i) = NaN;
            unity_mae(i) = NaN;
        end
    end
    
    bar_data = [gazebo_mae; unity_mae]';
    bar(bar_data);
    title('各关节平均绝对误差对比');
    xlabel('关节');
    ylabel('MAE (rad)');
    legend('Gazebo', 'Unity');
    set(gca, 'XTickLabel', joint_cols);
    grid on;
    

    subplot(2, 2, 3);
    gazebo_std = zeros(1, length(joint_cols));
    unity_std = zeros(1, length(joint_cols));
    
    for i = 1:length(joint_cols)
        joint = joint_cols{i};
        mapping = joint_mapping.(joint);
        
        try
            real_joint = real_data.(mapping.real);
            gazebo_joint = gazebo_data.(mapping.gazebo);
            unity_joint = unity_data.(mapping.unity);
            
            min_len = min([length(real_joint), length(gazebo_joint), length(unity_joint)]);
            real_joint = real_joint(1:min_len);
            gazebo_joint = gazebo_joint(1:min_len);
            unity_joint = unity_joint(1:min_len);
            
            gazebo_error = real_joint - gazebo_joint;
            unity_error = real_joint - unity_joint;
            
            gazebo_std(i) = std(gazebo_error, 'omitnan');
            unity_std(i) = std(unity_error, 'omitnan');
            
        catch ME
            fprintf('计算关节 %s 的标准差时出错: %s\n', joint, ME.message);
            gazebo_std(i) = NaN;
            unity_std(i) = NaN;
        end
    end
    
    bar_data = [gazebo_std; unity_std]';
    bar(bar_data);
    title('各关节误差标准差对比');
    xlabel('关节');
    ylabel('标准差 (rad)');
    legend('Gazebo', 'Unity');
    set(gca, 'XTickLabel', joint_cols);
    grid on;
    

    subplot(2, 2, 4);
    overall_metrics = [mean(gazebo_rmse, 'omitnan'), mean(unity_rmse, 'omitnan'); ...
                      mean(gazebo_mae, 'omitnan'), mean(unity_mae, 'omitnan'); ...
                      mean(gazebo_std, 'omitnan'), mean(unity_std, 'omitnan')];
    
    bar(overall_metrics);
    title('总体性能对比');
    xlabel('性能指标');
    ylabel('数值 (rad)');
    legend('Gazebo', 'Unity');
    set(gca, 'XTickLabel', {'平均RMSE', '平均MAE', '平均标准差'});
    grid on;
    
    sgtitle('机械臂数据准确性汇总分析', 'FontSize', 16, 'FontWeight', 'bold');
end

function generate_accuracy_report_final(accuracy_results, joint_cols)

    fprintf('\n=== 生成详细报告 ===\n');
    

    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    report_filename = sprintf('robot_accuracy_report_%s.txt', timestamp);
    
    try
        fid = fopen(report_filename, 'w');
        
        fprintf(fid, '机械臂数据准确性分析报告\n');
        fprintf(fid, '生成时间: %s\n', datestr(now));
        fprintf(fid, '=====================================\n\n');
        

        fprintf(fid, '总体统计:\n');
        fprintf(fid, '分析关节数: %d\n', length(joint_cols));
        fprintf(fid, '关节列表: %s\n\n', strjoin(joint_cols, ', '));
        

        for i = 1:length(joint_cols)
            joint = joint_cols{i};
            if isfield(accuracy_results, joint)
                fprintf(fid, '%s:\n', joint);
                fprintf(fid, '  Gazebo vs 真实机械臂:\n');
                fprintf(fid, '    平均误差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_real.mean_error);
                fprintf(fid, '    标准差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_real.std_error);
                fprintf(fid, '    最大误差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_real.max_error);
                fprintf(fid, '    RMSE: %.6f rad\n', accuracy_results.(joint).gazebo_vs_real.rmse);
                fprintf(fid, '    MAE: %.6f rad\n', accuracy_results.(joint).gazebo_vs_real.mae);
                
                fprintf(fid, '  Unity vs 真实机械臂:\n');
                fprintf(fid, '    平均误差: %.6f rad\n', accuracy_results.(joint).unity_vs_real.mean_error);
                fprintf(fid, '    标准差: %.6f rad\n', accuracy_results.(joint).unity_vs_real.std_error);
                fprintf(fid, '    最大误差: %.6f rad\n', accuracy_results.(joint).unity_vs_real.max_error);
                fprintf(fid, '    RMSE: %.6f rad\n', accuracy_results.(joint).unity_vs_real.rmse);
                fprintf(fid, '    MAE: %.6f rad\n', accuracy_results.(joint).unity_vs_real.mae);
                
                fprintf(fid, '  Gazebo vs Unity:\n');
                fprintf(fid, '    平均误差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_unity.mean_error);
                fprintf(fid, '    标准差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_unity.std_error);
                fprintf(fid, '    最大误差: %.6f rad\n', accuracy_results.(joint).gazebo_vs_unity.max_error);
                fprintf(fid, '    RMSE: %.6f rad\n', accuracy_results.(joint).gazebo_vs_unity.rmse);
                fprintf(fid, '    MAE: %.6f rad\n\n', accuracy_results.(joint).gazebo_vs_unity.mae);
            end
        end
        

        fprintf(fid, '性能排名:\n');
        

        gazebo_total_rmse = 0;
        unity_total_rmse = 0;
        valid_joints = 0;
        
        for i = 1:length(joint_cols)
            joint = joint_cols{i};
            if isfield(accuracy_results, joint)
                gazebo_total_rmse = gazebo_total_rmse + accuracy_results.(joint).gazebo_vs_real.rmse;
                unity_total_rmse = unity_total_rmse + accuracy_results.(joint).unity_vs_real.rmse;
                valid_joints = valid_joints + 1;
            end
        end
        
        if valid_joints > 0
            gazebo_avg_rmse = gazebo_total_rmse / valid_joints;
            unity_avg_rmse = unity_total_rmse / valid_joints;
            
            fprintf(fid, '平均RMSE:\n');
            fprintf(fid, '  Gazebo: %.6f rad\n', gazebo_avg_rmse);
            fprintf(fid, '  Unity: %.6f rad\n', unity_avg_rmse);
            
            if gazebo_avg_rmse < unity_avg_rmse
                fprintf(fid, '结论: Gazebo虚拟机械臂的准确性更好\n');
            else
                fprintf(fid, '结论: Unity虚拟机械臂的准确性更好\n');
            end
        end
        
        fclose(fid);
        fprintf('详细报告已保存到: %s\n', report_filename);
        
    catch ME
        fprintf('生成报告时出错: %s\n', ME.message);
    end
end
