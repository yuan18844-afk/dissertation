function calculate_unity_real_metrics_complete()

    
    fprintf('=== Unity和真实机械臂数据对比分析 (完整版) ===\n');
    

    [real_data, unity_data] = load_data_complete();
    
    if isempty(real_data) || isempty(unity_data)
        fprintf('数据加载失败\n');
        return;
    end
    

    calculate_metrics_complete(real_data, unity_data);
end

function [real_data, unity_data] = load_data_complete()
  
    
    fprintf('请选择数据文件:\n');
    

    [real_file, real_path] = uigetfile('*.csv', '选择真实机械臂数据文件');
    if real_file == 0
        real_data = [];
        unity_data = [];
        return;
    end
    real_filepath = fullfile(real_path, real_file);
    
    [unity_file, unity_path] = uigetfile('*.csv', '选择Unity虚拟机械臂数据文件');
    if unity_file == 0
        real_data = [];
        unity_data = [];
        return;
    end
    unity_filepath = fullfile(unity_path, unity_file);
    
    try

        real_data = readtable(real_filepath);
        unity_data = readtable(unity_filepath);
        
        fprintf('数据加载成功:\n');
        fprintf('真实机械臂: %s (行数: %d)\n', real_file, height(real_data));
        fprintf('Unity虚拟机械臂: %s (行数: %d)\n', unity_file, height(unity_data));

        fprintf('\n列名信息:\n');
        fprintf('真实机械臂列名: %s\n', strjoin(real_data.Properties.VariableNames, ', '));
        fprintf('Unity列名: %s\n', strjoin(unity_data.Properties.VariableNames, ', '));
        
    catch ME
        fprintf('读取文件时出错: %s\n', ME.message);
        real_data = [];
        unity_data = [];
    end
end

function calculate_metrics_complete(real_data, unity_data)

    fprintf('\n=== 开始计算指标 ===\n');

    check_data_quality_complete(real_data, unity_data);

    joint_mapping = create_joint_mapping_complete(real_data, unity_data);
    if isempty(joint_mapping)
        fprintf('无法创建关节映射\n');
        return;
    end

    [synced_real, synced_unity] = synchronize_data_complete(real_data, unity_data);

    joint_names = fieldnames(joint_mapping);
    results = struct();
    
    fprintf('\n各关节指标计算结果:\n');
    fprintf('%-15s %-12s %-12s %-12s %-12s\n', '关节', 'MAE(rad)', 'RMSE(rad)', 'Latency(ms)', '相关性');
    fprintf('%-15s %-12s %-12s %-12s %-12s\n', '----', '--------', '---------', '----------', '----');
    
    total_mae = 0;
    total_rmse = 0;
    total_latency = 0;
    total_correlation = 0;
    valid_joints = 0;
    valid_latency_joints = 0;
    
    for i = 1:length(joint_names)
        joint = joint_names{i};
        mapping = joint_mapping.(joint);
        
        try

            real_joint = synced_real.(mapping.real);
            unity_joint = synced_unity.(mapping.unity);
            

            min_len = min([length(real_joint), length(unity_joint)]);
            real_joint = real_joint(1:min_len);
            unity_joint = unity_joint(1:min_len);
            

            error = real_joint - unity_joint;
            

            mae = mean(abs(error), 'omitnan');
            rmse = sqrt(mean(error.^2, 'omitnan'));
            

            [latency, correlation] = calculate_latency_with_correlation_complete(real_joint, unity_joint);

            results.(joint).mae = mae;
            results.(joint).rmse = rmse;
            results.(joint).latency = latency;
            results.(joint).correlation = correlation;
            

            total_mae = total_mae + mae;
            total_rmse = total_rmse + rmse;
            total_correlation = total_correlation + correlation;
            valid_joints = valid_joints + 1;
            
            if ~isnan(latency)
                total_latency = total_latency + latency;
                valid_latency_joints = valid_latency_joints + 1;
            end
            

            if isnan(latency)
                fprintf('%-15s %-12.6f %-12.6f %-12s %-12.3f\n', ...
                    joint, mae, rmse, 'N/A', correlation);
            else
                fprintf('%-15s %-12.6f %-12.6f %-12.2f %-12.3f\n', ...
                    joint, mae, rmse, latency, correlation);
            end
            
        catch ME
            fprintf('处理关节 %s 时出错: %s\n', joint, ME.message);
        end
    end
    

    if valid_joints > 0
        avg_mae = total_mae / valid_joints;
        avg_rmse = total_rmse / valid_joints;
        avg_correlation = total_correlation / valid_joints;
        
        if valid_latency_joints > 0
            avg_latency = total_latency / valid_latency_joints;
        else
            avg_latency = NaN;
        end
        
        fprintf('%-15s %-12s %-12s %-12s %-12s\n', '----', '--------', '---------', '----------', '----');
        if isnan(avg_latency)
            fprintf('%-15s %-12.6f %-12.6f %-12s %-12.3f\n', ...
                '平均值', avg_mae, avg_rmse, 'N/A', avg_correlation);
        else
            fprintf('%-15s %-12.6f %-12.6f %-12.2f %-12.3f\n', ...
                '平均值', avg_mae, avg_rmse, avg_latency, avg_correlation);
        end
        

        generate_detailed_report_complete(results, joint_names, avg_mae, avg_rmse, avg_latency, avg_correlation);
    else
        fprintf('没有有效的关节数据可以计算\n');
    end
end

function check_data_quality_complete(real_data, unity_data)

    fprintf('\n=== 数据质量检查 ===\n');
    

    real_cols = real_data.Properties.VariableNames;
    unity_cols = unity_data.Properties.VariableNames;
    
    joint_cols = {};
    for i = 1:length(real_cols)
        col = real_cols{i};
        if contains(col, 'joint') && ~contains(col, 'time')
            joint_cols{end+1} = col;
        end
    end
    
    fprintf('检查 %d 个关节的数据质量:\n', length(joint_cols));
    
    for i = 1:length(joint_cols)
        joint = joint_cols{i};
        
        if ismember(joint, real_cols) && ismember(joint, unity_cols)
            real_joint = real_data.(joint);
            unity_joint = unity_data.(joint);
            

            real_nan_ratio = sum(isnan(real_joint)) / length(real_joint);
            unity_nan_ratio = sum(isnan(unity_joint)) / length(unity_joint);
            

            real_valid = real_joint(~isnan(real_joint));
            unity_valid = unity_joint(~isnan(unity_joint));
            
            if ~isempty(real_valid) && ~isempty(unity_valid)

                real_range = max(real_valid) - min(real_valid);
                unity_range = max(unity_valid) - min(unity_valid);
                
                fprintf('%-15s: NaN比例(真实/Unity): %.2f%%/%.2f%%, 范围: %.3f/%.3f\n', ...
                    joint, real_nan_ratio*100, unity_nan_ratio*100, real_range, unity_range);
            else
                fprintf('%-15s: 数据无效\n', joint);
            end
        end
    end
end

function joint_mapping = create_joint_mapping_complete(real_data, unity_data)

    real_cols = real_data.Properties.VariableNames;
    unity_cols = unity_data.Properties.VariableNames;
    

    real_joints = {};
    unity_joints = {};
    

    for i = 1:length(real_cols)
        col = real_cols{i};

        if (startsWith(col, 'joint') && length(col) > 5) || ...
           (contains(col, 'joint') && contains(col, 'panda')) || ...
           (contains(col, 'joint') && ~contains(col, 'time'))
            real_joints{end+1} = col;
        end
    end
    

    for i = 1:length(unity_cols)
        col = unity_cols{i};

        if (startsWith(col, 'joint') && length(col) > 5) || ...
           (contains(col, 'joint') && contains(col, 'panda')) || ...
           (contains(col, 'joint') && ~contains(col, 'time'))
            unity_joints{end+1} = col;
        end
    end
    
    fprintf('找到关节列:\n');
    fprintf('真实机械臂: %s\n', strjoin(real_joints, ', '));
    fprintf('Unity: %s\n', strjoin(unity_joints, ', '));
    
    if isempty(real_joints) || isempty(unity_joints)
        fprintf('错误: 未找到关节列\n');
        joint_mapping = [];
        return;
    end
    
    joint_mapping = struct();
    

    real_nums = zeros(1, length(real_joints));
    for i = 1:length(real_joints)
        num_str = regexp(real_joints{i}, '\d+', 'match');
        if ~isempty(num_str)
            real_nums(i) = str2double(num_str{1});
        end
    end
    [~, real_sort_idx] = sort(real_nums);
    real_joints = real_joints(real_sort_idx);
    
    unity_nums = zeros(1, length(unity_joints));
    for i = 1:length(unity_joints)
        num_str = regexp(unity_joints{i}, '\d+', 'match');
        if ~isempty(num_str)
            unity_nums(i) = str2double(num_str{1});
        end
    end
    [~, unity_sort_idx] = sort(unity_nums);
    unity_joints = unity_joints(unity_sort_idx);
    

    min_joints = min([length(real_joints), length(unity_joints)]);
    
    for i = 1:min_joints
        joint_name = sprintf('joint%d', i);
        joint_mapping.(joint_name).real = real_joints{i};
        joint_mapping.(joint_name).unity = unity_joints{i};
    end
    
    fprintf('创建了 %d 个关节映射\n', min_joints);
    

    joint_names = fieldnames(joint_mapping);
    for i = 1:length(joint_names)
        joint = joint_names{i};
        fprintf('映射 %s: %s -> %s\n', joint, joint_mapping.(joint).real, joint_mapping.(joint).unity);
    end
end

function [synced_real, synced_unity] = synchronize_data_complete(real_data, unity_data)

    
    fprintf('正在同步数据...\n');
    

    time_col = 'time';
    

    if ~ismember(time_col, real_data.Properties.VariableNames) || ...
       ~ismember(time_col, unity_data.Properties.VariableNames)
        fprintf('警告: 某些数据缺少时间列，使用索引同步\n');

        min_len = min([height(real_data), height(unity_data)]);
        synced_real = real_data(1:min_len, :);
        synced_unity = unity_data(1:min_len, :);
        return;
    end
    

    real_times = real_data.(time_col);
    unity_times = unity_data.(time_col);
    

    start_time = max([min(real_times), min(unity_times)]);
    end_time = min([max(real_times), max(unity_times)]);
    

    time_tolerance = 0.1; % 100ms
    real_idx = real_times >= (start_time - time_tolerance) & real_times <= (end_time + time_tolerance);
    unity_idx = unity_times >= (start_time - time_tolerance) & unity_times <= (end_time + time_tolerance);
    
    synced_real = real_data(real_idx, :);
    synced_unity = unity_data(unity_idx, :);
    
    fprintf('同步完成: 真实机械臂 %d 点, Unity %d 点\n', ...
        height(synced_real), height(synced_unity));
    

    if height(synced_real) < 100 || height(synced_unity) < 100
        fprintf('警告: 同步后的数据点较少，可能影响分析精度\n');
    end
end

function [latency, correlation] = calculate_latency_with_correlation_complete(real_signal, unity_signal)

    try

        valid_idx = ~isnan(real_signal) & ~isnan(unity_signal);
        real_clean = real_signal(valid_idx);
        unity_clean = unity_signal(valid_idx);
        

        if length(real_clean) < 50
            latency = NaN;
            correlation = NaN;
            return;
        end
        

        real_detrend = detrend(real_clean);
        unity_detrend = detrend(unity_clean);
        

        real_norm = (real_detrend - mean(real_detrend)) / std(real_detrend);
        unity_norm = (unity_detrend - mean(unity_detrend)) / std(unity_detrend);
        

        [c, lags] = xcorr(real_norm, unity_norm, 'coeff');
        

        [max_corr, max_idx] = max(abs(c));
        lag = lags(max_idx);
        

        sampling_freq = 100; % Hz
        latency = abs(lag) / sampling_freq * 1000; % 转换为毫秒

        if latency > 1000  
            latency = NaN;
        end
        

        if max_corr < 0.3  
            latency = NaN;
        end
        
        correlation = max_corr;
        
    catch
        latency = NaN;
        correlation = NaN;
    end
end

function generate_detailed_report_complete(results, joint_names, avg_mae, avg_rmse, avg_latency, avg_correlation)

    
    fprintf('\n=== 生成报告 ===\n');
    

    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    report_filename = sprintf('unity_real_metrics_complete_%s.txt', timestamp);
    
    try
        fid = fopen(report_filename, 'w');
        
        fprintf(fid, 'Unity和真实机械臂数据对比分析报告 (完整版)\n');
        fprintf(fid, '生成时间: %s\n', datestr(now));
        fprintf(fid, '=====================================\n\n');
        
        % 总体统计
        fprintf(fid, '总体统计:\n');
        fprintf(fid, '分析关节数: %d\n', length(joint_names));
        fprintf(fid, '关节列表: %s\n\n', strjoin(joint_names, ', '));
        
        % 各关节详细结果
        fprintf(fid, '各关节详细结果:\n');
        fprintf(fid, '%-15s %-12s %-12s %-12s %-12s\n', '关节', 'MAE(rad)', 'RMSE(rad)', 'Latency(ms)', '相关性');
        fprintf(fid, '%-15s %-12s %-12s %-12s %-12s\n', '----', '--------', '---------', '----------', '----');
        
        for i = 1:length(joint_names)
            joint = joint_names{i};
            if isfield(results, joint)
                if isnan(results.(joint).latency)
                    fprintf(fid, '%-15s %-12.6f %-12.6f %-12s %-12.3f\n', ...
                        joint, results.(joint).mae, results.(joint).rmse, 'N/A', results.(joint).correlation);
                else
                    fprintf(fid, '%-15s %-12.6f %-12.6f %-12.2f %-12.3f\n', ...
                        joint, results.(joint).mae, results.(joint).rmse, results.(joint).latency, results.(joint).correlation);
                end
            end
        end
        
        fprintf(fid, '%-15s %-12s %-12s %-12s %-12s\n', '----', '--------', '---------', '----------', '----');
        if isnan(avg_latency)
            fprintf(fid, '%-15s %-12.6f %-12.6f %-12s %-12.3f\n', ...
                '平均值', avg_mae, avg_rmse, 'N/A', avg_correlation);
        else
            fprintf(fid, '%-15s %-12.6f %-12.6f %-12.2f %-12.3f\n', ...
                '平均值', avg_mae, avg_rmse, avg_latency, avg_correlation);
        end
        
        % 性能分析
        fprintf(fid, '\n性能分析:\n');
        fprintf(fid, '平均MAE: %.6f rad (%.2f°)\n', avg_mae, avg_mae * 180/pi);
        fprintf(fid, '平均RMSE: %.6f rad (%.2f°)\n', avg_rmse, avg_rmse * 180/pi);
        if ~isnan(avg_latency)
            fprintf(fid, '平均延迟: %.2f ms\n', avg_latency);
        else
            fprintf(fid, '平均延迟: 无法计算\n');
        end
        fprintf(fid, '平均相关性: %.3f\n', avg_correlation);
        
        % 性能评级
        fprintf(fid, '\n性能评级:\n');
        
        % MAE评级
        if avg_mae < 0.02  % < 1.15°
            fprintf(fid, 'MAE性能: 优秀 (< 0.02 rad)\n');
        elseif avg_mae < 0.05  % < 2.87°
            fprintf(fid, 'MAE性能: 良好 (0.02-0.05 rad)\n');
        elseif avg_mae < 0.1   % < 5.73°
            fprintf(fid, 'MAE性能: 可接受 (0.05-0.1 rad)\n');
        else
            fprintf(fid, 'MAE性能: 需要改进 (> 0.1 rad)\n');
        end
        
        % 延迟评级
        if isnan(avg_latency)
            fprintf(fid, '延迟性能: 无法评估\n');
        elseif avg_latency < 50
            fprintf(fid, '延迟性能: 优秀 (< 50 ms)\n');
        elseif avg_latency < 200
            fprintf(fid, '延迟性能: 良好 (50-200 ms)\n');
        elseif avg_latency < 500
            fprintf(fid, '延迟性能: 可接受 (200-500 ms)\n');
        else
            fprintf(fid, '延迟性能: 需要改进 (> 500 ms)\n');
        end
        
        % 相关性评级
        if avg_correlation > 0.9
            fprintf(fid, '相关性性能: 优秀 (> 0.9)\n');
        elseif avg_correlation > 0.7
            fprintf(fid, '相关性性能: 良好 (0.7-0.9)\n');
        elseif avg_correlation > 0.5
            fprintf(fid, '相关性性能: 可接受 (0.5-0.7)\n');
        else
            fprintf(fid, '相关性性能: 需要改进 (< 0.5)\n');
        end
        
        % 建议
        fprintf(fid, '\n改进建议:\n');
        if avg_mae > 0.1
            fprintf(fid, '- 检查机械臂动力学模型参数\n');
            fprintf(fid, '- 优化轨迹规划算法\n');
        end
        if ~isnan(avg_latency) && avg_latency > 500
            fprintf(fid, '- 检查ROS通信延迟\n');
            fprintf(fid, '- 优化Unity渲染频率\n');
        end
        if avg_correlation < 0.5
            fprintf(fid, '- 检查数据同步问题\n');
            fprintf(fid, '- 验证传感器数据质量\n');
        end
        
        fclose(fid);
        fprintf('详细报告已保存到: %s\n', report_filename);
        
    catch ME
        fprintf('生成报告时出错: %s\n', ME.message);
    end
end