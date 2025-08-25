 function analyze_communication_latency_fixed()
    fprintf('=== Unity和Gazebo通信延迟性能分析 (修复版) ===\n');
    % 设置文件路径
    %real_path   = 'C:\Users\14595\Desktop\data\real_static1_20250728_062551.csv';
    %unity_path  = 'C:\Users\14595\Desktop\data\unity_static1_20250728_062551.csv';
    %gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_static1_20250728_062551.csv';

    %real_path   = 'C:\Users\14595\Desktop\data\real_slow3_20250728_065324.csv';
    %unity_path  = 'C:\Users\14595\Desktop\data\unity_slow3_20250728_065324.csv';
    %gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_slow3_20250728_065324.csv';
    real_path   = 'C:\Users\14595\Desktop\data\real_slow2_20250728_065133.csv';
    unity_path  = 'C:\Users\14595\Desktop\data\unity_slow2_20250728_065133.csv';
    gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_slow2_20250728_065133.csv';

    %real_path   = 'C:\Users\14595\Desktop\data\real_moveit_20250801_094114.csv';
    %unity_path  = 'C:\Users\14595\Desktop\data\unity_moveit_20250801_094114.csv';
    %gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_moveit_synth_ep01.csv';
    %real_path   = 'C:\Users\14595\Desktop\data\real_smooth1_20250728_072246.csv';
    %unity_path  = 'C:\Users\14595\Desktop\data\unity_smooth1_20250728_072246.csv';
    %gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_smooth1_20250728_072246.csv';
    % 设置文件路径
    real_path   = 'C:\Users\14595\Desktop\data\real_slow2_20250728_065133.csv';
    unity_path  = 'C:\Users\14595\Desktop\data\unity_slow2_20250728_065133.csv';
    gazebo_path = 'C:\Users\14595\Desktop\data\gazebo_slow2_20250728_065133.csv';

    save_plot = true;

    if ~exist(real_path, 'file') || ~exist(unity_path, 'file') || ~exist(gazebo_path, 'file')
        error('数据文件不存在，请检查路径！');
    end

    fprintf('文件路径检查完成，开始读取数据...\n');

    [real_data, unity_data, gazebo_data] = load_data_fixed(real_path, unity_path, gazebo_path);
    [real_time_ms, unity_time_ms, gazebo_time_ms] = standardize_timestamps_fixed(real_data, unity_data, gazebo_data);

    % 时间偏移统一起点（避免负延迟）
    min_start_time = max([real_time_ms(1), unity_time_ms(1), gazebo_time_ms(1)]);
    real_time_ms   = real_time_ms   - min_start_time;
    unity_time_ms  = unity_time_ms  - min_start_time;
    gazebo_time_ms = gazebo_time_ms - min_start_time;

    % 输出起始时间日志
    fprintf('\n⏱️ 起始时间（第一行数据）:\n');
    fprintf('Real   : %.9f\n', real_time_ms(1)/1000);
    fprintf('Unity  : %.9f\n', unity_time_ms(1)/1000);
    fprintf('Gazebo : %.9f\n', gazebo_time_ms(1)/1000);

    [unity_latency_ms, gazebo_latency_ms] = calculate_latency_fixed(real_time_ms, unity_time_ms, gazebo_time_ms);
    create_latency_boxplot_fixed(unity_latency_ms, gazebo_latency_ms, save_plot);
    generate_latency_report_fixed(unity_latency_ms, gazebo_latency_ms);
end


function [real_data, unity_data, gazebo_data] = load_data_fixed(real_path, unity_path, gazebo_path)

    
    try
        % 读取真实机器人数据
        real_data = readtable(real_path);
        fprintf('真实机器人数据加载成功: %s (行数: %d)\n', real_path, height(real_data));
        
        % 读取Unity数据
        unity_data = readtable(unity_path);
        fprintf('Unity数据加载成功: %s (行数: %d)\n', unity_path, height(unity_data));
        
        % 读取Gazebo数据
        gazebo_data = readtable(gazebo_path);
        fprintf('Gazebo数据加载成功: %s (行数: %d)\n', gazebo_path, height(gazebo_data));
        
        % 显示列名信息
        fprintf('\n列名信息:\n');
        fprintf('真实机器人: %s\n', strjoin(real_data.Properties.VariableNames, ', '));
        fprintf('Unity: %s\n', strjoin(unity_data.Properties.VariableNames, ', '));
        fprintf('Gazebo: %s\n', strjoin(gazebo_data.Properties.VariableNames, ', '));
        
    catch ME
        fprintf('读取数据时出错: %s\n', ME.message);
        real_data = [];
        unity_data = [];
        gazebo_data = [];
    end
end

function [real_time_ms, unity_time_ms, gazebo_time_ms] = standardize_timestamps_fixed(real_data, unity_data, gazebo_data)

    
    fprintf('\n=== 标准化时间戳 ===\n');
    
    % 获取时间列
    time_col = 'time';
    
    % 检查时间列是否存在
    if ~ismember(time_col, real_data.Properties.VariableNames) || ...
       ~ismember(time_col, unity_data.Properties.VariableNames) || ...
       ~ismember(time_col, gazebo_data.Properties.VariableNames)
        fprintf('错误: 某些数据缺少时间列\n');
        real_time_ms = [];
        unity_time_ms = [];
        gazebo_time_ms = [];
        return;
    end
    

    real_time = real_data.(time_col);
    unity_time = unity_data.(time_col);
    gazebo_time = gazebo_data.(time_col);
    

    real_time_ms = detect_and_convert_time_fixed(real_time, '真实机器人');
    unity_time_ms = detect_and_convert_time_fixed(unity_time, 'Unity');
    gazebo_time_ms = detect_and_convert_time_fixed(gazebo_time, 'Gazebo');
    
    fprintf('时间戳标准化完成\n');
end

function time_ms = detect_and_convert_time_fixed(time_data, platform_name)


    time_range = max(time_data) - min(time_data);
    time_mean = mean(time_data);
    
    fprintf('%s时间戳分析:\n', platform_name);
    fprintf('  时间范围: %.6f\n', time_range);
    fprintf('  平均时间: %.6f\n', time_mean);
    
    
    if time_range > 1e9  % 纳秒级
        time_ms = time_data / 1e6;  % 纳秒转毫秒
        fprintf('  检测到纳秒单位，转换为毫秒\n');
    elseif time_range > 1e6  % 微秒级
        time_ms = time_data / 1e3;  % 微秒转毫秒
        fprintf('  检测到微秒单位，转换为毫秒\n');
    elseif time_range > 1e3  % 毫秒级
        time_ms = time_data;  % 已经是毫秒
        fprintf('  检测到毫秒单位，无需转换\n');
    else  % 秒级
        time_ms = time_data * 1e3;  % 秒转毫秒
        fprintf('  检测到秒单位，转换为毫秒\n');
    end
    
    fprintf('  转换后时间范围: %.2f ms\n', max(time_ms) - min(time_ms));
end

function [unity_latency_ms, gazebo_latency_ms] = calculate_latency_fixed(real_time_ms, unity_time_ms, gazebo_time_ms)

    
    fprintf('\n=== 计算通信延迟 ===\n');
    [synced_real, synced_unity, synced_gazebo] = synchronize_timestamps_fixed(real_time_ms, unity_time_ms, gazebo_time_ms);
    
    if isempty(synced_real)
        fprintf('时间同步失败\n');
        unity_latency_ms = [];
        gazebo_latency_ms = [];
        return;
    end
    
    % 计算latency
    unity_latency_ms = synced_unity - synced_real;
    gazebo_latency_ms = synced_gazebo - synced_real;
    fprintf('延迟分析:\n');
    fprintf('Unity延迟范围: %.2f 到 %.2f ms\n', min(unity_latency_ms), max(unity_latency_ms));
    fprintf('Gazebo延迟范围: %.2f 到 %.2f ms\n', min(gazebo_latency_ms), max(gazebo_latency_ms));
    
    % 修正
    if all(unity_latency_ms < 0) && all(gazebo_latency_ms < 0)
        fprintf('警告: 所有延迟值都是负数，可能存在时间戳同步问题\n');
        fprintf('尝试修正延迟计算...\n');
        
        % 使用绝对延迟值
        unity_latency_ms = abs(unity_latency_ms);
        gazebo_latency_ms = abs(gazebo_latency_ms);
        
        fprintf('修正后延迟范围:\n');
        fprintf('Unity延迟范围: %.2f 到 %.2f ms\n', min(unity_latency_ms), max(unity_latency_ms));
        fprintf('Gazebo延迟范围: %.2f 到 %.2f ms\n', min(gazebo_latency_ms), max(gazebo_latency_ms));
    end
    
    % 移除异常值
    unity_latency_ms = remove_outliers_fixed(unity_latency_ms, 'Unity');
    gazebo_latency_ms = remove_outliers_fixed(gazebo_latency_ms, 'Gazebo');
    
    fprintf('延迟计算完成:\n');
    fprintf('Unity延迟样本数: %d\n', length(unity_latency_ms));
    fprintf('Gazebo延迟样本数: %d\n', length(gazebo_latency_ms));
end

function [synced_real, synced_unity, synced_gazebo] = synchronize_timestamps_fixed(real_time_ms, unity_time_ms, gazebo_time_ms)

    
    fprintf('正在同步时间戳...\n');

    start_time = max([min(real_time_ms), min(unity_time_ms), min(gazebo_time_ms)]);
    end_time = min([max(real_time_ms), max(unity_time_ms), max(gazebo_time_ms)]);
    
    fprintf('共同时间范围: %.2f ms - %.2f ms\n', start_time, end_time);
    
    % 选择时间范围内的数据
    real_idx = real_time_ms >= start_time & real_time_ms <= end_time;
    unity_idx = unity_time_ms >= start_time & unity_time_ms <= end_time;
    gazebo_idx = gazebo_time_ms >= start_time & gazebo_time_ms <= end_time;
    
    synced_real = real_time_ms(real_idx);
    synced_unity = unity_time_ms(unity_idx);
    synced_gazebo = gazebo_time_ms(gazebo_idx);
    
    fprintf('同步完成: 真实机器人 %d 点, Unity %d 点, Gazebo %d 点\n', ...
        length(synced_real), length(synced_unity), length(synced_gazebo));
    
    % 确保数据长度一致
    min_len = min([length(synced_real), length(synced_unity), length(synced_gazebo)]);
    synced_real = synced_real(1:min_len);
    synced_unity = synced_unity(1:min_len);
    synced_gazebo = synced_gazebo(1:min_len);
    
    fprintf('最终同步数据长度: %d 点\n', min_len);
end

function clean_latency = remove_outliers_fixed(latency_data, platform_name)

    % 使用IQR方法检测异常值
    Q1 = prctile(latency_data, 25);
    Q3 = prctile(latency_data, 75);
    IQR = Q3 - Q1;
    lower_bound = Q1 - 1.5 * IQR;
    upper_bound = Q3 + 1.5 * IQR;
    
    % 标记异常值
    outlier_idx = latency_data < lower_bound | latency_data > upper_bound;
    outlier_count = sum(outlier_idx);
    
    fprintf('%s延迟异常值检测:\n', platform_name);
    fprintf('  异常值数量: %d (%.2f%%)\n', outlier_count, outlier_count/length(latency_data)*100);
    fprintf('  异常值范围: < %.2f ms 或 > %.2f ms\n', lower_bound, upper_bound);
    
    % 移除异常值
    clean_latency = latency_data(~outlier_idx);
    
    fprintf('  清理后样本数: %d\n', length(clean_latency));
end

function create_latency_boxplot_fixed(unity_latency_ms, gazebo_latency_ms, save_plot)

    
    fprintf('\n=== 绘制延迟箱型图 ===\n');
    
    % 创建图形窗口
    figure('Name', 'Unity vs Gazebo 通信延迟对比', 'Position', [100, 100, 800, 600]);
    
    % 准备数据
    latency_data = [unity_latency_ms; gazebo_latency_ms];
    group_labels = [repmat({'Unity'}, length(unity_latency_ms), 1); ...
                   repmat({'Gazebo'}, length(gazebo_latency_ms), 1)];
   
    
    % 设置图形属性
    title('Unity vs Gazebo 通信延迟性能对比', 'FontSize', 16, 'FontWeight', 'bold');
    ylabel('延迟时间 (ms)', 'FontSize', 14);
    xlabel('仿真平台', 'FontSize', 14);
    grid on;
    
    % 设置颜色
    h = findobj(gca, 'Tag', 'Box');
    if length(h) >= 2
        set(h(1), 'Color', [0.2, 0.6, 1.0]);  % Unity蓝色
        set(h(2), 'Color', [1.0, 0.4, 0.2]);  % Gazebo橙色
    end
    
    % 添加统计信息
    unity_median = median(unity_latency_ms);
    gazebo_median = median(gazebo_latency_ms);
    unity_mean = mean(unity_latency_ms);
    gazebo_mean = mean(gazebo_latency_ms);
    
    text(0.7, 0.9, sprintf('Unity中位数: %.2f ms\nUnity平均值: %.2f ms\nGazebo中位数: %.2f ms\nGazebo平均值: %.2f ms', ...
        unity_median, unity_mean, gazebo_median, gazebo_mean), ...
        'Units', 'normalized', 'FontSize', 10, 'BackgroundColor', 'white', ...
        'EdgeColor', 'black', 'LineWidth', 1);
    
    % 修复y轴范围设置
    all_data = [unity_latency_ms; gazebo_latency_ms];
    data_min = min(all_data);
    data_max = max(all_data);
    
    if data_min < data_max
        ylim([data_min * 0.9, data_max * 1.1]);
    else
        ylim([data_min - 1, data_max + 1]);
    end
    
    % 保存图像
    if save_plot
        
        if ~exist('plots', 'dir')
            mkdir('plots');
        end
        
        
        filename = 'plots/latency_boxplot_fixed.png';
        print(filename, '-dpng', '-r300');
        fprintf('图像已保存: %s\n', filename);
        
        
        filename_eps = 'plots/latency_boxplot_fixed.eps';
        print(filename_eps, '-depsc', '-r300');
        fprintf('图像已保存: %s\n', filename_eps);
    end
    
    fprintf('箱型图绘制完成\n');
end

function generate_latency_report_fixed(unity_latency_ms, gazebo_latency_ms)
    % 生成延迟性能统计报告 
    
    fprintf('\n=== 延迟性能统计报告 ===\n');
    
    % 计算统计指标
    unity_stats = calculate_statistics_fixed(unity_latency_ms, 'Unity');
    gazebo_stats = calculate_statistics_fixed(gazebo_latency_ms, 'Gazebo');
    
    % 性能对比
    fprintf('\n性能对比:\n');
    if unity_stats.median < gazebo_stats.median
        fprintf('✓ Unity的中位数延迟更低 (%.2f ms vs %.2f ms)\n', unity_stats.median, gazebo_stats.median);
    else
        fprintf('✗ Gazebo的中位数延迟更低 (%.2f ms vs %.2f ms)\n', gazebo_stats.median, unity_stats.median);
    end
    
    if unity_stats.std < gazebo_stats.std
        fprintf('✓ Unity的延迟更稳定 (标准差: %.2f ms vs %.2f ms)\n', unity_stats.std, gazebo_stats.std);
    else
        fprintf('✗ Gazebo的延迟更稳定 (标准差: %.2f ms vs %.2f ms)\n', gazebo_stats.std, unity_stats.std);
    end
    
    % 保存报告
    save_report_to_file_fixed(unity_stats, gazebo_stats);
end

function stats = calculate_statistics_fixed(latency_data, platform_name)
    % 计算统计指标 
    
    stats.median = median(latency_data);
    stats.mean = mean(latency_data);
    stats.std = std(latency_data);
    stats.min = min(latency_data);
    stats.max = max(latency_data);
    stats.q1 = prctile(latency_data, 25);
    stats.q3 = prctile(latency_data, 75);
    stats.iqr = stats.q3 - stats.q1;
    
    fprintf('%s延迟统计:\n', platform_name);
    fprintf('  样本数: %d\n', length(latency_data));
    fprintf('  中位数: %.2f ms\n', stats.median);
    fprintf('  平均值: %.2f ms\n', stats.mean);
    fprintf('  标准差: %.2f ms\n', stats.std);
    fprintf('  最小值: %.2f ms\n', stats.min);
    fprintf('  最大值: %.2f ms\n', stats.max);
    fprintf('  Q1: %.2f ms\n', stats.q1);
    fprintf('  Q3: %.2f ms\n', stats.q3);
    fprintf('  IQR: %.2f ms\n', stats.iqr);
end

function save_report_to_file_fixed(unity_stats, gazebo_stats)
   
    
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    report_filename = sprintf('latency_analysis_report_fixed_%s.txt', timestamp);
    
    try
        fid = fopen(report_filename, 'w');
        
        fprintf(fid, 'Unity vs Gazebo 通信延迟性能分析报告 (修复版)\n');
        fprintf(fid, '生成时间: %s\n', datestr(now));
        fprintf(fid, '=====================================\n\n');
        
        % Unity
        fprintf(fid, 'Unity延迟统计:\n');
        fprintf(fid, '  中位数: %.2f ms\n', unity_stats.median);
        fprintf(fid, '  平均值: %.2f ms\n', unity_stats.mean);
        fprintf(fid, '  标准差: %.2f ms\n', unity_stats.std);
        fprintf(fid, '  最小值: %.2f ms\n', unity_stats.min);
        fprintf(fid, '  最大值: %.2f ms\n', unity_stats.max);
        fprintf(fid, '  Q1: %.2f ms\n', unity_stats.q1);
        fprintf(fid, '  Q3: %.2f ms\n', unity_stats.q3);
        fprintf(fid, '  IQR: %.2f ms\n\n', unity_stats.iqr);
        
        % Gazebo
        fprintf(fid, 'Gazebo延迟统计:\n');
        fprintf(fid, '  中位数: %.2f ms\n', gazebo_stats.median);
        fprintf(fid, '  平均值: %.2f ms\n', gazebo_stats.mean);
        fprintf(fid, '  标准差: %.2f ms\n', gazebo_stats.std);
        fprintf(fid, '  最小值: %.2f ms\n', gazebo_stats.min);
        fprintf(fid, '  最大值: %.2f ms\n', gazebo_stats.max);
        fprintf(fid, '  Q1: %.2f ms\n', gazebo_stats.q1);
        fprintf(fid, '  Q3: %.2f ms\n', gazebo_stats.q3);
        fprintf(fid, '  IQR: %.2f ms\n\n', gazebo_stats.iqr);
        
        % 性能对比
        fprintf(fid, '性能对比:\n');
        if unity_stats.median < gazebo_stats.median
            fprintf(fid, '✓ Unity的中位数延迟更低 (%.2f ms vs %.2f ms)\n', unity_stats.median, gazebo_stats.median);
        else
            fprintf(fid, '✗ Gazebo的中位数延迟更低 (%.2f ms vs %.2f ms)\n', gazebo_stats.median, unity_stats.median);
        end
        
        if unity_stats.std < gazebo_stats.std
            fprintf(fid, '✓ Unity的延迟更稳定 (标准差: %.2f ms vs %.2f ms)\n', unity_stats.std, gazebo_stats.std);
        else
            fprintf(fid, '✗ Gazebo的延迟更稳定 (标准差: %.2f ms vs %.2f ms)\n', gazebo_stats.std, unity_stats.std);
        end
        
        fclose(fid);
        fprintf('详细报告已保存到: %s\n', report_filename);
        
    catch ME
        fprintf('保存报告时出错: %s\n', ME.message);
    end
end