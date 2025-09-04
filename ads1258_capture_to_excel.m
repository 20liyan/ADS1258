function ads1258_capture_to_excel(comPort, baud, nRows, outFile)
% ads1258_capture_to_excel - 从 STM32 串口采集 ADS1258 差分数据并保存到 Excel
% 兼容 MATLAB 2013a
% 用例: ads1258_capture_to_excel('COM4',115200,66,'0.xlsx')

if nargin < 1, comPort = 'COM4'; end
if nargin < 2, baud    = 115200; end
if nargin < 3, nRows   = 1000;   end
if nargin < 4, outFile = 'ads_data.xlsx'; end

% 初始化串口对象
s = [];
try
    s = serial(comPort, 'BaudRate', baud, 'Terminator', 'LF', 'Timeout', 5);
    fopen(s);
    pause(0.1);
    flushinput(s);
catch ME
    if ~isempty(s) && isvalid(s)
        try fclose(s); delete(s); end
    end
    error('无法打开串口 %s: %s', comPort, ME.message);
end

% 预分配
data = NaN(nRows, 6);
row = 0;

fprintf('Capturing %d rows from %s @ %d baud...\n', nRows, comPort, baud);

% 主循环：接收并处理
success = false;
try
    while row < nRows
        % 读一行（阻塞直到 terminator 或超时）
        line = strtrim(fgetl(s));
        if isempty(line)
            continue;
        end

        % 按逗号分割，期望 12 个字段
        parts = strread(line, '%s', 'delimiter', ','); %#ok<DTSTRRD>
        if numel(parts) ~= 12
            continue;
        end

        raw = str2double(parts);
        if any(isnan(raw))
            continue;
        end

        % 原始码 -> 电压
        volts = raw * (2.5 / hex2dec('780000'));

        % 计算差分 DIFF0..DIFF5: (1-0),(3-2),...
        diffs = zeros(1,6);
        for k = 1:6
            diffs(k) = volts(2*k) - volts(2*k-1);
        end

        row = row + 1;
        data(row, :) = diffs;

        if mod(row,100) == 0 || row == nRows
            fprintf('  %d / %d rows\n', row, nRows);
        end
    end

    success = true; % 正常完成
catch ME
    warning('采集中断: %s', ME.message);
    % success 保持 false（表示非正常完成）
end

% 统一清理与保存（只调用一次）
finalize_and_save(s, outFile, data, row, nRows, success);

end


%% 辅助函数：关闭串口并保存数据（独立函数，避免引用外层工作区）
function finalize_and_save(s, outFile, data, row, nRows, success)
% 关闭串口
try
    if ~isempty(s) && isvalid(s)
        fclose(s);
        delete(s);
    end
catch
end

% 保存数据（只保存已采集行）
if row > 0
    dataToSave = data(1:row, :);
    varNames = arrayfun(@(k) ['DIFF', num2str(k)], 0:5, 'UniformOutput', false);
    try
        xlswrite(outFile, [varNames; num2cell(dataToSave)]);
    catch ME
        warning('保存 Excel 失败: %s. 尝试以 CSV 保存', ME.message);
        % 备用：写 CSV
        try
            csvFile = [outFile(1:end-4), '.csv'];
            fid = fopen(csvFile, 'w');
            if fid ~= -1
                % 写表头
                fprintf(fid, '%s', varNames{1});
                for ii = 2:numel(varNames)
                    fprintf(fid, ',%s', varNames{ii});
                end
                fprintf(fid, '\n');
                % 写数据
                for r = 1:size(dataToSave,1)
                    fprintf(fid, '%g', dataToSave(r,1));
                    for c = 2:size(dataToSave,2)
                        fprintf(fid, ',%g', dataToSave(r,c));
                    end
                    fprintf(fid, '\n');
                end
                fclose(fid);
                fprintf('已保存为 CSV: %s\n', csvFile);
            end
        catch
        end
    end

    if success && row >= nRows
        fprintf('正常完成: %d / %d rows 已保存到 %s\n', row, nRows, outFile);
    else
        fprintf('采集中断: 已保存 %d / %d rows 到 %s\n', row, nRows, outFile);
    end
else
    fprintf('没有采集到任何数据\n');
end

end
