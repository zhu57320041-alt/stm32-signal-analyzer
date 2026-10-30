%% 串口示波器 FFT 分析 — 动态版本
% 实时读取串口数据，动态绘制波形和FFT频谱

clear; clc; close all;

%% 串口配置
port = 'COM16';         % 根据实际串口号修改
baudrate = 921600;      % 波特率

serialPort = serialport(port, baudrate, "Timeout", 1);
flush(serialPort);
fprintf('串口 %s 已打开 (波特率 %d)\n', port, baudrate);

%% 初始化绘图窗口
figure('Position', [100 100 1200 500], 'Name', '串口示波器 FFT (动态)', 'NumberTitle', 'off');

% 时域子图
ax1 = subplot(1,2,1);
h_line = plot(NaN, NaN, 'b-', 'LineWidth', 1);
grid on; xlabel('时间 (ms)'); ylabel('电压 (mV)');
title('时域波形'); hold on;

% 频域子图
ax2 = subplot(1,2,2);
h_fft = plot(NaN, NaN, 'r-', 'LineWidth', 1.5);
grid on; xlabel('频率 (kHz)'); ylabel('幅值 (mV)');
title('FFT 频谱'); hold on;
h_peak = plot(NaN, NaN, 'ro', 'MarkerSize', 10, 'LineWidth', 2);

%% FFT 分析参数
adc_resolution = 4095;   % 12位 ADC
vref = 3.3;              % 参考电压
update_count = 0;

fprintf('开始实时采集...(按 Ctrl+C 停止)\n');
fprintf('帧号\t频率(Hz)\t幅值(mV)\tVpp(mV)\t\t偏置(V)\n');

%% 主循环
while ishandle(h_line)   % 窗口关闭则退出
    %% 找帧头 AA BB
    frame_found = false;
    for attempt = 1:200
        b = read(serialPort, 1, "uint8");
        if b == 0xAA
            b2 = read(serialPort, 1, "uint8");
            if b2 == 0xBB
                frame_found = true;
                break;
            end
        end
    end
    if ~frame_found, continue; end

    %% 解析帧头: 点数 + 采样率
    header = read(serialPort, 4, "uint8");
    num_samples = header(1) + header(2) * 256;
    fs = header(3) + header(4) * 256;
    L = num_samples;
    if L < 2, continue; end

    %% 读取 ADC 数据
    rawData = read(serialPort, L, "uint16");

    %% 换算电压
    voltage = (double(rawData) / adc_resolution) * vref;
    voltage = (double(rawData) / 4095) * 3.3;

    %% 时域参数
    t = (0:L-1) / fs;

    %% FFT
    Y = fft(voltage);
    P2 = abs(Y / L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2 * P1(2:end-1);
    f = fs * (0:(L/2)) / L;

    %% 找主峰
    [peak_val, peak_idx] = max(P1(5:end));
    peak_freq = f(peak_idx + 4);

    %% 时域特征
    vpp = max(voltage) - min(voltage);
    dc_offset = mean(voltage);

    %% 打印结果
    update_count = update_count + 1;
    fprintf('%d\t%.1f\t\t%.1f\t\t%.1f\t\t%.3f\n', ...
            update_count, peak_freq, peak_val*1000, vpp*1000, dc_offset);

    %% 更新绘图
    if ishandle(h_line)
        % 时域
        set(h_line, 'XData', t*1000, 'YData', voltage*1000);
        xlabel(ax1, sprintf('时间 (ms)  [%d点, %0.1f kHz]', L, fs/1000));
        ylabel(ax1, '电压 (mV)');
        if vpp > 0
            ylim(ax1, [min(voltage)*1000 - 50, max(voltage)*1000 + 50]);
        end

        % 频域
        set(h_fft, 'XData', f/1000, 'YData', P1*1000);
        set(h_peak, 'XData', peak_freq/1000, 'YData', peak_val*1000);
        xlim(ax2, [0 fs/2000]);
        title(ax2, sprintf('FFT 频谱 — 主峰: %.2f kHz, %.1f mV', ...
              peak_freq/1000, peak_val*1000));

        drawnow limitrate;
    end
end

%% 清理
clear serialPort;
fprintf('\n采集结束\n');
