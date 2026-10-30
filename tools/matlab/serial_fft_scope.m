%% 串口示波器 FFT 分析
% 从串口读取 ADC 原始数据 → 绘制波形 → FFT 分析
% 数据帧格式: AA BB + 点数(2B) + 采样率(2B) + N个uint16(小端)

clear; clc; close all;

%% 串口配置
port = 'COM16';          % 根据实际串口号修改
baudrate = 115200;      % 波特率

% 打开串口
serialPort = serialport(port, baudrate, "Timeout", 10);
flush(serialPort);      % 清空缓冲区
fprintf('串口 %s 已打开 (波特率 %d)\n', port, baudrate);

%% 寻找帧头并读取一帧数据
frame_found = false;
max_attempts = 100;

for attempt = 1:max_attempts
    % 读 1 字节，搜索帧头 AA
    b = read(serialPort, 1, "uint8");
    if b == 0xAA
        b2 = read(serialPort, 1, "uint8");
        if b2 == 0xBB
            frame_found = true;
            break;
        end
    end
end

if ~frame_found
    error('未找到帧头 AA BB，请检查串口连接');
end

%% 解析帧头: 采样点数 + 采样率
header = read(serialPort, 4, "uint8");
num_samples = header(1) + header(2) * 256;   % uint16 小端
fs = header(3) + header(4) * 256;            % 采样率 (Hz)
L = num_samples;

fprintf('帧同步成功: %d 点, 采样率 %d Hz\n', L, fs);

%% 读取 ADC 原始数据 (uint16 小端)
rawData = read(serialPort, L, "uint16");

clear serialPort;       % 关闭串口
fprintf('数据接收完成\n');

%% 将 ADC 原始值转换为电压
adc_resolution = 4095;   % 12 位 ADC (0-4095)
vref = 3.3;              % 参考电压 3.3V
voltage = (double(rawData) / adc_resolution) * vref;

% 注意: 该示波器前端有信号调理电路，实际电压 = 5 - 2*V_ADC
% 如果测量的是调理后的电压，使用下式; 否则直接使用 voltage
voltage = 5 - 2 * voltage;

%% 波形参数
T = 1/fs;
t = (0:L-1) * T;

%% FFT 分析
Y = fft(voltage);
P2 = abs(Y / L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2 * P1(2:end-1);

f = fs * (0:(L/2)) / L;
freq_resolution = fs / L;

%% 找主峰
[peak_val, peak_idx] = max(P1(5:end));
peak_freq = f(peak_idx + 4);

%% 打印结果
fprintf('\n=== FFT 分析结果 ===\n');
fprintf('采样率:        %.0f Hz\n', fs);
fprintf('采样点数:      %d\n', L);
fprintf('频率分辨率:    %.2f Hz\n', freq_resolution);
fprintf('主峰频率:      %.2f Hz\n', peak_freq);
fprintf('幅值:          %.4f V (= %.2f mV)\n', peak_val, peak_val*1000);
fprintf('直流偏置:      %.4f V\n', mean(voltage));
fprintf('时域 Vpp:      %.4f V (= %.2f mV)\n', max(voltage)-min(voltage), ...
        (max(voltage)-min(voltage))*1000);

%% 绘图
figure('Position', [100 100 1000 600]);

% 时域波形
subplot(2,2,1);
plot(t*1000, voltage*1000, 'b-', 'LineWidth',1);
xlabel('时间 (ms)'); ylabel('电压 (mV)');
title(sprintf('串口接收波形 (%d 点, %0.1f kHz)', L, fs/1000));
grid on; axis tight;

% 频谱
subplot(2,2,2);
plot(f/1000, P1*1000, 'r-', 'LineWidth',1.5);
xlabel('频率 (kHz)'); ylabel('幅值 (mV)');
title('FFT 频谱');
xlim([0 fs/2000]); grid on;
hold on;
plot(peak_freq/1000, peak_val*1000, 'ro', 'MarkerSize', 10, 'LineWidth',2);
legend('频谱', sprintf('主峰 %.2fkHz', peak_freq/1000));

% 频谱（半对数）
subplot(2,2,3);
semilogy(f/1000, P1*1000, 'r-', 'LineWidth',1);
xlabel('频率 (kHz)'); ylabel('幅值 (mV, log)');
title('FFT 频谱 (对数Y轴)');
xlim([0 fs/2000]); grid on;

% 主峰细节
subplot(2,2,4);
plot(f/1000, P1*1000, 'r-', 'LineWidth',1.5);
xlabel('频率 (kHz)'); ylabel('幅值 (mV)');
title('主峰细节');
xlim([peak_freq/1000 - 2, peak_freq/1000 + 2]); grid on;
hold on;
plot(peak_freq/1000, peak_val*1000, 'ro', 'MarkerSize', 10, 'LineWidth',2);
text(peak_freq/1000, peak_val*1000, ...
     sprintf('  %.2f kHz\\newline  %.2f mV', peak_freq/1000, peak_val*1000));

%% 窗函数对比（去掉直流后）
figure('Position', [150 150 800 400]);
windows = {'rectwin', 'hann', 'hamming', 'blackman'};
colors = {'b', 'r', 'g', 'm'};
dc_offset = mean(voltage);
for i = 1:4
    win = window(windows{i}, L)';
    sig_win = (voltage - dc_offset) .* win;
    Yw = fft(sig_win, L);
    Pw = abs(Yw / sum(win)) * 2;
    Pw = Pw(1:L/2+1);
    Pw(2:end-1) = Pw(2:end-1);
    plot(f/1000, Pw*1000, colors{i}, 'LineWidth', 1.2); hold on;
end
xlabel('频率 (kHz)'); ylabel('幅值 (mV)');
title('不同窗函数对比');
xlim([max(0,peak_freq/1000-2) peak_freq/1000+2]); grid on;
legend(windows);
