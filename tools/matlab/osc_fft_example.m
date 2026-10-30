%% 简易示波器 FFT 分析示例
% 参数: f=10kHz, Vpp=250mV, 偏置1.25V

clear; clc; close all;

%% 生成模拟信号（模拟示波器 ADC 采样）
fs = 200000;        % 采样率 200kHz
T = 1/fs;           % 采样周期
L = 1024;           % 采样点数（匹配示波器缓存）
t = (0:L-1) * T;    % 时间轴

f_sig = 10000;                  % 信号频率 10kHz
Vpp = 0.25;                     % 峰峰值 250mV
offset = 1.25;                  % 偏置 1.25V（示波器前端中点）
V_amp = Vpp / 2;                % 幅度

% 生成正弦波 + 噪声
noise_amp = 0.005;              % 5mV 噪声
signal = offset + V_amp * sin(2*pi*f_sig*t) + noise_amp * randn(1,L);

%% FFT 计算
Y = fft(signal);
P2 = abs(Y / L);               % 双边频谱
P1 = P2(1:L/2+1);               % 取单边
P1(2:end-1) = 2 * P1(2:end-1);

f = fs * (0:(L/2)) / L;         % 频率轴

%% 找出主峰
[peak_val, peak_idx] = max(P1(5:end));  % 跳过直流
peak_freq = f(peak_idx + 4);
freq_resolution = fs / L;

fprintf('=== FFT 分析结果 ===\n');
fprintf('采样率:      %.0f Hz\n', fs);
fprintf('采样点数:    %d\n', L);
fprintf('频率分辨率:  %.2f Hz\n', freq_resolution);
fprintf('主峰频率:    %.2f Hz\n', peak_freq);
fprintf('幅值:        %.4f V (= %.2f mV)\n', peak_val, peak_val*1000);

%% 计算 Vpp（时域）
measured_vpp = max(signal) - min(signal);
fprintf('时域 Vpp:    %.4f V (= %.2f mV)\n', measured_vpp, measured_vpp*1000);
fprintf('直流偏置:    %.4f V\n', mean(signal));

%% 绘图
figure('Position', [100 100 1000 600]);

% 时域波形
subplot(2,2,1);
plot(t*1000, signal*1000, 'b-', 'LineWidth',1);
xlabel('时间 (ms)'); ylabel('电压 (mV)');
title(sprintf('时域波形 (f=%dHz, Vpp=%.0fmV)', f_sig, Vpp*1000));
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

% 主峰附近细节
subplot(2,2,4);
plot(f/1000, P1*1000, 'r-', 'LineWidth',1.5);
xlabel('频率 (kHz)'); ylabel('幅值 (mV)');
title('主峰细节');
xlim([f_sig/1000 - 2, f_sig/1000 + 2]); grid on;
hold on;
plot(peak_freq/1000, peak_val*1000, 'ro', 'MarkerSize', 10, 'LineWidth',2);
text(peak_freq/1000, peak_val*1000, sprintf('  %.2f kHz\\newline  %.2f mV', peak_freq/1000, peak_val*1000));

%% 对比不同窗函数对频谱的影响
figure('Position', [150 150 800 400]);

windows = {'rectwin', 'hann', 'hamming', 'blackman'};
colors = {'b', 'r', 'g', 'm'};
for i = 1:4
    win = window(windows{i}, L)';
    sig_win = (signal - offset) .* win;
    Yw = fft(sig_win, L);
    Pw = abs(Yw / sum(win)) * 2;
    Pw = Pw(1:L/2+1);
    Pw(2:end-1) = Pw(2:end-1);
    plot(f/1000, Pw*1000, colors{i}, 'LineWidth', 1.2); hold on;
end
xlabel('频率 (kHz)'); ylabel('幅值 (mV)');
title('不同窗函数对比');
xlim([8 12]); grid on;
legend(windows);
