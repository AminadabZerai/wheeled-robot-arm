%% IMU Data Analysis Script
% Analyzes CSV data from imu_calibration_and_data.m
% Target: IMU calibration verification and sensor characterization
% This script will grow as the robot development progresses

clear; clc; close all;

%% 1. Load Data
[file, path] = uigetfile('*.csv', 'Select IMU Data CSV');
if isequal(file, 0)
    error('No file selected.');
end

data = readtable(fullfile(path, file));
t = data.Time_s;
accX = data.AccX; accY = data.AccY; accZ = data.AccZ;
gyrX = data.GyrX; gyrY = data.GyrY; gyrZ = data.GyrZ;

N = length(t);
dt_avg = mean(diff(t));
fs = 1 / dt_avg;
duration = t(end) - t(1);

fprintf('=== FILE INFO ===\n');
fprintf('File:          %s\n', file);
fprintf('Samples:       %d\n', N);
fprintf('Duration:      %.2f s\n', duration);
fprintf('Avg Rate:      %.1f Hz\n', fs);
fprintf('Avg dt:        %.4f s\n', dt_avg);
fprintf('\n');

%% 2. Accelerometer Statistics
fprintf('=== ACCELEROMETER STATISTICS (raw LSB) ===\n');
accMeans = [mean(accX), mean(accY), mean(accZ)];
accStds  = [std(accX),  std(accY),  std(accZ)];
accMins  = [min(accX),  min(accY),  min(accZ)];
accMaxs  = [max(accX),  max(accY),  max(accZ)];
accPkPk  = accMaxs - accMins;

fprintf('           X            Y            Z\n');
fprintf('Mean:   %+10.2f   %+10.2f   %+10.2f\n', accMeans);
fprintf('Std:    %10.2f   %10.2f   %10.2f\n', accStds);
fprintf('Min:    %+10.2f   %+10.2f   %+10.2f\n', accMins);
fprintf('Max:    %+10.2f   %+10.2f   %+10.2f\n', accMaxs);
fprintf('Pk-Pk:  %10.2f   %10.2f   %10.2f\n', accPkPk);

% Convert to g for physical interpretation (16384 LSB/g)
accMeans_g = accMeans / 16384;
accStds_g  = accStds / 16384;
accMag_g   = sqrt(accX.^2 + accY.^2 + accZ.^2) / 16384;

fprintf('\n--- In Physical Units (g) ---\n');
fprintf('Mean:   %+8.4f g   %+8.4f g   %+8.4f g\n', accMeans_g);
fprintf('Std:    %8.4f g   %8.4f g   %8.4f g\n', accStds_g);
fprintf('Magnitude Mean: %.4f g (ideal = 1.0000)\n', mean(accMag_g));
fprintf('Magnitude Std:  %.4f g\n', std(accMag_g));
fprintf('\n');

%% 3. Gyroscope Statistics
fprintf('=== GYROSCOPE STATISTICS (raw LSB) ===\n');
gyrMeans = [mean(gyrX), mean(gyrY), mean(gyrZ)];
gyrStds  = [std(gyrX),  std(gyrY),  std(gyrZ)];
gyrMins  = [min(gyrX),  min(gyrY),  min(gyrZ)];
gyrMaxs  = [max(gyrX),  max(gyrY),  max(gyrZ)];
gyrPkPk  = gyrMaxs - gyrMins;

fprintf('           X            Y            Z\n');
fprintf('Mean:   %+10.2f   %+10.2f   %+10.2f\n', gyrMeans);
fprintf('Std:    %10.2f   %10.2f   %10.2f\n', gyrStds);
fprintf('Min:    %+10.2f   %+10.2f   %+10.2f\n', gyrMins);
fprintf('Max:    %+10.2f   %+10.2f   %+10.2f\n', gyrMaxs);
fprintf('Pk-Pk:  %10.2f   %10.2f   %10.2f\n', gyrPkPk);

% Convert to deg/s (131 LSB/(deg/s) for ±250 dps)
gyrMeans_dps = gyrMeans / 131;
gyrStds_dps  = gyrStds / 131;

fprintf('\n--- In Physical Units (deg/s) ---\n');
fprintf('Mean:   %+8.4f     %+8.4f     %+8.4f\n', gyrMeans_dps);
fprintf('Std:    %8.4f     %8.4f     %8.4f\n', gyrStds_dps);
fprintf('\n');

%% 4. Bias Assessment (Calibration Quality)
fprintf('=== CALIBRATION QUALITY ===\n');
fprintf('Accel bias from ideal [0, 0, 1g]:\n');
fprintf('  X offset: %+.4f g  (ideal = 0)\n', accMeans_g(1));
fprintf('  Y offset: %+.4f g  (ideal = 0)\n', accMeans_g(2));
fprintf('  Z offset: %+.4f g  (ideal = 1g, measured = %.4f g)\n', ...
    accMeans_g(3) - 1.0, accMeans_g(3));

fprintf('Gyro bias (should be ~0 when stationary):\n');
fprintf('  X drift:  %+.4f deg/s\n', gyrMeans_dps(1));
fprintf('  Y drift:  %+.4f deg/s\n', gyrMeans_dps(2));
fprintf('  Z drift:  %+.4f deg/s\n', gyrMeans_dps(3));

% Drift accumulation estimate
fprintf('Gyro drift over 60s (if uncorrected):\n');
fprintf('  X: %.2f deg   Y: %.2f deg   Z: %.2f deg\n', ...
    abs(gyrMeans_dps) * 60);
fprintf('\n');

%% 5. Noise Density (useful for filter tuning later)
% Accelerometer noise density in g/√Hz
accND = accStds_g / sqrt(fs);
fprintf('=== NOISE DENSITY ===\n');
fprintf('Accel noise density: X=%.4f  Y=%.4f  Z=%.4f  g/sqrt(Hz)\n', accND);

% Gyro noise density in (deg/s)/√Hz
gyrND = gyrStds_dps / sqrt(fs);
fprintf('Gyro noise density:  X=%.4f  Y=%.4f  Z=%.4f  (deg/s)/sqrt(Hz)\n', gyrND);
fprintf('\n');

%% 6. Sampling Regularity
dt_all = diff(t);
dt_std = std(dt_all);
dt_jitter = max(dt_all) - min(dt_all);

fprintf('=== SAMPLING REGULARITY ===\n');
fprintf('dt mean:    %.4f s\n', dt_avg);
fprintf('dt std:     %.4f s\n', dt_std);
fprintf('dt jitter:  %.4f s (max-min)\n', dt_jitter);
fprintf('Missed samples (dt > 1.5x avg): %d\n', sum(dt_all > 1.5 * dt_avg));
fprintf('\n');

%% 7. Plots

% --- Figure 1: Time Series ---
figure('Name', 'IMU Time Series', 'Color', 'w', 'Position', [50 50 1000 700]);

subplot(3,1,1);
plot(t, accX, 'r', t, accY, 'g', t, accZ, 'b');
ylabel('Accel (LSB)'); title('Accelerometer Time Series');
legend('X', 'Y', 'Z'); grid on;

subplot(3,1,2);
plot(t, gyrX, 'm', t, gyrY, 'c', t, gyrZ, 'y');
ylabel('Gyro (LSB)'); title('Gyroscope Time Series');
legend('X', 'Y', 'Z'); grid on;

subplot(3,1,3);
plot(t(2:end), diff(t) * 1000, 'b');
ylabel('dt (ms)'); xlabel('Time (s)');
title('Sample Interval'); grid on;
yline(dt_avg * 1000, 'r--', sprintf('avg = %.1f ms', dt_avg * 1000));

% --- Figure 2: Histograms (Noise Distribution) ---
figure('Name', 'IMU Noise Distribution', 'Color', 'w', 'Position', [100 100 1000 600]);

subplot(2,3,1);
histogram(accX - mean(accX), 50, 'FaceColor', 'r', 'EdgeColor', 'none');
title('Acc X noise'); xlabel('LSB'); ylabel('Count'); grid on;

subplot(2,3,2);
histogram(accY - mean(accY), 50, 'FaceColor', 'g', 'EdgeColor', 'none');
title('Acc Y noise'); xlabel('LSB'); grid on;

subplot(2,3,3);
histogram(accZ - mean(accZ), 50, 'FaceColor', 'b', 'EdgeColor', 'none');
title('Acc Z noise'); xlabel('LSB'); grid on;

subplot(2,3,4);
histogram(gyrX - mean(gyrX), 50, 'FaceColor', 'm', 'EdgeColor', 'none');
title('Gyr X noise'); xlabel('LSB'); ylabel('Count'); grid on;

subplot(2,3,5);
histogram(gyrY - mean(gyrY), 50, 'FaceColor', 'c', 'EdgeColor', 'none');
title('Gyr Y noise'); xlabel('LSB'); grid on;

subplot(2,3,6);
histogram(gyrZ - mean(gyrZ), 50, 'FaceColor', 'y' , 'EdgeColor', 'none');
title('Gyr Z noise'); xlabel('LSB'); grid on;

% --- Figure 3: Power Spectral Density ---
figure('Name', 'IMU Power Spectral Density', 'Color', 'w', 'Position', [150 150 1000 500]);

nfft = 2^nextpow2(N);

subplot(1,2,1);
[pxx_ax, f_ax] = pwelch(accX - mean(accX), [], [], nfft, fs);
[pxx_ay, ~]    = pwelch(accY - mean(accY), [], [], nfft, fs);
[pxx_az, ~]    = pwelch(accZ - mean(accZ), [], [], nfft, fs);
semilogy(f_ax, pxx_ax, 'r', f_ax, pxx_ay, 'g', f_ax, pxx_az, 'b');
xlabel('Frequency (Hz)'); ylabel('PSD (LSB^2/Hz)');
title('Accelerometer PSD'); legend('X', 'Y', 'Z'); grid on;

subplot(1,2,2);
[pxx_gx, f_gx] = pwelch(gyrX - mean(gyrX), [], [], nfft, fs);
[pxx_gy, ~]    = pwelch(gyrY - mean(gyrY), [], [], nfft, fs);
[pxx_gz, ~]    = pwelch(gyrZ - mean(gyrZ), [], [], nfft, fs);
semilogy(f_gx, pxx_gx, 'm', f_gx, pxx_gy, 'c', f_gx, pxx_gz, 'k');
xlabel('Frequency (Hz)'); ylabel('PSD (LSB^2/Hz)');
title('Gyroscope PSD'); legend('X', 'Y', 'Z'); grid on;

% --- Figure 4: Accel Magnitude Stability ---
figure('Name', 'Accelerometer Magnitude', 'Color', 'w', 'Position', [200 200 800 300]);
plot(t, accMag_g, 'b');
yline(1.0, 'r--', '1g reference');
ylabel('|Accel| (g)'); xlabel('Time (s)');
title(sprintf('Accel Magnitude Stability — Mean: %.4f g, Std: %.4f g', ...
    mean(accMag_g), std(accMag_g)));
grid on;

fprintf('=== ANALYSIS COMPLETE ===\n');