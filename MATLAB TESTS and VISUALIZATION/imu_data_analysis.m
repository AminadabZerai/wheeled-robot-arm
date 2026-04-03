%% imu_analysis.m
% Offline analysis of IMU CSV data
% Covers: statistics, calibration quality, noise density, PSD, histograms

clear; clc; close all;

%% 1. Load
[file, path] = uigetfile('imu_data_*.csv', 'Select IMU CSV');
if isequal(file, 0); error('No file selected.'); end

data = readtable(fullfile(path, file));
t    = data.Time_s;
accX = data.AccX; accY = data.AccY; accZ = data.AccZ;
gyrX = data.GyrX; gyrY = data.GyrY; gyrZ = data.GyrZ;

N        = length(t);
dt_avg   = mean(diff(t));
fs       = 1 / dt_avg;
duration = t(end) - t(1);

fprintf('=== FILE INFO ===\n');
fprintf('File:      %s\n', file);
fprintf('Samples:   %d\n', N);
fprintf('Duration:  %.2f s\n', duration);
fprintf('Avg Rate:  %.1f Hz\n', fs);
fprintf('Avg dt:    %.4f s\n\n', dt_avg);

%% 2. Accelerometer Statistics
fprintf('=== ACCELEROMETER (LSB) ===\n');
accMeans = [mean(accX), mean(accY), mean(accZ)];
accStds  = [std(accX),  std(accY),  std(accZ)];
accMins  = [min(accX),  min(accY),  min(accZ)];
accMaxs  = [max(accX),  max(accY),  max(accZ)];

fprintf('           X            Y            Z\n');
fprintf('Mean:   %+10.2f   %+10.2f   %+10.2f\n', accMeans);
fprintf('Std:    %10.2f   %10.2f   %10.2f\n',     accStds);
fprintf('Min:    %+10.2f   %+10.2f   %+10.2f\n',  accMins);
fprintf('Max:    %+10.2f   %+10.2f   %+10.2f\n',  accMaxs);
fprintf('Pk-Pk:  %10.2f   %10.2f   %10.2f\n',     accMaxs - accMins);

% Physical units (16384 LSB/g for ±2g)
accMeans_g = accMeans / 16384;
accStds_g  = accStds  / 16384;
accMag_g   = sqrt(accX.^2 + accY.^2 + accZ.^2) / 16384;

fprintf('\n--- Physical Units (g) ---\n');
fprintf('Mean:   %+8.4f g   %+8.4f g   %+8.4f g\n', accMeans_g);
fprintf('Std:    %8.4f g   %8.4f g   %8.4f g\n',     accStds_g);
fprintf('|Acc| mean: %.4f g  (ideal = 1.0000)\n',     mean(accMag_g));
fprintf('|Acc| std:  %.4f g\n\n',                     std(accMag_g));

%% 3. Gyroscope Statistics
fprintf('=== GYROSCOPE (LSB) ===\n');
gyrMeans = [mean(gyrX), mean(gyrY), mean(gyrZ)];
gyrStds  = [std(gyrX),  std(gyrY),  std(gyrZ)];
gyrMins  = [min(gyrX),  min(gyrY),  min(gyrZ)];
gyrMaxs  = [max(gyrX),  max(gyrY),  max(gyrZ)];

fprintf('           X            Y            Z\n');
fprintf('Mean:   %+10.2f   %+10.2f   %+10.2f\n', gyrMeans);
fprintf('Std:    %10.2f   %10.2f   %10.2f\n',     gyrStds);
fprintf('Min:    %+10.2f   %+10.2f   %+10.2f\n',  gyrMins);
fprintf('Max:    %+10.2f   %+10.2f   %+10.2f\n',  gyrMaxs);
fprintf('Pk-Pk:  %10.2f   %10.2f   %10.2f\n',     gyrMaxs - gyrMins);

% Physical units (131 LSB/deg/s for ±250 dps)
gyrMeans_dps = gyrMeans / 131;
gyrStds_dps  = gyrStds  / 131;

fprintf('\n--- Physical Units (deg/s) ---\n');
fprintf('Mean:   %+8.4f     %+8.4f     %+8.4f\n', gyrMeans_dps);
fprintf('Std:    %8.4f     %8.4f     %8.4f\n\n',   gyrStds_dps);

%% 4. Calibration Quality
fprintf('=== CALIBRATION QUALITY ===\n');
fprintf('Accel bias from ideal [0, 0, 1g]:\n');
fprintf('  X: %+.4f g\n  Y: %+.4f g\n  Z measured: %.4f g (ideal 1.0)\n', ...
    accMeans_g(1), accMeans_g(2), accMeans_g(3));

fprintf('Gyro bias (ideal = 0 when stationary):\n');
fprintf('  X: %+.4f deg/s\n  Y: %+.4f deg/s\n  Z: %+.4f deg/s\n', gyrMeans_dps);
fprintf('Gyro drift over 60s:\n');
fprintf('  X: %.2f deg   Y: %.2f deg   Z: %.2f deg\n\n', abs(gyrMeans_dps) * 60);

%% 5. Noise Density
accND = accStds_g   / sqrt(fs);
gyrND = gyrStds_dps / sqrt(fs);

fprintf('=== NOISE DENSITY ===\n');
fprintf('Accel: X=%.5f  Y=%.5f  Z=%.5f  g/sqrt(Hz)\n',       accND);
fprintf('Gyro:  X=%.5f  Y=%.5f  Z=%.5f  (deg/s)/sqrt(Hz)\n\n', gyrND);

%% 6. Sampling Regularity
dt_all = diff(t);
fprintf('=== SAMPLING REGULARITY ===\n');
fprintf('dt mean:    %.4f s\n', dt_avg);
fprintf('dt std:     %.4f s\n', std(dt_all));
fprintf('dt jitter:  %.4f s\n', max(dt_all) - min(dt_all));
fprintf('Missed samples (dt > 1.5x avg): %d\n\n', sum(dt_all > 1.5 * dt_avg));

%% 7. Plots

% Fig 1: Time Series
figure('Name','IMU Time Series','Color','w','Position',[50 50 1000 600]);

subplot(3,1,1);
plot(t, accX,'r', t, accY,'g', t, accZ,'b');
ylabel('LSB'); title('Accelerometer Raw');
legend('X','Y','Z'); grid on;

subplot(3,1,2);
plot(t, gyrX,'m', t, gyrY,'c', t, gyrZ,[1 0.5 0]);
ylabel('LSB'); title('Gyroscope Raw');
legend('X','Y','Z'); grid on;

subplot(3,1,3);
plot(t(2:end), dt_all * 1000, 'b');
yline(dt_avg * 1000, 'r--', sprintf('avg = %.1f ms', dt_avg * 1000));
ylabel('dt (ms)'); xlabel('Time (s)'); title('Sample Interval'); grid on;

% Fig 2: Histograms
figure('Name','IMU Noise Distribution','Color','w','Position',[100 100 1000 550]);
axes_colors = {'r','g','b','m','c',[1 0.5 0]};
labels = {'Acc X','Acc Y','Acc Z','Gyr X','Gyr Y','Gyr Z'};
signals = {accX, accY, accZ, gyrX, gyrY, gyrZ};

for k = 1:6
    subplot(2,3,k);
    histogram(signals{k} - mean(signals{k}), 50, ...
        'FaceColor', axes_colors{k}, 'EdgeColor', 'none');
    title([labels{k} ' noise']); xlabel('LSB');
    if k == 1 || k == 4; ylabel('Count'); end
    grid on;
end

% Fig 3: PSD
figure('Name','IMU PSD','Color','w','Position',[150 150 1000 450]);
nfft = 2^nextpow2(N);

subplot(1,2,1);
[pax,f] = pwelch(accX-mean(accX),[],[],nfft,fs);
[pay,~] = pwelch(accY-mean(accY),[],[],nfft,fs);
[paz,~] = pwelch(accZ-mean(accZ),[],[],nfft,fs);
semilogy(f,pax,'r', f,pay,'g', f,paz,'b');
xlabel('Hz'); ylabel('LSB^2/Hz'); title('Accel PSD'); legend('X','Y','Z'); grid on;

subplot(1,2,2);
[pgx,f] = pwelch(gyrX-mean(gyrX),[],[],nfft,fs);
[pgy,~] = pwelch(gyrY-mean(gyrY),[],[],nfft,fs);
[pgz,~] = pwelch(gyrZ-mean(gyrZ),[],[],nfft,fs);
semilogy(f,pgx,'m', f,pgy,'c', f,pgz,[1 0.5 0]);
xlabel('Hz'); ylabel('LSB^2/Hz'); title('Gyro PSD'); legend('X','Y','Z'); grid on;

% Fig 4: Accel Magnitude
figure('Name','Accel Magnitude','Color','w','Position',[200 200 900 300]);
plot(t, accMag_g, 'b');
yline(1.0, 'r--', '1g reference');
ylabel('|Acc| (g)'); xlabel('Time (s)');
title(sprintf('Magnitude — Mean: %.4f g, Std: %.4f g', mean(accMag_g), std(accMag_g)));
grid on;

fprintf('=== IMU ANALYSIS COMPLETE ===\n');