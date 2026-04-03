%% IMU + Encoder Data Analysis Script
% Analyzes CSV data logged from the live stream script
% Covers: IMU calibration verification, noise, PSD, encoder kinematics

clear; clc; close all;

%% 1. Load Data
[file, path] = uigetfile('*.csv', 'Select IMU Data CSV');
if isequal(file, 0)
    error('No file selected.');
end

data = readtable(fullfile(path, file));
t      = data.Time_s;
accX   = data.AccX;   accY = data.AccY;   accZ = data.AccZ;
gyrX   = data.GyrX;   gyrY = data.GyrY;   gyrZ = data.GyrZ;
encRad = data.EncRad;
encVel = data.EncVel;

N        = length(t);
dt_avg   = mean(diff(t));
fs       = 1 / dt_avg;
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
fprintf('Std:    %10.2f   %10.2f   %10.2f\n',     accStds);
fprintf('Min:    %+10.2f   %+10.2f   %+10.2f\n',  accMins);
fprintf('Max:    %+10.2f   %+10.2f   %+10.2f\n',  accMaxs);
fprintf('Pk-Pk:  %10.2f   %10.2f   %10.2f\n',     accPkPk);

% Convert to g (16384 LSB/g for ±2g range)
accMeans_g = accMeans / 16384;
accStds_g  = accStds  / 16384;
accMag_g   = sqrt(accX.^2 + accY.^2 + accZ.^2) / 16384;

fprintf('\n--- In Physical Units (g) ---\n');
fprintf('Mean:   %+8.4f g   %+8.4f g   %+8.4f g\n', accMeans_g);
fprintf('Std:    %8.4f g   %8.4f g   %8.4f g\n',     accStds_g);
fprintf('Magnitude Mean: %.4f g (ideal = 1.0000)\n',  mean(accMag_g));
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
fprintf('Std:    %10.2f   %10.2f   %10.2f\n',     gyrStds);
fprintf('Min:    %+10.2f   %+10.2f   %+10.2f\n',  gyrMins);
fprintf('Max:    %+10.2f   %+10.2f   %+10.2f\n',  gyrMaxs);
fprintf('Pk-Pk:  %10.2f   %10.2f   %10.2f\n',     gyrPkPk);

% Convert to deg/s (131 LSB/deg/s for ±250 dps)
gyrMeans_dps = gyrMeans / 131;
gyrStds_dps  = gyrStds  / 131;

fprintf('\n--- In Physical Units (deg/s) ---\n');
fprintf('Mean:   %+8.4f     %+8.4f     %+8.4f\n', gyrMeans_dps);
fprintf('Std:    %8.4f     %8.4f     %8.4f\n',     gyrStds_dps);
fprintf('\n');

%% 4. Encoder Statistics
fprintf('=== ENCODER LF STATISTICS ===\n');

encRad_mean = mean(encRad);
encRad_std  = std(encRad);
encRad_min  = min(encRad);
encRad_max  = max(encRad);
encRad_range = encRad_max - encRad_min;

encVel_mean = mean(encVel);
encVel_std  = std(encVel);
encVel_min  = min(encVel);
encVel_max  = max(encVel);

% Convert radians to degrees and RPM for readability
encRad_deg  = rad2deg(encRad);
encVel_rpm  = encVel * (60 / (2 * pi));

fprintf('--- Angle ---\n');
fprintf('Mean:      %+8.4f rad  (%+8.2f deg)\n', encRad_mean, rad2deg(encRad_mean));
fprintf('Std:       %8.4f rad  (%8.2f deg)\n',   encRad_std,  rad2deg(encRad_std));
fprintf('Min:       %+8.4f rad  (%+8.2f deg)\n', encRad_min,  rad2deg(encRad_min));
fprintf('Max:       %+8.4f rad  (%+8.2f deg)\n', encRad_max,  rad2deg(encRad_max));
fprintf('Range:     %8.4f rad  (%8.2f deg)\n',   encRad_range, rad2deg(encRad_range));
fprintf('Total rot: %.3f rev\n', encRad_range / (2*pi));

fprintf('\n--- Angular Velocity ---\n');
fprintf('Mean:      %+8.4f rad/s  (%+8.2f RPM)\n', encVel_mean, mean(encVel_rpm));
fprintf('Std:       %8.4f rad/s  (%8.2f RPM)\n',   encVel_std,  std(encVel_rpm));
fprintf('Min:       %+8.4f rad/s  (%+8.2f RPM)\n', encVel_min,  min(encVel_rpm));
fprintf('Max:       %+8.4f rad/s  (%+8.2f RPM)\n', encVel_max,  max(encVel_rpm));
fprintf('\n');

%% 5. Bias & Calibration Quality
fprintf('=== CALIBRATION QUALITY ===\n');
fprintf('Accel bias from ideal [0, 0, 1g]:\n');
fprintf('  X offset: %+.4f g  (ideal = 0)\n', accMeans_g(1));
fprintf('  Y offset: %+.4f g  (ideal = 0)\n', accMeans_g(2));
fprintf('  Z offset: %+.4f g  (ideal = 1g, measured = %.4f g)\n', ...
    accMeans_g(3) - 1.0, accMeans_g(3));

fprintf('Gyro bias (should be ~0 when stationary):\n');
fprintf('  X drift: %+.4f deg/s\n', gyrMeans_dps(1));
fprintf('  Y drift: %+.4f deg/s\n', gyrMeans_dps(2));
fprintf('  Z drift: %+.4f deg/s\n', gyrMeans_dps(3));

fprintf('Gyro drift over 60s (if uncorrected):\n');
fprintf('  X: %.2f deg   Y: %.2f deg   Z: %.2f deg\n', abs(gyrMeans_dps) * 60);

% Encoder velocity noise: if wheel is stationary, std should be near 0
fprintf('Encoder velocity noise (std): %.4f rad/s (%.4f RPM)\n', ...
    encVel_std, encVel_std * 60 / (2*pi));
fprintf('\n');

%% 6. Noise Density
accND = accStds_g    / sqrt(fs);
gyrND = gyrStds_dps  / sqrt(fs);
encVelND = encVel_std / sqrt(fs);

fprintf('=== NOISE DENSITY ===\n');
fprintf('Accel noise density: X=%.5f  Y=%.5f  Z=%.5f  g/sqrt(Hz)\n', accND);
fprintf('Gyro  noise density: X=%.5f  Y=%.5f  Z=%.5f  (deg/s)/sqrt(Hz)\n', gyrND);
fprintf('Enc velocity noise density: %.5f (rad/s)/sqrt(Hz)\n', encVelND);
fprintf('\n');

%% 7. Sampling Regularity
dt_all    = diff(t);
dt_std    = std(dt_all);
dt_jitter = max(dt_all) - min(dt_all);

fprintf('=== SAMPLING REGULARITY ===\n');
fprintf('dt mean:    %.4f s\n', dt_avg);
fprintf('dt std:     %.4f s\n', dt_std);
fprintf('dt jitter:  %.4f s (max-min)\n', dt_jitter);
fprintf('Missed samples (dt > 1.5x avg): %d\n', sum(dt_all > 1.5 * dt_avg));
fprintf('\n');

%% 8. Plots

% --- Figure 1: IMU + Encoder Time Series ---
figure('Name', 'Full Sensor Time Series', 'Color', 'w', 'Position', [50 50 1100 800]);

subplot(4,1,1);
plot(t, accX, 'r', t, accY, 'g', t, accZ, 'b');
ylabel('Accel (LSB)'); title('Accelerometer Time Series');
legend('X','Y','Z'); grid on;

subplot(4,1,2);
plot(t, gyrX, 'm', t, gyrY, 'c', t, gyrZ, 'y');
ylabel('Gyro (LSB)'); title('Gyroscope Time Series');
legend('X','Y','Z'); grid on;

subplot(4,1,3);
plot(t, encRad, 'b', t, encRad_deg/100, 'r--'); % deg/100 for visual scale
ylabel('rad'); title('Encoder LF — Angle');
legend('rad', 'deg/100 (scaled)'); grid on;

subplot(4,1,4);
plot(t, encVel, 'k', t, encVel_rpm/10, 'b--'); % RPM/10 for visual scale
ylabel('rad/s'); xlabel('Time (s)');
title('Encoder LF — Angular Velocity');
legend('rad/s', 'RPM/10 (scaled)'); grid on;

% --- Figure 2: Noise Histograms (all 6 IMU axes) ---
figure('Name', 'IMU Noise Distribution', 'Color', 'w', 'Position', [100 100 1100 600]);

subplot(2,3,1); histogram(accX - mean(accX), 50, 'FaceColor','r','EdgeColor','none');
title('Acc X noise'); xlabel('LSB'); ylabel('Count'); grid on;

subplot(2,3,2); histogram(accY - mean(accY), 50, 'FaceColor','g','EdgeColor','none');
title('Acc Y noise'); xlabel('LSB'); grid on;

subplot(2,3,3); histogram(accZ - mean(accZ), 50, 'FaceColor','b','EdgeColor','none');
title('Acc Z noise'); xlabel('LSB'); grid on;

subplot(2,3,4); histogram(gyrX - mean(gyrX), 50, 'FaceColor','m','EdgeColor','none');
title('Gyr X noise'); xlabel('LSB'); ylabel('Count'); grid on;

subplot(2,3,5); histogram(gyrY - mean(gyrY), 50, 'FaceColor','c','EdgeColor','none');
title('Gyr Y noise'); xlabel('LSB'); grid on;

subplot(2,3,6); histogram(gyrZ - mean(gyrZ), 50, 'FaceColor',[1 0.5 0],'EdgeColor','none');
title('Gyr Z noise'); xlabel('LSB'); grid on;

% --- Figure 3: Encoder Histograms ---
figure('Name', 'Encoder Noise Distribution', 'Color', 'w', 'Position', [150 150 800 350]);

subplot(1,2,1);
histogram(encRad - mean(encRad), 50, 'FaceColor','b','EdgeColor','none');
title('Encoder Angle noise'); xlabel('rad'); ylabel('Count'); grid on;

subplot(1,2,2);
histogram(encVel - mean(encVel), 50, 'FaceColor','k','EdgeColor','none');
title('Encoder Velocity noise'); xlabel('rad/s'); grid on;

% --- Figure 4: Power Spectral Density (IMU) ---
figure('Name', 'IMU Power Spectral Density', 'Color', 'w', 'Position', [200 200 1000 500]);
nfft = 2^nextpow2(N);

subplot(1,2,1);
[pxx_ax,f_ax] = pwelch(accX-mean(accX),[],[],nfft,fs);
[pxx_ay,~]    = pwelch(accY-mean(accY),[],[],nfft,fs);
[pxx_az,~]    = pwelch(accZ-mean(accZ),[],[],nfft,fs);
semilogy(f_ax, pxx_ax,'r', f_ax, pxx_ay,'g', f_ax, pxx_az,'b');
xlabel('Frequency (Hz)'); ylabel('PSD (LSB^2/Hz)');
title('Accelerometer PSD'); legend('X','Y','Z'); grid on;

subplot(1,2,2);
[pxx_gx,f_gx] = pwelch(gyrX-mean(gyrX),[],[],nfft,fs);
[pxx_gy,~]    = pwelch(gyrY-mean(gyrY),[],[],nfft,fs);
[pxx_gz,~]    = pwelch(gyrZ-mean(gyrZ),[],[],nfft,fs);
semilogy(f_gx, pxx_gx,'m', f_gx, pxx_gy,'c', f_gx, pxx_gz,'k');
xlabel('Frequency (Hz)'); ylabel('PSD (LSB^2/Hz)');
title('Gyroscope PSD'); legend('X','Y','Z'); grid on;

% --- Figure 5: Encoder PSD ---
figure('Name', 'Encoder Power Spectral Density', 'Color', 'w', 'Position', [250 250 900 400]);

subplot(1,2,1);
[pxx_er, f_er] = pwelch(encRad - mean(encRad), [], [], nfft, fs);
semilogy(f_er, pxx_er, 'b');
xlabel('Frequency (Hz)'); ylabel('PSD (rad^2/Hz)');
title('Encoder Angle PSD'); grid on;

subplot(1,2,2);
[pxx_ev, f_ev] = pwelch(encVel - mean(encVel), [], [], nfft, fs);
semilogy(f_ev, pxx_ev, 'k');
xlabel('Frequency (Hz)'); ylabel('PSD ((rad/s)^2/Hz)');
title('Encoder Velocity PSD'); grid on;

% --- Figure 6: Accel Magnitude Stability ---
figure('Name', 'Accelerometer Magnitude', 'Color', 'w', 'Position', [300 300 900 300]);
plot(t, accMag_g, 'b');
yline(1.0, 'r--', '1g reference');
ylabel('|Accel| (g)'); xlabel('Time (s)');
title(sprintf('Accel Magnitude — Mean: %.4f g, Std: %.4f g', ...
    mean(accMag_g), std(accMag_g)));
grid on;

% --- Figure 7: Sampling Interval ---
figure('Name', 'Sampling Regularity', 'Color', 'w', 'Position', [350 350 900 300]);
plot(t(2:end), dt_all * 1000, 'b');
yline(dt_avg * 1000, 'r--', sprintf('avg = %.1f ms', dt_avg * 1000));
ylabel('dt (ms)'); xlabel('Time (s)');
title('Sample Interval'); grid on;

fprintf('=== ANALYSIS COMPLETE ===\n');