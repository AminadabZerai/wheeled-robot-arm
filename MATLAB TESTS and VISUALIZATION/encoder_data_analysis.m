%% encoder_analysis.m
% Offline analysis of encoder CSV data
% Covers: kinematics, velocity noise, PSD, rollover check

clear; clc; close all;

%% 1. Load
[file, path] = uigetfile('encoder_data_*.csv', 'Select Encoder CSV');
if isequal(file, 0); error('No file selected.'); end

data   = readtable(fullfile(path, file));
t      = data.Time_s;
encRad = data.EncRad;
encVel = data.EncVel;

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

%% 2. Angle Statistics
encRad_deg = rad2deg(encRad);
encRad_rev = encRad / (2 * pi);

fprintf('=== ANGLE STATISTICS ===\n');
fprintf('Mean:    %+8.4f rad  (%+8.2f deg)\n', mean(encRad),     mean(encRad_deg));
fprintf('Std:     %8.4f rad  (%8.2f deg)\n',   std(encRad),      std(encRad_deg));
fprintf('Min:     %+8.4f rad  (%+8.2f deg)\n', min(encRad),      min(encRad_deg));
fprintf('Max:     %+8.4f rad  (%+8.2f deg)\n', max(encRad),      max(encRad_deg));
fprintf('Range:   %8.4f rad  (%8.2f deg)\n',   range(encRad),    range(encRad_deg));
fprintf('Total rotations covered: %.3f rev\n\n', range(encRad) / (2*pi));

%% 3. Velocity Statistics
encVel_rpm  = encVel * (60 / (2 * pi));
encVel_dps  = rad2deg(encVel);

fprintf('=== ANGULAR VELOCITY STATISTICS ===\n');
fprintf('Mean:    %+8.4f rad/s  (%+8.2f RPM)\n', mean(encVel), mean(encVel_rpm));
fprintf('Std:     %8.4f rad/s  (%8.2f RPM)\n',   std(encVel),  std(encVel_rpm));
fprintf('Min:     %+8.4f rad/s  (%+8.2f RPM)\n', min(encVel),  min(encVel_rpm));
fprintf('Max:     %+8.4f rad/s  (%+8.2f RPM)\n', max(encVel),  max(encVel_rpm));
fprintf('Peak-to-peak: %.4f rad/s\n\n',           range(encVel));

%% 4. Rollover Check
% Sudden jumps > pi in encRad (after differencing) suggest missed rollover handling
d_enc    = diff(encRad);
n_jumps  = sum(abs(d_enc) > pi);

fprintf('=== ROLLOVER HEALTH CHECK ===\n');
fprintf('Large jumps (|delta| > pi rad) detected: %d\n', n_jumps);
if n_jumps == 0
    fprintf('  No rollover artifacts found. Rollover handling looks healthy.\n\n');
else
    fprintf('  WARNING: Possible rollover artifacts. Review as5600_update() logic.\n\n');
end

%% 5. Velocity Noise (useful if wheel stationary during test)
fprintf('=== VELOCITY NOISE ===\n');
fprintf('Std:             %.5f rad/s\n',           std(encVel));
fprintf('Noise density:   %.5f (rad/s)/sqrt(Hz)\n', std(encVel) / sqrt(fs));
fprintf('In RPM:          %.5f RPM\n\n',            std(encVel_rpm));

%% 6. Sampling Regularity
dt_all = diff(t);
fprintf('=== SAMPLING REGULARITY ===\n');
fprintf('dt mean:    %.4f s\n', dt_avg);
fprintf('dt std:     %.4f s\n', std(dt_all));
fprintf('dt jitter:  %.4f s\n', max(dt_all) - min(dt_all));
fprintf('Missed samples (dt > 1.5x avg): %d\n\n', sum(dt_all > 1.5 * dt_avg));

%% 7. Plots

% Fig 1: Time Series
figure('Name','Encoder Time Series','Color','w','Position',[50 50 1000 600]);

subplot(3,1,1);
plot(t, encRad, 'b');
ylabel('rad'); title('Encoder Angle (rad)'); grid on;

subplot(3,1,2);
yyaxis left;  plot(t, encVel, 'r');     ylabel('rad/s');
yyaxis right; plot(t, encVel_rpm, 'b'); ylabel('RPM');
title('Angular Velocity'); grid on;

subplot(3,1,3);
plot(t(2:end), dt_all * 1000, 'b');
yline(dt_avg * 1000, 'r--', sprintf('avg = %.1f ms', dt_avg * 1000));
ylabel('dt (ms)'); xlabel('Time (s)'); title('Sample Interval'); grid on;

% Fig 2: Histograms
figure('Name','Encoder Noise Distribution','Color','w','Position',[100 100 800 380]);

subplot(1,2,1);
histogram(encRad - mean(encRad), 50, 'FaceColor','b','EdgeColor','none');
title('Angle noise'); xlabel('rad'); ylabel('Count'); grid on;

subplot(1,2,2);
histogram(encVel - mean(encVel), 50, 'FaceColor','r','EdgeColor','none');
title('Velocity noise'); xlabel('rad/s'); grid on;

% Fig 3: PSD
figure('Name','Encoder PSD','Color','w','Position',[150 150 900 400]);
nfft = 2^nextpow2(N);

subplot(1,2,1);
[per, fer] = pwelch(encRad - mean(encRad), [], [], nfft, fs);
semilogy(fer, per, 'b');
xlabel('Hz'); ylabel('rad^2/Hz'); title('Angle PSD'); grid on;

subplot(1,2,2);
[pev, fev] = pwelch(encVel - mean(encVel), [], [], nfft, fs);
semilogy(fev, pev, 'r');
xlabel('Hz'); ylabel('(rad/s)^2/Hz'); title('Velocity PSD'); grid on;

% Fig 4: Rollover jump plot (diagnostic)
figure('Name','Encoder Rollover Diagnostic','Color','w','Position',[200 200 900 300]);
plot(t(2:end), d_enc, 'k');
yline(pi,  'r--', '+\pi'); yline(-pi, 'r--', '-\pi');
ylabel('\Delta rad'); xlabel('Time (s)');
title(sprintf('Step-to-step angle change — %d suspicious jumps detected', n_jumps));
grid on;

fprintf('=== ENCODER ANALYSIS COMPLETE ===\n');