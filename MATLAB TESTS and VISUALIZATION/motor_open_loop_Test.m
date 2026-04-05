%% motor_openloop_test.m
% Synced with Arduino 4-column format: Time_ms, Power_pct, Angle_rad, Vel_rads
% Tests: Deadband, PWM sweep, Step response

clear; clc; close all;

%% Connection
port     = "COM3";
baudrate = 115200;
device   = serialport(port, baudrate);
configureTerminator(device, "LF");
device.Timeout = 10;

pause(2); flush(device);

% Wait for Arduino READY message
fprintf('Waiting for Arduino calibration...\n');
while true
    try
        line = readline(device);
        if contains(line, "READY")
            fprintf('Arduino ready. Starting tests.\n\n');
            break;
        end
    catch
        continue;
    end
end

%% Save directory
save_dir = 'C:/Users/Amine/Desktop/Project_Make_it_Or_Break_It/wheeled-robot-arm/MATLAB TESTS and VISUALIZATION/CSVs/';
if ~exist(save_dir, 'dir'); mkdir(save_dir); end

%% Col indices — single source of truth, matches Arduino format
COL_TIME  = 1;
COL_POWER = 2;
COL_ANGLE = 3;
COL_VEL   = 4;
N_COLS    = 4;

%% ================================================================
%% Helper: run a fixed-power segment, return buffered data
%% ================================================================
function data = run_segment(device, power_pct, duration_s, label, N_COLS)
    fprintf('  >> %s: %.0f%% power for %.1f s\n', label, power_pct, duration_s);

    % Stop motor, flush stale buffer, then apply command
    writeline(device, '0.0');
    pause(1.5);
    flush(device);

    writeline(device, sprintf('%.2f', power_pct));
    pause(0.5); % let motor spin up

    device.Timeout = 2;
    warning('off', 'serialport:readline:timeout');

    data  = [];
    t_end = tic;
    while toc(t_end) < duration_s
        try
            line = readline(device);
        catch
            continue;
        end

        if ~ischar(line) && ~isstring(line); continue; end
        line = strtrim(string(line));
        if strlength(line) == 0; continue; end

        v = str2double(strsplit(line, ','));

        % Validate column count and no NaNs
        if length(v) == N_COLS && ~any(isnan(v))
            data = [data; v];
        end
    end

    warning('on', 'serialport:readline:timeout');
    device.Timeout = 10;

    fprintf('     Captured %d samples.\n', size(data,1));
end

%% ================================================================
%% TEST 1: Deadband Identification
%% ================================================================
fprintf('=== TEST 1: Deadband Identification ===\n');

deadband_powers  = 5:5:60;
n_db             = length(deadband_powers);
deadband_results = zeros(n_db, 3); % [power, mean_vel, std_vel]

for k = 1:n_db
    pwr = deadband_powers(k);
    seg = run_segment(device, pwr, 2.5, 'Deadband probe', N_COLS);

    if ~isempty(seg)
        mean_vel = mean(abs(seg(:, COL_VEL)));
        std_vel  = std(abs(seg(:, COL_VEL)));
    else
        mean_vel = 0;
        std_vel  = 0;
    end

    deadband_results(k,:) = [pwr, mean_vel, std_vel];
    fprintf('    %3.0f%%  |  mean: %.4f rad/s  |  std: %.4f\n', ...
        pwr, mean_vel, std_vel);
end

% Stop motor
writeline(device, '0.0');
pause(1.5); flush(device);

% Find deadband threshold
vel_threshold = 0.05;
deadband_idx  = find(deadband_results(:,2) > vel_threshold, 1, 'first');
if ~isempty(deadband_idx)
    deadband_pct = deadband_results(deadband_idx, 1);
    fprintf('\n  Deadband: %.0f%%\n\n', deadband_pct);
else
    deadband_pct = NaN;
    fprintf('\n  WARNING: Wheel did not move. Check wiring.\n\n');
end

%% ================================================================
%% TEST 2: PWM to Velocity Map
%% ================================================================
fprintf('=== TEST 2: PWM to Velocity Map ===\n');

sweep_powers  = [30, 40, 50, 60, 70, 80, 100, ...
                -30,-40,-50,-60,-70,-80,-100];
n_sw          = length(sweep_powers);
sweep_results = zeros(n_sw, 3); % [power, mean_vel, std_vel]
sweep_buffer  = cell(n_sw, 1); % store raw segments for later

for k = 1:n_sw
    pwr = sweep_powers(k);
    seg = run_segment(device, pwr, 3.0, 'Sweep', N_COLS);
    sweep_buffer{k} = seg;

    if ~isempty(seg)
        % Skip first 1500ms — steady state only
        t0         = seg(1, COL_TIME);
        steady_idx = (seg(:, COL_TIME) - t0) > 1500;
        if sum(steady_idx) > 5
            steady_vel = seg(steady_idx, COL_VEL);
        else
            steady_vel = seg(:, COL_VEL);
        end
        sweep_results(k,:) = [pwr, mean(steady_vel), std(steady_vel)];
    end

    fprintf('    %+4.0f%%  |  vel: %+7.4f rad/s  |  std: %.4f\n', ...
        pwr, sweep_results(k,2), sweep_results(k,3));
end

% Stop motor
writeline(device, '0.0');
pause(1.5); flush(device);

% Linear fit above deadband
if ~isnan(deadband_pct)
    above_db = abs(sweep_powers') >= deadband_pct;
else
    above_db = true(n_sw, 1);
end

if sum(above_db) > 3
    p = polyfit(sweep_powers(above_db)', sweep_results(above_db, 2), 1);
    K = p(1);
    fprintf('\n  Plant gain K = %.4f (rad/s)/%% power\n\n', K);
else
    K = NaN; p = [NaN NaN];
    fprintf('\n  Not enough points above deadband for linear fit.\n\n');
end

%% ================================================================
%% TEST 3: Step Response
%% ================================================================
fprintf('=== TEST 3: Step Response ===\n');

step_powers  = [40, 60, 100]; % start above deadband
step_dur     = 4.0;
step_buffer  = cell(length(step_powers), 1);

for k = 1:length(step_powers)
    pwr = step_powers(k);
    seg = run_segment(device, pwr, step_dur, sprintf('Step %.0f%%', pwr), N_COLS);
    step_buffer{k} = seg;
end

% Stop motor
writeline(device, '0.0');
pause(1.0); flush(device);
fprintf('\nAll tests complete. Generating plots...\n\n');

%% ================================================================
%% PLOTS
%% ================================================================

% --- Figure 1: Deadband ---
figure('Name','Test 1 — Deadband','Color','w','Position',[50 50 750 420]);
bar(deadband_results(:,1), deadband_results(:,2), 'FaceColor',[0.2 0.5 0.8]);
hold on;
errorbar(deadband_results(:,1), deadband_results(:,2), deadband_results(:,3), ...
    'k.', 'LineWidth', 1);
yline(vel_threshold, 'r--', 'Threshold', 'LabelHorizontalAlignment','left');
if ~isnan(deadband_pct)
    xline(deadband_pct, 'g--', sprintf('Deadband = %.0f%%', deadband_pct), ...
        'LabelVerticalAlignment','bottom');
end
xlabel('Power (%)'); ylabel('Mean |velocity| (rad/s)');
title('Test 1 — Deadband Identification'); grid on;

% --- Figure 2: PWM to Velocity Map ---
figure('Name','Test 2 — PWM to Velocity Map','Color','w','Position',[100 100 750 450]);
errorbar(sweep_results(:,1), sweep_results(:,2), sweep_results(:,3), ...
    'o', 'MarkerFaceColor','b', 'Color','b', 'LineWidth',1.2);
hold on;
if ~isnan(K)
    x_fit = linspace(min(sweep_powers)-5, max(sweep_powers)+5, 200);
    plot(x_fit, polyval(p, x_fit), 'r--', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('K = %.4f (rad/s)/%%', K));
end
xline(0,'k-'); yline(0,'k-');
if ~isnan(deadband_pct)
    xline( deadband_pct, 'g:', 'Deadband');
    xline(-deadband_pct, 'g:');
end
xlabel('Commanded Power (%)'); ylabel('Steady-State Velocity (rad/s)');
title('Test 2 — Plant Gain K');
legend('Measured (±1σ)', 'Linear fit', 'Location','northwest');
grid on;

% --- Figure 3: Step Responses ---
figure('Name','Test 3 — Step Responses','Color','w','Position',[150 150 900 500]);
colors = {'b','r','g'};
hold on;
for k = 1:length(step_powers)
    seg = step_buffer{k};
    if isempty(seg); continue; end
    t_rel = (seg(:, COL_TIME) - seg(1, COL_TIME)) / 1000;
    plot(t_rel, seg(:, COL_VEL), 'Color', colors{k}, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('%.0f%% power', step_powers(k)));
end
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
title('Test 3 — Step Response');
legend('show','Location','southeast'); grid on;

% --- Figure 4: Forward vs Reverse Symmetry ---
fwd_mask = sweep_powers > 0;
rev_mask = sweep_powers < 0;
fwd_pwr  = sweep_powers(fwd_mask);
fwd_vel  = sweep_results(fwd_mask, 2);
rev_pwr  = abs(sweep_powers(rev_mask));
rev_vel  = abs(sweep_results(rev_mask, 2));
[rev_pwr_s, si] = sort(rev_pwr);
rev_vel_s = rev_vel(si);
[fwd_pwr_s, si] = sort(fwd_pwr);
fwd_vel_s = fwd_vel(si);

figure('Name','Test 2b — Symmetry','Color','w','Position',[200 200 750 420]);
plot(fwd_pwr_s, fwd_vel_s, 'b-o', 'DisplayName','Forward', 'LineWidth',1.2); hold on;
plot(rev_pwr_s, rev_vel_s, 'r-o', 'DisplayName','Reverse (|vel|)', 'LineWidth',1.2);
xlabel('|Power| (%)'); ylabel('|Velocity| (rad/s)');
title('Test 2b — Forward vs Reverse Symmetry');
legend('show'); grid on;

%% ================================================================
%% SUMMARY
%% ================================================================
fprintf('=== OPEN LOOP SUMMARY ===\n');
fprintf('Deadband:      %.0f%%\n', deadband_pct);
fprintf('Plant gain K:  %.4f (rad/s)/%% power\n', K);
fprintf('Max velocity:  %.4f rad/s\n', max(abs(sweep_results(:,2))));

% Time constant from first step segment
seg_step = step_buffer{1};
tau = NaN;
if ~isempty(seg_step) && size(seg_step,1) > 10
    t_rel   = (seg_step(:, COL_TIME) - seg_step(1, COL_TIME)) / 1000;
    vel     = seg_step(:, COL_VEL);
    v_ss    = mean(vel(max(1,end-10):end));
    tau_idx = find(vel >= 0.632 * v_ss, 1, 'first');
    if ~isempty(tau_idx) && tau_idx > 1
        tau = t_rel(tau_idx);
        fprintf('Time constant: %.3f s\n', tau);
    else
        fprintf('Time constant: could not be determined.\n');
    end
end

if ~isnan(K) && ~isnan(tau) && tau > 0 && K ~= 0
    Kp = 1 / K;
    Ki = Kp / tau;
    Kd = Kp * tau * 0.1;
    fprintf('\n--- Suggested PID starting point ---\n');
    fprintf('Kp ~ %.4f\n', Kp);
    fprintf('Ki ~ %.4f\n', Ki);
    fprintf('Kd ~ %.4f\n', Kd);
else
    fprintf('\nCould not compute PID gains — review plots.\n');
end

%% ================================================================
%% SAVE
%% ================================================================
timestamp = datestr(now,'yyyy-mm-dd_HH-MM-SS');

% Sweep summary
sweep_table = array2table(sweep_results, ...
    'VariableNames', {'Power_pct','MeanVel_rads','StdVel_rads'});
writetable(sweep_table, fullfile(save_dir, sprintf('openloop_sweep_%s.csv', timestamp)));

% Raw step data
for k = 1:length(step_powers)
    seg = step_buffer{k};
    if isempty(seg); continue; end
    step_table = array2table(seg, ...
        'VariableNames', {'Time_ms','Power_pct','Angle_rad','Vel_rads'});
    writetable(step_table, fullfile(save_dir, ...
        sprintf('openloop_step%d_%s.csv', step_powers(k), timestamp)));
end

fprintf('\nData saved to %s\n', save_dir);
clear device;