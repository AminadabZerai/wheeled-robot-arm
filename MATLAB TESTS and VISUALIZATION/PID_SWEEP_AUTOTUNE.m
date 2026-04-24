%% pid_autotune_sweep.m
% Sends PID gains and setpoint over serial, logs response per combination,
% computes metrics, ranks results, and saves everything to CSV.

clear; clc; close all;

%% ================================================================
%% CONFIG — edit these
%% ================================================================
port     = "COM3";
baud     = 115200;
save_dir = 'C:/Users/Amine/Desktop/Project_Make_it_Or_Break_It/wheeled-robot-arm/MATLAB TESTS and VISUALIZATION/CSVs/';

SETTLE_WINDOW = 10.0;  % double it — system needs time to accumulate
SETPOINT      = 10.0;
STOP_PAUSE    = 2.0;    % seconds motor off between tests

% Gain grid to sweep — based on analysis: Ki must be much lower
KP_VALUES     = [6.0, 8.0, 10.0, 12.0];
KI_VALUES     = [5.0, 10.0, 20.0, 30.0];
KD_VALUE      = 0.0;

% Derived
n_combos  = length(KP_VALUES) * length(KI_VALUES);
fprintf('Total combinations: %d\n', n_combos);
fprintf('Estimated time: %.0f minutes\n\n', ...
    n_combos * (SETTLE_WINDOW + STOP_PAUSE) / 60);

%% ================================================================
%% CONNECTION
%% ================================================================
device = serialport(port, baud);
configureTerminator(device, "LF");
flush(device);
pause(2);
disp("Connected.");

device.Timeout = 30;
disp("Waiting for READY...");
while true
    try
        line = strtrim(string(readline(device)));
        if contains(line, "READY"); disp("Arduino READY"); break; end
    catch; continue;
    end
end
device.Timeout = 2;
warning('off', 'serialport:readline:timeout');

%% ================================================================
%% FIGURE — live view
%% ================================================================
fig = figure('Name','PID Auto Sweep','Color','w','Position',[50 50 1100 650]);

subplot(2,2,1);
h_vel = animatedline('Color','b','LineWidth',1.5,'DisplayName','Velocity');
h_sp  = animatedline('Color','r','LineStyle','--','LineWidth',1.2,'DisplayName','Setpoint');
ylabel('Velocity (rad/s)'); legend('show'); grid on;
title('Current Test — Velocity');

subplot(2,2,2);
h_pow = animatedline('Color','c','LineWidth',1.2);
ylabel('Power (%)'); grid on;
title('Current Test — Motor Power');

subplot(2,2,3);
h_os  = bar(0, 'FaceColor',[0.8 0.3 0.2]);
ylabel('Overshoot (%)'); grid on;
title('Overshoot per Test');

subplot(2,2,4);
h_st  = bar(0, 'FaceColor',[0.2 0.5 0.8]);
ylabel('Settling Time (s)'); grid on;
title('Settling Time per Test');

status_lbl = uicontrol('Style','text','Units','normalized', ...
    'Position',[0.01 0.96 0.98 0.03], ...
    'String','Starting...','HorizontalAlignment','left','FontSize',10);

%% ================================================================
%% HELPER: flush and stop motor cleanly between tests
%% ================================================================
function stop_motor(device, pause_s)
    writeline(device, 'R:');
    writeline(device, 'S:0.00');
    pause(pause_s);
    flush(device);
end

%% ================================================================
%% HELPER: collect N seconds of serial data into matrix
%% ================================================================
function data = collect_segment(device, duration_s)
    data  = [];
    t_end = tic;
    while toc(t_end) < duration_s
        try
            line = strtrim(string(readline(device)));
        catch
            continue;
        end
        if strlength(line) == 0; continue; end
        v = str2double(split(line, ','));
        if length(v) == 4 && ~any(isnan(v))
            data = [data; v'];
        end
    end
end

%% ================================================================
%% HELPER: compute metrics from a response segment
%% ================================================================
function m = compute_metrics(data, setpoint, dt_col, sp_col, vel_col, pow_col)
    if isempty(data)
        m = struct('overshoot',NaN,'settle_t',NaN,'ss_error',NaN,'ss_vel',NaN);
        return;
    end

    t_rel = (data(:, dt_col) - data(1, dt_col)) / 1000;
    vel   = data(:, vel_col);
    sp    = setpoint;

    % Steady state from last 20% of segment
    tail_idx  = round(0.8 * length(vel)):length(vel);
    ss_vel    = mean(vel(tail_idx));
    ss_error  = abs(ss_vel - sp) / sp * 100;
    overshoot = max(0, (max(vel) - sp) / sp * 100);

    % Settling time: last time error exceeds 10% band
    band      = 0.10 * sp;
    out_idx   = find(abs(vel - sp) > band, 1, 'last');
    if ~isempty(out_idx) && out_idx < length(t_rel)
        settle_t = t_rel(out_idx + 1);
    else
        settle_t = t_rel(end);
    end

    m = struct('overshoot', overshoot, 'settle_t', settle_t, ...
               'ss_error', ss_error,   'ss_vel',   ss_vel);
end

%% ================================================================
%% MAIN SWEEP LOOP
%% ================================================================
results  = [];  % [Kp, Ki, Kd, overshoot, settle_t, ss_error, ss_vel]
all_segs = {};  % raw data per combination
combo_labels = {};
combo_idx    = 0;

for kp = KP_VALUES
    for ki = KI_VALUES
        combo_idx = combo_idx + 1;
        label = sprintf('Kp=%.1f Ki=%.2f', kp, ki);
        combo_labels{end+1} = label;

        set(status_lbl, 'String', sprintf('[%d/%d] Testing %s — setpoint %.1f rad/s', ...
            combo_idx, n_combos, label, SETPOINT));
        drawnow;

        % --- Send gains ---
        stop_motor(device, STOP_PAUSE);
        clearpoints(h_vel); clearpoints(h_sp); clearpoints(h_pow);

        writeline(device, sprintf('P:%.4f', kp));  pause(0.05);
        writeline(device, sprintf('I:%.4f', ki));  pause(0.05);
        writeline(device, sprintf('D:%.4f', KD_VALUE)); pause(0.05);
        writeline(device, sprintf('S:%.2f', SETPOINT));
        flush(device);

        % --- Collect and plot live ---
        seg  = [];
        t_end = tic;
        t0    = [];

        while toc(t_end) < SETTLE_WINDOW
            try
                line = strtrim(string(readline(device)));
            catch
                drawnow limitrate; continue;
            end
            if strlength(line) == 0; continue; end
            v = str2double(split(line, ','));
            if length(v) ~= 4 || any(isnan(v)); continue; end

            seg = [seg; v'];

            if isempty(t0); t0 = v(1); end
            t = (v(1) - t0) / 1000;

            addpoints(h_vel, t, v(3));
            addpoints(h_sp,  t, v(2));
            addpoints(h_pow, t, v(4));

            subplot(2,2,1);
            xlim([max(0, t-SETTLE_WINDOW) max(t, 0.1)]);
            subplot(2,2,2);
            xlim([max(0, t-SETTLE_WINDOW) max(t, 0.1)]);

            drawnow limitrate;
        end

        % --- Compute metrics ---
        m = compute_metrics(seg, SETPOINT, 1, 2, 3, 4);
        all_segs{combo_idx} = seg;
        results(end+1, :)   = [kp, ki, KD_VALUE, ...
                                m.overshoot, m.settle_t, m.ss_error, m.ss_vel];

        fprintf('[%d/%d] %s → OS=%.1f%%  Settle=%.2fs  SSErr=%.1f%%  SSVel=%.3f\n', ...
            combo_idx, n_combos, label, ...
            m.overshoot, m.settle_t, m.ss_error, m.ss_vel);

        % Update bar charts
        subplot(2,2,3);
        bar(results(:,4), 'FaceColor',[0.8 0.3 0.2]);
        set(gca,'XTickLabel', combo_labels, 'XTickLabelRotation', 45);
        ylabel('Overshoot (%)'); grid on;

        subplot(2,2,4);
        bar(results(:,5), 'FaceColor',[0.2 0.5 0.8]);
        set(gca,'XTickLabel', combo_labels, 'XTickLabelRotation', 45);
        ylabel('Settling Time (s)'); grid on;

        drawnow;
    end
end

% Stop motor at end
stop_motor(device, 1.0);
clear device;

%% ================================================================
%% RESULTS TABLE & RANKING
%% ================================================================
fprintf('\n=== SWEEP COMPLETE ===\n\n');

T = array2table(results, 'VariableNames', ...
    {'Kp','Ki','Kd','Overshoot_pct','SettleTime_s','SSError_pct','SSVel_rads'});

% Score: weighted combination — lower is better
% Weight: overshoot 40%, settling time 40%, SS error 20%
os_norm  = results(:,4) / max(results(:,4) + 1e-6);
st_norm  = results(:,5) / max(results(:,5) + 1e-6);
sse_norm = results(:,6) / max(results(:,6) + 1e-6);
score    = 0.4*os_norm + 0.4*st_norm + 0.2*sse_norm;

[~, rank_idx] = sort(score);

fprintf('%-5s  %-6s  %-6s  %-12s  %-12s  %-12s  %s\n', ...
    'Rank','Kp','Ki','Overshoot%','SettleTime','SSError%','Score');
fprintf('%s\n', repmat('-',1,75));
for r = 1:length(rank_idx)
    idx = rank_idx(r);
    fprintf('%-5d  %-6.2f  %-6.3f  %-12.1f  %-12.3f  %-12.1f  %.4f\n', ...
        r, results(idx,1), results(idx,2), results(idx,4), ...
        results(idx,5), results(idx,6), score(idx));
end

best_idx = rank_idx(1);
fprintf('\n=== RECOMMENDED GAINS ===\n');
fprintf('  Kp = %.4f\n', results(best_idx,1));
fprintf('  Ki = %.4f\n', results(best_idx,2));
fprintf('  Kd = %.4f\n', results(best_idx,3));

%% ================================================================
%% SAVE
%% ================================================================
if ~exist(save_dir,'dir'); mkdir(save_dir); end
timestamp = datestr(now,'yyyy-mm-dd_HH-MM-SS');

% Summary table
writetable(T, fullfile(save_dir, sprintf('pid_sweep_summary_%s.csv', timestamp)));

% Raw segments per combination
for k = 1:length(all_segs)
    seg = all_segs{k};
    if isempty(seg); continue; end
    kp_k = results(k,1); ki_k = results(k,2);
    seg_T = array2table(seg, 'VariableNames', ...
        {'Time_ms','Setpoint_rads','Velocity_rads','Power_pct'});
    fname = sprintf('pid_sweep_kp%.1f_ki%.3f_%s.csv', kp_k, ki_k, timestamp);
    writetable(seg_T, fullfile(save_dir, fname));
end

fprintf('\nAll data saved to %s\n', save_dir);

%% ================================================================
%% FINAL COMPARISON PLOT
%% ================================================================
figure('Name','Sweep Results — All Combinations','Color','w', ...
    'Position',[100 100 1200 500]);

subplot(1,3,1);
bar(results(rank_idx, 4), 'FaceColor',[0.8 0.3 0.2]);
set(gca,'XTickLabel', combo_labels(rank_idx), 'XTickLabelRotation',45);
ylabel('Overshoot (%)'); title('Overshoot — ranked'); grid on;

subplot(1,3,2);
bar(results(rank_idx, 5), 'FaceColor',[0.2 0.5 0.8]);
set(gca,'XTickLabel', combo_labels(rank_idx), 'XTickLabelRotation',45);
ylabel('Settling Time (s)'); title('Settling Time — ranked'); grid on;

subplot(1,3,3);
bar(results(rank_idx, 6), 'FaceColor',[0.3 0.7 0.3]);
set(gca,'XTickLabel', combo_labels(rank_idx), 'XTickLabelRotation',45);
ylabel('SS Error (%)'); title('Steady-State Error — ranked'); grid on;