%% pid_autotune_sweep_ff.m
% Feedforward-enabled PID sweep
% Arduino stream: Time_ms, Setpoint, Velocity, Power, Kp, Ki, Kd (7 cols)
% MATLAB sends: P: I: D: F: S: R: commands

clear; clc; close all;

%% ================================================================
%% CONFIG
%% ================================================================
port     = "COM3";
baud     = 115200;
save_dir = 'C:/Users/Amine/Desktop/Project_Make_it_Or_Break_It/wheeled-robot-arm/MATLAB TESTS and VISUALIZATION/CSVs/';

SETPOINT      = 10.0;   % rad/s
SETTLE_WINDOW = 8.0;    % seconds per combination
STOP_PAUSE    = 2.0;    % seconds between tests

% Gain grid — tighter ranges now that feedforward handles steady-state
KP_VALUES  = [4.0, 6.0, 8.0, 10.0];
KI_VALUES  = [2.0, 5.0, 10.0, 15.0];
KD_VALUE   = 0.0;
KFF_VALUE  = 4.366;   % fixed — derived from open-loop K = 0.2291

N_COLS     = 7;
n_combos   = length(KP_VALUES) * length(KI_VALUES);
fprintf('Combinations: %d\n', n_combos);
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
        disp("RX: " + line);
        if contains(line, "READY"); disp("Arduino READY"); break; end
    catch; continue;
    end
end
device.Timeout = 2;
warning('off', 'serialport:readline:timeout');

% Send fixed feedforward gain once at startup
writeline(device, sprintf('F:%.4f', KFF_VALUE));
pause(0.1);
fprintf('Feedforward set: Kff = %.4f\n\n', KFF_VALUE);

%% ================================================================
%% FIGURE
%% ================================================================
fig = figure('Name','PID+FF Auto Sweep','Color','w','Position',[50 50 1200 700]);

subplot(2,3,1);
h_vel = animatedline('Color','b','LineWidth',1.5,'DisplayName','Velocity');
h_sp  = animatedline('Color','r','LineStyle','--','LineWidth',1.2,'DisplayName','Setpoint');
ylabel('Velocity (rad/s)'); legend('show'); grid on;
title('Current Test — Velocity');

subplot(2,3,4);
h_pow = animatedline('Color','k','LineWidth',1.2);
ylabel('Power (%)'); xlabel('Time (s)'); grid on;
title('Current Test — Power');

subplot(2,3,2);
h_os_bar = bar(nan, 'FaceColor',[0.8 0.3 0.2]);
ylabel('Overshoot (%)'); grid on;
title('Overshoot per Test');

subplot(2,3,5);
h_st_bar = bar(nan, 'FaceColor',[0.2 0.5 0.8]);
ylabel('Settling Time (s)'); grid on;
title('Settling Time per Test');

subplot(2,3,3);
h_ss_bar = bar(nan, 'FaceColor',[0.3 0.7 0.3]);
ylabel('SS Error (%)'); grid on;
title('SS Error per Test');

subplot(2,3,6);
h_sc_bar = bar(nan, 'FaceColor',[0.6 0.3 0.8]);
ylabel('Score (lower = better)'); grid on;
title('Composite Score — ranked');

status_lbl = uicontrol('Style','text','Units','normalized', ...
    'Position',[0.01 0.97 0.98 0.03], ...
    'String','Starting...','HorizontalAlignment','left','FontSize',10);

%% ================================================================
%% HELPERS
%% ================================================================
function stop_motor(device, pause_s)
    writeline(device, 'R:');
    writeline(device, 'S:0.00');
    pause(pause_s);
    flush(device);
end

function m = compute_metrics(seg, setpoint)
    if isempty(seg) || size(seg,1) < 20
        m = struct('overshoot',NaN,'settle_t',NaN,'ss_error',NaN,...
                   'ss_vel',NaN,'vel_std',NaN);
        return;
    end

    t_rel = (seg(:,1) - seg(1,1)) / 1000;
    vel   = seg(:,3);
    sp    = setpoint;

    % Steady state from last 20% of segment
    tail     = max(1, round(0.8*length(vel)));
    ss_vel   = mean(vel(tail:end));
    ss_error = abs(ss_vel - sp) / abs(sp) * 100;
    overshoot= max(0, (max(vel) - sp) / abs(sp) * 100);
    vel_std  = std(vel);

    % Settling: first time enters 10% band and stays for 1s
    band       = 0.10 * abs(sp);
    settle_t   = t_rel(end); % default — not settled
    window_pts = round(1.0 / mean(diff(t_rel)));
    for i = 1:length(vel)
        if abs(vel(i) - sp) <= band
            horizon = min(length(vel), i + window_pts);
            if all(abs(vel(i:horizon) - sp) <= band)
                settle_t = t_rel(i);
                break;
            end
        end
    end

    m = struct('overshoot', overshoot, 'settle_t',  settle_t, ...
               'ss_error',  ss_error,  'ss_vel',    ss_vel, ...
               'vel_std',   vel_std);
end

%% ================================================================
%% MAIN SWEEP
%% ================================================================
results      = [];
all_segs     = {};
combo_labels = {};
combo_idx    = 0;

for kp = KP_VALUES
    for ki = KI_VALUES
        combo_idx = combo_idx + 1;
        label     = sprintf('Kp=%.1f Ki=%.1f', kp, ki);
        combo_labels{end+1} = label;

        set(status_lbl, 'String', ...
            sprintf('[%d/%d]  %s  |  Kff=%.3f  Setpoint=%.1f rad/s', ...
            combo_idx, n_combos, label, KFF_VALUE, SETPOINT));
        drawnow;

        % Stop, flush, send new gains
        stop_motor(device, STOP_PAUSE);
        clearpoints(h_vel); clearpoints(h_sp); clearpoints(h_pow);

        writeline(device, sprintf('P:%.4f', kp));   pause(0.05);
        writeline(device, sprintf('I:%.4f', ki));   pause(0.05);
        writeline(device, sprintf('D:%.4f', KD_VALUE)); pause(0.05);
        flush(device);
        writeline(device, sprintf('S:%.2f', SETPOINT));

        % Collect live
        seg = [];
        t0  = [];
        t_end = tic;

        while toc(t_end) < SETTLE_WINDOW
            try
                line = strtrim(string(readline(device)));
            catch
                drawnow limitrate; continue;
            end
            if strlength(line) == 0; continue; end

            v = str2double(split(line, ','));
            if length(v) ~= N_COLS || any(isnan(v)); continue; end

            % Verify Arduino actually received correct gains
            if abs(v(5) - kp) > 0.01 || abs(v(6) - ki) > 0.01
                continue; % skip samples from previous gain set
            end

            seg = [seg; v'];
            if isempty(t0); t0 = v(1); end
            t = (v(1) - t0) / 1000;

            addpoints(h_vel, t, v(3));
            addpoints(h_sp,  t, v(2));
            addpoints(h_pow, t, v(4));

            subplot(2,3,1); xlim([max(0,t-SETTLE_WINDOW) max(t,0.1)]);
            subplot(2,3,4); xlim([max(0,t-SETTLE_WINDOW) max(t,0.1)]);
            drawnow limitrate;
        end

        % Metrics
        m = compute_metrics(seg, SETPOINT);
        all_segs{combo_idx} = seg;
        results(end+1,:)    = [kp, ki, KD_VALUE, KFF_VALUE, ...
                                m.overshoot, m.settle_t, m.ss_error, ...
                                m.ss_vel, m.vel_std];

        fprintf('[%d/%d] %s → OS=%.1f%%  Settle=%.2fs  SSErr=%.2f%%  SSVel=%.3f\n', ...
            combo_idx, n_combos, label, ...
            m.overshoot, m.settle_t, m.ss_error, m.ss_vel);

        % Update bar charts
        os_data = results(:,5);
        st_data = results(:,6);
        ss_data = results(:,7);

        subplot(2,3,2);
        bar(os_data,'FaceColor',[0.8 0.3 0.2]);
        set(gca,'XTickLabel',combo_labels,'XTickLabelRotation',45);
        ylabel('Overshoot (%)'); grid on;

        subplot(2,3,5);
        bar(st_data,'FaceColor',[0.2 0.5 0.8]);
        set(gca,'XTickLabel',combo_labels,'XTickLabelRotation',45);
        ylabel('Settling Time (s)'); grid on;

        subplot(2,3,3);
        bar(ss_data,'FaceColor',[0.3 0.7 0.3]);
        set(gca,'XTickLabel',combo_labels,'XTickLabelRotation',45);
        ylabel('SS Error (%)'); grid on;

        drawnow;
    end
end

stop_motor(device, 1.0);

%% ================================================================
%% RANKING
%% ================================================================
fprintf('\n=== SWEEP COMPLETE ===\n\n');

os_norm  = results(:,5) / (max(results(:,5)) + 1e-6);
st_norm  = results(:,6) / (max(results(:,6)) + 1e-6);
sse_norm = results(:,7) / (max(results(:,7)) + 1e-6);
score    = 0.4*os_norm + 0.4*st_norm + 0.2*sse_norm;

[score_sorted, rank_idx] = sort(score);

fprintf('%-5s  %-6s  %-6s  %-12s  %-12s  %-12s  %s\n', ...
    'Rank','Kp','Ki','Overshoot%','SettleTime','SSError%','Score');
fprintf('%s\n', repmat('-',1,72));
for r = 1:length(rank_idx)
    idx = rank_idx(r);
    fprintf('%-5d  %-6.1f  %-6.1f  %-12.1f  %-12.3f  %-12.2f  %.4f\n', ...
        r, results(idx,1), results(idx,2), results(idx,5), ...
        results(idx,6), results(idx,7), score(idx));
end

best = rank_idx(1);
fprintf('\n=== RECOMMENDED GAINS (with Feedforward) ===\n');
fprintf('  Kp  = %.4f\n', results(best,1));
fprintf('  Ki  = %.4f\n', results(best,2));
fprintf('  Kd  = %.4f\n', results(best,3));
fprintf('  Kff = %.4f\n', results(best,4));

% Update composite score bar chart
subplot(2,3,6);
bar(score_sorted, 'FaceColor',[0.6 0.3 0.8]);
set(gca,'XTickLabel', combo_labels(rank_idx), 'XTickLabelRotation', 45);
ylabel('Score (lower = better)'); grid on;
title('Composite Score — ranked');
drawnow;

%% ================================================================
%% SAVE
%% ================================================================
if ~exist(save_dir,'dir'); mkdir(save_dir); end
timestamp = datestr(now,'yyyy-mm-dd_HH-MM-SS');

% Summary
T = array2table(results, 'VariableNames', ...
    {'Kp','Ki','Kd','Kff','Overshoot_pct','SettleTime_s', ...
     'SSError_pct','SSVel_rads','VelStd_rads'});
writetable(T, fullfile(save_dir, sprintf('pid_ff_sweep_summary_%s.csv', timestamp)));

% Raw segments
for k = 1:length(all_segs)
    seg = all_segs{k};
    if isempty(seg); continue; end
    kp_k = results(k,1); ki_k = results(k,2);
    seg_T = array2table(seg, 'VariableNames', ...
        {'Time_ms','Setpoint_rads','Velocity_rads','Power_pct', ...
         'Kp','Ki','Kd'});
    fname = sprintf('pid_ff_kp%.1f_ki%.1f_%s.csv', kp_k, ki_k, timestamp);
    writetable(seg_T, fullfile(save_dir, fname));
end

fprintf('\nAll data saved to %s\n', save_dir);
clear device;