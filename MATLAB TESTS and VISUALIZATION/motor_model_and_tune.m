%% motor_model_and_tune.m
% Loads open-loop CSV, fits first-order transfer function,
% runs pidtune() for P / PI / PID, simulates closed-loop responses,
% and exports gains for use in Simulink and the Arduino PID library.

clear; clc; close all;

%% ================================================================
%% 1. Load open-loop data
%% ================================================================
csv_dir  = 'C:/Users/Amine/Desktop/Project_Make_it_Or_Break_It/wheeled-robot-arm/MATLAB TESTS and VISUALIZATION/CSVs/';

% Load sweep summary
[sweep_file, sweep_path] = uigetfile(fullfile(csv_dir, 'openloop_sweep_*.csv'), ...
    'Select sweep CSV');
if isequal(sweep_file, 0); error('No file selected.'); end
sweep = readtable(fullfile(sweep_path, sweep_file));

% Load 40% step response (used for τ identification)
[step_file, step_path] = uigetfile(fullfile(csv_dir, 'openloop_step40_*.csv'), ...
    'Select step40 CSV');
if isequal(step_file, 0); error('No file selected.'); end
step_data = readtable(fullfile(step_path, step_file));

fprintf('=== LOADED DATA ===\n');
fprintf('Sweep: %s\n', sweep_file);
fprintf('Step:  %s\n\n', step_file);

%% ================================================================
%% 2. Re-derive K from sweep data
%% ================================================================
% Use only points above deadband (non-zero velocity)
valid = abs(sweep.MeanVel_rads) > 0.1;
powers = sweep.Power_pct(valid);
vels   = sweep.MeanVel_rads(valid);

p     = polyfit(powers, vels, 1);
K     = p(1);   % (rad/s) / % power
K_off = p(2);   % offset (should be near 0)

fprintf('=== PLANT IDENTIFICATION ===\n');
fprintf('Plant gain K = %.4f (rad/s)/%% power\n', K);

%% ================================================================
%% 3. Re-derive τ from step response
%% ================================================================
t_ms  = step_data.Time_ms;
vel   = step_data.Vel_rads;
t_s   = (t_ms - t_ms(1)) / 1000;  % relative time in seconds

% Steady-state = mean of last 10 samples
v_ss  = mean(vel(end-10:end));

% τ = time to reach 63.2% of steady state
tau_idx = find(vel >= 0.632 * v_ss, 1, 'first');
if isempty(tau_idx) || tau_idx <= 1
    error('Could not identify τ — check step response data.');
end
tau = t_s(tau_idx);

fprintf('Time constant τ = %.4f s\n', tau);
fprintf('Steady-state velocity at 40%% = %.4f rad/s\n\n', v_ss);

%% ================================================================
%% 4. Build transfer function  G(s) = K / (τs + 1)
%% ================================================================
G = tf(K, [tau 1]);

fprintf('=== TRANSFER FUNCTION ===\n');
fprintf('G(s) = %.4f / (%.4fs + 1)\n\n', K, tau);

%% ================================================================
%% 5. Auto-tune PID gains with pidtune()
%%    Try three controller types and compare
%% ================================================================
fprintf('=== PIDTUNE RESULTS ===\n');

% P only
C_p   = pidtune(G, 'P');
fprintf('P  controller:  Kp = %.4f\n', C_p.Kp);

% PI
C_pi  = pidtune(G, 'PI');
fprintf('PI controller:  Kp = %.4f  Ki = %.4f\n', C_pi.Kp, C_pi.Ki);

% PID
C_pid = pidtune(G, 'PID');
fprintf('PID controller: Kp = %.4f  Ki = %.4f  Kd = %.4f\n\n', ...
    C_pid.Kp, C_pid.Ki, C_pid.Kd);

% PID with target bandwidth (faster response — optional)
opts = pidtuneOptions('CrossoverFrequency', 2/tau, 'PhaseMargin', 60);
C_pid_fast = pidtune(G, 'PID', opts);
fprintf('PID (fast, 60deg margin): Kp=%.4f  Ki=%.4f  Kd=%.4f\n\n', ...
    C_pid_fast.Kp, C_pid_fast.Ki, C_pid_fast.Kd);

%% ================================================================
%% 6. Simulate closed-loop step responses
%% ================================================================
t_sim  = 0:0.001:2;   % 2 second simulation at 1ms resolution
r      = ones(size(t_sim)); % unit step setpoint (1 rad/s target)

% Build closed-loop systems
CL_p    = feedback(C_p   * G, 1);
CL_pi   = feedback(C_pi  * G, 1);
CL_pid  = feedback(C_pid * G, 1);
CL_fast = feedback(C_pid_fast * G, 1);

y_p    = lsim(CL_p,    r, t_sim);
y_pi   = lsim(CL_pi,   r, t_sim);
y_pid  = lsim(CL_pid,  r, t_sim);
y_fast = lsim(CL_fast, r, t_sim);

%% ================================================================
%% 7. Compute performance metrics for each controller
%% ================================================================
function metrics = get_metrics(y, t, r_val)
    % Settling time (within 2% of setpoint)
    settled  = find(abs(y - r_val) <= 0.02 * r_val, 1, 'last');
    settle_t = t(end); % default if never settles
    for k = length(y):-1:1
        if abs(y(k) - r_val) > 0.02 * r_val
            settle_t = t(min(k+1, length(t)));
            break;
        end
    end
    overshoot = max(0, (max(y) - r_val) / r_val * 100);
    ss_error  = abs(y(end) - r_val) / r_val * 100;
    metrics   = struct('settle_t', settle_t, 'overshoot', overshoot, ...
                       'ss_error', ss_error);
end

m_p    = get_metrics(y_p,    t_sim, 1);
m_pi   = get_metrics(y_pi,   t_sim, 1);
m_pid  = get_metrics(y_pid,  t_sim, 1);
m_fast = get_metrics(y_fast, t_sim, 1);

fprintf('=== PERFORMANCE METRICS (1 rad/s step) ===\n');
fprintf('%-20s  %10s  %12s  %10s\n', 'Controller', 'Settle(s)', 'Overshoot%%', 'SS Error%%');
fprintf('%-20s  %10.3f  %12.1f  %10.2f\n', 'P',        m_p.settle_t,    m_p.overshoot,    m_p.ss_error);
fprintf('%-20s  %10.3f  %12.1f  %10.2f\n', 'PI',       m_pi.settle_t,   m_pi.overshoot,   m_pi.ss_error);
fprintf('%-20s  %10.3f  %12.1f  %10.2f\n', 'PID',      m_pid.settle_t,  m_pid.overshoot,  m_pid.ss_error);
fprintf('%-20s  %10.3f  %12.1f  %10.2f\n', 'PID fast', m_fast.settle_t, m_fast.overshoot, m_fast.ss_error);
fprintf('\n');

%% ================================================================
%% 8. Plots
%% ================================================================

% --- Fig 1: Raw step data + fitted model ---
figure('Name','Plant Identification','Color','w','Position',[50 50 900 400]);

subplot(1,2,1);
hold on;
% Measured step response
plot(t_s, vel, 'b', 'LineWidth', 1.2, 'DisplayName', 'Measured');
% Model prediction: v(t) = v_ss * (1 - exp(-t/tau))
v_model = v_ss * (1 - exp(-t_s / tau));
plot(t_s, v_model, 'r--', 'LineWidth', 1.5, 'DisplayName', ...
    sprintf('Model: K=%.3f, τ=%.3f', K, tau));
xline(tau, 'g:', sprintf('τ = %.3fs', tau), 'LabelVerticalAlignment','bottom');
yline(0.632 * v_ss, 'g:', '63.2%');
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
title('Step Response: Measured vs Model');
legend('show','Location','southeast'); grid on;

subplot(1,2,2);
powers_plot = linspace(min(powers), max(powers), 100);
plot(sweep.Power_pct, sweep.MeanVel_rads, 'bo', ...
    'MarkerFaceColor','b', 'DisplayName', 'Measured'); hold on;
plot(powers_plot, polyval(p, powers_plot), 'r--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Fit: K=%.4f', K));
xline(0,'k-'); yline(0,'k-');
xlabel('Power (%)'); ylabel('Velocity (rad/s)');
title('PWM-Velocity Map: Measured vs Fit');
legend('show','Location','northwest'); grid on;

% --- Fig 2: Closed-loop step responses ---
figure('Name','Closed-Loop Simulation','Color','w','Position',[100 100 900 500]);
plot(t_sim, r,     'k--', 'LineWidth', 1.0, 'DisplayName', 'Setpoint'); hold on;
plot(t_sim, y_p,   'b',   'LineWidth', 1.2, 'DisplayName', ...
    sprintf('P  (Kp=%.2f)', C_p.Kp));
plot(t_sim, y_pi,  'r',   'LineWidth', 1.2, 'DisplayName', ...
    sprintf('PI (Kp=%.2f Ki=%.2f)', C_pi.Kp, C_pi.Ki));
plot(t_sim, y_pid, 'g',   'LineWidth', 1.2, 'DisplayName', ...
    sprintf('PID (Kp=%.2f Ki=%.2f Kd=%.3f)', C_pid.Kp, C_pid.Ki, C_pid.Kd));
plot(t_sim, y_fast,'m',   'LineWidth', 1.2, 'DisplayName', ...
    sprintf('PID fast (Kp=%.2f Ki=%.2f Kd=%.3f)', C_pid_fast.Kp, C_pid_fast.Ki, C_pid_fast.Kd));
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
title('Closed-Loop Step Response Comparison (Simulated)');
legend('show','Location','southeast'); grid on;
ylim([0 1.5]);

% --- Fig 3: Bode plot of plant ---
figure('Name','Plant Bode Plot','Color','w','Position',[150 150 800 500]);
bode(G);
title('Plant Bode Plot — G(s)');
grid on;

% --- Fig 4: Step info comparison bar chart ---
figure('Name','Controller Comparison','Color','w','Position',[200 200 800 400]);
ctrl_names = {'P','PI','PID','PID fast'};
settle_times  = [m_p.settle_t, m_pi.settle_t, m_pid.settle_t, m_fast.settle_t];
overshoots    = [m_p.overshoot, m_pi.overshoot, m_pid.overshoot, m_fast.overshoot];

subplot(1,2,1);
bar(settle_times, 'FaceColor',[0.2 0.5 0.8]);
set(gca,'XTickLabel', ctrl_names);
ylabel('Settling Time (s)'); title('Settling Time'); grid on;

subplot(1,2,2);
bar(overshoots, 'FaceColor',[0.8 0.3 0.2]);
set(gca,'XTickLabel', ctrl_names);
ylabel('Overshoot (%)'); title('Overshoot'); grid on;

%% ================================================================
%% 9. Export gains for Simulink and Arduino
%% ================================================================

% Choose PID as default — change to C_pid_fast if faster response needed
C_chosen = C_pid;

pid_gains = struct();
pid_gains.Kp          = C_chosen.Kp;
pid_gains.Ki          = C_chosen.Ki;
pid_gains.Kd          = C_chosen.Kd;
pid_gains.K           = K;
pid_gains.tau         = tau;
pid_gains.deadband    = 30.0;   % % power — from Test 1
pid_gains.dt          = 0.010;  % 10ms control interval
pid_gains.output_min  = -100.0;
pid_gains.output_max  =  100.0;

save(fullfile(csv_dir, 'pid_gains.mat'), 'pid_gains', 'G', 'K', 'tau');
fprintf('=== EXPORTED ===\n');
fprintf('Gains saved to pid_gains.mat\n');
fprintf('\nArduino PID library starting values:\n');
fprintf('  float Kp       = %.4f;\n', pid_gains.Kp);
fprintf('  float Ki       = %.4f;\n', pid_gains.Ki);
fprintf('  float Kd       = %.4f;\n', pid_gains.Kd);
fprintf('  float deadband = %.1f;  // %% power\n', pid_gains.deadband);
fprintf('  float dt       = %.3f;  // seconds\n', pid_gains.dt);