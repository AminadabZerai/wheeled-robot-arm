%% motor_simulink_builder.m
% Loads gains and opens an existing Simulink model selected by the user

clear; clc;

csv_dir = 'C:/Users/Amine/Desktop/Project_Make_it_Or_Break_It/wheeled-robot-arm/MATLAB TESTS and VISUALIZATION/CSVs/';

%% Load gains from tuning script
load(fullfile(csv_dir, 'pid_gains.mat'));
fprintf('Loaded gains: Kp=%.4f  Ki=%.4f  Kd=%.4f\n', ...
    pid_gains.Kp, pid_gains.Ki, pid_gains.Kd);

%% ================================================================
%% Select and open existing Simulink model
%% ================================================================
[model_file, model_path] = uigetfile('*.slx', 'Select Simulink model');

if isequal(model_file, 0)
    error('No Simulink model selected.');
end

model_fullpath = fullfile(model_path, model_file);
[~, model_name, ~] = fileparts(model_file);

open_system(model_fullpath);
fprintf('Opened model: %s\n', model_file);

% Set PID block gains programmatically
pid_block = [model_name '/PID'];
set_param(pid_block, 'P', num2str(pid_gains.Kp));
set_param(pid_block, 'I', num2str(pid_gains.Ki));
set_param(pid_block, 'D', num2str(pid_gains.Kd));

%% ================================================================
%% Run simulation
%% ================================================================
fprintf('\nRunning simulation...\n');
sim_out = sim(model_name, 'StopTime', '3');

%% Extract data (StructureWithTime expected)
data  = sim_out.sim_output;
t_sim = data.time;
y_sim = data.signals.values;

%% ================================================================
%% Plot results
%% ================================================================
figure('Name','Simulink Closed-Loop Result','Color','w','Position',[100 100 900 450]);

subplot(1,2,1);
plot(t_sim, y_sim, 'b', 'LineWidth', 1.5, 'DisplayName', 'Velocity'); hold on;
yline(10, 'r--', 'Setpoint (10 rad/s)');
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
title('Simulated Closed-Loop Step Response');
legend('show','Location','southeast'); grid on;

subplot(1,2,2);
info = stepinfo(y_sim, t_sim, 10);
metrics = {'Rise Time','Settle Time','Overshoot'};
values  = [info.RiseTime, info.SettlingTime, info.Overshoot];
bar(values, 'FaceColor',[0.2 0.6 0.4]);
set(gca,'XTickLabel', metrics);
title('Step Response Metrics');
grid on;

%% ================================================================
%% Print results
%% ================================================================
fprintf('\n=== SIMULATION RESULTS ===\n');
fprintf('Rise time:     %.3f s\n', info.RiseTime);
fprintf('Settling time: %.3f s\n', info.SettlingTime);
fprintf('Overshoot:     %.1f %%\n', info.Overshoot);
fprintf('Steady-state:  %.4f rad/s\n', y_sim(end));