clear; clc; close all;

%% Connection
port   = "COM3";
baud   = 115200;
device = serialport(port, baud);
configureTerminator(device, "LF");
flush(device);
pause(2);
disp("Connected to Arduino.");

%% Wait for READY
disp("Waiting for READY...");
device.Timeout = 30;
while true
    try
        line = strtrim(string(readline(device)));
        disp("RX: " + line);
        if contains(line, "READY")
            disp("Arduino READY"); break;
        end
    catch; continue;
    end
end
device.Timeout = 2;

%% Figure with UI input box — NON-BLOCKING setpoint entry
fig = figure('Name','Live PID Monitor','Color','w','Position',[100 100 900 600]);

% Setpoint input panel at top
uicontrol('Style','text','Units','normalized', ...
    'Position',[0.01 0.93 0.15 0.05],'String','Setpoint (rad/s):');
sp_box = uicontrol('Style','edit','Units','normalized', ...
    'Position',[0.17 0.93 0.12 0.05],'String','0');
uicontrol('Style','pushbutton','Units','normalized', ...
    'Position',[0.30 0.93 0.10 0.05],'String','Send', ...
    'Callback', @(~,~) send_setpoint(device, sp_box));

% Status label
status_lbl = uicontrol('Style','text','Units','normalized', ...
    'Position',[0.42 0.93 0.45 0.05], ...
    'String','Waiting for data...','HorizontalAlignment','left');

% Velocity subplot
subplot(2,1,1);
h_vel = animatedline('Color','b','LineWidth',1.5,'DisplayName','Velocity');
h_sp  = animatedline('Color','y','LineStyle','--','LineWidth',1.2,'DisplayName','Setpoint');
ylabel('Velocity (rad/s)'); legend('show'); grid on;
title('Velocity Tracking');

% Power subplot
subplot(2,1,2);
h_pow = animatedline('Color','w','LineWidth',1.2);
ylabel('Motor Power (%)'); xlabel('Time (s)'); grid on;
title('Motor Power Output');

%% Callback: send setpoint when button pressed
function send_setpoint(device, sp_box)
    val = str2double(get(sp_box, 'String'));
    if ~isnan(val)
        writeline(device, sprintf('%.2f', val));
        fprintf('Setpoint sent: %.2f rad/s\n', val);
    end
end

%% Storage
all_data   = zeros(10000, 4); % [Time_ms, Setpoint, Velocity, Power]
n          = 0;
time_window = 10;
start_time  = [];

%% Main streaming loop
try
    while isvalid(fig)
        try
            line = readline(device);
        catch
            drawnow; continue;
        end

        if ~(isstring(line) || ischar(line)); continue; end
        line = strtrim(string(line));
        if strlength(line) == 0; continue; end

        data = str2double(split(line, ','));
        if length(data) ~= 4 || any(isnan(data)); continue; end

        t_ms  = data(1);
        sp_rx = data(2);
        vel   = data(3);
        power = data(4);

        if isempty(start_time); start_time = t_ms; end
        t = (t_ms - start_time) / 1000;

        % Store sample
        n = n + 1;
        if n > size(all_data, 1)
            all_data = [all_data; zeros(10000, 4)];
        end
        all_data(n, :) = [t_ms, sp_rx, vel, power];

        % Update plots
        addpoints(h_vel, t, vel);
        addpoints(h_sp,  t, sp_rx);
        addpoints(h_pow, t, power);

        t_lo = max(0, t - time_window);
        t_hi = max(t, t_lo + 0.001);

        subplot(2,1,1); xlim([t_lo t_hi]);
        subplot(2,1,2); xlim([t_lo t_hi]);

        set(status_lbl, 'String', sprintf('t=%.1fs  sp=%.2f  vel=%.3f rad/s  pwr=%.1f%%', ...
            t, sp_rx, vel, power));

        drawnow limitrate;
    end

catch ME
    fprintf('Loop stopped: %s\n', ME.message);
end

% Save block — runs after loop exits for any reason
try; clear device; catch; end

save_dir = 'C:/Users/Amine/Desktop/Project_Make_it_Or_Break_It/wheeled-robot-arm/MATLAB TESTS and VISUALIZATION/CSVs/';
if ~exist(save_dir, 'dir'); mkdir(save_dir); end

if n > 0
    all_data = all_data(1:n, :);
    filename = fullfile(save_dir, sprintf('pid_response_%s.csv', ...
               datestr(now, 'yyyy-mm-dd_HH-MM-SS')));
    T = array2table(all_data, 'VariableNames', ...
        {'Time_ms','Setpoint_rads','Velocity_rads','Power_pct'});
    writetable(T, filename);
    fprintf('Saved %d samples to %s\n', n, filename);
else
    fprintf('No data collected — nothing saved.\n');
end

fprintf('Session ended.\n');