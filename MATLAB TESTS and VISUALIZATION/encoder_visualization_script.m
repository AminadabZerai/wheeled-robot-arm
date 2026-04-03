%% encoder_stream.m
% Live visualization and CSV logging of encoder data only
% Arduino serial format: encRad,encVel

clear; clc; close all;

%% 1. Connection
port     = "COM3";
baudrate = 115200;
device   = serialport(port, baudrate);
configureTerminator(device, "LF");
device.Timeout = 30;

pause(1); flush(device);
fprintf('Waiting for Arduino...\n');
readline(device);
fprintf('Encoder stream started.\n');

%% 2. Figure
fig = figure('Name', 'Encoder Live Stream', 'NumberTitle', 'off', 'Color', 'w');

subplot(2,1,1);
hEncRad = animatedline('Color', 'b', 'DisplayName', 'Angle (rad)');
ax1 = gca; ax1.YLim = [-10 10];
title('Encoder LF — Angle'); ylabel('rad');
legend('show'); grid on;

subplot(2,1,2);
hEncVel = animatedline('Color', 'r', 'DisplayName', 'Velocity (rad/s)');
ax2 = gca; ax2.YLim = [-50 50];
title('Encoder LF — Angular Velocity'); ylabel('rad/s'); xlabel('Samples');
legend('show'); grid on;

%% 3. Storage
bufferSize = 10000;
allData    = zeros(bufferSize, 3); % [Time, EncRad, EncVel]
i          = 0;
radMax     = 10;
velMax     = 50;
tic;

%% 4. Acquisition Loop
try
    while isvalid(fig)
        rawLine = readline(device);
        if ismissing(rawLine) || strlength(rawLine) == 0; continue; end

        dataVector = str2double(strsplit(rawLine, ','));

        if length(dataVector) == 2 && ~any(isnan(dataVector))
            i = i + 1;
            if i > size(allData, 1)
                allData = [allData; zeros(bufferSize, 3)];
            end

            allData(i, :) = [toc, dataVector(:)'];

            encRad = dataVector(1);
            encVel = dataVector(2);

            addpoints(hEncRad, i, encRad);
            addpoints(hEncVel, i, encVel);

            if abs(encRad) > radMax * 0.9; radMax = abs(encRad) * 1.5; ax1.YLim = [-radMax radMax]; end
            if abs(encVel) > velMax * 0.9; velMax = abs(encVel) * 1.5; ax2.YLim = [-velMax velMax]; end

            if i > 200
                ax1.XLim = [i-200 i];
                ax2.XLim = [i-200 i];
            end

            drawnow limitrate;
        end
    end

catch ME
    if ~strcmp(ME.identifier, 'MATLAB:serial:readline:interrupted')
        fprintf('Stopped: %s\n', ME.message);
    end

finally
    clear device;
    if i > 0
        allData  = allData(1:i, :);
        filename = sprintf('encoder_data_%s.csv', datestr(now, 'yyyy-mm-dd_HH-MM-SS'));
        header   = {'Time_s','EncRad','EncVel'};
        fid      = fopen(filename, 'w');
        fprintf(fid, '%s,%s,%s\n', header{:});
        fclose(fid);
        dlmwrite(filename, allData, '-append', 'precision', '%.6f');
        fprintf('Saved %d samples to %s\n', i, filename);
    else
        fprintf('No data collected — nothing saved.\n');
    end
end