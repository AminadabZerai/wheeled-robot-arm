%% imu_stream.m
% Live visualization and CSV logging of IMU data only
% Arduino serial format: accX,accY,accZ,gyrX,gyrY,gyrZ

clear; clc; close all;

%% 1. Connection
port     = "COM3";
baudrate = 115200;
device   = serialport(port, baudrate);
configureTerminator(device, "LF");
device.Timeout = 30;

pause(1); flush(device);
fprintf('Waiting for calibration to finish...\n');
readline(device); % discard first line (may be partial)
fprintf('IMU stream started.\n');

%% 2. Figure
fig = figure('Name', 'IMU Live Stream', 'NumberTitle', 'off', 'Color', 'w');

subplot(2,1,1);
hAccX = animatedline('Color', 'r', 'DisplayName', 'Acc X');
hAccY = animatedline('Color', 'g', 'DisplayName', 'Acc Y');
hAccZ = animatedline('Color', 'b', 'DisplayName', 'Acc Z');
ax1 = gca; ax1.YLim = [-20 20];
title('Accelerometer (m/s^2)'); ylabel('m/s^2');
legend('show'); grid on;

subplot(2,1,2);
hGyrX = animatedline('Color', 'm', 'DisplayName', 'Gyr X');
hGyrY = animatedline('Color', 'c', 'DisplayName', 'Gyr Y');
hGyrZ = animatedline('Color', [1 0.5 0], 'DisplayName', 'Gyr Z');
ax2 = gca; ax2.YLim = [-10 10];
title('Gyroscope (rad/s)'); ylabel('rad/s'); xlabel('Samples');
legend('show'); grid on;

%% 3. Storage
bufferSize = 10000;
allData    = zeros(bufferSize, 7); % [Time, AccX, AccY, AccZ, GyrX, GyrY, GyrZ]
i          = 0;
accMax     = 20;
gyrMax     = 10;
tic;

%% 4. Acquisition Loop
try
    while isvalid(fig)
        rawLine = readline(device);
        if ismissing(rawLine) || strlength(rawLine) == 0; continue; end

        dataVector = str2double(strsplit(rawLine, ','));

        if length(dataVector) == 6 && ~any(isnan(dataVector))
            i = i + 1;
            if i > size(allData, 1)
                allData = [allData; zeros(bufferSize, 7)];
            end

            allData(i, :) = [toc, dataVector(:)'];

            accX = dataVector(1); accY = dataVector(2); accZ = dataVector(3);
            gyrX = dataVector(4); gyrY = dataVector(5); gyrZ = dataVector(6);

            addpoints(hAccX, i, accX); addpoints(hAccY, i, accY); addpoints(hAccZ, i, accZ);
            addpoints(hGyrX, i, gyrX); addpoints(hGyrY, i, gyrY); addpoints(hGyrZ, i, gyrZ);

            peakAcc = max(abs([accX, accY, accZ]));
            peakGyr = max(abs([gyrX, gyrY, gyrZ]));
            if peakAcc > accMax * 0.9; accMax = peakAcc * 1.5; ax1.YLim = [-accMax accMax]; end
            if peakGyr > gyrMax * 0.9; gyrMax = peakGyr * 1.5; ax2.YLim = [-gyrMax gyrMax]; end

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
        filename = sprintf('imu_data_%s.csv', datestr(now, 'yyyy-mm-dd_HH-MM-SS'));
        header   = {'Time_s','AccX','AccY','AccZ','GyrX','GyrY','GyrZ'};
        fid      = fopen(filename, 'w');
        fprintf(fid, '%s,%s,%s,%s,%s,%s,%s\n', header{:});
        fclose(fid);
        dlmwrite(filename, allData, '-append', 'precision', '%.6f');
        fprintf('Saved %d samples to %s\n', i, filename);
    else
        fprintf('No data collected — nothing saved.\n');
    end
end