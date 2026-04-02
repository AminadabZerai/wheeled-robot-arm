% 1. Connection Setup
port = "COM3";
baudrate = 115200;
device = serialport(port, baudrate);
configureTerminator(device, "LF");
device.Timeout = 30;

pause(1);
flush(device);

fprintf('Waiting for Arduino to finish calibration...\n');
firstLine = readline(device);
fprintf('Data stream detected. Starting plot.\n');

% 2. Visualization Setup
fig = figure('Name', 'Robot IMU Live Stream', 'NumberTitle', 'off');

subplot(3,1,1);
hAccX = animatedline('Color', 'r', 'DisplayName', 'Acc X');
hAccY = animatedline('Color', 'g', 'DisplayName', 'Acc Y');
hAccZ = animatedline('Color', 'b', 'DisplayName', 'Acc Z');
ax1 = gca;
ax1.YLim = [-20000 20000];
title('Accelerometer Data (Raw)');
ylabel('LSB');
legend('show');
grid on;

subplot(3,1,2);
hGyrX = animatedline('Color', 'm', 'DisplayName', 'Gyr X');
hGyrY = animatedline('Color', 'c', 'DisplayName', 'Gyr Y');
hGyrZ = animatedline('Color', 'w', 'DisplayName', 'Gyr Z');
ax2 = gca;
ax2.YLim = [-5000 5000];
title('Gyroscope Data (Raw)');
ylabel('LSB');
xlabel('Samples');
legend('show');
grid on;

subplot(3,1,3);
encLF = animatedline('Color', 'y', 'DisplayName', 'LF Encoder');
ax3 = gca;
ax3.YLim = [0 5000];
title('Encoder LF Data (Raw)');
ylabel('LSB');
xlabel('Samples');
legend('show');
grid on;

% 3. Pre-allocate storage (grows if needed)
bufferSize = 10000;
allData = zeros(bufferSize, 7); % [timestamp, accX, accY, accZ, gyrX, gyrY, gyrZ, encLF]

% 4. Data Acquisition Loop
i = 0;
accMax = 20000;
gyrMax = 5000;
encLFMax = 5000;
tic; % start timer for timestamps

while isvalid(fig)
    try
        rawLine = readline(device);
        if ismissing(rawLine) || strlength(rawLine) == 0
            continue;
        end

        dataVector = str2double(strsplit(rawLine, ','));

        if length(dataVector) == 7 && ~any(isnan(dataVector))
            i = i + 1;

            % Grow buffer if needed
            if i > size(allData, 1)
                allData = [allData; zeros(bufferSize, 8)];
            end

            % Store with timestamp
            allData(i, :) = [toc, dataVector];

            % Plot
            addpoints(hAccX, i, dataVector(1));
            addpoints(hAccY, i, dataVector(2));
            addpoints(hAccZ, i, dataVector(3));

            addpoints(hGyrX, i, dataVector(4));
            addpoints(hGyrY, i, dataVector(5));
            addpoints(hGyrZ, i, dataVector(6));

            addpoints(encLF, i, dataVector(7));

            % Auto-scale
            peakAcc = max(abs(dataVector(1:3)));
            peakGyr = max(abs(dataVector(4:6)));
            peakEncLF = max(abs(dataVector(7)));
            if peakAcc > accMax * 0.9
                accMax = peakAcc * 1.5;
                ax1.YLim = [-accMax accMax];
            end
            if peakGyr > gyrMax * 0.9
                gyrMax = peakGyr * 1.5;
                ax2.YLim = [-gyrMax gyrMax];
            end
            if peakEncLF > encLFMax * 0.9
                encLFMax = peakEncLF * 1.5;
                ax2.YLim = [0 encLFMax];
            end

            if i > 200
                ax1.XLim = [i-200 i];
                ax2.XLim = [i-200 i];
            end

            drawnow limitrate;
        end
    catch ME
        fprintf('Error: %s\n', ME.message);
        break;
    end
end

clear device;

% 5. Save to CSV
allData = allData(1:i, :); % trim unused rows
filename = sprintf('imu_data_%s.csv', datestr(now, 'yyyy-mm-dd_HH-MM-SS'));

header = {'Time_s', 'AccX', 'AccY', 'AccZ', 'GyrX', 'GyrY', 'GyrZ', 'EncLF'};
fid = fopen(filename, 'w');
fprintf(fid, '%s,%s,%s,%s,%s,%s,%s,%s\n', header{:});
fclose(fid);

dlmwrite(filename, allData, '-append', 'precision', '%.6f');

fprintf('Streaming stopped. %d samples saved to %s\n', i, filename);