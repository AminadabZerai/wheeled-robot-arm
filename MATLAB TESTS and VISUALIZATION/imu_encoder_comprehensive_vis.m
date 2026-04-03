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

subplot(4,1,1);
hAccX = animatedline('Color', 'r', 'DisplayName', 'Acc X');
hAccY = animatedline('Color', 'g', 'DisplayName', 'Acc Y');
hAccZ = animatedline('Color', 'b', 'DisplayName', 'Acc Z');
ax1 = gca;
ax1.YLim = [-20 20];
title('Accelerometer (m/s^2)');
ylabel('m/s^2');
legend('show');
grid on;

subplot(4,1,2);
hGyrX = animatedline('Color', 'm', 'DisplayName', 'Gyr X');
hGyrY = animatedline('Color', 'c', 'DisplayName', 'Gyr Y');
hGyrZ = animatedline('Color', [1 0.5 0], 'DisplayName', 'Gyr Z');
ax2 = gca;
ax2.YLim = [-10 10];
title('Gyroscope (rad/s)');
ylabel('rad/s');
legend('show');
grid on;

subplot(4,1,3);
hEncRad = animatedline('Color', 'y', 'DisplayName', 'Angle (rad)');
ax3 = gca;
ax3.YLim = [-10 10];
title('Encoder LF - Angle');
ylabel('rad');
legend('show');
grid on;

subplot(4,1,4);
hEncVel = animatedline('Color', 'w', 'DisplayName', 'Velocity (rad/s)');
ax4 = gca;
ax4.YLim = [-50 50];
title('Encoder LF - Angular Velocity');
ylabel('rad/s');
xlabel('Samples');
legend('show');
grid on;

% 3. Pre-allocate storage
% Columns: [Time, AccX, AccY, AccZ, GyrX, GyrY, GyrZ, EncRad, EncVel]
bufferSize = 10000;
allData = zeros(bufferSize, 9);

% 4. Data Acquisition Loop
i = 0;
accMax  = 20;
gyrMax  = 10;
radMax  = 10;
velMax  = 50;
tic;

while isvalid(fig)
    try
        rawLine = readline(device);
        if ismissing(rawLine) || strlength(rawLine) == 0
            continue;
        end

        dataVector = str2double(strsplit(rawLine, ','));

        % Expect exactly 8 values from Arduino
        if length(dataVector) == 8 && ~any(isnan(dataVector))
            i = i + 1;

            % Grow buffer if needed
            if i > size(allData, 1)
                allData = [allData; zeros(bufferSize, 9)];
            end

            % Store with timestamp
            allData(i, :) = [toc, dataVector(:)'];

            % Unpack for clarity
            accX = dataVector(1); accY = dataVector(2); accZ = dataVector(3);
            gyrX = dataVector(4); gyrY = dataVector(5); gyrZ = dataVector(6);
            encRad = dataVector(7);
            encVel = dataVector(8);

            % Plot
            addpoints(hAccX, i, accX);
            addpoints(hAccY, i, accY);
            addpoints(hAccZ, i, accZ);

            addpoints(hGyrX, i, gyrX);
            addpoints(hGyrY, i, gyrY);
            addpoints(hGyrZ, i, gyrZ);

            addpoints(hEncRad, i, encRad);
            addpoints(hEncVel, i, encVel);

            % Auto-scale Y axes
            peakAcc = max(abs([accX, accY, accZ]));
            peakGyr = max(abs([gyrX, gyrY, gyrZ]));
            peakRad = abs(encRad);
            peakVel = abs(encVel);

            if peakAcc > accMax * 0.9
                accMax = peakAcc * 1.5;
                ax1.YLim = [-accMax accMax];
            end
            if peakGyr > gyrMax * 0.9
                gyrMax = peakGyr * 1.5;
                ax2.YLim = [-gyrMax gyrMax];
            end
            if peakRad > radMax * 0.9
                radMax = peakRad * 1.5;
                ax3.YLim = [-radMax radMax];
            end
            if peakVel > velMax * 0.9
                velMax = peakVel * 1.5;
                ax4.YLim = [-velMax velMax];
            end

            % Sliding window on X axis
            if i > 200
                ax1.XLim = [i-200 i];
                ax2.XLim = [i-200 i];
                ax3.XLim = [i-200 i];
                ax4.XLim = [i-200 i];
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
allData = allData(1:i, :);
filename = sprintf('imu_data_%s.csv', datestr(now, 'yyyy-mm-dd_HH-MM-SS'));

header = {'Time_s','AccX','AccY','AccZ','GyrX','GyrY','GyrZ','EncRad','EncVel'};
fid = fopen(filename, 'w');
fprintf(fid, '%s,%s,%s,%s,%s,%s,%s,%s,%s\n', header{:});
fclose(fid);

dlmwrite(filename, allData, '-append', 'precision', '%.6f');
fprintf('Streaming stopped. %d samples saved to %s\n', i, filename);