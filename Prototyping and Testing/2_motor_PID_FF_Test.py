import matplotlib.pyplot as plt
import serial
from serial.tools.list_ports import comports
import time
import csv
import os
import threading

# Use a style that looks good on most screens
plt.style.use('dark_background')

# --- PLOT STYLE SELECTION ---
plot_choice = ''
while plot_choice not in ['1', '2']:
    plot_choice = input("Select plot style:\n  1: Subplots (Separate graphs)\n  2: Superimposed (Combined graph)\nEnter choice (1 or 2): ")

# --- SERIAL PORT SETUP ---
print("\nAvailable serial ports:")
for portItem in comports():
    print(portItem)

# --- USER CONFIGURATION ---
SERIAL_PORT = 'COM3'  # <-- IMPORTANT: Change this to your Arduino's COM port
BAUD_RATE = 115200
CSV_FILENAME = "SI_PID_Test_2_Motor_1.csv" # <-- Change this name before each test
# --- END USER CONFIGURATION ---

try:
    serialArduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"\nConnected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
    exit()

time.sleep(2)

# --- BACKGROUND THREAD FOR USER COMMANDS ---
def serial_input_thread():
    """Handles sending user commands to the Arduino in a separate thread."""
    while True:
        try:
            user_input = input("Enter command (e.g., f,150 or r,300 or f,0): ")
            if user_input:
                serialArduino.write((user_input.strip() + '\n').encode())
        except (KeyboardInterrupt, EOFError):
            print("\nInput thread terminated.")
            break

# --- DATA STORAGE ---
time_series = []
setpoint_speed_series = []
measured_speed_LF_series = []
measured_speed_RF_series = []

# --- CSV FILE SETUP ---
# NOTE: This is a hardcoded file path. Make sure it exists on your computer.
file_path = "C:/Users/Amine/Desktop/Project_Make_it_Or_Break_It/wheeled-robot-arm/Prototyping and Testing/DATA_LOG/"
CSV_PATH = os.path.join(file_path, CSV_FILENAME)
print(f"Logging data to: {CSV_PATH}")

# Ensure the directory exists before trying to create the file
os.makedirs(os.path.dirname(CSV_PATH), exist_ok=True)

csv_file = open(CSV_PATH, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "Time_s", "Setpoint_dps",
    "Measured_LF_dps", "PWM_LF", "Delta_LF_deg",
    "Measured_RF_dps", "PWM_RF", "Delta_RF_deg",
    "Error_LF_dps"
])

# --- LIVE PLOT SETUP (Conditional) ---
plt.ion()
if plot_choice == '1':
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 8))
    fig.suptitle('PID Tracking Performance (Subplots)', fontsize=16)
else: # plot_choice == '2'
    fig, ax = plt.subplots(1, 1, figsize=(12, 7))
    fig.suptitle('PID Tracking Performance (Superimposed)', fontsize=16)
plt.show()

# --- MAIN LOOP ---
threading.Thread(target=serial_input_thread, daemon=True).start()
try:
    while True:
        if serialArduino.in_waiting > 0:
            try:
                data_str = serialArduino.readline().decode('utf-8').strip()
                
                data_fields = data_str.split('\t')
                if len(data_fields) == 9:
                    t_sec, setpoint, measured_LF, _, _, measured_RF, _, _, _ = map(float, data_fields)

                    csv_writer.writerow(data_fields)
                    csv_file.flush()

                    time_series.append(t_sec)
                    setpoint_speed_series.append(setpoint)
                    measured_speed_LF_series.append(measured_LF)
                    measured_speed_RF_series.append(measured_RF)

                    # --- CONDITIONAL PLOTTING LOGIC ---
                    if plot_choice == '1': # Subplot Mode
                        ax1.clear()
                        ax1.set_title('Left Front Motor')
                        ax1.set_ylabel('Speed (deg/s)')
                        ax1.grid(True, linestyle='--', alpha=0.6)
                        ax1.plot(time_series, measured_speed_LF_series, label="Measured LF", color="cyan")
                        ax1.plot(time_series, setpoint_speed_series, label="Setpoint", color="yellow", linestyle="--")
                        ax1.legend(loc="upper left")

                        ax2.clear()
                        ax2.set_title('Right Front Motor')
                        ax2.set_xlabel('Time (s)')
                        ax2.set_ylabel('Speed (deg/s)')
                        ax2.grid(True, linestyle='--', alpha=0.6)
                        ax2.plot(time_series, measured_speed_RF_series, label="Measured RF", color="lime")
                        ax2.plot(time_series, setpoint_speed_series, label="Setpoint", color="yellow", linestyle="--")
                        ax2.legend(loc="upper left")
                    
                    else: # Superimposed Mode
                        ax.clear()
                        ax.set_title('Motor Speed vs. Setpoint')
                        ax.set_xlabel('Time (s)')
                        ax.set_ylabel('Speed (deg/s)')
                        ax.grid(True, linestyle='--', alpha=0.6)
                        ax.plot(time_series, setpoint_speed_series, label="Setpoint", color="yellow", linestyle="--", linewidth=2)
                        ax.plot(time_series, measured_speed_LF_series, label="Measured LF", color="cyan")
                        ax.plot(time_series, measured_speed_RF_series, label="Measured RF", color="lime")
                        ax.legend(loc="upper left")

                    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
                    plt.pause(0.001)

            except (ValueError, IndexError) as e:
                print(f"Skipping line due to parsing error: {e}, Line: '{data_str}'")
            except Exception as e:
                print(f"An unexpected error occurred: {e}")

except KeyboardInterrupt:
    print("\nExiting and saving final plot...")
    serialArduino.close()
    csv_file.close()
    plt.ioff()
    plt.show()