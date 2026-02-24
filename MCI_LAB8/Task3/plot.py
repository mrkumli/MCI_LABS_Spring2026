import serial
import matplotlib.pyplot as plt
from drawnow import drawnow

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyUSB0', 115200)

plt.ion()

tempValues = []
xValues = []
yValues = []
zValues = []
time_ms = []
cnt = 0

# === Plotting Function ===
def makeFig():
    plt.clf()
    plt.title('Gyroscope Temperature & Angular Velocity')
    plt.grid(True)
    plt.xlabel('Time (samples)')
    plt.ylabel('Value')
    plt.plot(time_ms, tempValues, 'm.-', label='Temp (LSB)')
    plt.plot(time_ms, xValues,    'r.-', label='X (dps)')
    plt.plot(time_ms, yValues,    'g.-', label='Y (dps)')
    plt.plot(time_ms, zValues,    'b.-', label='Z (dps)')
    plt.legend(loc='upper left')

# === Main Loop ===
while True:
    while sinWaveData.inWaiting() == 0:
        pass

    try:
        line = sinWaveData.readline().decode().strip()

        if line:
            print(repr(line))  # ADD THIS to see raw data
            values = line.split(',')
            values = line.split(',')

            if len(values) == 4:
                temp  = int(values[0])
                x_dps = float(values[1]) / 100.0
                y_dps = float(values[2]) / 100.0
                z_dps = float(values[3]) / 100.0

                tempValues.append(temp)
                xValues.append(x_dps)
                yValues.append(y_dps)
                zValues.append(z_dps)
                time_ms.append(cnt)
                cnt += 1

                drawnow(makeFig)
                plt.pause(0.0001)

                if len(tempValues) > 500:
                    tempValues.pop(0)
                    xValues.pop(0)
                    yValues.pop(0)
                    zValues.pop(0)
                    time_ms.pop(0)

    except ValueError:
        print("Skipping bad line:", line)
    except Exception as e:
        print("Error:", e)