import serial
import matplotlib.pyplot as plt
from drawnow import drawnow

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyUSB0', 115200)

plt.ion()  # Enable interactive mode

adcValues = []
time_ms = []
cnt = 0  # Time index

# === Plotting Function ===
def makeFig():
    plt.clf()
    plt.title('Gyroscope Raw Temperature Register')
    plt.grid(True)
    plt.xlabel('Time (ms)')
    plt.ylabel('Raw Temperature')
    plt.ylim(0, 20)  # Adjusted for values like 12, 13, etc.
    plt.plot(time_ms, adcValues, 'r.-', label='Raw Temperature')
    plt.legend(loc='upper left')

# === Main Loop ===
while True:
    while sinWaveData.inWaiting() == 0:
        pass  # Wait for data

    try:
        line = sinWaveData.readline().decode().strip()

        if line:  # Make sure line is not empty
            raw = int(line)  # Single integer per line: 13, 13, 12, etc.

            adcValues.append(raw)
            time_ms.append(cnt)
            cnt += 1

            drawnow(makeFig)
            plt.pause(0.0001)

            # Keep only last 500 samples
            if len(adcValues) > 500:
                adcValues.pop(0)
                time_ms.pop(0)

    except ValueError:
        print("Skipping non-integer line:", line)
    except Exception as e:
        print("Error:", e)