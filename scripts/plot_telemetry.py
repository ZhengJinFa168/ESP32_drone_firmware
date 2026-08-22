#!/usr/bin/env python3
"""Real-time telemetry plotter – reads CSV lines from serial port."""
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

PORT = "/dev/ttyUSB0"
BAUD = 115200
HISTORY = 200

ser = serial.Serial(PORT, BAUD)
roll_buf, pitch_buf, yaw_buf = deque(maxlen=HISTORY), deque(maxlen=HISTORY), deque(maxlen=HISTORY)

fig, ax = plt.subplots()
lines = [ax.plot([], [], label=l)[0] for l in ("Roll", "Pitch", "Yaw")]
ax.legend(); ax.set_ylim(-180, 180)

def update(_):
    try:
        row = ser.readline().decode().strip().split(",")
        roll_buf.append(float(row[0]))
        pitch_buf.append(float(row[1]))
        yaw_buf.append(float(row[2]))
    except Exception:
        pass
    for line, buf in zip(lines, (roll_buf, pitch_buf, yaw_buf)):
        line.set_data(range(len(buf)), list(buf))
    ax.set_xlim(0, HISTORY)
    return lines

ani = animation.FuncAnimation(fig, update, interval=20)
plt.show()
