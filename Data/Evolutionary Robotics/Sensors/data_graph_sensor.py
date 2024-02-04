import csv
import matplotlib.pyplot as plt
from collections import deque

def rolling_average(data, window_size):
    """Calculate the rolling average of a list."""
    averages = []
    window = deque(maxlen=window_size)
    total = 0

    for value in data:
        window.append(value)
        total += value

        if len(window) == window_size:
            averages.append(total / window_size)
            total -= window[0]

    return averages

csv_file_path = 'sensor_data.csv'

#read the CSV file and extract the values
with open(csv_file_path, 'r') as file:
    reader = csv.reader(file)
    data = list(reader)

#manually specify column names, including the new columns "leftVel" and "rightVel"
columns = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7", "gleft", "gmiddle", "gright", "leftVel", "rightVel"]

#transpose the data
data_transposed = list(map(list, zip(*data)))

#plot the first five values in the first subplot
fig1, ax1 = plt.subplots(figsize=(10, 4))
for i in range(6):  #include "gleft", "gmiddle", "gright", "leftVel", and "rightVel"
    values = [float(row[i]) if i < len(row) else None for row in data[:1000]]
    averages = rolling_average(values, window_size=5)  #adjust the window size as needed
    ax1.plot(averages, label=columns[i], alpha=0.5)  #adjust alpha here
ax1.set_xlabel('Index')
ax1.set_ylabel('Values')
ax1.legend()
fig1.savefig('ERdistance.png')
plt.show()

#plot the middle values in the second subplot
fig2, ax2 = plt.subplots(figsize=(10, 4))
for i in range(6, 14):  #adjust range for "middle" values
    values = [float(row[i]) if i < len(row) else None for row in data[:1000]]
    averages = rolling_average(values, window_size=5)  #adjust the window size as needed
    ax2.plot(averages, label=columns[i], alpha=0.5)  #adjust alpha here
ax2.set_xlabel('Index')
ax2.set_ylabel('Values')
ax2.legend()
fig2.savefig('ERlight.png')
plt.show()

#plot the last ten values in the third subplot
fig3, ax3 = plt.subplots(figsize=(10, 4))
for i in range(14, len(columns)):
    values = [float(row[i]) if i < len(row) else None for row in data[:1000]]
    averages = rolling_average(values, window_size=5)  #adjust the window size as needed
    ax3.plot(averages, label=columns[i], alpha=0.5)  #adjust alpha here
ax3.set_xlabel('Index')
ax3.set_ylabel('Values')
ax3.legend()
fig3.savefig('ERground_wheels.png')
plt.show()
