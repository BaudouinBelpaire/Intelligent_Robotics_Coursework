import csv
import matplotlib.pyplot as plt
from collections import deque


#rolling average function because raw data was unreadable
def rolling_average(data, window_size):
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

#read csv file with data
with open(csv_file_path, 'r') as file:
    reader = csv.reader(file)
    data = list(reader)

#manually specify column names, including the new columns "leftVel" and "rightVel"
columns = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7", "ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7", "gleft", "gmiddle", "gright", "leftVel", "rightVel"]
data_transposed = list(map(list, zip(*data)))

#plot all proximity sensors
fig1, ax1 = plt.subplots(figsize=(10, 4))
for i in range(8):
    values = [float(row[i]) if i < len(row) else None for row in data[:]]
    averages = rolling_average(values, window_size=5)
    ax1.plot(averages, label=columns[i], alpha=0.5)  
ax1.set_xlabel('Index')
ax1.set_ylabel('Values')
ax1.legend()
fig1.savefig('BBRdistance.png')
plt.show()

#plot the light sensor values
fig2, ax2 = plt.subplots(figsize=(10, 4))
for i in range(8, 16):  
    values = [float(row[i]) if i < len(row) else None for row in data[:]]
    averages = rolling_average(values, window_size=5)
    ax2.plot(averages, label=columns[i], alpha=0.5)
ax2.set_xlabel('Index')
ax2.set_ylabel('Values')
ax2.legend()
fig2.savefig('BBRlight.png')
plt.show()

#plot "gleft", "gmiddle", "gright", "leftVel", and "rightVel"
fig3, ax3 = plt.subplots(figsize=(10, 4))
for i in range(16, len(columns)):
    values = [float(row[i]) if i < len(row) else None for row in data[:]]
    averages = rolling_average(values, window_size=5) 
    ax3.plot(averages, label=columns[i], alpha=0.5)
ax3.set_xlabel('Index')
ax3.set_ylabel('Values')
ax3.legend()
fig3.savefig('BBRground_wheels.png')
plt.show()
