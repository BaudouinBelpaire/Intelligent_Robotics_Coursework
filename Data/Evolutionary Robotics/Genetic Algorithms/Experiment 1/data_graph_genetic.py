import matplotlib.pyplot as plt
import pandas as pd

filename='genetic_data.csv'

# Read data from the file
data = pd.read_csv(filename)

generation = data.iloc[:, 0]
best = data.iloc[:, 1]
avg = data.iloc[:, 2]

df = pd.DataFrame({"Generation": generation, "Best": best, "Average": avg})

plt.plot(generation, best,label="Best")
plt.plot(generation, avg,label="Average")


plt.xlabel('Generation')
plt.ylabel('Fitness')
plt.legend()
plt.show()