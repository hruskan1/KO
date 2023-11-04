import sys
import matplotlib.pyplot as plt

# Read data points -------------------------------------------------------
if len(sys.argv) < 2:
    raise Exception("The first argument should contain a path to the data file.")

with open(sys.argv[1],"r") as f:
    n = int(f.readline().split()[0])  # the number of data points
    x = [0]*n
    y = [0]*n
    for i in range(n):
        x[i], y[i] = [int(j) for j in f.readline().split()]

# TODO: Construct the graph, and call Bellman-Ford to solve it
# Fill these:
x_new = [elem + 20 for elem in x]  # dummy data; just shifting the points
y_new = [elem - 20 for elem in y]  # to show how the visualization works

# Plot the graph
plt.plot(x,y)
plt.plot(x_new,y_new, 'r--')
plt.show()
