import math
import sys
import numpy as np
from matplotlib import pyplot as plt

# Get the file name from user
file_path = input("Enter file name: ")
# Open the file
f = open(file_path, "r")
# Create a list for the inputs
dots = []
# Get the input line by line
while True:
    # Read the line if it is empty then terminate the loop
    line = f.readline()
    if not line:
        break
    # split the line
    id, x, y = line.split()
    # add x and y coordinates to dots list
    dots.append([int(x), int(y)])
f.close()

# compute length of the input beforehand for optimization
length_of_input = len(dots)

# calculate the mean
mean_x = 0
mean_y = 0
for i in range(length_of_input):
    mean_x += dots[i][0]
    mean_y += dots[i][1]
mean_y /= length_of_input
mean_x /= length_of_input

# calculate the distances for every point and mean
distances_to_mean = []
for i in range(length_of_input):
    distances_to_mean.append(math.sqrt((dots[i][0] - mean_x) ** 2 + (dots[i][1] - mean_y) ** 2))

# create lists for,
# - visited, visited nodes
# - not_visited, not yet visited nodes
# - selected_indexes = indexes that are selected consequently
visited = []
not_visited = [i for i in range(length_of_input)]
selected_indexes = []
min_distances = sys.maxsize

# search the nearest element for ceil(n/2) times
for i in range(math.ceil(length_of_input / 2)):
    index_ = 0
    min_dist_to_mean = sys.maxsize
    for j in not_visited:
        if distances_to_mean[j] < min_dist_to_mean:
            index_ = j
            min_dist_to_mean = distances_to_mean[j]
    selected_indexes.append(index_)
    visited.append(dots[index_])
    not_visited.remove(index_)

# print the result to an output file

fout = open("half-input.txt", "w")

fout.write(str(len(visited)) + "\n")
for i in range(len(visited)):
    fout.write(str(selected_indexes[i]) + " " + str(visited[i][0]) + " " + str(visited[i][1]) + "\n")
fout.close()
