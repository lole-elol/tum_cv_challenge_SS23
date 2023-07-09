#!/usr/bin/env python3

import numpy as np

# Specify the path to your .txt file
file_path = 'test/point_cloud/delivery_points3D.txt'

# Read the file and extract the lines
with open(file_path, 'r') as file:
    lines = file.readlines()

# Remove comment lines starting with '#'
lines = [line for line in lines if not line.startswith('#')]

# Split the lines and extract the numeric data
data = []
for line in lines:
    line_data = line.strip().split(' ')
    numeric_data = [float(value) for value in line_data[:8]]
    #track_data = [int(value) for value in line_data[9:]]
    data.append(numeric_data)

# Convert the data to a numpy array
matrix = np.array(data)

# Display the imported matrix
np.savetxt("test/point_cloud/delivery_points3D_massaged.txt", matrix)