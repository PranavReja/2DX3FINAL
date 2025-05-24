import serial
import open3d as o3d
import numpy as np
import time
import math

# Scan parameters
layer_count = 3                # Number of vertical layers
points_per_layer = 32          # Readings per 360° sweep
layer_distance = 100         # Vertical spacing between layers (units match sensor output)

elevation = 0  # Z position, tracked implicitly

# Setup serial connection (adjust COM port and baud rate as needed)
ser = serial.Serial('COM8', 115200, timeout=5)
print("Serial connection opened:", ser.name)

# Clear serial buffers
ser.reset_input_buffer()
ser.reset_output_buffer()

# Stores all scan points
all_points = []

# Wait for MCU start signal ('T')
print("Waiting for start signal from MCU...")
while True:
    trigger = ser.readline().decode().strip()
    if trigger == "T":
        print("Start signal received.")
        break

# Collect layered scans
for level in range(layer_count):
    current_layer = []
    collected = 0

    # Read distance-angle pairs until full layer is captured
    while collected < points_per_layer:
        incoming = ser.readline().decode().strip()
        if incoming != 'T':
            print("Received:", incoming)
        parsed = incoming.split(',')

        if len(parsed) == 2:
            angle_index = int(parsed[0])
            distance = float(parsed[1])
            angle_rad = angle_index * 11.25 * (math.pi / 180)

            # Alternate rotation direction on odd layers
            if level % 2 == 1:
                angle_rad = (2 * math.pi) - angle_rad

            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)
            z = level * layer_distance

            current_layer.append((x, y, z))
            collected += 1

    # Wait for MCU to send "COMPLETE" after stepper motor finishes sweep
    while True:
        status = ser.readline().decode().strip()
        if status == "COMPLETE":
            print(f"Layer {level} complete.")
            break

    # Append to master point list
    all_points.extend(current_layer)

# Close serial port after data collection
ser.close()

# Save points to XYZ file
with open("scanned_data.xyz", "w") as file:
    for x, y, z in all_points:
        file.write(f"{x:.2f} {y:.2f} {z:.2f}\n")

# Create Open3D point cloud object
points_array = np.array(all_points)
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points_array)

# Define line connections for visualization
connections = []

# Connect points within each circular layer
for level in range(layer_count):
    base = level * points_per_layer
    for j in range(points_per_layer - 1):
        connections.append([base + j, base + j + 1])
    connections.append([base + points_per_layer - 1, base])  # close loop

# Connect points between layers (zigzag wraparound)
for j in range(points_per_layer):
    for level in range(layer_count - 1):
        src = level * points_per_layer + j
        dest = (level + 1) * points_per_layer + (j if j == 0 else (points_per_layer - j))
        connections.append([src, dest])

# Build and color the line mesh
line_visual = o3d.geometry.LineSet()
line_visual.points = o3d.utility.Vector3dVector(points_array)
line_visual.lines = o3d.utility.Vector2iVector(connections)
line_visual.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in connections])  # red lines

# Display the final point cloud with connecting lines
o3d.visualization.draw_geometries([point_cloud, line_visual])
