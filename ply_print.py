import open3d as o3d
import numpy as np

# Load the PLY back
pcd = o3d.io.read_point_cloud("lidar_output.ply")

# Convert to numpy array
points = np.asarray(pcd.points)   # (N,3) -> x,y,z
colors = np.asarray(pcd.colors)   # (N,3) -> r,g,b in [0,1]

# Print first 10 points for inspection
for i in range(10):
    print(f"Point {i}: x={points[i,0]:.3f}, y={points[i,1]:.3f}, z={points[i,2]:.3f}, "
          f"R={colors[i,0]:.2f}, G={colors[i,1]:.2f}, B={colors[i,2]:.2f}")
