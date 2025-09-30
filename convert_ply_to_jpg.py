import open3d as o3d
import os
import numpy as np
from PIL import Image

# Input/Output paths
lidar_path = "/carla_0.9.15/PythonAPI/examples/BAVT/"  # The .ply files are stored in this path variable.

folder = "/carla_0.9.15/PythonAPI/examples/BAVT/" # This is where we save our screenshots (.jpg).
os.makedirs(folder, exist_ok=True)

# Create one Visualizer instance
vis = o3d.visualization.Visualizer()
vis.create_window(visible=False)

for filename in os.listdir(lidar_path):
    if filename.endswith(".ply"):
        filepath = os.path.join(lidar_path, filename)
        print(f"Processing: {filename}")
        
        pcd = o3d.io.read_point_cloud(filepath)

        vis.clear_geometries()   # Clear previous geometry
        vis.add_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        
        out_jpg = os.path.join(folder, filename.replace(".ply", ".jpg"))
        
        img_array = np.asarray(vis.capture_screen_float_buffer())
        img = Image.fromarray((img_array * 255).astype(np.uint8))
        img.save(out_jpg, "JPEG")
        
vis.destroy_window()
