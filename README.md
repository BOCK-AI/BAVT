# BAVT

There are two documents here:
1. LiDAR Top-Down Visualization with PID Lane-Following (CARLA)
2. LiDAR PLY to JPEG Converter (Open3D)

# LiDAR Top-Down Visualization with PID Lane-Following (CARLA)

## Overview

This script connects to a running [CARLA simulator](https://carla.org/) instance, spawns a **Tesla Model 3** vehicle, attaches a **LiDAR sensor**, and:

* Converts LiDAR point cloud data into a **2D top-down image** (radar-like view).
* Uses a **PID controller** to keep the vehicle aligned with the road waypoints.
* Updates the **spectator camera** to smoothly follow the vehicle.
* Displays the LiDAR top-down map in an OpenCV window.

Press **`q`** to quit safely.

---

## Requirements

* **CARLA Simulator 0.9.15** running on your machine (default port: 2000).
* **Python 3.8+**
* Python dependencies:

  ```bash
  pip install carla opencv-python numpy
  ```

---

## Running the Script

1. Start the CARLA server:

   ```bash
   ./CarlaUE4.sh -quality-level=Epic -world-port=2000
   ```

   (Windows users: run `CarlaUE4.exe` instead.)

2. Run the script:

   ```bash
   python lidar_pid.py
   ```

---

## Intended Behavior

* The world switches to **synchronous mode** (fixed 20 FPS).
* A **Tesla Model 3** spawns at the first map spawn point.
* A **64-channel LiDAR** is attached to the vehicle.
* The car drives itself using a simple **PID lane-following controller**.
* The **spectator camera** follows the vehicle with a fixed offset.
* A live **OpenCV window** appears:

  * Grid lines every 10 meters.
  * Vehicle at the center.
  * LiDAR points shown as **green dots** (brightness = intensity).

---

## Controls

* **`q`** → Quit simulation & cleanup.
* **Ctrl+C** → Emergency stop (also cleans up).

---

## Code Structure

### Functions & Classes

* **`process_lidar_frames(data)`**
  Converts LiDAR raw data → top-down 2D image. Places frames into a queue for display.

* **`PIDController(Kp, Ki, Kd)`**
  Simple PID controller for steering correction.

  * `run_step(error, dt)` → returns control signal.

* **`main()`**
  Orchestrates the workflow:

  * Connects to CARLA.
  * Spawns vehicle, LiDAR, and spectator.
  * Applies PID-based controls each tick.
  * Displays LiDAR visualization.
  * Cleans up actors on exit.

---

## Cleanup & Exit

When quitting:

* LiDAR sensor stops streaming.
* Vehicle and sensors are destroyed.
* World reverts to **asynchronous mode**.
* OpenCV windows are closed.

---

## Customization

* **Visualization**

  * `IMG_SIZE`: size of the top-down image (default `500x500`).
  * `SCALE`: pixels per meter (default `5.0`).
  * `MAX_RANGE`: LiDAR maximum range (default `50 m`).

* **LiDAR Sensor**

  * `channels`, `range`, `points_per_second`, FOV attributes can be tuned in the script.

* **Controller**

  * Adjust `Kp`, `Ki`, `Kd` in `PIDController` to change steering responsiveness.

---

## Example Output

* Window: `"LiDAR Top-Down View"`
* A green radar-like visualization of obstacles around the car.
* The Tesla drives forward and stays aligned with the road.

---

***

# LiDAR PLY to JPEG Converter (Open3D)

## Overview

This script converts a series of **LiDAR point cloud files (`.ply`)** into **JPEG images** using [Open3D](http://www.open3d.org/).

* Reads `.ply` point clouds from an input folder.
* Renders each frame in an **off-screen Open3D visualizer**.
* Captures the rendered image and saves it as a `.jpg`.

---

## Requirements

* **Python 3.8+**
* Dependencies:

  ```bash
  pip install open3d pillow numpy
  ```
* A folder containing **LiDAR `.ply` files** (e.g., exported from CARLA).

---

## Input/Output

* **Input folder (`lidar_path`)**
  Path containing `.ply` files (LiDAR point clouds).
  Example:

  ```
  /home/.../lidar_output/
  ```

* **Output folder (`folder`)**
  Path where converted `.jpg` images will be stored.
  Example:

  ```
  /home/.../lidar_faster/jpg/
  ```

The script creates the output folder automatically if it does not exist.

---

## Running the Script

```bash
python ply_to_jpg.py
```

Replace `lidar_path` and `folder` variables in the script with your own directories.

---

## Workflow

1. Initializes a single **Open3D Visualizer** (off-screen window).
2. Iterates through all `.ply` files in the input folder.
3. For each file:

   * Loads the point cloud (`o3d.io.read_point_cloud`).
   * Clears the old geometry and adds the new one.
   * Renders the scene off-screen.
   * Captures a float buffer image, converts to `uint8`, and saves as `.jpg`.
4. Closes the visualizer after processing all files.

---

## Example Output

Input:

```
lidar_output/
 ├─ frame_0001.ply
 ├─ frame_0002.ply
 ├─ frame_0003.ply
```

Output:

```
jpg/
 ├─ frame_0001.jpg
 ├─ frame_0002.jpg
 ├─ frame_0003.jpg
```

Each `.jpg` is a **2D rendered view** of the point cloud.

---

## Notes

* Rendering is done **headless** (`visible=False`), so no window will pop up.
* Image quality depends on Open3D’s default rendering (no custom camera set).
* To change image resolution, adjust Open3D’s visualizer settings before capture.
* You can modify file formats (e.g., save as `.png`) by changing the `Image.save()` call.

---

## Cleanup

* The script automatically destroys the visualizer with `vis.destroy_window()` after finishing.

---
