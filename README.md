# BAVT

# **Documentation for CARLA Sensor Visualization & Vehicle Control**

### **Usage of this document** ( This Documentation is primarily focused on Linux )
1. Download this repo or file lidar3D_and_radar.py / Clone this repo (run these commands on terminal)
```
          git clone https://github.com/BOCK-AI/BAVT/
          cd BAVT
```

2. Install dependencies, if you do not already have them:
``` pip install carla open3d opencv-python pillow numpy matplotlib ```

3. Run CARLA server (in a separate terminal):

``` ./CarlaUE4.sh   ``` on Linux.

Run ```CarlaUE4.exe.``` on Windows.

4. Run the python file in terminal/ prompt:
``` python3 lidar3D_and_radar.py```

5. Output is expected to be seen. If some error occurs, it may be due to incorrect dependencies, or other things.

This script integrates **CARLA simulator** with **Open3D** and **OpenCV** to:

* Spawn a Tesla Model 3 vehicle.
* Attach sensors (LiDAR, RADAR, RGB Camera).
* Stream data in **synchronous mode**.
* Visualize LiDAR and RADAR as Open3D point clouds.
* Display the RGB camera feed using OpenCV.
* Control the vehicle with a simple **PID-based lane-following controller**.

---

## **Global Constants**

* `VIRIDIS`: Colormap (plasma, used for LiDAR intensity).
* `VID_RANGE`: Normalized range for colormap interpolation.
* `COOL`: Colormap (winter, used for RADAR velocity coloring).
* `COOL_RANGE`: Normalized range for RADAR colormap.
* `IM_WIDTH`, `IM_HEIGHT`: Image size for RGB camera.

---

## **Functions**

### 1. `add_open3d_axis(vis)`

Adds a **coordinate axis** (X = red, Y = green, Z = blue) to the Open3D visualization window.

**Args**:

* `vis` (`o3d.visualization.Visualizer`): Open3D visualizer instance.

---

### 2. `lidar_callback(point_cloud, point_list)`

Processes LiDAR sensor data and updates the Open3D point cloud.

**Steps**:

* Reads raw LiDAR buffer(x, y, z, intensity).
* Normalizes and color-maps intensity values using `VIRIDIS`.
* Updates `point_list` (Open3D point cloud) with new points and colors.

**Args**:

* `point_cloud` (`carla.LidarMeasurement`): CARLA LiDAR data.
* `point_list` (`o3d.geometry.PointCloud`): Open3D point cloud container.

---

### 3. `radar_callback(meas, point_list)`

Processes RADAR sensor detections and updates the Open3D point cloud.

**Steps**:

* Converts CARLA radar detection data (depth, azimuth, altitude, velocity) into 3D points.
* Normalizes velocity magnitude and applies `COOL` colormap.
* Updates `point_list` with points and colors.
* Mirrors X-axis to fit Open3D coordinate convention.

**Args**:

* `meas` (`carla.RadarMeasurement`): List of RADAR detections.
* `point_list` (`o3d.geometry.PointCloud`): Open3D point cloud container.

---

### 4. `camera_callback(image, data_dict)`

Processes RGB camera frames.

**Steps**:

* Converts CARLA image buffer into a `(height, width, 4)` NumPy array (BGRA).
* Stores result inside a shared dictionary (`data_dict`).

**Args**:

* `image` (`carla.Image`): Raw CARLA image.
* `data_dict` (`dict`): Shared dictionary holding the current frame.

---

### 5. `class PIDController`

A basic **PID controller** implementation for steering correction.

#### `__init__(self, Kp, Ki, Kd)`

Initializes PID gains.

* `Kp`: Proportional gain.
* `Ki`: Integral gain.
* `Kd`: Derivative gain.

#### `run_step(self, error, dt)`

Computes PID output.

* `error`: Current control error.
* `dt`: Time step.

**Returns**: Steering correction value.

---

### 6. `main()`

Main entry point of the script.

**Responsibilities**:

1. Connects to CARLA (`localhost:2000`).
2. Configures **synchronous simulation mode**.
3. Spawns:

   * Vehicle (Tesla Model 3).
   * LiDAR, RADAR, and RGB Camera sensors.
4. Sets up **Open3D visualizer** and **OpenCV window**.
5. Registers sensor callbacks:

   * `lidar_callback` → updates LiDAR point cloud.
   * `radar_callback` → updates RADAR point cloud.
   * `camera_callback` → updates RGB frames.
6. Implements **spectator camera tracking** (follows behind the car).
7. Implements a **PID lane-following controller**:

   * Gets current waypoint.
   * Computes cross-track error (via vector cross product).
   * Adjusts steering accordingly.
8. Runs a simulation loop:

   * Calls `world.tick()` for sync stepping.
   * Updates Open3D visualizer with sensor data.
   * Displays RGB camera in OpenCV.
   * Applies throttle & steering control to vehicle.
   * Breaks loop on **`q` key press**.
9. Cleans up: Stops/destroys actors, resets CARLA settings, closes windows.

---

## **Key Control Flow**

1. **Initialization**:

   * Create CARLA client → set sync mode → spawn vehicle + sensors.

2. **Visualization**:

   * LiDAR → Open3D point cloud with `VIRIDIS`.
   * RADAR → Open3D point cloud with `COOL`.
   * Camera → OpenCV window.

3. **Control Loop**:

   * Advance sim (`world.tick()`).
   * Update visualizers.
   * Compute steering via PID controller.
   * Apply vehicle throttle & steering.

4. **Shutdown**:

   * Stop sensors.
   * Destroy actors.
   * Reset CARLA to async mode.
   * Close OpenCV + Open3D windows.

---

## **Usage**

Run the script with CARLA server running (port 2000):

```bash
python3 radar.py
or
./radar.py
```

* Press **`q`** in the OpenCV window to exit.
* OR
* Press Ctrl + C to stop the program.

---

## **Dependencies**

* `carla` (Python API from CARLA simulator).
* `open3d` (3D visualization).
* `cv2` (OpenCV for RGB display).
* `numpy`, `matplotlib`, `math`, `time`.

---
