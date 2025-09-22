# BAVT

There are two documents here:
1. LiDAR Top-Down Visualization with PID Lane-Following (CARLA)
2. LiDAR PLY to JPEG Converter (Open3D)

---

### **Usage of this document** ( This Documentation is primarily focused on Ubuntu 22.04 LTS )

##### Pre-Requisites:
We need **Python 3.8 to 3.10**. So please make sure you have that in your system. Download this version of python using: 

Mainly for Windows users:
``` 
https://www.python.org/downloads/release/python-3100/
https://www.youtube.com/watch?v=PLgTZqNsMdQ
```
###### Mainly for Linux users: ( I am using Ubuntu 22.04 LTS ) 
These instructions are to install python 3.10.12.
Update system packages and install necessary dependencies:
```
    sudo apt update
    sudo apt install software-properties-common -y
```
Add the deadsnakes PPA repository.


```
    sudo add-apt-repository ppa:deadsnakes/ppa
    sudo apt update
```
Press ENTER when prompted to confirm adding the repository. Install Python 3.10.


```
    sudo apt install python3.10 python3.10-venv python3.10-dev -y
```
This command installs Python 3.10, the virtual environment module (venv), and development headers (dev). Verify the installation.

```
    python3.10 --version
```
This command should output the installed Python 3.10 version, confirming the successful installation.



1. Must have a Carla 0.9.15 version simulator installed on your system.
   Install Carla 0.9.15 at:
```
https://github.com/carla-simulator/carla/releases
https://github.com/carla-simulator/carla/releases/tag/0.9.15
```
(Just click on the suitable link under 0.9.15 on the same page:
<img width="1150" height="724" alt="image" src="https://github.com/user-attachments/assets/7ef4b032-7443-4d99-a09b-71e96575f50b" />
)


Usually, Carla would be installed in Downloads folder/ directory, as a ZIP file.
Extract it in home directory / C: (C Drive) / wherever comfortable.

2. To run Carla on ubuntu, follow these steps:<br>
             - Open new terminal window.<br>
             - Make sure you are in home directory. Terminal window should look like this.<br>
                       ```
                       username@username-device_name:~$
                       ```<br>
             - Extract the carla .zip or .tar into Home directory or (C:) C drive.<br>
             - After extracting, rename the folder to ```carla_0.9.15```.<br>
             - Then execute the following commands in the same terminal/ prompt:<br>
                    ```
                    cd carla_0.9.15 (linux)
                    ```
                     ```
                    cd ../.. and cd carla_0.9.15 (windows)
                    ```<br>
             - To run carla on your system, execute this in the terminal:<br>
                    ```
                    ./CarlaUE4.sh (linux)
                    ./CarlaUE4.exe (windows)
                    ```<br>
                    OR<br>
                    (For windows) Simply open File Explorer and go to same folder and click on CarlaUE4.exe.<br>
   Congratulations! You are now ready to get started with the world of Carla.
   
   
   
4. Setup Carla using these resources:
   ```
   https://carla.readthedocs.io/en/latest/start_quickstart/
   https://www.youtube.com/watch?v=jIK9sanumuU
   https://www.youtube.com/watch?v=AaJekfFR1KQ
   https://carla.readthedocs.io/en/latest/tuto_first_steps/
   https://carla.readthedocs.io/en/latest/tutorials/
   ```
---

#### Instructions to use this file:

1. Download this repo or file lidar3D_and_radar.py / Clone this repo (run these commands on terminal for linux/ cmd for windows)
```
          git clone -b 2-D-LIDAR https://github.com/BOCK-AI/BAVT/
          cd BAVT
```

2. Install dependencies, if you do not already have them:
``` pip install carla opencv-python numpy ```

3. Run CARLA server (in a separate terminal):

``` ./CarlaUE4.sh   ``` on Linux
run ```CarlaUE4.exe.``` on Windows

4. Run the python file in terminal/ prompt:
``` python3 rgb_cam.py```

5. What you’ll see:
   A Tesla Model 3 driving in CARLA’s world, following the road.
   A live OpenCV window open up and show 2d lidar feed.
   A live camera window showing RGB feed from the vehicle’s front camera.
If some error occurs, it may be due to incorrect dependencies, or other things.

7. To Exit: CTRL + C stops execution. OR - Press q in the OpenCV window to stop. All actors are destroyed automatically (vehicle + camera). 

This script integrates **CARLA simulator** with **Open3D** and **OpenCV** to:

* Spawns a **Tesla Model 3** in the CARLA simulator.
* Attaches a **64-channel LiDAR sensor** to the vehicle.
* Converts LiDAR point clouds into **2D top-down images**.
* Implements a **PID controller** to follow driving waypoints automatically.
* Displays a **live LiDAR top-down visualization** using OpenCV.

Press **`q`** in the OpenCV window to safely exit the simulation.

---

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

## Configuration

* **LiDAR Settings**:

  * Channels: `64`
  * Range: `50 m`
  * Horizontal FOV: `360°`
  * Vertical FOV: `-10° to 10°`
  * Rotation frequency: `20 Hz`
  * Points per second: `200,000`

* **Top-Down Visualization**:

  * Image size: `500 × 500 px`
  * Scale: `5 pixels per meter`
  * Center of image: vehicle location

* 

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
