# BAVT

### **Usage of this document** ( This Documentation is primarily focused on Linux )

##### Pre-Requisites:
1. Must have a Carla 0.9.15 version simulator installed on your system.
   Install Carla 0.9.15 at:
```
https://github.com/carla-simulator/carla/releases
```
(Just click on the suitable link under 0.9.15 on the same page:
<img width="1150" height="724" alt="image" src="https://github.com/user-attachments/assets/7ef4b032-7443-4d99-a09b-71e96575f50b" />
)


Usually, Carla would be installed in Downloads folder/ directory, as a ZIP file.
Extract it in home directory / Desktop / wherever comfortable.

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
-

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


1. Download this repo or file lidar3D_and_radar.py / Clone this repo (run these commands on terminal for linux/ cmd for windows)
```
          git clone -b RGB-Camera https://github.com/BOCK-AI/BAVT/
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
   A live camera window showing RGB feed from the vehicle’s front camera.
If some error occurs, it may be due to incorrect dependencies, or other things.

6. To Exit: CTRL + C stops execution. OR - Press q in the OpenCV window to stop. All actors are destroyed automatically (vehicle + camera). 

This script integrates **CARLA simulator** with **Open3D** and **OpenCV** to:

- **Connects** to CARLA server at `localhost:2000`  
- **Spawns** a Tesla Model 3 with a front RGB camera (FOV 150°)  
- **Captures & processes** frames into OpenCV format for live display  
- **Controls** the vehicle using a PID controller (constant throttle = 0.4)  
- **Runs** simulation in synchronous mode (20 FPS)  
- **Cleans up** actors and resets world settings on exit  

⚠️ Notes / Tweaks:
If CARLA lags, reduce resolution (e.g., IM_WIDTH = 320, IM_HEIGHT = 240).
Change FOV (cam_bp.set_attribute("fov", "90")) for a more natural view.
Add other sensors (LIDAR, RADAR) the same way.
PID values (Kp, Ki, Kd) may need tuning depending on the map.


# **Technical Report: CARLA RGB Camera Streaming with PID-Based Lane Following**

## 1. Overview

This Python script implements a **CARLA autonomous driving simulation** featuring:

1. **RGB camera streaming** — captures live frames from a vehicle-mounted camera and displays them in real-time.
2. **PID-based lane-following controller** — automatically steers a Tesla Model 3 along driving waypoints.
3. **Synchronous simulation mode** — ensures deterministic frame updates and sensor sampling.

The simulation enables real-time testing of basic autonomous navigation pipelines and provides a visualization of camera feeds.

---

## 2. System Requirements

* **CARLA Simulator** (tested with version 0.9.15).

* **Python 3.8+**

* Dependencies:

  ```bash
  pip install carla opencv-python numpy
  ```

* **Hardware Requirements**: GPU recommended for real-time rendering, CPU must support multi-threaded simulation for synchronous mode.

---

## 3. Script Architecture

### 3.1 Components

1. **Vehicle Actor**

   * Tesla Model 3 (`vehicle.tesla.model3`) spawned at a predefined spawn point.
   * Controlled via the CARLA Python API.

2. **RGB Camera Sensor**

   * Attributes:

     * Resolution: `640 × 480` pixels
     * Field of View: `150°`
   * Mounted at `(x=1.6 m, z=1.3 m)` relative to the vehicle.
   * Streams frames into a **thread-safe queue** for asynchronous display.

3. **PID Controller**

   * Proportional (`Kp=1.0`), Integral (`Ki=0.0`), Derivative (`Kd=0.2`) gains.
   * Computes **cross-track error** relative to next waypoint.
   * Generates **steering corrections** for lane following.

4. **Simulation Loop**

   * Runs in **synchronous mode** at `20 FPS` (`fixed_delta_seconds = 0.05`).
   * Each tick performs:

     1. Vehicle pose update.
     2. Next waypoint computation.
     3. Cross-track error calculation.
     4. PID-based steering.
     5. Camera frame retrieval and display.

---

### 3.2 Data Flow

```text
CARLA Simulator
   │
   ├─ Vehicle Actor (Tesla Model 3)
   │      │
   │      └─ Camera Sensor ──> process_img() ──> image_queue
   │
   └─ PID Controller ──> VehicleControl → Vehicle Steering
```

* `process_img()` converts the raw camera RGBA buffer into a BGR frame for OpenCV.
* The `image_queue` ensures thread-safe communication between the CARLA sensor callback and the main rendering loop.

---

## 4. Key Features

* **Real-Time Visualization**: OpenCV window displays live camera feed.
* **Automatic Lane Following**: Vehicle follows driving lanes using PID steering control.
* **Synchronous Simulation**: Ensures deterministic frame and sensor updates.
* **Thread-Safe Sensor Handling**: Sensor callback does not block the simulation.

---

## 5. Usage Instructions

1. **Start CARLA Simulator**:

   ```bash
   ./CarlaUE4.sh -quality-level=Epic -world-port=2000
   ```

2. **Run the script**:

   ```bash
   python carla_rgb_pid.py
   ```

3. **Exit Simulation**:

   * Press **`q`** in the OpenCV window.
   * Cleanup is automatic: sensors stop, actors destroyed, world reverted to asynchronous mode.

---

## 6. Algorithm Details

### 6.1 Cross-Track Error Calculation

* Let `v_forward` be the vehicle's forward vector.
* Let `target_vector` be the vector from the vehicle to the next waypoint.
* Cross-track error `e` computed as:

$$
e = (v_{\text{forward}} \times target_{\text{vector}})_z
$$

* The PID controller outputs a steering command based on `e`.

### 6.2 PID Controller

* **Control Law**:

$$
steer = K_p \cdot e + K_i \cdot \int e\, dt + K_d \cdot \frac{de}{dt}
$$

* Steering values are clipped to `[−1.0, 1.0]`.
* Throttle is fixed at `0.4`.

---

## 7. Limitations

* Only **forward throttle control**; braking not implemented.
* Camera frame resolution fixed; high FPS may stress CPU/GPU.
* PID gains tuned manually; may require adjustment for other maps or speeds.
* No collision avoidance implemented.

---

## 8. Cleanup and Safety

* On exit, all actors (vehicle and sensors) are destroyed.
* World reverts to asynchronous mode.
* OpenCV windows closed to prevent memory leaks.

---

## 9. Customization

* **Camera**: Change resolution or FOV in `cam_bp.set_attribute()`.
* **PID Gains**: Modify `Kp`, `Ki`, `Kd` in `PIDController` to adjust steering response.
* **Throttle**: Adjust `control.throttle` for higher/lower speed.
* **Waypoints**: Adjust `next_wp = waypoint.next(2.0)[0]` distance for smoother or sharper turns.
