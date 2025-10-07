# BAVT - `rgb_cam.py` Documentation

### **Usage of this document** ( This Documentation is primarily focused on Linux )

##### Pre-Requisites:
We need **Python 3.8 to 3.10**. So please make sure you have that in your system. If you have the required python versions already, then the next sub-section can be skipped.

If you don't have python on your system, or have a different version, then please read the next Sub-section.


#### Python Installation (Recommend using python 3.10.11)
##### Mainly for Linux users: ( I am using Ubuntu 22.04 LTS ) | Python.org 

1.  **Go to the official Python downloads page** for version 3.10.11: [https://www.python.org/downloads/release/python-31011/](https://www.python.org/downloads/release/python-31011/)

2.  **Scroll down** to the **Files** section at the bottom of the page.

3.  **Choose the correct installer** for your operating system.

      * **Windows:** Look for "**Windows installer (64-bit)**".
      * **macOS:** Look for "**macOS 64-bit universal2 installer**".
      * **Linux:** It's best to use your distribution's package manager (like `apt`, `yum`, or `dnf`). For example, on Ubuntu, you might use `sudo apt install python3.10`.

4.  **Download and run the installer**, following the on-screen instructions. Be sure to check the box that says "**Add Python 3.10 to PATH**" during installation on Windows.

-----
#### CARLA Simulator Installation (Version 0.9.15) 

1.  You **must** have CARLA 0.9.15 installed. Download it from the official releases page:
    [https://github.com/carla-simulator/carla/releases](https://github.com/carla-simulator/carla/releases)
    Click on the appropriate download link under the `0.9.15` release section.

2. After downloading, you will typically get a ZIP or `tar.gz` file.
     * **Extract** this file to a comfortable location, such as your home directory on Linux or `C:\` drive on Windows.
     * **Rename** the extracted folder to `carla_0.9.15` for consistency.  
-----


#### Running the CARLA Simulator
* **Open a new terminal window.** <br>
* **Navigate to the CARLA directory.** Assuming you extracted it to your Home directory or C: (C drive).  <br>
       * **Linux:**
          ```bash
            cd ~/carla_0.9.15
          ```
          * **Windows CMD (Adjust path as required):**
                  ```
                  cd
                  cd C:\carla_0.9.15
                  ``` <br>
               * **Execute the CARLA server:**
                    * **Linux:**
                         ```bash
                         ./CarlaUE4.sh
                         ```
               * **Windows:**
               ```cmd 
               CarlaUE4.exe        
               ```
      * Alternatively, on Windows, you can simply navigate to the `carla_0.9.15` folder in File Explorer and double-click `CarlaUE4.exe`.
      
Congratulations\! You are now ready to get started with the world of CARLA.

   
   
#### Additional CARLA Setup Resources:
   For more detailed setup and tutorials, refer to these official CARLA documentation links:
  * [https://www.youtube.com/watch?v=jIK9sanumuU](https://www.youtube.com/watch?v=jIK9sanumuU)
  * [https://www.youtube.com/watch?v=AaJekfFR1KQ](https://www.youtube.com/watch?v=AaJekfFR1KQ)
  * [https://carla.readthedocs.io/en/latest/start\_quickstart/](https://carla.readthedocs.io/en/latest/start_quickstart/)
  * [https://carla.readthedocs.io/en/latest/tuto\_first\_steps/](https://carla.readthedocs.io/en/latest/tuto_first_steps/)
  * [https://carla.readthedocs.io/en/latest/tutorials/](https://carla.readthedocs.io/en/latest/tutorials/)

-----


### Instructions to use `rgb_cam.py`:

Firstly, run this command, which makes sure you clone the repo into the right folder.  Start from HOME directory (```~/```) in your linux terminal. For Windows, Start from C: Folder in cmd.
```
cd carla_0.9.15/PythonAPI/examples
```
Now, we are ready to clone the repository.

1.  **Clone the Repository**
    
    *FOR WINDOWS:*   
    It is easier for Linux users to use Git than Windows users.
    Windows User can simply download zip, extract its contents into the right folder:
    ```(C:/carla_0.9.15/PythonAPI/examples)```
    
    *FOR LINUX:*   
    Clone the `RGB-Camera` branch of this repository and navigate into the `BAVT` directory. Run these commands in your terminal or command prompt:
    (**ONLY ONCE THIS HAS TO BE CLONED.**)
    ```  
    git https://github.com/BOCK-AI/BAVT/   
    cd BAVT   
    ```
    
3.  **Install Dependencies**
    Install the required Python libraries.

    ```bash
    pip install carla opencv-python numpy
    ```

4.  **Run CARLA Server**
    In a **separate terminal**, start the CARLA simulator (same as described in the "Running the CARLA Simulator" section above).

      * **Linux:**
        ```bash
        ./CarlaUE4.sh
        ```
      * **Windows:**
        ```cmd
        CarlaUE4.exe
        ```

5.  **Run the Python Script**
    Once the CARLA server is running, execute the `rgb_cam.py` file in your first terminal:

    ```bash
    python3 rgb_cam.py
    ```

6.  **What You’ll See**

      * A **Tesla Model 3** driving automatically in the CARLA world, following the road.
      * A live OpenCV window displaying the **RGB camera feed** from the vehicle's front camera.
      * In the simulator, you will be positioned to spectate *behind* the car. (This is done using the Spectator class of CARLA. Can be modified. I find it convenient.)
      * If any error occurs, it might be due to incorrect dependencies or other configuration issues.

7.  **How to Exit**
    To stop the script and clean up the simulation, you can either:

      * Press **`CTRL + C`** in the terminal where `rgb_cam.py` is running.
      * Press the **`q`** key while the OpenCV window is active.
        All actors (vehicle + camera) and windows are destroyed automatically.

-----

**Some Handy Terminal Shortcuts:**
 - ctrl + shift + v - paste
 - ctrl + shift + c - copy
 - tab - Autocomplete, if enabled.
 - ctrl + shift + t - for a new tab in the current terminal window.
 - ctrl + shift + n - for a new terminal window.

-----

## Technical Report: CARLA RGB Camera Streaming with PID-Based Lane Following

### 1\. Overview

This Python script implements a **CARLA autonomous driving simulation** featuring:

1.  **RGB camera streaming** — captures live frames from a vehicle-mounted camera and displays them in real-time.
2.  **PID-based lane-following controller** — automatically steers a Tesla Model 3 along driving waypoints.
3.  **Synchronous simulation mode** — ensures deterministic frame updates and sensor sampling.

The simulation enables real-time testing of basic autonomous navigation pipelines and provides a visualization of camera feeds.

-----

### 2\. System Requirements

  * **CARLA Simulator** (tested with version 0.9.15).
  * **Python 3.8+**
  * **Dependencies**:
    ```bash
    pip install carla opencv-python numpy
    ```
  * **Hardware Requirements**: GPU recommended for real-time rendering; CPU must support multi-threaded simulation for synchronous mode.

-----

### 3\. Script Architecture

#### 3.1 Components

1.  **Vehicle Actor**

      * A **Tesla Model 3** (`vehicle.tesla.model3`) is spawned at a predefined spawn point.
      * It is controlled via the CARLA Python API.

2.  **RGB Camera Sensor**

      * **Attributes**:
          * Resolution: `640 × 480` pixels
          * Field of View (FOV): `150°`
      * **Placement**: Mounted at `(x=1.6 m, z=1.3 m)` relative to the vehicle.
      * **Streaming**: Streams frames into a **thread-safe queue** for asynchronous display.

3.  **PID Controller**

      * **Gains**: Proportional (`Kp=1.0`), Integral (`Ki=0.0`), Derivative (`Kd=0.2`).
      * **Function**: Computes **cross-track error** relative to the next waypoint and generates **steering corrections** for lane following.

4.  **Simulation Loop**

      * Runs in **synchronous mode** at `20 FPS` (`fixed_delta_seconds = 0.05`).
      * Each tick performs:
        1.  Vehicle pose update.
        2.  Next waypoint computation.
        3.  Cross-track error calculation.
        4.  PID-based steering.
        5.  Camera frame retrieval and display.

-----

#### 3.2 Data Flow

```text
CARLA Simulator
    │
    ├─ Vehicle Actor (Tesla Model 3)
    │       │
    │       └─ Camera Sensor ──> process_img() ──> image_queue
    │
    └─ PID Controller ──> VehicleControl → Vehicle Steering
```

  * `process_img()` converts the raw camera RGBA buffer into a BGR frame suitable for OpenCV.
  * The `image_queue` ensures thread-safe communication between the CARLA sensor callback and the main rendering loop.

-----

### 4\. Key Features

  * **Real-Time Visualization**: An OpenCV window displays the live camera feed from the vehicle.
  * **Automatic Lane Following**: The vehicle autonomously follows driving lanes using PID steering control.
  * **Synchronous Simulation**: Ensures deterministic and consistent frame and sensor updates.
  * **Thread-Safe Sensor Handling**: The sensor callback processes data without blocking the main simulation thread.

-----

### 5\. Usage Instructions (Detailed)

1.  **Start CARLA Simulator**:

    ```bash
    ./CarlaUE4.sh -quality-level=Epic -world-port=2000
    ```

    *(Note: Adjust `./CarlaUE4.sh` to `CarlaUE4.exe` for Windows as appropriate.)*

2.  **Run the script**:

    ```bash
    python carla_rgb_pid.py
    ```

    *(Note: The script name in this section `carla_rgb_pid.py` seems to be a placeholder, please ensure it matches the actual filename, which is `rgb_cam.py` as per the document's context.)*

3.  **Exit Simulation**:

      * Press **`q`** in the OpenCV window.
      * Cleanup is automatic: sensors stop, actors are destroyed, and the world is reverted to asynchronous mode.

-----

### 6\. Algorithm Details

#### 6.1 Cross-Track Error Calculation

  * Let `v_forward` be the vehicle's forward vector.

  * Let `target_vector` be the vector from the vehicle to the next waypoint.

  * The cross-track error `e` is computed as:

    e = (v\_{\\text{forward}} \\times target\_{\\text{vector}})\_z
    
  * The PID controller then outputs a steering command based on this error `e`.

#### 6.2 PID Controller

  * **Control Law**:

    $$
    $$$$steer = K\_p \\cdot e + K\_i \\cdot \\int e, dt + K\_d \\cdot \\frac{de}{dt}

    $$
    $$$$
    $$
  * Steering values are clipped to `[−1.0, 1.0]`.

  * The throttle is fixed at `0.4`.

-----

### 7\. Limitations

  * Only **forward throttle control** is implemented; braking is not handled by the PID controller.
  * The camera frame resolution is fixed; very high frame rates might strain the CPU/GPU.
  * PID gains are manually tuned and may require adjustment for optimal performance on different maps or at varying speeds.
  * No collision avoidance mechanism is implemented.

-----

### 8\. Cleanup and Safety

  * Upon exiting the script, all spawned actors (vehicle and sensors) are automatically destroyed.
  * The CARLA world settings are reverted to asynchronous mode.
  * OpenCV windows are properly closed to prevent potential memory leaks.

-----

### 9\. Customization

  * **Camera**: You can change the resolution or Field of View (FOV) by modifying `cam_bp.set_attribute()`.
  * **PID Gains**: Adjust `Kp`, `Ki`, `Kd` in the `PIDController` class to fine-tune the steering response.
  * **Throttle**: Modify `control.throttle` to achieve higher or lower speeds.
  * **Waypoints**: Adjust the distance for `next_wp = waypoint.next(2.0)[0]` to influence how smoothly or sharply the vehicle takes turns.

-----

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




--- 
---
OWN DRAFT!

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
