# BAVT - `lidar3d_and_radar.py` Documentation

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

### Instructions to use `lidar3d_and_radar.py`:

Firstly, run this command, which makes sure you clone the repo into the right folder. Start from HOME directory (```~/```) in your linux terminal. For Windows, Start from C: Folder in cmd.

* The *first* terminal
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
    Clone the `3-D-LIDAR` branch of this repository and navigate into the `BAVT` directory. Run these commands in your terminal or command prompt:      
    ```
    git clone -b 3-D-LIDAR https://github.com/BOCK-AI/BAVT/
    cd BAVT
    ```

3.  **Install Dependencies**
    Install the required Python libraries.

    ```bash
    pip install carla numpy cv2 open3d matplotlib
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
    Once the CARLA server is running, execute the `lidar3d_and_radar.py` file in your *first* terminal: 

    ```bash
    python3 lidar3d_and_radar.py
    ```

6.  **What You’ll See**

      * A **Tesla Model 3** driving automatically in the CARLA world, following the road.
      * A live OpenCV window displaying the **RGB camera feed** from the vehicle's front camera.
      * A live Open3D Window ('Carla Lidar/Radar') which is an interactive 3D window where you can use your mouse to rotate, pan, and zoom the view. It displays two overlapping point clouds updated in real-time. (Lidar - yellow gradient points, Radar - Blue points)
      * In the simulator, you will be positioned to spectate behind the car. (This is done using the Spectator class of CARLA. Can be modified. I find it convenient.)
      
Note: Shortcuts for the **Open3D** window: 
Ctrl + '+' or Ctrl + '=' -> to make points bigger.
Ctrl + '-' -> to make points smaller.
        
6.  **How to Exit**
    To stop the script and clean up the simulation, you can either:

      * Press **`CTRL + C`** in the terminal where `lidar3d_and_radar.py` is running.
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
      
## **Technical Report: CARLA 3D-LIDAR and RADAR Streaming with PID-Based Lane Following**
The lidar3d_and_radar.py script demonstrates a comprehensive CARLA simulation environment. It spawns a vehicle and equips it with LiDAR, RADAR, and an RGB camera. The data from these sensors is processed and visualized in real-time using Open3D for 3D point clouds and OpenCV for the camera feed.

The script implements a simple autonomous driving agent using a **PID controller** to make the vehicle follow the lanes of the road.  

### \#\# Dependencies

Before running, ensure you have the required libraries installed and a CARLA server is active.

  * **CARLA Server:** The script requires an active CARLA simulator instance to connect to.
  * **Python Libraries:**
      * `carla`: The official CARLA Python client.
      * `numpy`: For numerical operations.
      * `opencv-python` (cv2): For displaying the camera feed.
      * `open3d`: For 3D point cloud visualization.
      * `matplotlib`: For colormap generation.


### \#\# Core Components

#### **Configuration**

Global constants at the beginning of the file define the image dimensions (`IM_WIDTH`, `IM_HEIGHT`) and pre-generate colormaps from `matplotlib` for coloring the point clouds.

  * `VIRIDIS` (Plasma): Used to color LiDAR points based on **intensity**.
  * `COOL` (Winter): Used to color RADAR points based on **velocity**.

#### **PID Controller**

The `PIDController` class implements a Proportional-Integral-Derivative controller. This is a common control loop mechanism used to minimize the error between a measured value and a desired setpoint.

  * **Proportional ($K_p$)**: Reacts to the current error. A larger error results in a stronger correction.
  * **Integral ($K_i$)**: Accumulates past errors. This helps eliminate steady-state errors.
  * **Derivative ($K_d$)**: Reacts to the rate of change of the error. This helps dampen oscillations and prevent overshooting.

In this script, it's used to calculate the necessary **steering angle** to minimize the car's distance from the center of the lane (the cross-track error).

#### **Main Loop (`main` function)**

The `main()` function orchestrates the entire simulation:

1.  **Initialization**: Connects to the CARLA client and sets the world to **synchronous mode**. This is critical as it ensures the simulation only advances when `world.tick()` is called, allowing all sensors to provide data for the same moment in time.
2.  **Actor Spawning**: Spawns a `vehicle.tesla.model3` and attaches three sensors to it: a LiDAR, a RADAR, and an RGB Camera. Each sensor's attributes (range, FoV, points per second, etc.) are configured here.
3.  **Callbacks**: For each sensor, a `listen()` method is called with a lambda function. This function is executed asynchronously whenever the sensor produces new data.
      * `lidar.listen(lambda data: lidar_callback(data, point_list))`
      * `radar.listen(lambda data: radar_callback(data, radar_list))`
      * `camera.listen(lambda image: camera_callback(image, camera_data))`
4.  **Visualization Setup**: Creates an OpenCV window for the camera and an Open3D visualizer for the point clouds.
5.  **Execution Loop**: The `while True:` loop is the heart of the script. In each iteration, it:
      * Calls `world.tick()` to advance the simulation by one step (`0.05` seconds).
      * Updates the Open3D geometries with the latest sensor data.
      * Calculates the vehicle's **cross-track error** by comparing its forward vector to the vector pointing towards a future waypoint.
      * Uses the `PIDController` to compute the required steering correction based on this error.
      * Applies throttle and the computed steer value to the vehicle.
      * Displays the new camera image.
6.  **Cleanup**: The `finally` block ensures that when the script is terminated, all actors are destroyed and the world is reverted to asynchronous mode, preventing orphaned actors in the simulator.


### \#\# Function Reference

`add_open3d_axis(vis)`

  * Adds a 3D coordinate axis (X, Y, Z) to the Open3D visualizer for better orientation.

`lidar_callback(point_cloud, point_list)`

  * **Triggered by**: LiDAR sensor data.
  * **Action**: Processes raw LiDAR data (`point_cloud.raw_data`). It reshapes the data into `(x, y, z, intensity)` points. Colors are calculated based on intensity and mapped using the 'plasma' colormap. The provided Open3D `point_list` object is updated with the new points and colors.

`radar_callback(meas, point_list)`

  * **Triggered by**: RADAR sensor data.
  * **Action**: Processes a list of radar detections (`meas`). It converts each detection from spherical coordinates (azimuth, altitude, depth) to Cartesian coordinates `(x, y, z)`. Colors are calculated based on the absolute velocity and mapped using the 'winter' colormap. The provided Open3D `radar_list` object is updated.

`camera_callback(image, data_dict)`

  * **Triggered by**: RGB Camera data.
  * **Action**: Reshapes the raw image data into a NumPy array `(height, width, 4)` and stores it in the `camera_data` dictionary.

`PIDController.run_step(error, dt)`

  * **Parameters**: The current `error` (cross-track error) and `dt` (delta time, `0.05s`).
  * **Returns**: A control value (steering correction) calculated using the PID formula.




















