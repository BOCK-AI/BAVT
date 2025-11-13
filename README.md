# BAVT - IMU Sensor

### **Usage of this document** ( This Documentation is primarily focused on Linux )

##### Pre-Requisites:
We need **Python 3.8 to 3.10**. So please make sure you have that in your system. If you have the required python versions already, then the next sub-section can be skipped.

If you don't have python on your system, or have a different version, then please read the next Sub-section.


#### Python Installation (Recommend using python 3.10.11)
##### Mainly for Linux users: ( I am using Ubuntu 22.04 LTS ) | Python.org 

* *FOR LINUX:*      
* **Linux:** It's best to use your distribution's package manager (like `apt`, `yum`, or `dnf`). For example, on Ubuntu, you might use   
  ```   
  sudo apt install python3.10
  ```    

* *FOR WINDOWS:*             
1. Please follow this tutorial for CRYSTAL CLEAR instructions on how to download python.     
   (Change python 3.8 to 3.10 wherever needed.).    
   `https://tutedude.com/blogs/product/how-to-install-python-on-windows-10-a-guide-for-developers-new-to-python/`      
2. **Go to the official Python downloads page** for version 3.10.11: [https://www.python.org/downloads/release/python-31011/](https://www.python.org/downloads/release/python-31011/)


3.  **Scroll down** to the **Files** section at the bottom of the page.

4.  **Choose the correct installer** for your operating system.

      * **Windows:** Look for "**Windows installer (64-bit)**".
      * **macOS:** Look for "**macOS 64-bit universal2 installer**".

5.  **Download and run the installer**, following the on-screen instructions. Be sure to check the box that says "**Add Python 3.10 to PATH**" during installation on Windows.

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
* **Navigate to the CARLA directory.** Assuming you extracted it to your Home directory or C: (C drive).
* * **Linux:**      
```    
cd ~/carla_0.9.15
```     
* **Windows CMD (Adjust path as required):**    
```    
cd   
cd C:\carla_0.9.15     
```      
* **Execute the CARLA server:**   
  * **Linux:**        
       ```
       ./CarlaUE4.sh      
       ```   
   * **Windows:**   
   ```         
   CarlaUE4.exe              
   ```           
Alternatively, on Windows, you can simply navigate to the `carla_0.9.15` folder in File Explorer and double-click `CarlaUE4.exe`.
      
Congratulations\! You are now ready to get started with the world of CARLA.

   
   
#### Additional CARLA Setup Resources:
   For more detailed setup and tutorials, refer to these official CARLA documentation links:
  * [https://www.youtube.com/watch?v=jIK9sanumuU](https://www.youtube.com/watch?v=jIK9sanumuU)
  * [https://www.youtube.com/watch?v=AaJekfFR1KQ](https://www.youtube.com/watch?v=AaJekfFR1KQ)
  * [https://carla.readthedocs.io/en/latest/start\_quickstart/](https://carla.readthedocs.io/en/latest/start_quickstart/)
  * [https://carla.readthedocs.io/en/latest/tuto\_first\_steps/](https://carla.readthedocs.io/en/latest/tuto_first_steps/)
  * [https://carla.readthedocs.io/en/latest/tutorials/](https://carla.readthedocs.io/en/latest/tutorials/)

-----

### Instructions to use `imu_sensor.py`:

Firstly, run this command, which makes sure you clone the repo into the right folder.   
Start from HOME directory (```~/```) in your linux terminal.    
For Windows, Start from C: Folder in cmd.

* The *first* terminal   
  *FOR WINDOWS:*
  ```
  cd C:/carla_0.9.15/PythonAPI/examples
  ```
  *FOR LINUX:*
  ```
  cd carla_0.9.15/PythonAPI/examples
  ```
Now, we are ready to clone the repository.

1.  **Clone the Repository and switch branch**
  - **(NOTE: Clone only the first time. Do not need to clone repeatedly.)**                     
  - **(NOTE: If repository is already cloned, Only need to SWITCH BRANCH. Go to **SWITCHING BRANCH:** subsection below. )**
           
    **<ins>FOR WINDOWS:</ins>**                                 
    **CLONING: (FIRST TIME ONLY)**                
    It is easier for Linux users to use Git than Windows users. Thus, I suggest downloading zip and extracting the contents into te BAVT folder we will create below.                      
    To Create a BAVT folder: (only first time)         
    ```
    cd C:/carla_0.9.15/PythonAPI/examples
    mkdir BAVT
    cd BAVT    
    ```    
    **Extract zip contents into the BAVT Folder**          
    Download the zip, extract its contents into the below BAVT folder:            
    ```
    (C:/carla_0.9.15/PythonAPI/examples/BAVT)
    ```

    **<ins>FOR LINUX:</ins>**                                                  
    **CLONING: (FIRST TIME ONLY)**                 
    Run these commands in your terminal or command prompt:      
    ```
    git clone https://github.com/BOCK-AI/BAVT/
    cd BAVT
    ```
    **SWITCHING BRANCH:**                         
    Now we must navigate to the right branch on terminal: (This command is used for seeing All the existing Branches in the GitHub repo.)
    ```
    git branch -a
    ```
    Now we switch to IMU-Sensor branch : (Switches branches using ```git checkout```)
    ```
    git checkout IMU-Sensor
    ```
    You can see all branches on *your* system using:
    ```
    git branch
    ```   

3.  **Install Dependencies**
    Install the required Python libraries.

    ```bash
    pip install carla numpy opencv-python
    ``` 

4.  **Run CARLA Server**
    In a **separate terminal**, start the CARLA simulator (same as described in the "Running the CARLA Simulator" section above).
    Execute these lines first:
    ```
    cd
    cd carla_0.9.15
    ```
    Then continue with these:   
      * **Linux:**
        ```bash
        ./CarlaUE4.sh
        ```
      * **Windows:**
        ```cmd
        CarlaUE4.exe
        ```
        (If this doesn't work, then go to your file explorer, open C:, open the carla_0.9.15 folder. Double click on the CarlaUE4.exe. Carla is now going to start running.)

6.  **Run the Python Script**
    Once the CARLA server is running, execute the `imu_sensor.py` file in your *first* terminal: 

    ```bash
    python3 imu_sensor.py
    ```

7.  **What You’ll See**

      * A **Tesla Model 3** driving automatically in the CARLA world, following the road.
      * IMU Sensor readings will be seen in terminal. Accelerometer readings, Gyroscope readings, Compass Direction readings.
      * Explanation is provided below.
      * In the simulator, you will be positioned to spectate behind the car. (This is done using the Spectator class of CARLA. It can be modified.)
                <
    #### IMU Sensor output Understanding:                        
      Think of the IMU as a sensor that feels the **pushes and spins** of the vehicle.
      #####  1. ACCELEROMETER (x, y, z)
      It tells you **how the car is being pushed**.
      Imagine you're sitting in a car:
        ######  **x-axis (forward/backward push)**
        * Car accelerates → you feel pushed **back** into the seat → **+x**
        * Car brakes → you lurch **forward** → **–x**
        ######  **y-axis (sideways push)**
        * Car turns left → you feel pushed **right** → **+y**
        * Car turns right → you feel pushed **left** → **–y**
        ######  **z-axis (up/down push)**
        * Gravity always pushes down → ~9.8 m/s²
        * If car goes over a bump → z briefly increases
        * If car drops down → z decreases
      #####  2. GYROSCOPE (x, y, z)
      This measures **how fast the car is rotating**.
      Think of holding a phone and rotating it:
      ######  **x-axis (roll)** 
      Car tilts **sideways** (like when climbing a slanted road)
      ######  **y-axis (pitch)**
      Car tilts **forward/back** (braking → nose dips down, accelerating → nose lifts)
      ######  **z-axis (yaw)**
      Car rotates **left or right** (turning at an intersection)

      ##### **A SIMPLE, REAL EXAMPLE**
      Suppose your car is:
      * Driving straight and accelerating
      * Slightly turning left
      * Going up a small incline
    
      IMU might read:             
        * Accelerometer:
        ```x  = +2.5   (car speeding up)
    
           y  = +0.3   (turning left, small sideways push)
    
           z  = 9.8    (gravity)                  
        ```                  
        * Gyroscope:                    
        ```
           x   = 0.0    (vehicle not rolling)
                                  
           y = +0.1   (nose slightly going up)                   

           z = 0.4    (yaw: turning left)                        
        ```                                
        It makes sense now, hopefully!                                    
 
    ##### Overview:
      **Accelerometer = pushes**
      * x = forward/back
      * y = left/right
      * z = up/down
      **Gyroscope = spins**
      * x = roll
      * y = pitch
      * z = yaw
   
      

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
## **Technical Report: CARLA IMU Sensor Data Streaming with PID-Based Lane Following**

### 1. Overview
This Python script provides a comprehensive example of interfacing with the CARLA autonomous driving simulator. Its primary functions are:
1.  **Spawning** a vehicle and an IMU (Inertial Measurement Unit) sensor.
2. **Attaching** the IMU to the vehicle.
3.  **Listening** to the IMU sensor data using a callback and printing formatted data (Accelerometer, Gyroscope, Compass) to the console.
4.  **Implementing** a PID-controlled autonomous driving agent that follows the road's waypoints.
5.  **Managing** the simulation in synchronous mode for deterministic behavior.
6.  **Providing** robust setup and cleanup, including a "chase cam" spectator view.
   
### 2. Features  
* **Synchronous Mode:** Ensures that simulation physics and sensor data are perfectly aligned by using a fixed time-step (`fixed_delta_seconds = 0.05`).
* **Sensor Callback:** Uses the `sensor.listen()` method to asynchronously (from the script's main thread) process IMU data as soon as it's available from the simulator tick.
* **Live Data Display:** Prints formatted IMU data to the console without spamming new lines by using the `\r` carriage return.
* **PID Controller:** Includes a self-contained `PIDController` class to manage steering control.
* **Waypoint Following:** Implements a control loop that:
  * Finds the vehicle's current waypoint.
  * Calculates a target waypoint 2.0 meters ahead.
  * Calculates the **cross-track error** using a vector cross-product.
  * Feeds this error into the PID controller to get a steering correction.
  * **Spectator "Chase Cam":** The spectator camera is programmatically set to follow the vehicle from behind. 
* **Robust Cleanup:** Uses a `try...finally` block to ensure all actors are destroyed and the simulator is returned to asynchronous mode on exit, even if an error occurs.

### 3. Prerequisites  
* A running CARLA simulator instance (defaults to `localhost:2000`).
* Python 3.x
* Required Python libraries:
* `carla`: The CARLA client library.
* `numpy`: For vector and cross-product calculations.
* `opencv-python` (`cv2`): Used only for the `cv2.waitKey()` function to detect the 'q' exit key


### 4. How to Run
1. Start the CARLA simulator.
2. Execute the script from your terminal:```bash    python your_script_name.py    ```
3. An empty OpenCV window titled "CARLA Control" will appear. Keep this window in focus.
4. Press the **'q'** key to gracefully exit the script, clean up actors, and stop the simulation.


-----

### 5. Core Components & Architecture
The script is built around the `main()` function and two key helper components: the `imu_callback` function and the `PIDController` class.
#### 5.1. Setup and Connection (in `main()`)
1.  **Client Connection:** A `carla.Client` is instantiated and connects to the simulator.
2.  **World & Settings:** The script gets the `world` object and retrieves its settings.
3.  **Synchronous Mode:** This is a critical step.
* `settings.synchronous_mode = True` tells the simulator to pause and wait for a "tick" command from the client. 
* `settings.fixed_delta_seconds = 0.05` sets the physics time-step to 0.05 seconds (20 FPS).
* `world.apply_settings(settings)` commits these changes.

#### 5.2. Actor Spawning (in `main()`)
1. **Vehicle:** A `vehicle.tesla.model3` is spawned at a random spawn point and added to the `actors` list for later cleanup.
2. **Spectator Camera:** A nested function `update_spectator()` is defined. It calculates a position 4m behind and 2.5m above the vehicle *relative to the vehicle's transform* and moves the spectator camera there.
3. **IMU Sensor:**
   * An `sensor.other.imu` blueprint is found.
   * Its `sensor_tick` is set to match the world's `fixed_delta_seconds` to ensure 1:1 data generation with physics steps.
   * The IMU is spawned and **attached** to the `vehicle` at a slight offset.
   * The sensor is added to the `actors` list.
#### 5.3. IMU Data Handling (`imu_callback` & `imu.listen()`)  
* **`imu.listen(imu_callback)`:** This line registers the `imu_callback` function. After every `world.tick()`, the simulator will generate new IMU data and automatically call this function, passing the `imu_data` object to it.
* **`imu_callback(imu_data)`:**
  * **Input:** `imu_data` (a `carla.ImuMeasurement` object).
  * **Action:** It accesses the `accelerometer`, `gyroscope`, and `compass` attributes.
  * **Note on Compass:** The script correctly identifies that `imu_data.compass` provides the heading in **radians**. It converts this to degrees by multiplying by `180/pi`.
  * **Output:** Prints the formatted data to a single, updating line in the console.
#### 5.4. Control Logic (`PIDController` & Main Loop)
1.  **`PIDController` Class:**
   * `__init__(self, Kp, Ki, Kd)`: Initializes the controller with Proportional, Integral, and Derivative gains.
   * `run_step(self, error, dt)`: The core logic. It calculates the integral and derivative terms based on the current `error` and time delta (`dt`), then returns the final control output: $Kp \cdot error + Ki \cdot integral + Kd \cdot derivative$.
2.  **Main Loop Logic (`while True`)**
     * **`world.tick()`:** The **most important** call. This commands the simulator to advance one step (0.05s). All physics are calculated, and all sensors (like the IMU) are triggered.      
     **`update_spectator()`:** The "chase cam" is updated to the vehicle's new position.
     * **Waypoint Calculation:**
       1.  The vehicle's current location is used to find the nearest `waypoint` on the road.
       2.  `waypoint.next(2.0)[0]` gets the next waypoint 2.0 meters down the lane. This is the **target**.
          * **Error Calculation (Cross-Track Error):**
            1.  `v_forward`: The vehicle's current forward-facing vector.
            2.  `target_vector`: The vector from the vehicle's location to the target waypoint's location.
            3.  `np.cross(...)`: The 3D cross product is calculated between the *forward vector* and the *target vector*. The **Z-component** of this resulting vector is the cross-track error.
               * If `z > 0`, the target is to the vehicle's right.
               * If `z < 0`, the target is to the vehicle's left.
               * If `z == 0`, the target is dead ahead.
          * **Control Application:**
         1.  The `steer_correction` is calculated by feeding the `cross_track_error` into `pid.run_step()`.
         2.  The output is clamped (using `np.clip`) to the valid range `[-1.0, 1.0]`.
         3.  A new `carla.VehicleControl` object is created with a constant throttle (0.4) and the calculated `steer_correction`.
         4.  `vehicle.apply_control(control)` sends the command to the vehicle.
          * **Exit Condition:** `cv2.waitKey(1)` checks if a key has been pressed. If it's 'q', the loop breaks.

#### 5.5. Cleanup (`finally` block)
This block executes **no matter what**, ensuring the simulator is not left in a broken state.
1.  **Stop Sensor:** `imu.stop()` is called to stop the `listen()` callback.
2.  **Destroy Actors:** Iterates through the `actors` list (containing the vehicle and IMU) and calls `a.destroy()` on each.
3.  **Revert Settings:** The script gets the world settings again, sets `synchronous_mode = False`, and applies them. This is critical for returning the simulator to its normal, real-time state.
4.  **Close Windows:** `cv2.destroyAllWindows()` closes the OpenCV window.

-----
