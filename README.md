# BAVT - `6sensors.py` Documentation
    
***NOTE: THE BELOW CODE HAS BEEN TAKEN FROM THE ABOVE TUTORIAL: [An In-Depth Look at Carla's Sensors](https://www.youtube.com/watch?v=om8klsBj4rc
)***

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

### Instructions to use `6sensors.py`:

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
  - **(NOTE: Clone only the first time. Do not need to clone repeatedly.)**
Now, we are ready to clone the repository.

1.  **Clone the Repository and switch branch**

    *FOR WINDOWS:*   
    It is easier for Linux users to use Git than Windows users. Thus, I suggest downloading zip and extracting the contents into te BAVT folder we will create below.                      
    To Create a BAVT folder: (only first time)         
    ```
    cd C:/carla_0.9.15/PythonAPI/examples
    mkdir BAVT
    cd BAVT    
    ```      
    Download the zip, extract its contents into the below BAVT folder:            
    ```
    (C:/carla_0.9.15/PythonAPI/examples/BAVT)
    ```
           
    *FOR LINUX:*   
    Run these commands in your terminal or command prompt:      
    ```
    git clone https://github.com/BOCK-AI/BAVT/
    cd BAVT
    ```
    Now we must navigate to the right branch on terminal: (This command is used for seeing All the existing Branches in the GitHub repo.)
    ```
    git branch -a
    ```
    Now we switch to 3-D-LIDAR branch : (Switches branches using ```git checkout```)
    ```
    git checkout 6-SENSORS
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
    Once the CARLA server is running, execute the `6sensors.py` file in your *first* terminal: 

    ```bash
    python3 6sensors.py
    ```

7.  **What You’ll See**

      * A **Tesla Model 3** driving automatically in the CARLA world, following the road.
      * A live OpenCV window ('All Cameras') displaying the **RGB camera feed**, **Semantic Segmentation camera feed**, **Instance Segmentation camera feed**, **Depth camera feed**, **Dynamic Vision Sensing camera feed**, **Optical Flow Camera feed** from the vehicle's front camera.
      * Each camera will be visible with clarity. 
      * In the simulator, you will be positioned to spectate behind the car. (This is done using the Spectator class of CARLA. It can be modified.)
          
<img width="725" height="825" alt="image" src="https://github.com/user-attachments/assets/ec153c2f-8a13-442b-80fa-bab1ff00e312" />

      

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
      
## **Technical Report: CARLA 6-SENSORS Streaming with PID-Based Lane Following**

Here is the technical report formatted using GitHub-flavored Markdown.

---

### **Technical Report: Multi-Sensor Data Collection and Closed-Loop Vehicle Control in the CARLA Simulator**

The primary objective of the script is to establish a robust, multi-modal data collection platform by interfacing with a comprehensive suite of camera-based sensors. The system spawns a primary ego-vehicle and attaches six distinct sensors: **RGB, semantic segmentation, instance segmentation, depth, DVS (Dynamic Vision Sensor), and optical flow**.

The simulation is configured to run in **synchronous mode** to ensure data consistency and determinism. Data from each sensor is processed via asynchronous callbacks and aggregated for real-time visualization in a single, tiled OpenCV window. Concurrently, the script implements a closed-loop vehicle control system using a **PID (Proportional-Integral-Derivative) controller** for basic lane-keeping, demonstrating a complete "sense-plan-act" loop.

---

### **1. Introduction**

#### **1.1. Objective**

The script's main goal is to create a testbed for autonomous vehicle perception and control. It achieves this by:
1.  **Configuring** a CARLA simulation environment for deterministic, repeatable experiments (synchronous mode).
2.  **Spawning** a vehicle and a comprehensive suite of 6 different camera sensors.
3.  **Implementing** a data-handling pipeline to receive, process, and visualize data from all sensors in real-time.
4.  **Demonstrating** basic autonomous control by using a PID controller to perform lane-following based on the vehicle's position relative to road waypoints.

#### **1.2. Core Technologies**
* **CARLA Simulator (0.9.x):** A high-fidelity, open-source simulator for autonomous driving research.
* **Python 3:** The scripting and control language.
* **OpenCV (`cv2`):** Used for real-time image processing, visualization, and user input (quitting the simulation).
* **NumPy:** Employed for efficient numerical operations and array manipulation of sensor data.

---

### **2. System Architecture and Methodology**

The system's operation is built around a central simulation loop, which is managed in synchronous mode.

#### **2.1. Environment Configuration**

The script begins by connecting to the CARLA client and modifying the world settings.
* **Synchronous Mode:** `settings.synchronous_mode = True` is set. This is a critical step. It pauses the simulation's physics engine, which will only advance upon receiving a "tick" command from the client (`world.tick()`).
* **Fixed Timestep:** `settings.fixed_delta_seconds = 0.05` sets the simulation to advance in fixed 50ms increments, corresponding to a simulation rate of 20 FPS. This ensures that all sensor data and control commands are processed at a consistent frequency, which is essential for data logging and stable control.

#### **2.2. Actor and Sensor Suite Spawning**

1.  **Vehicle:** A `vehicle.tesla.model3` is spawned at a pre-defined spawn point.
2.  **Sensor Suite:** A list of sensor definitions is iterated over. For each, a blueprint is found, attributes (image width/height) are set, and the sensor is spawned and **attached** to the ego-vehicle. This ensures the sensors move with the vehicle.
3.  **Sensor Callbacks:** Each sensor's `listen()` method is used to register a callback function. This function is automatically invoked by the CARLA client (in a separate thread) whenever the sensor produces new data.

#### **2.3. Vehicle Control: PID Lane-Keeper**

A `PIDController` class is defined to manage vehicle steering. The control logic is executed within the main `while` loop at each simulation tick.

1.  **Error Calculation (Cross-Track Error):**
    * The vehicle's current location and the nearest `waypoint` on the road are fetched.
    * A target waypoint `next_wp` is projected 2.0 meters ahead of the current waypoint.
    * The vehicle's forward vector (`v_forward`) and the vector to the target waypoint (`target_vector`) are computed.
    * The **Cross-Track Error (CTE)** is calculated using the 2D cross-product of these two vectors:
        `cte = cross_product(v_forward, target_vector)`
        This yields a scalar value whose sign indicates the direction of the error (left/right) and whose magnitude represents the distance from the target.

2.  **PID Computation:**
    The CTE is fed into the PID controller (`pid.run_step(cross_track_error, 0.05)`), which calculates the necessary steering correction based on the formula:
    $$
    \text{output} = (K_p \cdot e(t)) + (K_i \cdot \int_0^t e(\tau)d\tau) + (K_d \cdot \frac{de(t)}{dt})
    $$
    * **$K_p = 1.0$ (Proportional):** Reacts to the current error.
    * **$K_i = 0.0$ (Integral):** Corrects for accumulated past error (disabled in this configuration).
    * **$K_d = 0.2$ (Derivative):** Dampens oscillations by reacting to the rate of change of the error.

3.  **Actuation:** The calculated `steer_correction` is clipped to a `[-1.0, 1.0]` range and applied to the vehicle using `carla.VehicleControl` along with a constant throttle of `0.4`.

---

### **3. Key Component Analysis**

#### **3.1. Sensor Data Processing**

The script uses a shared dictionary `sensor_data` to store the most recent frame from each sensor. The callbacks are responsible for processing the raw data and placing it in this dictionary.

* **RGB / Instance:** The raw 4-channel (BGRA) data is reshaped into a NumPy array.
* **Semantic Segmentation:** The raw data (with object tags) is converted using `carla.ColorConverter.CityScapesPalette` to produce a human-readable, color-coded segmentation map.
* **Depth:** The raw depth data (float) is converted using `carla.ColorConverter.LogarithmicDepth` to a visualizable 8-bit image, which provides better contrast for nearby objects.
* **DVS (Dynamic Vision Sensor):** This callback is unique. It processes a buffer of *events* rather than an image. It correctly reconstructs a 3-channel image by iterating through the events and setting pixel colors based on polarity (`pol`): **red** for positive events (brightness increase) and **blue** for negative (brightness decrease).
* **Optical Flow:** This callback uses the built-in `data.get_color_coded_flow()` method to convert the 2D flow vectors (x, y velocities) into a color-coded image (HSV) for visualization. It also includes a persistence feature, saving each flow image to the `carla_output/optical_flow` directory.

#### **3.2. Main Loop and Visualization**

The main `while` loop is the system's heartbeat. In each iteration, it:
1.  **Ticks** the simulation (`world.tick()`).
2.  **Updates** the spectator camera to follow the ego-vehicle.
3.  **Executes** the PID control logic to steer the vehicle.
4.  **Renders** the visualization. It fetches the latest images from the `sensor_data` dictionary and uses `np.concatenate` to tile them into a 2x3 grid.
5.  **Displays** the final tiled image using `cv2.imshow("All Cameras", ...)`.
6.  **Checks** for the 'q' key to break the loop.

#### **3.3. Resource Management**

A `finally` block is used to ensure proper cleanup, even if an error occurs.
1.  **Reverts to Asynchronous Mode:** This is a critical step. It returns the CARLA server to its default state, allowing other clients to connect and run normally.
2.  **Destroys Actors:** It iterates through the `sensors` and `actors` lists and sends a batch command to the client to destroy them, preventing "zombie" actors from accumulating in the simulator.
3.  **Closes Windows:** `cv2.destroyAllWindows()` closes the OpenCV display.

---

### **4. Results and Output**

The script successfully produces two primary outputs:

1.  **A real-time, 6-panel video feed** displaying the synchronized output from all attached sensors. This provides a comprehensive view of the vehicle's perceived environment. \
2.  **A vehicle that autonomously follows** the center of its lane on the CARLA map, driven by the PID controller.
3.  **A directory of saved images** (`carla_output/optical_flow`) containing the frame-by-frame optical flow data, which could be used for offline analysis.

### **5. Conclusion and Future Work**

This script serves as an excellent foundation for more advanced ADAS/AD research. It successfully integrates a complex sensor suite with a closed-loop control system in a deterministic, synchronous environment.

Potential areas for future development include:
* **Data Logging:** Expanding the save functionality from just optical flow to all sensor modalities (e.g., saving RGB images, semantic masks, and vehicle metadata) to create a training dataset.
* **Perception-Based Control:** Replacing the waypoint-based PID controller with a computer vision model. For example, using the semantic segmentation feed to detect lane lines and feeding the detected lane curvature into the PID controller.
* **Model Training:** Using the collected multi-modal data to train deep learning models for tasks like end-to-end driving, sensor fusion, or object detection.

