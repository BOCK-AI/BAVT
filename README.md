# BAVT - `depth_cam.py` Documentation

#### NOTE: For the more Basic Documentation (Installation of carla, python, etc.), refer to the READMEs under either of the 3-D-LIDAR, 2-D-LIDAR, RGB_CAM branches.

---

### Instructions to use `depth_cam.py`:

Firstly, run this command, to be in the right folder.   
Start from HOME directory (```~/```) in your linux terminal.    
For Windows, Start from C: Folder in cmd. I am assuming you have created the BAVT Folder already. If not, please refer Tutorial 1 - Basic Intro to Carla OR 3-D-LIDAR branch README Doc.

* The *first* terminal   
  *FOR WINDOWS:* (CMD)     
  ```
  cd C:/carla_0.9.15/PythonAPI/examples/BAVT
  ```
  *FOR LINUX:* (Terminal)
  ```
  cd carla_0.9.15/PythonAPI/examples
  ```
  ***(Clone only the first time. Do not need to clone repeatedly. Once cloned, we can just switch between branches as convenient.)***
Now, we are ready to clone the repository.

1.  **Clone the Repository and switch branch**

    *FOR WINDOWS:*    
    It is easier for Linux users to use Git than Windows users. Thus, I suggest downloading zip and extracting the contents into the BAVT folder we will create below.                      
    
    Download the zip, extract its contents into the below BAVT folder:           
    ```
    (C:/carla_0.9.15/PythonAPI/examples/BAVT)
    ```
            
    *FOR LINUX:*   
    Clone the `Depth-Sensing-camera` branch of this repository and navigate into the `BAVT` directory. Run these commands in your terminal or command prompt:      
    Here, I am assuming you have cloned BAVT 'main' branch from previous **3-D-LIDAR** README Documentation.
    Now we must navigate to the right branch on terminal: This command is used for seeing All the existing Branches in the GitHub repo.
    ```
    git branch -a
    ```
    Now we switch to Depth-Sensing-camera branch : (Switch branches using ```git checkout```)
    ```
    git checkout Depth-Sensing-camera
    ```
    You can see all branches on your system using:
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
    Once the CARLA server is running, execute the `depth_cam.py` file in your *first* terminal: 

    ```bash
    python3 depth_cam.py
    ```

7.  **What You’ll See**

      * A **Tesla Model 3** driving automatically in the CARLA world, following the road.
      * An OpenCV window showing the Logarithmic Depth output is seen (Black - closer, White - farther), titled "Depth sensing camera".
      * In the simulator, you will be positioned to spectate behind the car. (This is done using the Spectator class of CARLA. Can be modified. I find it convenient.)
  
     
6.  **How to Exit**
    To stop the script and clean up the simulation, you can either:

      * Press **`CTRL + C`** in the terminal where `lidar3d_and_radar.py` is running.
      All actors (vehicle + camera) and windows are destroyed automatically.

-----

**Some Handy Terminal Shortcuts:**
 - ctrl + shift + v - paste
 - ctrl + shift + c - copy
 - tab - Autocomplete, if enabled.
 - ctrl + shift + t - for a new tab in the current terminal window.
 - ctrl + shift + n - for a new terminal window.
               
-----
    
## Technical Report: CARLA Autonomous Agent with Depth Sensing Camera

This report provides a technical analysis of a Python script designed to implement an autonomous lane-following vehicle in the CARLA simulator. The agent utilizes a PID controller for steering logic and a depth camera for environmental perception, visualized via OpenCV.

***
### `depth_callback(image, data_dict)`

* **Purpose**: This function serves as the callback handler for the depth camera sensor. It is invoked asynchronously by the CARLA simulator whenever a new depth frame is generated.

* **Parameters**:
    * `image` (`carla.Image`): The sensor data object provided by the CARLA `listen()` method. It contains the raw image data and metadata like height and width.
    * `data_dict` (`dict`): A mutable dictionary passed by reference from the main thread, used to share sensor data.

* **Execution**:
    1.  `image.convert(carla.ColorConverter.LogarithmicDepth)`: The primary operation of this function. It transforms the raw depth data, which is stored as a float representing distance in meters, into an 8-bit logarithmic scale. This conversion is crucial for visualization, as it maps the wide range of depth values into a limited grayscale range (0-255) that can be displayed as an image. Far-away objects appear darker, while closer objects appear lighter.
    2.  `np.reshape(...)`: The raw data buffer from the `image` object is copied and reshaped into a 4-channel NumPy array with dimensions `(height, width, 4)`. This format is compatible with OpenCV for rendering.
    3.  `data_dict['depth_image'] = ...`: The processed NumPy array is assigned to the `depth_image` key in the shared dictionary, making it accessible to the main loop for display.

***
### `PIDController` (Class)

* **Purpose**: This class implements a Proportional-Integral-Derivative (PID) controller, a standard feedback control loop mechanism widely used in industrial control systems and robotics. Its objective is to minimize the error between a measured process variable and a desired setpoint by adjusting a control output.

#### **`__init__(self, Kp, Ki, Kd)`**
* **Purpose**: The constructor initializes the controller's parameters.
* **Parameters**:
    * `Kp` (`float`): The **Proportional gain**. It determines the influence of the current error on the output.
    * `Ki` (`float`): The **Integral gain**. It determines the influence of the sum of past errors, used to eliminate steady-state offsets.
    * `Kd` (`float`): The **Derivative gain**. It determines the influence of the rate of change of the error, used to dampen oscillations and predict future error.

#### **`run_step(self, error, dt)`**
* **Purpose**: Calculates a single control output based on the current error. This method is called once per simulation tick.
* **Parameters**:
    * `error` (`float`): The current process error. In this script, it represents the **cross-track error**—the lateral distance of the vehicle from the target waypoint path.
    * `dt` (`float`): The delta time, or the time elapsed since the last step. In this synchronous simulation, it is a fixed value (`0.05s`).
* **Execution**:
    1.  **Integral Term**: `self.integral += error * dt`. The current error is multiplied by the time step and added to an accumulator. This term grows over time if a persistent error exists.
    2.  **Derivative Term**: `derivative = (error - self.prev_error) / dt`. The rate of change of the error is calculated by finding the difference between the current and previous error and dividing by the time step.
    3.  **State Update**: `self.prev_error = error`. The current error is stored for the next iteration's derivative calculation.
    4.  **Output Calculation**: The final control output (steering angle) is computed as the weighted sum of the three terms: `self.Kp * error + self.Ki * self.integral + self.Kd * derivative`.
* **Returns**: A `float` representing the calculated control output.

***
### `main()`

* **Purpose**: The main execution function that orchestrates the entire simulation lifecycle.

* **Execution**:
    1.  **Initialization**: It establishes a connection with the CARLA server and retrieves the `world` object.
    2.  **Synchronous Mode Configuration**: It configures the simulator to run in synchronous mode with a fixed time step (`fixed_delta_seconds = 0.05`). This is a critical step that ensures deterministic behavior by giving the client script full control over the simulation's progression via `world.tick()`.
    3.  **Actor Spawning**: It spawns a `vehicle.tesla.model3` at a predefined spawn point and a `sensor.camera.depth` attached to it. All spawned actors are tracked in `actor_list` for later cleanup.
    4.  **Control Loop**: The `while(True)` loop constitutes the core of the agent's operation. On each iteration, it performs the following sequence:
        * `world.tick()`: Sends a signal to the server to advance the simulation by one time step. This triggers sensor data generation and physics updates.
        * **Waypoint Navigation**: It retrieves the vehicle's current location, finds the nearest waypoint on the map, and determines the next target waypoint 2.0 meters ahead.
        * **Error Calculation**: It calculates the **cross-track error** by computing the vector cross product between the vehicle's forward vector and the target vector to the next waypoint. The Z-component of the resulting vector gives a signed value representing the vehicle's lateral deviation from the path.
        * **PID Control**: It calls `pid.run_step()` with the cross-track error to get a steering correction value. This value is clamped to the `[-1.0, 1.0]` range.
        * **Vehicle Control**: It applies the calculated steering value and a constant throttle (`0.4`) to the vehicle using a `carla.VehicleControl` object.
        * **Visualization**: It displays the latest depth image (retrieved from `sensor_data`) in an OpenCV window.
    5.  **Termination and Cleanup**: The loop breaks when the 'q' key is pressed. The script then proceeds to destroy all spawned actors, revert the simulation to asynchronous mode, and close all OpenCV windows to ensure a clean exit.

