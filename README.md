# BAVT - `gnss.py` Documentation

#### NOTE: For the more Basic Documentation (Installation of carla, python, etc.), refer to the READMEs under either of the 3-D-LIDAR, 2-D-LIDAR, RGB_CAM branches.

---

### Instructions to use `gnss.py`:

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
    Clone the `GNSS-sensor` branch of this repository and navigate into the `BAVT` directory. Run these commands in your terminal or command prompt:      
    Here, I am assuming you have cloned BAVT 'main' branch from previous **3-D-LIDAR** README Documentation.
    Now we must navigate to the right branch on terminal: This command is used for seeing All the existing Branches in the GitHub repo.
    ```
    git branch -a
    ```
    Now we switch to GNSS-sensor branch : (Switch branches using ```git checkout```)
    ```
    git checkout GNSS-sensor
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
    Once the CARLA server is running, execute the `gnss.py` file in your *first* terminal: 

    ```bash
    python3 gnss.py
    ```

7.  **What You’ll See**

      * A **Tesla Model 3** driving automatically in the CARLA world, following the road.
      * Live Location readings from simulator in terminal every 0.05 seconds. This includes: Latitude, Longitude, Altitude, Frame, Timestamp.
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
     
## Technical Documentation: CARLA Autonomous Agent with GNSS Sensor

This document provides a technical analysis of a Python script that implements an autonomous lane-following vehicle in the CARLA simulator. The agent uses a GNSS sensor for localization and a PID controller to compute steering commands based on waypoint navigation. The simulation is executed in synchronous mode to ensure deterministic behavior.
    
***
### ## `gnss_callback(gnss_data)`

* **Purpose**: This function is a callback handler designed to process data packets from the CARLA GNSS sensor. It is invoked by the simulator's `listen()` method each time a new GNSS measurement is available.

* **Parameters**:
    * `gnss_data` (`carla.GnssMeasurement`): An object containing the sensor data for a specific timestamp. It includes attributes for latitude, longitude, altitude, the simulation frame number, and the timestamp.

* **Execution**: The function's sole action is to print a formatted string to the standard output. It extracts the **latitude**, **longitude**, **altitude**, **frame**, and **timestamp** from the `gnss_data` object, providing a real-time log of the vehicle's geographic position as determined by the simulated GNSS device.

***
### ## `PIDController` (Class)

* **Purpose**: This class implements a Proportional-Integral-Derivative (PID) controller. It is a feedback control mechanism that calculates a corrective output to guide a system towards a desired setpoint by minimizing the error between the current state and the target state.

#### **`__init__(self, Kp, Ki, Kd)`**
* **Purpose**: The constructor method initializes the PID controller's gain parameters.
* **Parameters**:
    * `Kp` (`float`): **Proportional gain**. Dictates the response to the current error.
    * `Ki` (`float`): **Integral gain**. Addresses accumulated past errors to correct steady-state offset.
    * `Kd` (`float`): **Derivative gain**. Responds to the rate of error change to dampen oscillations.

#### **`run_step(self, error, dt)`**
* **Purpose**: This method computes a single control output based on the current system error. It is designed to be called once per simulation tick.
* **Parameters**:
    * `error` (`float`): The current error value. In this script, this is the **cross-track error**—a signed float representing the vehicle's lateral distance from the desired path.
    * `dt` (`float`): The fixed time delta of the simulation tick (`0.05s`).
* **Execution**:
    1.  **Integral Term Calculation**: The integral term is updated by accumulating the product of the current error and the time delta (`self.integral += error * dt`).
    2.  **Derivative Term Calculation**: The derivative term is computed as the rate of change of the error: `(error - self.prev_error) / dt`.
    3.  **State Update**: The current error is stored in `self.prev_error` for the next iteration's derivative calculation.
    4.  **Output Computation**: The final control output (steering value) is calculated as the weighted sum of the three terms: `Kp * error + Ki * integral + Kd * derivative`.
* **Returns**: A `float` representing the calculated steering correction.

***
### ## `main()`

* **Purpose**: The main function that serves as the entry point and orchestrator for the entire simulation. It handles setup, the main control loop, and cleanup.

* **Execution**:
    1.  **Client and World Setup**: Establishes a connection to the CARLA server and configures the world to operate in **synchronous mode** with a fixed time step (`fixed_delta_seconds = 0.05`). This is crucial for ensuring that the PID controller and physics engine operate in a stable, deterministic manner.
    2.  **Actor Spawning**: Spawns a random vehicle model at a predefined location. It then spawns a `sensor.other.gnss` and attaches it to the vehicle. All created actors are appended to `actor_list` for robust cleanup. The GNSS sensor is set to call `gnss_callback` with its data.
    3.  **Control Loop**: The script enters an infinite `while True` loop that constitutes the agent's core logic. In each iteration:
        * `world.tick()`: Advances the simulation by one discrete time step.
        * **Spectator Camera**: Updates the spectator's transform to follow the vehicle in a chase-cam view.
        * **Waypoint Navigation**: Determines the vehicle's current waypoint and identifies a target waypoint 2.0 meters ahead along the road.
        * **Error Calculation**: The **cross-track error** is calculated using the vector cross product (`np.cross`) of the vehicle's forward vector and the vector to the target waypoint. This yields a signed scalar indicating the direction and magnitude of deviation from the path.
        * **PID Control**: The calculated error is fed into the `pid.run_step()` method to obtain a steering correction, which is then clamped to the valid range of `[-1.0, 1.0]`.
        * **Vehicle Control**: A `carla.VehicleControl` object is created with a constant throttle (`0.4`) and the calculated steering correction, which is then applied to the vehicle.
    4.  **Cleanup**: The `finally` block ensures a clean shutdown, even if errors occur. It stops the GNSS sensor, iterates through `actor_list` to destroy all actors, and reverts the world to asynchronous mode.

