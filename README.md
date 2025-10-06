# BAVT - `gnss.py` Documentation

#### NOTE: For the more Basic Documentation (Installation of carla, python, etc.), refer either of the 3-D-LIDAR, 2-D-LIDAR, RGB_CAM branches.

---

### Instructions to use `gnss.py`:

Firstly, run this command, which makes sure you clone the repo into the right folder.   
Start from HOME directory (```~/```) in your linux terminal.    
For Windows, Start from C: Folder in cmd. I am assuming you have created the BAVT Folder already. If not, please refer Tutorial 1 - Basic Intro to Carla OR 3-D-LIDAR branch README Doc

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
    It is easier for Linux users to use Git than Windows users. Thus, I suggest downloading zip and extracting the contents into te BAVT folder we will create below.                      
    
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
    pip install carla 
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
    Once the CARLA server is running, execute the `lidar3d_and_radar.py` file in your *first* terminal: 

    ```bash
    python3 lidar3d_and_radar.py
    ```

7.  **What You’ll See**

      * A **Tesla Model 3** driving automatically in the CARLA world, following the road.
      * A live OpenCV window displaying the **RGB camera feed** from the vehicle's front camera.
      * A live Open3D Window ('Carla Lidar/Radar') which is an interactive 3D window where you can use your mouse to rotate, pan, and zoom the view. It displays two overlapping point clouds updated in real-time. (Lidar - yellow gradient points, Radar - Green points)
      * In the simulator, you will be positioned to spectate behind the car. (This is done using the Spectator class of CARLA. Can be modified. I find it convenient.)
      
**Note**: Shortcuts for the **Open3D** window:   
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
