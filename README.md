#  BAVT - `2d_lidar.py` & `convert_ply_to_jpg.py` Documentation

There are two documents here:
1. LiDAR Top-Down Visualization with PID Lane-Following (CARLA)
2. LiDAR PLY to JPEG Converter (Open3D)

---

### **Usage of this document** ( This Documentation is primarily focused on Linux )

##### Pre-Requisites:
We need **Python 3.8 to 3.10**. So please make sure you have that in your system. If you have the required python versions already, then the next sub-section can be skipped.

If you don't have python on your system, or have a different version, then please read the next Sub-section.


#### Python Installation (Recommend using python 3.10.11)
##### Mainly for Linux users: ( I am using Ubuntu 22.04 LTS ) | Python.org 

*FOR LINUX:*      
* It's best to use your distribution's package manager (like `apt`, `yum`, or `dnf`). For example, on Ubuntu, you might use   
  ```   
  sudo apt install python3.10
  ```    

*FOR WINDOWS:*
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

### Instructions to use `2d_lidar.py`:

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

1.  **Clone the Repository Branch**
    *FOLLOW FROM HERE ON:*    
    *FOR WINDOWS:*   
    It is easier for Linux users to use Git than Windows users.
    Windows User can simply download zip, extract its contents into the right folder:
    ```(C:/carla_0.9.15/PythonAPI/examples)```
    
    *FOR LINUX:*   
    Clone the `2-D-LIDAR` branch of this repository and navigate into the `BAVT` directory. Run these commands in your terminal or command prompt:
    ***IT IS NOT REQUIRED TO CLONE THE REPOSITORY MULTIPLE TIMES. PLEASE IGNORE THIS IF YOU HAVE ALREADY CLONED IT IN A PREVIOUS TUTORIAL.***
    ```    git clone https://github.com/BOCK-AI/BAVT/   ```       
    
    **Enter** the BAVT directory.       
    ``` 
    cd BAVT     
    ```   
    Now we must navigate to the right branch on terminal: this command is used for seeing All the existing Branches in the GutHub repo.
    ```       
    git branch -a
    ```    
    Now we switch to 2-D-LIDAR branch : (Switch branches using ```git checkout```)
    ```
    git checkout origin/2-D-LIDAR
    ```
    You can see all branches on your system using:
    ```
    git branch
    ```   
     
2.  **Install Dependencies**
    Install the required Python libraries.
    
    ```
    pip install carla random cv2 queue numpy time
    ```

3.  **Run CARLA Server**
    In a **separate terminal**, start the CARLA simulator (same as described in the "Running the CARLA Simulator" section above).  

      * **Linux:**  
        ```
        ./CarlaUE4.sh
        ```  
      * **Windows:**  
        ```
        CarlaUE4.exe
        ```  

4.  **Run the Python Script**  
    Once the CARLA server is running, execute the `2d_lidar.py` file in your first terminal:

    ```bash
    python3 2d_lidar.py
    ```

5.  **What You’ll See**  
      * A **Tesla Model 3** driving automatically in the CARLA world, following the road.
      * A live OpenCV window displaying the **RGB camera feed** from the vehicle's front camera.
      * A live Open3D Window ('Top Down Lidar'), it is fixed. It displays 2d point cloud data updated in real-time. (Green - 2D LIDAR)
      * In the simulator, you will be positioned to spectate behind the car. (This is done using the Spectator class of CARLA. Can be modified. I find it convenient.)
      
**Note**: Shortcuts for the **Open3D** window:   
Ctrl + '+' or Ctrl + '=' -> to make points bigger.  
Ctrl + '-' -> to make points smaller.   

6.  **How to Exit**  
    To stop the script and clean up the simulation, you can either:
        
      * Press **`CTRL + C`** in the terminal where `2d_lidar.py` is running.
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


## **Technical Report: CARLA 2D-LIDAR Streaming with PID-Based Lane Following**

It spawns a car with a LiDAR sensor, makes it follow lanes through waypoints on the map, and visualizes the LiDAR data in real-time. 
The code has three main parts: **LiDAR processing**, a **PID controller** for driving, and a **main simulation loop**.

### LiDAR Data Processing
-   A **LiDAR sensor** is attached to the car, which scans the environment with lasers.
-   The `process_lidar_frames` function acts as a listener. Every time the sensor gets a reading, this function is triggered.
-   It converts the raw 3D point cloud data into a 2D **top-down image** (like a bird's-eye view map).
-   To avoid slowing down the simulation, it pushes this image into a **queue**. The main loop can then grab images from this queue whenever it's ready. If the queue is full, the new frame is simply dropped.

### PID Controller for Driving
-   A **PID Controller** is a simple but powerful control algorithm used to minimize errors. Think of it like how you steer a car:
    -   **Proportional (P):** If you're far from the lane center, you turn the wheel sharply. The correction is proportional to the error.
    -   **Integral (I):** If you're consistently a little bit off to one side (like in a long curve), you start holding the wheel at a slight angle to correct this persistent error.
    -   **Derivative (D):** As you approach the center of the lane, you start straightening the wheel to avoid overshooting. This predicts future error.
-   In the script, the "error" is the **cross-track error**, which is the car's distance from the center of its lane. The PID controller calculates the necessary steering angle to reduce this error to zero.

### Main Simulation Loop
-   The script connects to the CARLA server and sets it to **synchronous mode**. This means the simulation only advances one step at a time when the script tells it to (`world.tick()`), ensuring the sensor data and vehicle commands are perfectly aligned.
-   In each step of the main `while` loop, the script does the following:
    1.  **Advances the simulation** one frame.
    2.  **Calculates the cross-track error** by finding the path ahead and seeing how far the car is from it.
    3.  **Feeds the error** to the PID controller to get a steering correction.
    4.  **Applies control** to the vehicle (throttle and the calculated steering).
    5.  **Checks the queue** for a new LiDAR image and displays it on the screen using OpenCV.
-   The loop continues until you press the 'q' key.

### Cleanup
-   When the script is stopped, the `finally` block runs. It's very important as it **destroys all the created actors** (car, sensor) and **switches the CARLA server back to normal asynchronous mode**. This prevents the server from being left in a frozen state.



