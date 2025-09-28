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
* **Navigate to the CARLA directory.** Assuming you extracted it to your Home directory or C: (C drive).  <br>
       * **Linux:**    
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
    Clone the `2-D-LIDAR` branch of this repository and navigate into the `BAVT` directory. Run these commands in your terminal or command prompt:
    
    ***IF THE PREVIOUS TUTORIALS WERE FOLLOWED IN ORDER, THEN PLEASE SKIP CLONING THE REPO AGAIN:***    
    ``` 
    cd BAVT     
    ```

    *FOLLOW FROM HERE ON:*    
    *FOR WINDOWS:*   
    It is easier for Linux users to use Git than Windows users.
    Windows User can simply download zip, extract its contents into the right folder:
    ```(C:/carla_0.9.15/PythonAPI/examples)```
    
    *FOR LINUX:*   
    Clone the `3-D-LIDAR` branch of this repository and navigate into the `BAVT` directory. Run these commands in your terminal or command prompt:      
    ```
    git clone https://github.com/BOCK-AI/BAVT/
    cd BAVT
    ```
    Now we must navigate to the right branch on terminal: this command is used for seeing All the existing Branches in the GutHub repo.
    ```
    git branch -a
    ```
    Now we switch to 3-D-LIDAR branch : (Switch branches using ```git checkout```)
    ```
    git checkout origin/3-D-LIDAR
    ```
    You can see all branches on your system using:
    ```
    git branch
    ```   
     
3.  **Install Dependencies**
    Install the required Python libraries.
    
    ```
    pip install carla random cv2 queue numpy time
    ```

4.  **Run CARLA Server**
    In a **separate terminal**, start the CARLA simulator (same as described in the "Running the CARLA Simulator" section above).  

      * **Linux:**  
        ```
        ./CarlaUE4.sh
        ```  
      * **Windows:**  
        ```
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
      * A live Open3D Window ('Top Down Lidar'), it is fixed. It displays 2d point cloud data updated in real-time. (Green - 2D LIDAR)
      * In the simulator, you will be positioned to spectate behind the car. (This is done using the Spectator class of CARLA. Can be modified. I find it convenient.)
      * If any error occurs, it might be due to incorrect dependencies or other configuration issues.

7.  **How to Exit**  
    To stop the script and clean up the simulation, you can either:

      * Press **`CTRL + C`** in the terminal where `rgb_cam.py` is running.
      * Press the **`q`** key while the OpenCV window is active.
        All actors (vehicle + camera) and windows are destroyed automatically.
b
-----


