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

Firstly, run this command, which makes sure you clone the repo into the right folder.
```
cd carla_0.9.15/PythonAPI/examples
```
Now, we are ready to clone the repository.

1.  **Clone the Repository**
    Clone the `3-D-LIDAR` branch of this repository and navigate into the `BAVT` directory. Run these commands in your terminal or command prompt:

    ```bash
    git clone -b 3-D-LIDAR https://github.com/BOCK-AI/BAVT/
    cd BAVT
    ```

2.  **Install Dependencies**
    Install the required Python libraries.

    ```bash
    pip install carla random cv2 queue numpy time
    ```

3.  **Run CARLA Server**
    In a **separate terminal**, start the CARLA simulator (same as described in the "Running the CARLA Simulator" section above).

      * **Linux:**
        ```bash
        ./CarlaUE4.sh
        ```
      * **Windows:**
        ```cmd
        CarlaUE4.exe
        ```

4.  **Run the Python Script**
    Once the CARLA server is running, execute the `lidar3d_and_radar.py` file in your first terminal:

    ```bash
    python3 lidar3d_and_radar.py
    ```

5.  **What You’ll See**

      * A **Tesla Model 3** driving automatically in the CARLA world, following the road.
      * A live OpenCV window displaying the **RGB camera feed** from the vehicle's front camera.
      * If any error occurs, it might be due to incorrect dependencies or other configuration issues.

6.  **How to Exit**
    To stop the script and clean up the simulation, you can either:

      * Press **`CTRL + C`** in the terminal where `lidar3d_and_radar.py` is running.
      * Press the **`q`** key while the OpenCV window is active.
        All actors (vehicle + camera) are destroyed automatically.

-----
**Some Handy Terminal Shortcuts:**
 - ctrl + shift + v - paste
 - ctrl + shift + c - copy
 - tab - Autocomplete, if enabled.
 - ctrl + shift + t - for a new tab in the current terminal window.
 - ctrl + shift + n - for a new terminal window.

-----

***Instructions to use lidar3d_and_radar.py:**



































