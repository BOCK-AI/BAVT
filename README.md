# BAVT - `all.py` Documentation
    
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




















