#!/usr/bin/env python
import carla
import random
import cv2
import queue # Note: queue is imported but not used in this version, could be used for advanced data handling
import numpy as np
import time

# --- Constants ---
# These were in your original code but not used for the IMU.
# They are good to keep if you add a LiDAR or camera later.
IMG_SIZE = 500
CENTER = IMG_SIZE // 2
SCALE = 5.0  # pixels per meter
MAX_RANGE = 50  # meters
IM_WIDTH = 640
IM_HEIGHT = 480

# --- Sensor Callback ---

def imu_callback(imu_data):
    """
    Callback function for the IMU sensor.
    This function is called by the CARLA simulator every time a new
    IMU measurement is available.
    """
    accel = imu_data.accelerometer
    gyro = imu_data.gyroscope
    
    # CARLA's 'compass' is a single float representing heading in degrees (relative to North)
    compass_heading = imu_data.compass * 180/3.14159265359
    
    # Print the values neatly on a single line.
    # \r (carriage return) moves the cursor to the start of the line without a newline.
    # 'end=""' prevents the print function from adding its own newline.
    # The trailing spaces help clear any leftover characters from the previous print.
    print(f"\r[IMU] Accel: (x={accel.x: 8.2f}, y={accel.y: 8.2f}, z={accel.z: 8.2f}) | "
          f"Gyro: (x={gyro.x: 8.2f}, y={gyro.y: 8.2f}, z={gyro.z: 8.2f}) | "
          f"Compass: {compass_heading: 6.2f}°   ", end="")

# --- PID Controller ---

class PIDController:
    """
    A simple Proportional-Integral-Derivative (PID) controller.
    """
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0
    
    def run_step(self, error, dt):
        """
        Calculate the control output based on the error and time delta.
        """
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

# --- Main Execution ---

def main():
    actors = []
    client = None
    world = None

    try:
        # 1. Connect to client and get world
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # 2. Set synchronous mode for physics and sensor consistency
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        world.apply_settings(settings)

        blueprints = world.get_blueprint_library()

        # 3. Spawn vehicle
        vehicle_bp = blueprints.find("vehicle.tesla.model3")
        spawn_point = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actors.append(vehicle)
        print(f"Spawned actor '{vehicle.type_id}' at {spawn_point.location}")

        # 4. Place spectator behind the vehicle
        spectator = world.get_spectator()
        
        def update_spectator():
            vehicle_transform = vehicle.get_transform()
            # Place spectator 4m behind and 2.5m above the vehicle
            offset = carla.Location(x=-4, z=2.5)
            spectator_loc = vehicle_transform.transform(offset)
            spectator.set_transform(carla.Transform(spectator_loc, vehicle_transform.rotation))
        
        update_spectator() # Set initial position

        # 5. Setup IMU sensor
        imu_bp = blueprints.find("sensor.other.imu")
        # Set sensor to tick at the same rate as the world
        imu_bp.set_attribute("sensor_tick", str(settings.fixed_delta_seconds))
        
        # Spawn IMU and attach to vehicle
        imu_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.8)) # Centered, slightly above vehicle root
        imu = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
        actors.append(imu)
        
        # Register the callback function
        imu.listen(imu_callback)

        # 6. Setup PID controller for steering
        pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.2)
        
        # This window is from your original code, used for the exit key 'q'
        cv2.namedWindow("CARLA Control", cv2.WINDOW_AUTOSIZE)

        # 7. Prime the simulation
        for _ in range(2):
            world.tick()

        # 8. Main simulation loop
        while True:
            # Advance simulation (required in synchronous mode)
            world.tick()

            # Update spectator to chase the vehicle
            update_spectator()

            # --- Simple waypoint following logic ---
            vehicle_transform = vehicle.get_transform()
            waypoint = world.get_map().get_waypoint(vehicle.get_location(), project_to_road=True,
                                                    lane_type=carla.LaneType.Driving)
            
            # Get a waypoint 2 meters ahead
            next_wp = waypoint.next(2.0)[0] 
            
            v_loc = vehicle_transform.location
            v_forward = vehicle_transform.get_forward_vector()
            target_vector = next_wp.transform.location - v_loc

            # Calculate steering error
            # Cross product [v_forward] x [target_vector]
            # The 'z' component tells us if the target is to the left (negative) or right (positive)
            cross_track_error = np.cross(
                [v_forward.x, v_forward.y, 0],
                [target_vector.x, target_vector.y, 0]
            )[2]

            # Get PID correction
            steer_correction = pid.run_step(cross_track_error, settings.fixed_delta_seconds)
            steer_correction = np.clip(steer_correction, -1.0, 1.0)

            # Apply control
            control = carla.VehicleControl()
            control.throttle = 0.4
            control.steer = steer_correction
            vehicle.apply_control(control)

            # Check for 'q' key to exit
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("\nExiting loop.")
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user (Ctrl+C)")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        # --- Cleanup ---
        print("\nCleaning up actors and settings...")
        if imu is not None:
            try:
                imu.stop()
                print("IMU sensor stopped.")
            except Exception as e:
                print(f"Error stopping IMU: {e}")

        for a in actors:
            try:
                a.destroy()
            except Exception as e:
                print(f"Error destroying actor {a.type_id}: {e}")
        print(f"Destroyed {len(actors)} actors.")

        if client is not None and world is not None:
            try:
                # Revert to asynchronous mode
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
                print("Reverted to asynchronous mode.")
            except Exception as e:
                print(f"Error reverting world settings: {e}")

        cv2.destroyAllWindows()
        print("All Cleaned Up! Exiting.")

if __name__ == "__main__":
    main()
