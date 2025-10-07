#!/usr/bin/env python

import carla
import random
import numpy as np
import cv2
import time

def gnss_callback(gnss_data):
    print(
        f"Lat: {gnss_data.latitude:.6f}, "
        f"Lon: {gnss_data.longitude:.6f}, "
        f"Alt: {gnss_data.altitude:.2f} m, "
        f"Frame: {gnss_data.frame}, "
        f"Timestamp: {gnss_data.timestamp:.3f} s"
    )

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def run_step(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

def main():
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    # Enable synchronous mode for stable readings
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  # 20 FPS
    world.apply_settings(settings)
    
    bplib = world.get_blueprint_library()
    vehicle_bp = random.choice(bplib.filter('vehicle'))
    spawn_points = world.get_map().get_spawn_points()
    
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
    actor_list.append(vehicle)  
    
    # GNSS Sensor
    gnss_bp = bplib.find('sensor.other.gnss')
    gnss_transform = carla.Transform(
        carla.Location(x=1.0, y=0.0, z=2.0),
        carla.Rotation(yaw=0.0, roll=90.0, pitch=0.0)
    )
    gnss_sensor = world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle)
    actor_list.append(gnss_sensor)

    gnss_sensor.listen(gnss_callback)
    
    pid = PIDController(Kp = 1.0, Ki = 0.0, Kd = 0.2)
    
    try:
        while True:
            
            # Advance the sim (required in sync mode)
            world.tick()  # <-- critical

            spectator = world.get_spectator()
            location = vehicle.get_transform().transform(carla.Location(x = -4, z = 2.5))
            rotation = vehicle.get_transform().rotation
            transform = carla.Transform(location, rotation)
            spectator.set_transform(transform)
        
            
            # Control
            waypoint = world.get_map().get_waypoint(vehicle.get_location(), project_to_road = True, lane_type=carla.LaneType.Driving)	
            next_wp = waypoint.next(2.0)[0]
            
            vehicle_transform = vehicle.get_transform()
            v_loc = vehicle_transform.location
            v_forward = vehicle_transform.get_forward_vector()
            
            target_vector = next_wp.transform.location - v_loc
            cross_track_error = np.cross(
                [v_forward.x, v_forward.y, 0], 
                [target_vector.x, target_vector.y, 0]
            )[2]
            
            steer_correction = pid.run_step(cross_track_error, 0.05)
            steer_correction = np.clip(steer_correction, -1.0, 1.0)
            
            control = carla.VehicleControl()
            control.throttle = 0.4
            control.steer = steer_correction
            vehicle.apply_control(control)

            # Small sleep to ease CPU thrash (optional)
            time.sleep(0.002)        
    
    
    
    
    
    finally:
        print("Stopping GNSS sensor...")
        gnss_sensor.stop()
        print("Destroying actors...")
        for actor in actor_list:
            actor.destroy()
            
        if world is not None:
            try:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
                print("Reverted to asynchronous mode.")
            except Exception:
                pass

        print("Cleanup complete.")

if __name__ == "__main__":
    main()
