#!/usr/bin/env python

import carla
import random

def gnss_callback(gnss_data):
    print(
        f"Lat: {gnss_data.latitude:.6f}, "
        f"Lon: {gnss_data.longitude:.6f}, "
        f"Alt: {gnss_data.altitude:.2f} m, "
        f"Frame: {gnss_data.frame}, "
        f"Timestamp: {gnss_data.timestamp:.3f} s"
    )

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
    
    try:
        while True:
            spectator = world.get_spectator()
            location = vehicle.get_transform().transform(carla.Location(x = -4, z = 2.5))
            rotation = vehicle.get_transform().rotation
            transform = carla.Transform(location, rotation)
            spectator.set_transform(transform)
            world.tick()
    
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
