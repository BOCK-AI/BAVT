#!/usr/bin/env python

import carla
import random
import math
import cv2
import numpy as np
import time
import open3d as o3d
from matplotlib import cm

# --- Colormaps ---
VIRIDIS = np.array(cm.get_cmap('plasma').colors)  # <-- fix name (keep plasma if you like)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

COOL_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
COOL = np.array(cm.get_cmap('winter')(COOL_RANGE))[:, :3]

IM_WIDTH = 640
IM_HEIGHT = 480

def add_open3d_axis(vis):
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    axis.lines = o3d.utility.Vector2iVector(np.array([[0, 1], [0, 2], [0, 3]]))
    axis.colors = o3d.utility.Vector3dVector(np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    vis.add_geometry(axis)

def lidar_callback(point_cloud, point_list):
    data = np.frombuffer(point_cloud.raw_data, dtype=np.float32).reshape(-1, 4).copy()
    intensity = data[:, -1]
    # Guard log domain
    eps = 1e-6
    intensity = np.clip(intensity, eps, None)
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))

    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])
    ]

    points = data[:, :-1]
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)


def radar_callback(meas, point_list):
    radar_data = np.zeros((len(meas), 4), dtype=np.float32)

    for i, d in enumerate(meas):
        # CARLA radar: depth forward, azimuth around Z, altitude up/down
        x = d.depth * math.cos(d.altitude) * math.cos(d.azimuth)
        y = d.depth * math.cos(d.altitude) * math.sin(d.azimuth)  # <-- fix extra cos(altitude)
        z = d.depth * math.sin(d.altitude)
        radar_data[i, :] = [x, y, z, d.velocity]

    intensity = np.abs(radar_data[:, -1])
    eps = 1e-6
    intensity = np.clip(intensity, eps, None)
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))

    int_color = np.c_[
        np.interp(intensity_col, COOL_RANGE, COOL[:, 0]),
        np.interp(intensity_col, COOL_RANGE, COOL[:, 1]),
        np.interp(intensity_col, COOL_RANGE, COOL[:, 2])
    ]

    points = radar_data[:, :-1]
    points[:, 0] = -points[:, 0]  # mirror X if you prefer left-handed view
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)

def camera_callback(image, data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

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
    actors = []
    sensors = []
    client = None
    world = None
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # --- Synchronous mode ---
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        world.apply_settings(settings)

        bp = world.get_blueprint_library()
        vehicle_bp = bp.find('vehicle.tesla.model3')

        spawn_points = world.get_map().get_spawn_points()
        vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
        actors.append(vehicle)
        print(f"Spawned {vehicle.type_id}")
        
        spectator = world.get_spectator()
        location = vehicle.get_transform().transform(carla.Location(x = -4, z = 2.5))
        rotation = vehicle.get_transform().rotation
        transform = carla.Transform(location, rotation)
        spectator.set_transform(transform)
        
        # --- LIDAR ---
        lidar_bp = bp.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '100.0')
        lidar_bp.set_attribute('noise_stddev', '0.1')
        lidar_bp.set_attribute('upper_fov', '15.0')
        lidar_bp.set_attribute('lower_fov', '-25.0')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('rotation_frequency', '20')
        lidar_bp.set_attribute('points_per_second', '500000')
        lidar_bp.set_attribute('sensor_tick', '0.05')  # <-- match sync tick

        lidar = world.spawn_actor(
            lidar_bp,
            carla.Transform(carla.Location(z=2.0)),
            attach_to=vehicle
        )
        sensors.append(lidar)
        print("LIDAR spawned.")

        # --- RADAR ---
        radar_bp = bp.find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', '30.0')
        radar_bp.set_attribute('vertical_fov', '30.0')
        radar_bp.set_attribute('range', '30')
        radar_bp.set_attribute('points_per_second', '10000')
        radar_bp.set_attribute('sensor_tick', '0.05')  # <-- match sync tick

        radar = world.spawn_actor(
            radar_bp,
            carla.Transform(carla.Location(z=2.0)),
            attach_to=vehicle
        )
        sensors.append(radar)
        print("RADAR spawned.")

        # --- RGB Camera ---
        cam_bp = bp.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(IM_WIDTH))   # <-- ensure sizes
        cam_bp.set_attribute('image_size_y', str(IM_HEIGHT))
        cam_bp.set_attribute('fov', '90')
        cam_bp.set_attribute('sensor_tick', '0.05')

        camera = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(z=2.5, x=-3.0), carla.Rotation(0, 0, 0)),
            attach_to=vehicle
        )
        sensors.append(camera)

        # --- Data holders ---
        point_list = o3d.geometry.PointCloud()
        radar_list = o3d.geometry.PointCloud()

        image_w = cam_bp.get_attribute("image_size_x").as_int()
        image_h = cam_bp.get_attribute("image_size_y").as_int()
        camera_data = {"image": np.zeros((image_h, image_w, 4), dtype=np.uint8)}

        lidar.listen(lambda data: lidar_callback(data, point_list))
        radar.listen(lambda data: radar_callback(data, radar_list))
        camera.listen(lambda image: camera_callback(image, camera_data))

        # --- Windows ---
        cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)

        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='Carla Lidar/Radar', width=960, height=540, left=270)
        opt = vis.get_render_option()
        opt.background_color = np.array([0.05, 0.05, 0.05], dtype=np.float32)
        opt.point_size = 1.0
        opt.show_coordinate_frame = True
        add_open3d_axis(vis)

        # Prime with a couple of ticks so sensors start streaming
        for _ in range(3):
            world.tick()

        added = False
        
        pid = PIDController(Kp = 1.0, Ki = 0.0, Kd = 0.2)
        
        while True:
            # Advance the sim (required in sync mode)
            world.tick()  # <-- critical

            if not added:
                vis.add_geometry(point_list)
                vis.add_geometry(radar_list)
                added = True

            vis.update_geometry(point_list)
            vis.update_geometry(radar_list)
            vis.poll_events()
            vis.update_renderer()
            
            
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
            
            cv2.imshow('RGB Camera', camera_data['image'])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Small sleep to ease CPU thrash (optional)
            time.sleep(0.002)

    finally:
        print("destroying actors!")
        # Stop sensors first
        for s in sensors:
            try:
                s.stop()
            except Exception:
                pass
        # Destroy all
        for actor in sensors + actors:
            try:
                actor.destroy()
            except Exception:
                pass

        if world is not None:
            try:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
                print("Reverted to asynchronous mode.")
            except Exception:
                pass

        cv2.destroyAllWindows()
        print("All Cleaned Up!")

if __name__ == "__main__":
    main()

