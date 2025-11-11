'''
#!/usr/bin/env python

import carla
import numpy as np
import time
import open3d as o3d

class RadarData:
    def __init__(self):
        self.points = np.empty((0, 3))
# To get a numpy [[vel, azimuth, altitude, depth],...[,,,]]:

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

# Creating object of the class above.
sensor_data = RadarData() 
    
# NOTE that 'data.raw_data' arg is a LIST. 
def radar_callback(data):
    # sensor_data.raw_data = data.raw_data
    points = []
    for detection in data:
        x = detection.depth * np.cos(detection.altitude) * np.cos(detection.azimuth)
        y = detection.depth * np.cos(detection.altitude) * np.sin(detection.azimuth)
        z = detection.depth * np.sin(detection.altitude)
        points.append([x,y,z])
    sensor_data.points = np.array(points) 
    
    # Added from lidar3d_and_radar.py
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

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        original_settings = world.get_settings()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find('vehicle.tesla.model3')
        spawn_point = world.get_map().get_spawn_points()[0]
        
        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)
        print("Vehicle spawned.")
        
        radar_bp = blueprint_library.find("sensor.other.radar")
        transform = carla.Transform(carla.Location(x = 0, y = 0 , z = 0))
        radar = world.spawn_actor(radar_bp, transform, attach_to = vehicle)
        
        radar.listen(lambda data: radar_callback(data))
        print("Radar sensor listening.")
        
        pid = PIDController(Kp=0.8, Ki=0.0, Kd=0.1)
        
        
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="CARLA Radar Visualization", width=960, height=540, left = 270)
        opt = vis.get_render_option()
        opt.background_color = np.array([0.05, 0.05, 0.05], dtype=np.float32)
        opt.point_size = 1.0
        opt.show_coordinate_frame = True
        add_open3d_axis(vis)
        
        """
        pcd = o3d.geometry.PointCloud()
        pcd.paint_uniform_color([1,0,0]) # RED points
        vis.add_geometry(pcd)
        """
        
        view_control = vis.get_view_control()
        
        
        # STATIC OPEN3D CAMERA
        v_transform = vehicle.get_transform()
        center = v_transform.location
        eye = center - v_transform.get_forward_vector() * 10 + carla.Location(z = 5)
        up = [0, 0, 1]
        view_control.set_lookat(
            np.array([center.x, center.y, center.z]),
            np.array([eye.x, eye.y, eye.z]),
            np.array(up)
        )
        
        while True:
            world.tick()
            
            spectator = world.get_spectator()
            location = vehicle.get_transform().transform(carla.Location(x=-4, z=2.5))
            spectator.set_transform(carla.Transform(location, vehicle.get_transform().rotation))
            
            """
            # To get a numpy [[vel, azimuth, altitude, depth],...[,,,]]:
            print("velocity: ", sensor_data.raw_data[0], end = "    ")
            print("azimuthal angle: ", sensor_data.raw_data[1], end = "    ")
            print("altitude: ", sensor_data.raw_data[2], end = "    ")
            print("depth: ", sensor_data.raw_data[3])
             print("--------------------------------------------------------------------------------------")
            """
            
            """
            # DYNAMIC OPEN3D CAMERA
            
            v_transform = vehicle.get_transform()
            
            center = v_transform.location
            eye = center - v_transform.get_forward_vector() * 10 + carla.Location(z = 5)
            
            up = [0, 0, 1]
            view_control.set_lookat(
                np.array([center.x, center.y, center.z]),
                np.array([eye.x, eye.y, eye.z]),
                np.array(up)
            )
            """
            
            
            
            if sensor_data.points.size > 0:               
                pcd.points = o3d.utility.Vector3dVector(sensor_data.points)
                vis.update_geometry(pcd)
                
            vis.poll_events()
            vis.update_renderer()
            
            print(f"Visualizing {len(sensor_data.points)} radar points.")
            
            waypoint = world.get_map().get_waypoint(vehicle.get_location(), project_to_road=True)
            next_wp = waypoint.next(2.0)[0]
            
            v_forward = vehicle.get_transform().get_forward_vector()
            target_vector = next_wp.transform.location - vehicle.get_location()
            
            cross_track_error = np.cross([v_forward.x, v_forward.y, 0], [target_vector.x, target_vector.y, 0])[2]
            
            steer_correction = np.clip(pid.run_step(cross_track_error, 0.05), -1.0, 1.0)
            
            control = carla.VehicleControl(throttle=0.4, steer=steer_correction)
            vehicle.apply_control(control)
            
    except KeyboardInterrupt:
        print("\nScript interrupted by user.")
    
    finally:
        print("Cleaning up...")
        if world is not None:
            world.apply_settings(original_settings)
            print("Reverted to asynchronous mode.")

        
        print("Destroying actors...")
        if 'radar' in locals() and radar.is_listening:
            radar.stop() # Stop the listener
            
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print("All actors destroyed.")

        print("Cleanup complete.")

if __name__ == "__main__":
    main()


---------------------------------------------


#!/usr/bin/env python

import carla
import numpy as np
import time

# --- FIXED: RadarData class now includes all expected attributes for clarity ---
class RadarData:
    def __init__(self):
        # Store a list of detections, not a single value
        self.detections = []

# Global object to store sensor data
sensor_data = RadarData()

# --- FIXED: Correctly process the iterable RadarMeasurement data ---
def radar_callback(data):
    """
    This callback is invoked every time the radar sensor sends a measurement.
    'data' is a carla.RadarMeasurement object.
    """
    # Create a list of dictionaries, where each dictionary represents one detection
    detections = []
    for detection in data:
        detections.append({
            'altitude': detection.altitude,
            'azimuth': detection.azimuth,
            'depth': detection.depth,
            'velocity': detection.velocity
        })
    sensor_data.detections = detections


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
    world = None # Initialize world to None for the finally block

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        original_settings = world.get_settings()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find('vehicle.tesla.model3')
        spawn_point = world.get_map().get_spawn_points()[0]

        vehicle = world.spawn_actor(bp, spawn_point)
        actor_list.append(vehicle)
        print("Vehicle spawned.")

        radar_bp = blueprint_library.find("sensor.other.radar")
        # --- IMPROVED: Place radar at the front of the vehicle for better results ---
        transform = carla.Transform(carla.Location(x=2.0, z=1.0)) # 2m forward, 1m up
        radar = world.spawn_actor(radar_bp, transform, attach_to=vehicle)
        actor_list.append(radar) # Add radar to the list to ensure it's destroyed

        radar.listen(lambda data: radar_callback(data))
        print("Radar sensor listening.")

        pid = PIDController(Kp=0.8, Ki=0.0, Kd=0.1)

        while True:
            world.tick()

            spectator = world.get_spectator()
            # Follow camera
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                    carla.Rotation(pitch=-90)))

            # --- FIXED: Print the number of detections, and data for the first one if it exists ---
            if sensor_data.detections:
                first_detection = sensor_data.detections[0]
                print(f"Total Detections: {len(sensor_data.detections)} | "
                      f"Closest -> Alt: {first_detection['altitude']:.2f}, "
                      f"Az: {first_detection['azimuth']:.2f}, "
                      f"Depth: {first_detection['depth']:.2f}, "
                      f"Vel: {first_detection['velocity']:.2f}")
            else:
                print("No radar detections.")

            waypoint = world.get_map().get_waypoint(vehicle.get_location(), project_to_road=True)
            # Get next waypoint 5 meters ahead
            next_wp = waypoint.next(5.0)[0]

            v_forward = vehicle.get_transform().get_forward_vector()
            target_vector = next_wp.transform.location - vehicle.get_location()

            # Calculate steering error using the cross product
            cross_track_error = np.cross([v_forward.x, v_forward.y, 0], [target_vector.x, target_vector.y, 0])[2]
            
            steer_correction = np.clip(pid.run_step(cross_track_error, 0.05), -1.0, 1.0)

            control = carla.VehicleControl(throttle=0.4, steer=steer_correction)
            vehicle.apply_control(control)

    except KeyboardInterrupt:
        print("\nScript interrupted by user.")

    finally:
        print("Cleaning up...")
        if world is not None:
            world.apply_settings(original_settings)
            print("Reverted to asynchronous mode.")

        # --- FIXED: Check for 'radar' object and stop it before destroying actors ---
        if 'radar' in locals() and radar.is_listening:
            radar.stop()
            print("Radar stopped.")
            
        if actor_list:
            print("Destroying actors...")
            client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
            print(f"{len(actor_list)} actors destroyed.")

        print("Cleanup complete.")

if __name__ == "__main__":
    main()


'''





















































#!/usr/bin/env python

import carla
import math
import cv2
import numpy as np
import time
import open3d as o3d
from matplotlib import cm

# --- Colormap for Radar Velocity ---
COOL_RANGE = np.linspace(0.0, 1.0, 256)
COOL = np.array(cm.get_cmap('winter')(COOL_RANGE))[:, :3]

IM_WIDTH = 640
IM_HEIGHT = 480

class PIDController: # PID - (P , I , D)
	def __init__(self, Kp, Ki, Kd):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.integral = 0.0
		self.prev_error = 0.0
		
	def run_step(self, error, dt):
		self.integral += error*dt
		derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
		self.prev_error = error
		return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


def add_open3d_axis(vis):
    """Adds a coordinate system axis to the Open3D visualizer."""
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(np.array([
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]))
    axis.lines = o3d.utility.Vector2iVector(np.array([[0, 1], [0, 2], [0, 3]]))
    axis.colors = o3d.utility.Vector3dVector(np.array([
        [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]))
    vis.add_geometry(axis)

def radar_callback(meas, point_list):
    """
    Processes radar data, converting to Cartesian points and coloring by velocity.
    """
    radar_data = np.zeros((len(meas), 4), dtype=np.float32)

    for i, d in enumerate(meas):
        x = d.depth * math.cos(d.altitude) * math.cos(d.azimuth)
        y = d.depth * math.cos(d.altitude) * math.sin(d.azimuth)
        z = d.depth * math.sin(d.altitude)
        radar_data[i, :] = [x, y, z, d.velocity]

    # Color points based on their velocity
    velocity = np.abs(radar_data[:, -1])
    # Normalize velocity to a 0-1 range for colormapping
    # (Cap at 25 m/s for reasonable color distribution)
    norm_velocity = np.clip(velocity / 25.0, 0.0, 1.0)
    
    # Interpolate colors from the COOL colormap
    int_color = COOL[ (norm_velocity * (COOL.shape[0] - 1)).astype(int) ]

    points = radar_data[:, :-1]
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)
    
def camera_callback(image, data_dict):
    """Stores the raw image data in the data_dict."""
    data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

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
        
        # --- RADAR ---
        radar_bp = bp.find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', '100.0')
        radar_bp.set_attribute('vertical_fov', '50.0')
        radar_bp.set_attribute('range', '1000')
        radar_bp.set_attribute('points_per_second', '100000') # Reduced for clarity
        radar = world.spawn_actor(
            radar_bp,
            carla.Transform(carla.Location(x=2.0, z=1.5)), # Placed at the front
            attach_to=vehicle
        )
        sensors.append(radar)
        print("RADAR spawned.")

        # --- RGB Camera ---
        cam_bp = bp.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(IM_WIDTH))
        cam_bp.set_attribute('image_size_y', str(IM_HEIGHT))
        cam_bp.set_attribute('fov', '90')
        camera = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(x=1.5, z=2.4)),
            attach_to=vehicle
        )
        sensors.append(camera)
        print("RGB Camera spawned.")

        # --- Data holders ---
        radar_list = o3d.geometry.PointCloud()
        camera_data = {"image": np.zeros((IM_HEIGHT, IM_WIDTH, 4), dtype=np.uint8)}

        # --- Listeners ---
        radar.listen(lambda data: radar_callback(data, radar_list))
        camera.listen(lambda image: camera_callback(image, camera_data))

        # --- Windows ---
        cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='Carla Radar', width=960, height=540)
        opt = vis.get_render_option()
        opt.background_color = np.array([0.05, 0.05, 0.05], dtype=np.float32)
        opt.point_size = 3.0 # Made points bigger for visibility
        opt.show_coordinate_frame = True
        add_open3d_axis(vis)


        pid = PIDController(Kp = 1.0, Ki = 0.0, Kd = 0.2)
        # Prime with a couple of ticks so sensors start streaming
        for _ in range(3):
            world.tick()

        added = False
        
        while True:
            # Advance the simulation
            world.tick()
            
            spectator = world.get_spectator()
            location = vehicle.get_transform().transform(carla.Location(x = -4, z = 2.5))
            rotation = vehicle.get_transform().rotation
            transform = carla.Transform(location, rotation)
            spectator.set_transform(transform)

            if not added:
                vis.add_geometry(radar_list)
                added = True

            vis.update_geometry(radar_list)
            vis.poll_events()
            vis.update_renderer()
            

            
            
            #Control
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

            time.sleep(0.002)

    finally:
        print("Destroying actors!")
        if 'settings' in locals() and world is not None:
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
            print("Reverted to asynchronous mode.")
        
        for s in sensors:
            s.stop()
        for actor in actors + sensors:
            actor.destroy()

        cv2.destroyAllWindows()
        print("All Cleaned Up!")

if __name__ == "__main__":
    main()
