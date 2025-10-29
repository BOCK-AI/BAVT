import carla
import numpy as np
import cv2
import os

# Define the output directory for saved images
OUTPUT_DIR = "carla_output"

# Lists to keep track of actors and sensors
actors = []
sensors = []

# PID Controller class
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


try:
    # --- 1. Client and World Setup ---
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    
    # --- Synchronous mode ---
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  # 10 FPS
    world.apply_settings(settings)
    print("Now in synchronous mode.")


    # --- 2. Spawn Vehicle ---
    vehicle_bp = bp_lib.find("vehicle.tesla.model3")
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    actors.append(vehicle)
    print(f"Spawned actor: {vehicle.type_id}")

    # --- 3. Define Sensor Blueprints ---
    # Define image size
    IMG_WIDTH = 240
    IMG_HEIGHT = 360
    
    # Common camera transform
    camera_init_trans = carla.Transform(carla.Location(z=2))

    # Define a list of sensor blueprints to spawn
    sensor_definitions = [
        {'name': 'rgb', 'bp': 'sensor.camera.rgb'},
        {'name': 'semantic', 'bp': 'sensor.camera.semantic_segmentation'},
        {'name': 'instance', 'bp': 'sensor.camera.instance_segmentation'},
        {'name': 'depth', 'bp': 'sensor.camera.depth'},
        {'name': 'dvs', 'bp': 'sensor.camera.dvs'},
        {'name': 'optical_flow', 'bp': 'sensor.camera.optical_flow'}
    ]

    for sensor_def in sensor_definitions:
        cam_bp = bp_lib.find(sensor_def['bp'])
        cam_bp.set_attribute("image_size_x", str(IMG_WIDTH))
        cam_bp.set_attribute("image_size_y", str(IMG_HEIGHT))
        camera_actor = world.spawn_actor(cam_bp, camera_init_trans, attach_to=vehicle)
        sensors.append(camera_actor)
        print(f"Spawned sensor: {sensor_def['name']}")
        sensor_def['actor'] = camera_actor # Store actor for listening

    # --- 4. Prepare Data Storage and Callbacks ---
    # Dictionary to hold the latest sensor data
    sensor_data = {
        'rgb': np.zeros((IMG_HEIGHT, IMG_WIDTH, 4), dtype=np.uint8),
        'semantic': np.zeros((IMG_HEIGHT, IMG_WIDTH, 4), dtype=np.uint8),
        'instance': np.zeros((IMG_HEIGHT, IMG_WIDTH, 4), dtype=np.uint8),
        'depth': np.zeros((IMG_HEIGHT, IMG_WIDTH, 4), dtype=np.uint8),
        'dvs': np.zeros((IMG_HEIGHT, IMG_WIDTH, 4), dtype=np.uint8),
        'optical_flow': np.zeros((IMG_HEIGHT, IMG_WIDTH, 4), dtype=np.uint8)
    }
    
    # Create directory for saving optical flow images if it doesn't exist
    opt_flow_output_path = os.path.join(OUTPUT_DIR, "optical_flow")
    os.makedirs(opt_flow_output_path, exist_ok=True)

    # --- Callback Functions ---
    def process_image(data, key):
        array = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
        sensor_data[key] = array

    def process_semantic(data, key):
        data.convert(carla.ColorConverter.CityScapesPalette)
        array = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
        sensor_data[key] = array
    
    def process_depth(data, key):
        data.convert(carla.ColorConverter.LogarithmicDepth)
        array = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
        sensor_data[key] = array

    def process_dvs(data, key):
        events = np.frombuffer(data.raw_data, dtype=np.dtype([
            ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', bool)
        ]))
        dvs_img = np.zeros((data.height, data.width, 3), dtype=np.uint8)
        # Set pixel color based on polarity: red for positive, blue for negative
        dvs_img[events['y'], events['x'], events['pol'] * 2] = 255
        sensor_data[key][:, :, :3] = dvs_img # Update only RGB channels

    def process_optical_flow(data, key):
        # get_color_coded_flow() returns a carla.Image object
        flow_image = data.get_color_coded_flow()
        array = np.reshape(np.copy(flow_image.raw_data), (flow_image.height, flow_image.width, 4))
        sensor_data[key] = array
        # Save the image using OpenCV
        filepath = os.path.join(opt_flow_output_path, f"{data.frame:06d}.png")
        cv2.imwrite(filepath, array)

    # --- 5. Start Listening to Sensors ---
    # Assign callbacks to each sensor
    sensor_definitions[0]['actor'].listen(lambda data: process_image(data, 'rgb'))
    sensor_definitions[1]['actor'].listen(lambda data: process_semantic(data, 'semantic'))
    sensor_definitions[2]['actor'].listen(lambda data: process_image(data, 'instance'))
    sensor_definitions[3]['actor'].listen(lambda data: process_depth(data, 'depth'))
    sensor_definitions[4]['actor'].listen(lambda data: process_dvs(data, 'dvs'))
    sensor_definitions[5]['actor'].listen(lambda data: process_optical_flow(data, 'optical_flow'))
    
    # --- Here, I am simply initilizing an Object of the PID class ---
    pid = PIDController(Kp = 1.0, Ki = 0.0, Kd = 0.2)
    
    
    
    # --- 6. Main Loop ---
    # Create a named window for display
    cv2.namedWindow("All Cameras", cv2.WINDOW_AUTOSIZE)

    while True:
        # Setting World to Tick, else world will never start ticking.
        world.tick()
            
        # Setting up a spectator to follow out ego vehicle in the simulation
        spectator = world.get_spectator()
        location = vehicle.get_transform().transform(carla.Location(x = -4, z = 2.5))
        rotation = vehicle.get_transform().rotation
        transform = carla.Transform(location, rotation)
        spectator.set_transform(transform)
        
        # Set vehicle to Keep to the lane, by manually adjusting the PID controller.
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
    
        
        # Tile images for display
        top_row = np.concatenate((
            sensor_data['rgb'], sensor_data['semantic'], sensor_data['instance']
        ), axis=1)
        
        bottom_row = np.concatenate((
            sensor_data['depth'], sensor_data['dvs'], sensor_data['optical_flow']
        ), axis=1)

        tiled_image = np.concatenate((top_row, bottom_row), axis=0)

        # Display the tiled image
        cv2.imshow("All Cameras", tiled_image)


        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) == ord('q'):
            break

finally:
    # --- 7. Cleanup ---
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    print("Reverted to asynchronous mode.")
    
    print("Cleaning up...")
    client.apply_batch([carla.command.DestroyActor(s) for s in sensors])
    client.apply_batch([carla.command.DestroyActor(a) for a in actors])
    cv2.destroyAllWindows()
    print("All cleaned up!")
    
    
