#!/usr/bin/env python

import carla
import random
import cv2
import queue
import numpy as np
import time

IMG_SIZE = 500
CENTER = IMG_SIZE // 2
SCALE = 5.0  # pixels per meter
MAX_RANGE = 50  # meters
IM_WIDTH = 640
IM_HEIGHT = 480

# Thread-safe queue for processed lidar frames (images)
lidar_queue = queue.Queue(maxsize=4)


def process_lidar_frames(data):
    """
    Callback runs in CARLA thread. Process the raw_data -> top-down image
    and push a single image into the lidar_queue. Do not call cv2.imshow
    or waitKey here.
    """
    points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
    img = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)

    # Draw polar grid
    for r in range(10, MAX_RANGE, 10):
        cv2.circle(img, (CENTER, CENTER), int(r * SCALE), (50, 50, 50), 1)
    cv2.line(img, (0, CENTER), (IMG_SIZE, CENTER), (50, 50, 50), 1)
    cv2.line(img, (CENTER, 0), (CENTER, IMG_SIZE), (50, 50, 50), 1)

    # Draw points (top-down: x forward, y left)
    for x, y, z, intensity in points:
        if x * x + y * y > MAX_RANGE**2:
            continue
        img_x = int(CENTER + y * SCALE)  # Swap axes to make top-down
        img_y = int(CENTER - x * SCALE)

        if 0 <= img_x < IMG_SIZE and 0 <= img_y < IMG_SIZE:
            col_val = int(min(255, intensity * 255))
            # Draw as green scaled by intensity
            img[img_y, img_x] = (0, col_val, 0)

    # Put processed image into queue (drop if full to avoid blocking)
    try:
        lidar_queue.put_nowait(img)
    except queue.Full:
        # If the queue is full, drop this frame (keeps display responsive)
        pass


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def run_step(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


def main():
    actors = []
    client = None
    try:
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # Synchronous mode (so sensors update on world.tick())
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        world.apply_settings(settings)

        blueprints = world.get_blueprint_library()

        # Spawn vehicle
        vehicle_bp = blueprints.find("vehicle.tesla.model3")
        spawn_point = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actors.append(vehicle)

        # Place spectator behind the vehicle initially
        spectator = world.get_spectator()
        vehicle_transform = vehicle.get_transform()
        offset = carla.Location(x=-4, z=2.5)
        spectator_loc = vehicle_transform.transform(offset)  # correct usage
        spectator.set_transform(carla.Transform(spectator_loc, vehicle_transform.rotation))

        # Setup LiDAR sensor
        lidar_bp = blueprints.find("sensor.lidar.ray_cast")
        lidar_bp.set_attribute("range", str(MAX_RANGE))
        lidar_bp.set_attribute("rotation_frequency", "20")
        lidar_bp.set_attribute("channels", "64")
        lidar_bp.set_attribute("points_per_second", "200000")
        lidar_bp.set_attribute("upper_fov", "10")
        lidar_bp.set_attribute("lower_fov", "-10")
        lidar_bp.set_attribute("horizontal_fov", "360")
        # match sensor_tick roughly to world step (optional)
        lidar_bp.set_attribute("sensor_tick", str(0.05))

        lidar_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.5))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        actors.append(lidar)

        # Register callback (fast processing and queueing)
        lidar.listen(process_lidar_frames)

        # PID controller for following waypoints
        pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.2)

        cv2.namedWindow("LiDAR Top-Down View", cv2.WINDOW_AUTOSIZE)

        # Prime some ticks so sensors start streaming
        for _ in range(2):
            world.tick()

        while True:
            # Advance simulation (required in synchronous mode)
            world.tick()

            # Update spectator to chase the vehicle smoothly
            vehicle_transform = vehicle.get_transform()
            offset = carla.Location(x=-4, z=2.5)
            spectator_loc = vehicle_transform.transform(offset)
            spectator.set_transform(carla.Transform(spectator_loc, vehicle_transform.rotation))

            # Simple waypoint following (steer only)
            waypoint = world.get_map().get_waypoint(vehicle.get_location(), project_to_road=True,
                                                    lane_type=carla.LaneType.Driving)
            next_wp = waypoint.next(2.0)[0]

            v_loc = vehicle_transform.location
            v_forward = vehicle_transform.get_forward_vector()
            target_vector = next_wp.transform.location - v_loc
            cross_track_error = np.cross(
                [v_forward.x, v_forward.y, 0],
                [target_vector.x, target_vector.y, 0]
            )[2]

            steer_correction = pid.run_step(cross_track_error, settings.fixed_delta_seconds or 0.05)
            steer_correction = np.clip(steer_correction, -1.0, 1.0)

            control = carla.VehicleControl()
            control.throttle = 0.4
            control.steer = steer_correction
            vehicle.apply_control(control)

            # Display the latest lidar image if available
            if not lidar_queue.empty():
                lidar_img = lidar_queue.get()
                cv2.imshow("LiDAR Top-Down View", lidar_img)

            # handle keypress
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Exiting loop.")
                break

            # small sleep to reduce CPU load (optional)
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        # Stop and destroy sensors/actors
        try:
            lidar.stop()
        except Exception:
            pass

        print("Destroying actors...")
        for a in actors:
            try:
                a.destroy()
            except Exception:
                pass

        if client and world is not None:
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
