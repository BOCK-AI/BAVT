#!/usr/bin/env python

import carla 
import random
import cv2
import queue
import numpy as np
import time
import threading

image_queue = queue.Queue()




IM_WIDTH = 640
IM_HEIGHT = 480










def depth_callback(image, data_dict):
	image.convert(carla.ColorConverter.LogarithmicDepth)
	data_dict['depth_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
	



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

def main():
	actor_list = []
	#try:
	client = carla.Client('localhost', 2000)
	client.set_timeout(10.0)
	world = client.get_world()

	# set up synchronous mode
	settings = world.get_settings()
	settings.synchronous_mode = True
	settings.fixed_delta_seconds = 0.05     #20 FPS
	world.apply_settings(settings)
	
	blueprint_library = world.get_blueprint_library()
	bp = blueprint_library.find('vehicle.tesla.model3')
	print(bp)
	spawn_points = world.get_map().get_spawn_points()
	#spawn_point = random.choice(world.get_map().get_spawn_points())
	
	vehicle = world.spawn_actor(bp, spawn_points[0])
	actor_list.append(vehicle)
	
	sensor_data = {
	'depth_image' : np.zeros((IM_HEIGHT, IM_WIDTH, 4))
	}
	      
	cam_bp = blueprint_library.find('sensor.camera.depth')
	cam_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
	cam_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")
	cam_bp.set_attribute("fov", "150")
	cam_bp.set_attribute("sensor_tick", str(0.05))
	cam_spawn_point = carla.Transform(carla.Location(2, 0, 2), carla.Rotation(0, 0, 0))
	
	camera = world.spawn_actor(cam_bp, cam_spawn_point, attach_to=vehicle)
	actor_list.append(camera)
	
	camera.listen(lambda image: depth_callback(image, sensor_data))
	#camera.listen(lambda image: image.save_to_disk('frame_pictures/new_depth_cam_output/%.6d.jpg' % image.frame, carla.ColorConverter.LogarithmicDepth))
	print("CARLA simulation started Depth cam.")
	
			# We don't set sensor_tick here as we are in synchronous mode and 
	        # will tick manually...
	        
	
	
	
	pid = PIDController(Kp = 1.0, Ki = 0.0, Kd = 0.2)
	
	while(True):
		world.tick() #To advance the simulation world
		
		spectator = world.get_spectator()
		location = vehicle.get_transform().transform(carla.Location(x = -4, z = 2.5))
		rotation = vehicle.get_transform().rotation
		transform = carla.Transform(location, rotation)
		spectator.set_transform(transform)
		
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
		
		cv2.imshow('All cameras', sensor_data['depth_image'])
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			print("Exiting Loop.")
			break
			
		time.sleep(0.01)				
	
#finally:
	print("destroying actors!")
	camera.stop()
	for actor in actor_list: 
		actor.destroy()
		cv2.destroyAllWindows()
		
	if client:
		settings = world.get_settings()
		settings.synchronous_mode = False
		world.apply_settings(settings)
		print("Reverted to asynchronous mode.")
		
	cv2.destroyAllWindows()
	print("All Cleaned Up!")
		
	
if __name__ == "__main__":
	main()
