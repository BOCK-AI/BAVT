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





def process_img(image):
	i = np.array(image.raw_data)    #raw_data is a flattened array	
	i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))

	frame_bgr = cv2.cvtColor(i2, cv2.COLOR_BGRA2BGR) #height, width, rgb values(from rgba values so we choose 0:3)
	image_queue.put(frame_bgr)
	
	#img_rgb = i2[:, :, :3] #height, width, rgb values( #from rgba values so we choose 0:3)#cv2.imshow("Camera Feeding",img_bgr) #cv2.waitKey(1)	#cv2.waitKey(1)	#return i3/255.0


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
	try:
		client = carla.Client('localhost', 2000)
		client.set_timeout(10.0)
		world = client.get_world()

		# Set up synchronous mode
		settings = world.get_settings()
		settings.synchronous_mode = True
		settings.fixed_delta_seconds = 0.05    #20 FPS
		world.apply_settings(settings)
		
		blueprint_library = world.get_blueprint_library()
		bp = blueprint_library.find('vehicle.tesla.model3')
		print(bp)
		spawn_points = world.get_map().get_spawn_points()
		#spawn_point = random.choice(world.get_map().get_spawn_points())
		
		vehicle = world.spawn_actor(bp, spawn_points[0])
		actor_list.append(vehicle)
		print(f"Spawned {vehicle.type_id}") 
		
		#spect = world.get_spectator()
		#spect.set_transform()
		
		
		# Camera sensor
		cam_bp = blueprint_library.find('sensor.camera.rgb')
		cam_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
		cam_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")
		cam_bp.set_attribute("fov", "150")
		#cam_bp.set_attribute("sensor_tick", str(0.5))
		cam_spawn_point = carla.Transform(carla.Location(x = 1.6, z = 1.3), carla.Rotation(0, 0, 0))		
		camera = world.spawn_actor(cam_bp, cam_spawn_point, attach_to=vehicle)
		actor_list.append(camera)
		camera.listen(lambda data : process_img(data))
		#camera.listen(lambda image: image.save_to_disk('frame_pictures/rgb_cam_output/%.6d.jpg' % image.frame))
		print("CARLA simulation started RGB cam.")
		
		        # We don't set sensor_tick here as we are in synchronous mode and 
		        # will tick manually... 
		
		
		pid = PIDController(Kp = 1.0, Ki = 0.0, Kd = 0.2)
		
		while(True):
			world.tick() #To advance the simulation world
			
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
					
			if not image_queue.empty():
				frame = image_queue.get()
				cv2.imshow("Camera Feed", frame)
			
			if cv2.waitKey(1) & 0xFF == ord('q'):
				print("Exiting Loop.")
				break
				
			time.sleep(0.01)				
		
	finally:
		print("destroying actors!")
		camera.stop()
		for actor in actor_list: 
			actor.destroy()
			
		if client:
			settings = world.get_settings()
			settings.synchronous_mode = False
			world.apply_settings(settings)
			print("Reverted to asynchronous mode.")
			
		cv2.destroyAllWindows()
		print("All Cleaned Up!")
		
	
if __name__ == "__main__":
	main()
