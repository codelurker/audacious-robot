import sys
import random
import pygame
from pygame.locals import *
from math import *

pygame.init()
fpsClock = pygame.time.Clock()

class Simulation:
	def __init__(self):
		self.robots = []
		self.grid = []
	def add_robot(self, robot):
		self.robots.append(robot)
	def set_grid(self,grid):
		self.grid = grid
		error = ValueError("Grid must be 20 rows and 32 columns!")
		if len(self.grid)!=20:
			raise error
		for l in self.grid:
			if len(l)!=32:
				raise error
		
			
	def run(self):
		self.window = pygame.display.set_mode((640,480))
		
		pygame.display.set_caption('Audacious Robot Simulator')

		self.font = pygame.font.Font ('freesansbold.ttf',32)

		while True:
			self.window.fill((0,0,0))
			
			for r, row in enumerate(self.grid):
				for col, tile in enumerate(row):
					if tile in ("W","w"):
						pygame.draw.rect(self.window, (230,0,0),
									(col*20, r*20,20, 20))
			for robot in self.robots:
				robot.update(self.grid)
				l = 10.0
				pygame.draw.circle(self.window, (255,0,0), (int(robot.hardware.x),int(robot.hardware.y)),10,1)
				dx = robot.hardware.x+ cos(robot.hardware.orientation)*l
				dy = robot.hardware.y+sin(robot.hardware.orientation)*l
				pygame.draw.line(self.window, (255,0,0),
					(int(robot.hardware.x),int(robot.hardware.y)),
					(int(dx),int(dy)),1)
			for event in pygame.event.get():
				if event.type == QUIT:
					pygame.quit()
					sys.exit()
				elif event.type == KEYDOWN:
					if event.key ==  K_ESCAPE:
						pygame.event.post(pygame.event.Event(QUIT))
			pygame.display.update()
			fpsClock.tick(30)
		

class Robot:
	def __init__(self, hardware, controller, measurement_delay = 5, control_delay = 5):
		""" Creates a robot instance with hardware and software. """
		self.hardware = hardware
		self.controller = controller
		self.measurement_queue =[]
		self.control_queue =[]
		self.measurement_delay = measurement_delay
		self.control_delay = control_delay
			
	def update(self, grid=[]):
		""" Negotiates the update between hardware and controller. """
		
		# Only apply control
		if len(self.control_queue)>=self.control_delay:
			control = self.control_queue.pop(0)
		else:
			control = None
		
		update = self.hardware.update(grid, control)
		self.measurement_queue.append(update)
		if len(self.measurement_queue)>=self.measurement_delay:
			measurement= self.measurement_queue.pop(0)
			control = self.controller.update(measurement)
			self.control_queue.append(control)
		

class IdealizedBicycle:
	""" This hardware model implements idealized Bicycle motion with gaussian noise.
		The implementation was adapted from Udacity's cs373.
	"""
	def __init__(self, length=10.0, distance=0.0, distance_noise=0.3, steering_noise=0.01, measurement_noise=0.3,
		orientation_noise= 0.01):
		self.length = length
		self.distance = distance # Distance to travel in next frame. Should usually be 0 at first frame
		self.distance_noise = distance_noise
		self.steering_noise = steering_noise
		self.measurement_noise = measurement_noise
		self.x = 0.0
		self.y = 0.0
		self.orientation = 0.0
		self.length = length
		self.steering_noise    = 0.0
		self.distance_noise    = 0.0
		self.measurement_noise = 0.0
		self.orientation_noise = 0.0
		self.num_collisions    = 0
		self.num_steps         = 0
		self.max_steering_angle = pi/4
		self.steering = 0
		self.distance = 0
		self.tolerance = 0.001

	def update(self, grid, control):
		# control will be none until the first control command is received
		if control is not None:
			self.steering, self.distance = control
		oldx, oldy = self.x, self.y
		if self.steering > self.max_steering_angle:
		    self.steering = self.max_steering_angle
		if self.steering < -self.max_steering_angle:
		    self.steering = -self.max_steering_angle
		
		if self.distance < 0.0:
		    self.distance = 0.0

		# apply noise
		steering2 = random.gauss(self.steering, self.steering_noise)
		distance2 = random.gauss(self.distance, self.distance_noise)


		# Execute motion
		turn = tan(steering2) * distance2 / self.length

		if abs(turn) < self.tolerance:

		    # approximate by straight line motion

		    self.x = self.x + (distance2 * cos(self.orientation))
		    self.y = self.y + (distance2 * sin(self.orientation))
		    self.orientation = (self.orientation + turn) % (2.0 * pi)
		else:
		    # approximate bicycle model for motion
		    radius = distance2 / turn
		    cx = self.x - (sin(self.orientation) * radius)
		    cy = self.y + (cos(self.orientation) * radius)
		    self.orientation = (self.orientation + turn) % (2.0 * pi)
		    self.x = cx + (sin(self.orientation) * radius)
		    self.y = cy - (cos(self.orientation) * radius)
		
		# Check bounds
		if self.x<5 or self.x>635:
			self.x = oldx
		if self.y<5 or self.y>395:
			self.y = oldy
		
		# Check for collisions with grid
		if len(grid)>0:
			l = 10.0
			dx = self.x+ cos(self.orientation)*l
			dy = self.y+sin(self.orientation)*l		

			row = int(int(dy) / 20)
			col = int(int(dx) / 20)
			if grid[row][col] in ("W","w"):
				self.x = oldx
				self.y = oldy
		
		return (random.gauss(self.x, self.measurement_noise),
			random.gauss(self.y, self.measurement_noise),
			random.gauss(self.orientation, self.orientation_noise))