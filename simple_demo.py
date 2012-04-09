from audacious_robot import *

grid =	("W               W              W",
		 "                                ",
		 "        WWWW  WWW WWW    WWW    ",
		 "                                ",
		 "            WWWWW               ",
		 "             W    W             ",
		 "                                ",
		 "       WW                       ",
		 "      WW              W         ",
		 "     WW               W         ",
		 "          WW          W         ",
		 "                      W    W    ",
		 "                      W    W    ",
		 "      WWWWWW          W    W    ",
		 "                           W    ",
		 "            WWWWWWWW       W    ",
		 "                           W    ",
		 "                           W    ",
		 "                                ",
		 "W               W              W",)

class RandomController:
	def __init__(self):
		pass
		self.steerdir=1
	
	def update(self, measurement):
		steering = self.steerdir * pi/4
		if random.random()<0.05:
			self.steerdir *=-1
		distance = 1.0
		return steering, distance

myrobot = Robot(IdealizedBicycle(), RandomController())
myrobot.hardware.x = 320
myrobot.hardware.y = 200

sim = Simulation()
sim.set_grid(grid)
sim.add_robot(myrobot)
sim.run()
