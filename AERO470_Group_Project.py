# AERO 470 Group Project
# Matt Luzuriaga, Ryan Wendt, Drew Chapman

import numpy as np 
from vpython import *

class BOIDS():
    def __init__ (self, num_boids):
        self.num_boids = num_boids

    def Initialize_Swarm(self):
        self.boid_pos = []
        for i in range(self.num_boids):
            x = np.random.uniform(-30, 30, 2)
            self.boid_pos.append(x)

    def Update_Position(self):
        self.boid_pos = np.add(self.boid_pos, self.dr)
        for i in range(len(self.boid_pos)):
            for j in range(2):
                if self.boid_pos[i][j] > 30:
                    self.boid_pos[i][j] -= 60
                elif self.boid_pos[i][j] < -30:
                    self.boid_pos[i][j] += 60
        return self.boid_pos

    def Initial_Velocity(self):
        self.dr = []
        for i in range(self.num_boids):
            dir = np.random.uniform(0, 2*np.pi)
            self.dr.append([np.cos(dir), np.sin(dir)])


## SIMULATION LOOP
            
# Setting the Scene -- it would be cool if we could but in like a background texture for this !!
scene.range = 30 
scene.center = vector(0, 0, 0)  
scene.camera.pos = vector(0, 0, 50) 
boundary_thickness = 0.1
boundary_color = color.blue
box(pos=vector(0, 0, -boundary_thickness/2), size=vector(60, 60, boundary_thickness), color=boundary_color)
box(pos=vector(0, 0, boundary_thickness/2), size=vector(60, 60, boundary_thickness), color=boundary_color)

# Initialization of Swarm + Vpython Setup
num_boids = 13
boid_runloop = BOIDS(num_boids)
boid_runloop.Initialize_Swarm()  
boid_sim = []
for pos in boid_runloop.boid_pos:  
    boid_sim.append(sphere(pos=vector(pos[0], pos[1], 0), radius=1, color=color.red))  

boid_runloop.Initial_Velocity() # Generating an initial velocity of all boids 

# Run the fuggin loop
n = 0
while n < 1000:
    rate(20)
    boid_update = boid_runloop.Update_Position() 
    for i, pos in enumerate(boid_update):
        boid_sim[i].pos = vector(pos[0], pos[1], 0)
    n += 1  