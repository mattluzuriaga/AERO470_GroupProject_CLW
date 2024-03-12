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
        return self.boid_pos

    def Initial_Velocity(self):
        self.dr = []
        for i in range(self.num_boids):
            dir = np.random.uniform(0, 2*np.pi)
            self.dr.append([np.cos(dir), np.sin(dir)])


# SIMULATION LOOP
num_boids = 13
boid_runloop = BOIDS(num_boids)
boid_runloop.Initialize_Swarm()  
boid_sim = []
for pos in boid_runloop.boid_pos:  
    boid_sim.append(sphere(pos=vector(pos[0], pos[1], 0), radius=1, color=color.red))  

boid_runloop.Initial_Velocity()  

n = 0
while n < 100:
    rate(5)
    boid_update = boid_runloop.Update_Position() 
    for i, pos in enumerate(boid_update):
        boid_sim[i].pos = vector(pos[0], pos[1], 0)
    n += 1  