# AERO 470 Group Project
# Matt Luzuriaga, Ryan Wendt, Drew Chapman

import numpy as np 
from vpython import *
import copy

class boid():
        def __init__(self,selfnum):
            self.num = selfnum
            pos1 = np.random.uniform(-20, 20)
            pos2 = np.random.uniform(-20, 20)
            pos3 = np.random.uniform(-20, 20)
            self.pos = [pos1,pos2,pos3]
            self.boid_obj = sphere(pos=vector(self.pos[0],self.pos[1],self.pos[2]),radius=2,color=color.yellow)
            vel1 = np.random.uniform(-6, 6)
            vel2 = np.random.uniform(-6, 6)
            vel3 = np.random.uniform(-6, 6)
            self.v = [vel1,vel2,vel3]

        def Separate(self,pop_pos): # Boids need to locally separate from each other. Avoid determins avoidance safety factor
            closex = 0
            closey = 0
            closez = 0
            safetycirc = 4 # Radius of safety around each boid. Parameter can be tuned.
            avoid = .005 # Avoidance factor, parameter can be tuned

            for k in range(len(pop_pos)):
                if k == self.num:
                    pass
                elif np.linalg.norm(np.array(self.pos)-np.array(pop_pos[k])) < safetycirc:
                    closex += self.pos[0]-copy.deepcopy(pop_pos[k][0])
                    closey += self.pos[1]-copy.deepcopy(pop_pos[k][1])
                    closez += self.pos[2]-copy.deepcopy(pop_pos[k][2])
            vx = closex*avoid
            vy = closey*avoid
            vz = closez*avoid
            self.v[0] += vx
            self.v[1] += vy
            self.v[2] += vz
            return(self)

        def Align(self,pop_pos,popv):
            vx_avg = 0
            vy_avg = 0
            vz_avg = 0
            neighbors = 0
            visiblerange = 20 # radius that an individual boid can see. Tunable paramter
            matchfact = .25 # How closely boids will align with nearby boids. Tunable parameter

            for k in range(len(pop_pos)):
                if k == self.num:
                    pass
                elif np.linalg.norm(np.array(self.pos)-np.array(pop_pos[k])) < visiblerange:
                    vx_avg += copy.deepcopy(popv[k][0])
                    vy_avg += copy.deepcopy(popv[k][1])
                    vz_avg += copy.deepcopy(popv[k][2])
                    neighbors += 1

            if neighbors > 0: # Average out sum of nearby velocities
                vx_avg = vx_avg/neighbors
                vy_avg = vy_avg/neighbors
                vz_avg = vz_avg/neighbors

            self.v[0] += (vx_avg-self.v[0])*matchfact
            self.v[1] += (vy_avg-self.v[1])*matchfact
            self.v[2] += (vz_avg-self.v[2])*matchfact
            return(self)

        def flock(self,pop_pos):
            x_avg_pos = 0
            y_avg_pos = 0
            z_avg_pos = 0
            neighbors = 0
            visiblerange = 12 # radius that an individual boid can see. Tunable paramter
            centering_factor = .0001 # amount that boids will want to center themselves. Tunable parameter.

            for k in range(len(pop_pos)):
                if k == self.num:
                    pass
                elif np.linalg.norm(np.array(self.pos)-np.array(pop_pos[k])) < visiblerange:
                    x_avg_pos += copy.deepcopy(pop_pos[k][0])
                    y_avg_pos += copy.deepcopy(pop_pos[k][1])
                    z_avg_pos += copy.deepcopy(pop_pos[k][2])
                    neighbors += 1

            if neighbors > 0: # Average out sum of nearby boid posiitions
                x_avg_pos = x_avg_pos/neighbors
                y_avg_pos = y_avg_pos/neighbors
                z_avg_pos = z_avg_pos/neighbors

            self.pos[0] += (x_avg_pos-self.pos[0])*centering_factor
            self.pos[1] += (y_avg_pos-self.pos[1])*centering_factor
            self.pos[2] += (z_avg_pos-self.pos[2])*centering_factor
            return(self)

        def UpdatePos(self): 
            self.pos = np.array(self.pos)+np.array(self.v)
            nextpos = vector(self.pos[0],self.pos[1],self.pos[2])
            self.boid_obj.pos = nextpos
            return(self)

class BOIDS():
    def __init__ (self, num_boids):
        self.num_boids = num_boids
        self.pop = []
        self.boid_pop_pos = []
        self.boid_pop_v = []


        for k in range(num_boids):
            self.pop.append(boid(k))
            self.boid_pop_pos.append(self.pop[k].pos)
            self.boid_pop_v.append(self.pop[k].v)


## SIMULATION LOOP

# Setting the Scene -- it would be cool if we could but in like a background texture for this !!
scene.center = vector(0, 0, 0)  
scene.camera.pos = vector(0, 0, 200) 
scene.camera.axis = vector(0, 0, -200) 

# Initialization of Swarm + Vpython Setup
num_boids = 50
BoidPop = BOIDS(num_boids)

while True:
    rate(30)
    # Align, separate, and flock
    for k in BoidPop.pop:
        k.Separate(BoidPop.boid_pop_pos)
        k.Align(BoidPop.boid_pop_pos,BoidPop.boid_pop_v)
        k.flock(BoidPop.boid_pop_pos)

    # Implement speed limits:
    for k in BoidPop.pop:
        maxspeed = 6
        minspeed = 3

        speed = np.linalg.norm(np.array(k.v))

        if speed > maxspeed:
            Vx = (k.v[0]/speed)*maxspeed
            Vy = (k.v[1]/speed)*maxspeed
            Vz = (k.v[2]/speed)*maxspeed
            k.v = [Vx,Vy,Vz]
        elif np.linalg.norm(np.array(k.v))<minspeed:
            Vx = (k.v[0]/speed)*minspeed
            Vy = (k.v[1]/speed)*minspeed
            Vz = (k.v[2]/speed)*minspeed
            k.v = [Vx,Vy,Vz]

    # Implement Turn Factor
    lb = -35
    ub = 35
    turn = .25 # tunable parameter that makes boids turn before hitting edge of scren

    for k in BoidPop.pop:
        if k.pos[0] < lb:
            k.v[0] = k.v[0] + turn
        if k.pos[0] > ub:
            k.v[0] = k.v[0] - turn
        if k.pos[1] < lb:
            k.v[1] = k.v[1] + turn
        if k.pos[1] > ub:
            k.v[1] = k.v[1] - turn
        if k.pos[2] < lb:
            k.v[2] = k.v[2] + turn
        if k.pos[2] > ub:
            k.v[2] = k.v[2] - turn

    # Update Position
    for k in BoidPop.pop:
        k.UpdatePos()

