# AERO 470 Group Project
# Matt Luzuriaga, Ryan Wendt, Drew Chapman

import numpy as np 
from vpython import *
import copy
import Control_Functions

class boid():
        def __init__(self, selfnum):
            self.num = selfnum
            pos1 = np.random.uniform(-100, 100)  # random initialization position coord
            pos2 = np.random.uniform(-100, 100)
            pos3 = np.random.uniform(-100, 100)
            self.pos = [pos1, pos2, pos3]  # store coords in list

            # define the base vertices of the pyramid
            v0 = vector(0, 0, 0)
            v1 = vector(2, 0, 0)
            v2 = vector(1, 2, 0)

            # define the apex of the pyramid
            apex = vector(1, 1, 1)  # Adjust the position of the apex as needed

            # translate the pyramid to the random position
            self.boid_obj = pyramid(
                pos=vector(pos1, pos2, pos3),
                size=vector(2, 2, 3),  # Adjust the size of the pyramid as needed
                axis=(apex - vector(pos1, pos2, pos3)),
                up=v2 - v0,
                color=color.yellow
            )

            vel1 = np.random.uniform(-6, 6)
            vel2 = np.random.uniform(-6, 6)
            vel3 = np.random.uniform(-6, 6)
            self.v = [vel1, vel2, vel3]

        def Separate(self,pop_pos): # Boids need to locally separate from each other. Avoid determins avoidance safety factor
            closex = 0
            closey = 0
            closez = 0
            safetycirc = 3 # Radius of safety around each boid. Parameter can be tuned.
            avoid = .0005 # Avoidance factor, parameter can be tuned

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
            matchfact = .3 # How closely boids will align with nearby boids. Tunable parameter

            for k in range(len(pop_pos)):
                tempdist = np.linalg.norm(np.array(self.pos)-np.array(pop_pos[k]))
                if k == self.num:
                    pass
                elif tempdist < visiblerange:
                    vx_avg += copy.deepcopy(popv[k][0])
                    vy_avg += copy.deepcopy(popv[k][1])
                    vz_avg += copy.deepcopy(popv[k][2])
                    neighbors += 1
                    if tempdist < 2:
                        global collision
                        collision += 1

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
        
        def avoid_hawk(self, pop_pos, hawk_pop_pos):
            closex = 0
            closey = 0
            closez = 0
            safetycirc =  10 # Radius of safety around each boid. Parameter can be tuned.
            avoid = .1 # Avoidance factor, parameter can be tuned

            for k in range(len(hawk_pop_pos)):
                if k == self.num:
                    pass
                elif np.linalg.norm(np.array(self.pos)-np.array(hawk_pop_pos[k])) < safetycirc:
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

        def UpdatePos(self): 
            self.pos = np.array(self.pos)+np.array(self.v)
            nextpos = vector(self.pos[0],self.pos[1],self.pos[2])
            self.boid_obj.pos = nextpos
            return(self)

class Hawk():
    def __init__(self, selfnum):
            self.num = selfnum
            pos1 = np.random.uniform(-20, 20)  # random initialization position coord
            pos2 = np.random.uniform(-20, 20)
            pos3 = np.random.uniform(-20, 20)
            self.pos = [pos1, pos2, pos3]  # store coords in list

            # define the base vertices of the pyramid
            v0 = vector(0, 0, 0)
            v1 = vector(2, 0, 0)
            v2 = vector(1, 2, 0)

            # define the apex of the pyramid
            apex = vector(1, 1, 1)  # Adjust the position of the apex as needed

            # translate the pyramid to the random position
            if mode == "Hawk":
                self.boid_obj = sphere(pos=vector(pos1, pos2, pos3),radius = .01)
            elif mode == "observer":
                self.boid_obj = pyramid(
                    pos=vector(pos1, pos2, pos3),
                    size=vector(4, 4, 6),  # Adjust the size of the pyramid as needed
                    axis=(apex - vector(pos1, pos2, pos3)),
                    up=v2 - v0,
                    color=color.red
                )
            else:
                Warning("Mode does not exist. Please input different viewing mode.")

            vel1 = np.random.uniform(-12, 12)
            vel2 = np.random.uniform(-12, 12)
            vel3 = np.random.uniform(-12, 12)
            self.v = [vel1, vel2, vel3]

    def targeting_onoff(self, hawk_pop_pos, pop_pos, boid_pop_v):
        Hawk_Range = 40 # visibility of the hawk
        self.targeting = .02 # Targeting factor

        # initialize
        self.boid_min_dist = float('inf')
        self.closest_boid_index = -1

        for i, boid_pos in enumerate(pop_pos):
            relpos = np.array(boid_pos) - np.array(self.pos)
            sight = np.dot(self.v,relpos)
            if sight > .55*np.linalg.norm(self.v):
                # calc distance
                distance = np.linalg.norm(np.array(self.pos) - np.array(boid_pos))
                # check if boid is within hawks range 
                if distance < Hawk_Range and distance < self.boid_min_dist:
                    self.boid_min_dist = distance
                    self.closest_boid_index = i 

                
        return self.boid_min_dist, self.closest_boid_index

    def target(self, hawk_pop_pos, pop_pos, boid_pop_v):
            
            # if boid os within range
            if self.closest_boid_index != -1:
                closest_boid_pos = pop_pos[self.closest_boid_index]
                closest_boid_vel = boid_pop_v[self.closest_boid_index]

                # move hawk towards closest boid
                targeting_vector = np.array(closest_boid_pos) - np.array(self.pos)
                self.v += targeting_vector * self.targeting 

    def UpdatePos(self): 
            self.pos = np.array(self.pos)+np.array(self.v)
            nextpos = vector(self.pos[0],self.pos[1],self.pos[2])
            self.boid_obj.pos = nextpos
            return(self)

class HAWKS():
    def __init__ (self, num_hawks):
        self.num_boids = num_hawks
        self.pop = []
        self.hawk_pop_pos = []
        self.hawk_pop_v = []

        for k in range(num_hawks):
            self.pop.append(Hawk(k))
            self.hawk_pop_pos.append(self.pop[k].pos)
            self.hawk_pop_v.append(self.pop[k].v)

class BOIDS():
    def __init__ (self, input_class, num_boids):
        self.num_boids = num_boids
        self.pop = []
        self.boid_pop_pos = []
        self.boid_pop_v = []

        for k in range(num_boids):
            self.pop.append(input_class(k))
            self.boid_pop_pos.append(self.pop[k].pos)
            self.boid_pop_v.append(self.pop[k].v)

    def change_color(self, index, color):
        self.pop[index].boid_obj.color = color

## SIMULATION LOOP

# Setting the Scene -- it would be cool if we could but in like a background texture for this !!
scene.center = vector(0, 0, 0)  
scene.camera.pos = vector(0, 0, 200) 
scene.camera.axis = vector(0, 0, -200) 

# Initialization of Swarm + Vpython Setup
collision = 0
mode = "observer"
num_boids = 50
BoidPop = BOIDS(boid, num_boids)
HawkPop = HAWKS(1)
count = 0

iprev = 0 # switch tells when to turn boid color back to yellow (AKA hawk has switched and started tracking new boid)

while True:
    count = count +1
    # rate(20)
    
    # Align, separate, and flock
    if count == 500:
        print("Hi")

    for k in BoidPop.pop:
        k.Separate(BoidPop.boid_pop_pos)
        k.Align(BoidPop.boid_pop_pos, BoidPop.boid_pop_v)
        k.flock(BoidPop.boid_pop_pos)
        k.avoid_hawk(BoidPop.boid_pop_pos, HawkPop.hawk_pop_pos)

    for k in HawkPop.pop:
        test = k.targeting_onoff(HawkPop.hawk_pop_pos, BoidPop.boid_pop_pos, BoidPop.boid_pop_v) 
        print(test)
        if test != float('inf'):
            k.target(HawkPop.hawk_pop_pos, BoidPop.boid_pop_pos, BoidPop.boid_pop_v)  
            BoidPop.change_color(test[1], color.green)  # Change the color of the targeted boid to green   
        if test[1] != iprev:
            BoidPop.change_color(iprev, color.yellow)
        iprev = copy.deepcopy(test[1])

    # Implement speed limits:
    minspeed_boid = 3
    maxspeed_boid = 6
    minspeed_hawk = 3
    maxspeed_hawk = 8
    Control_Functions.Speed_Limit(BoidPop.pop, minspeed_boid, maxspeed_boid)
    Control_Functions.Speed_Limit(HawkPop.pop, minspeed_boid, maxspeed_boid)

    # Implement Turn Factor
    lb = -35
    ub = 35
    turn = .25 # tunable parameter that makes boids turn before hitting edge of scren
    Control_Functions.Turn_Factor(BoidPop.pop, lb, ub, turn)
    Control_Functions.Turn_Factor(HawkPop.pop, lb, ub, .4)

    # Update Position
    for k in BoidPop.pop:
        k.UpdatePos()
    for k in HawkPop.pop:
        k.UpdatePos()

    if mode == "Hawk":
        scene.camera.pos = vector(HawkPop.pop[0].pos[0],HawkPop.pop[0].pos[1],HawkPop.pop[0].pos[2])
        scene.camera.axis = -scene.camera.pos

