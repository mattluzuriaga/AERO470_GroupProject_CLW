import numpy as np

 # Implement speed limits:
def Speed_Limit(input_pop):
    for k in input_pop:
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

def Turn_Factor(input_pop, lb, ub, turn):
    lb = -35
    ub = 35
    turn = .25 # tunable parameter that makes boids turn before hitting edge of scren

    for k in input_pop:
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
