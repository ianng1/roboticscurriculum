import time
def move_forward_time(env, x_vel, runtime):
    
    action = {'x_velocity': x_vel, 
            'y_velocity': 0.0, 
            'yaw_rate': 0.0, 
            'height': -0.11068185047760919, 
            'pitch': -0.008044656932101048, 
            'com_x_shift': -0.0004617729765720894, 
            'z_clearance': 0.08629007873657413, 
            'alpha': 0.4622902730448037, 
            'beta': 0.5406641271611506, 
            'delta_y': 0.11221057680972596
            }
    start_time = time.time()
    while(time.time() - start_time < runtime):
        env.step(action)


def turn_for_time(env, rotational_vel, runtime):
    action = {'x_velocity': 0.0, 
            'y_velocity': 0.0, 
            'yaw_rate': rotational_vel, 
            'height': -0.11068185047760919, 
            'pitch': -0.008044656932101048, 
            'com_x_shift': -0.0004617729765720894, 
            'z_clearance': 0.08629007873657413, 
            'alpha': 0.4622902730448037, 
            'beta': 0.5406641271611506, 
            'delta_y': 0.11221057680972596
            }
    start_time = time.time()
    while(time.time() - start_time < runtime):
        env.step(action)

def turn_for_distance(env, angle, speed):
    time = angle/speed
    turn_for_time(env, speed, time)
    


def change_height(env):
    start_time = time.time()
    last_loop = start_time
    cur = -0.11
    while(abs(cur) < 0.21):
        if time.time() - last_loop >= 0.01:
            action = {'x_velocity': 0.0, 
            'y_velocity': 0.0, 
            'yaw_rate': 0, 
            'height': ((time.time() - start_time) / 5) * -0.21 - 0.11, 
            'pitch': -0.008044656932101048, 
            'com_x_shift': -0.0004617729765720894, 
            'z_clearance': 0.08629007873657413, 
            'alpha': 0.4622902730448037, 
            'beta': 0.5406641271611506, 
            'delta_y': 0.11221057680972596
            }
            env.step(action)
            cur = action['height']
            print(cur)
            last_loop - time.time()