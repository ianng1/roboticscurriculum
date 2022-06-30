from pupper_controller.src.pupperv2 import pupper, pupper_env
import time
from collections import defaultdict
def create_pupper_env(action="F"):
  env = pupper_env.PupperEnv(render=True, action=action)
  return env


try:
    env = create_pupper_env()
    obs = env.reset()
    start_time_wall = time.time()
    env_start_time_wall = time.time()
    print("Starting loop")
    while True:
        action = defaultdict(float, {'x_velocity': 5.0, 'y_velocity': 0.0, 'yaw_rate': 0.0, 'height': -0.11068185047760919, 'pitch': -0.008044656932101048, 'com_x_shift': -0.0004617729765720894, 'z_clearance': 0.08629007873657413, 'alpha': 0.4622902730448037, 'beta': 0.5406641271611506, 'delta_y': 0.11221057680972596})
        obs, r, done, _ = env.step(action)

except:
    pass