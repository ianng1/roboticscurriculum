import time
import math
import gym
import numpy as np
import os

from pupper_controller.src.pupperv2 import pupper


class PupperEnv(gym.Env):

    def __init__(
        self,
        run_on_robot=False,
        render=True,
        render_meshes=False,
        action="F",
        plane_tilt=0.0
    ):
        """
        Create pupper gym environment.
        
        Args:
            run_on_robot: Whether to simulate or run on robot
            render: If simulating, whether to open the visualization window
            render_meshes: If simulating, whether to use visually detailed model or basic model
            plane_tilt: Tilt in radians of the ground plane
        """
        self.action = action # F, B, L, R

        self.action_keys = ["x_velocity", "y_velocity", "yaw_rate", "height", "pitch", "x_com_shift", 
                            "z_clearance", "alpha", "beta", "overlap_time", "swing_time", "delta_x", "delta_y"]

        # Defines lower and upper bounds on possible actions
        # Order of elements:
        # x velocity, y velocity, yaw rate, height, pitch, x_com_shift
        self.action_space = gym.spaces.Box(
            np.array([-1.2, -0.4, -2.0, -0.14, -0.1, -0.02, 0.065, 0.3, 0.3, 0.01, 0.01, 0.05, 0.05]),
            np.array([1.2, 0.4, 2.0, -0.08, 0.1, 0.02, 0.10, 0.7, 0.7, 0.1, 0.1, 0.15, 0.15]),
            dtype=np.float32)

        # Defines expected lower and upper bounds on observations
        # Order of elements
        # roll, pitch, the 12 joint angles
        self.observation_space = gym.spaces.Box(
            np.array([-0.5*math.pi, -0.5*math.pi] + 12*[-0.5*math.pi]),
            np.array([0.5*math.pi, 0.5*math.pi] + 12*[0.5*math.pi]),
            dtype=np.float32)

        self.env_step_counter = 0

        self.plane_tilt = plane_tilt

        self.pupper = pupper.Pupper(
            run_on_robot, render=render, render_meshes=render_meshes, plane_tilt=plane_tilt)
        
        # Overwrite the config values to match the specified action limits
        self.pupper.config.min_x_velocity = -1.2
        self.pupper.config.max_x_velocity = 1.2
        self.pupper.config.min_yaw_rate = -4.0
        self.pupper.config.max_yaw_rate = 4.0
        
        self.env_time_step = self.pupper.config.dt

    def reset(self):
        ob = self.pupper.reset()
        self.pupper.slow_stand(do_sleep=False)
        self.pupper.start_trot()
        return self.pupper.get_observation()

    def forward_reward(self, observation):
        dx = self.pupper.body_velocity()[0] * self.pupper.config.dt
        return 1.0 + dx

    def backward_reward(self, observation):
        dx = self.pupper.body_velocity()[0] * self.pupper.config.dt
        return 1.0 - dx
    
    def turn_left_reward(self, observation):
        dx = self.pupper.angular_velocity()[0] * self.pupper.config.dt
        return 1.0 + dx

    def turn_right_reward(self, observation):
        dx = self.pupper.angular_velocity()[0] * self.pupper.config.dt
        return 1.0 - dx

    def getReward(self, observation):
        if self.action == "F":
            return self.forward_reward(observation)
        elif self.action == "B":
            return self.backward_reward(observation)
        elif self.action == "L":
            return self.turn_left_reward(observation)
        elif self.action == "R":
            return self.turn_right_reward(observation)
        return self.forward_reward(observation)

    def terminate(self, observation):
        roll = observation[0]
        pitch = observation[1]

        # Terminate if body (id=-1) touches ground
        # Foot ids are 3, 7, 11, 15
        contact_points = self.pupper.hardware_interface._bullet_client.getContactPoints(
            self.pupper.hardware_interface.robot_id, self.pupper.hardware_interface.floor_id)
        for contact_point in contact_points:
            pupper_link_id = contact_point[3]
            if pupper_link_id == -1:
                return True

        return False

    def step(self, actions):
        if isinstance(actions, dict):
            action_dict = actions
        elif self.action == "F":
            action_dict = {'x_velocity': actions[0],
                           'y_velocity': actions[1],
                           'yaw_rate': actions[2],
                           'height': actions[3],
                           'pitch': actions[4],
                           'com_x_shift': actions[5],
                           'z_clearance': actions[6],
                           'alpha': actions[7],
                           'beta': actions[8],
                        #    'overlap_time': actions[9],
                        #    'swing_time': actions[10],
                        #    'delta_x': actions[11],
                           'delta_y': actions[12],
                           }
        elif self.action == "B":
            action_dict = {'x_velocity': actions[0],
                           'y_velocity': actions[1],
                           'yaw_rate': actions[2],
                           'height': actions[3],
                           'pitch': actions[4],
                           'com_x_shift': actions[5],
                           'z_clearance': actions[6],
                           'alpha': actions[7],
                           'beta': actions[8],
                        #    'overlap_time': actions[9],
                        #    'swing_time': actions[10],
                        #    'delta_x': actions[11],
                           'delta_y': actions[12],
                           }
        elif self.action == "L":
            action_dict = {'x_velocity': actions[0],
                           'y_velocity': actions[1],
                           'yaw_rate': actions[2],
                           'height': actions[3],
                           'pitch': actions[4],
                           'com_x_shift': actions[5],
                           'z_clearance': actions[6],
                           'alpha': actions[7],
                           'beta': actions[8],
                        #    'overlap_time': actions[9],
                        #    'swing_time': actions[10],
                        #    'delta_x': actions[11],
                           'delta_y': actions[12],
                           }
        elif self.action == "R":
            action_dict = {'x_velocity': actions[0],
                           'y_velocity': actions[1],
                           'yaw_rate': actions[2],
                           'height': actions[3],
                           'pitch': actions[4],
                           'com_x_shift': actions[5],
                           'z_clearance': actions[6],
                           'alpha': actions[7],
                           'beta': actions[8],
                        #    'overlap_time': actions[9],
                        #    'swing_time': actions[10],
                        #    'delta_x': actions[11],
                           'delta_y': actions[12],
                           }
        observation = self.pupper.step(action_dict)
        reward = self.getReward(observation)
        done = self.terminate(observation)
        self.env_step_counter += 1
        return observation, reward, done, {}

    def shutdown(self):
        # TODO: Added this function to attempt to gracefully close
        # the serial connection to the Teensy so that the robot
        # does not jerk, but it doesn't actually work
        self.pupper.shutdown()
