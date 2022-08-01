import numpy as np
import time
from pupper_controller.src.common import controller
from pupper_controller.src.common import command
from pupper_controller.src.common import robot_state
from pupper_controller.src.pupperv2 import config

from pupper_controller.src.pupperv2 import kinematics
from pupper_hardware_interface import interface
from pupper_controller.src.pupperv2 import pybullet_interface
from pupper_controller.src.pupperv2 import robot_utils
from collections import defaultdict


class Pupper:
    def __init__(self,
                 run_on_robot=False,
                 render=True,
                 render_meshes=False,
                 plane_tilt=0.0,
                 LOG=False, ):
        self.config = config.Config()
        self.command = None

        self.run_on_robot = run_on_robot
        if self.run_on_robot:
            self.hardware_interface = interface.Interface(
                port=robot_utils.get_teensy_serial_port(),
                initial_max_current=7.0,)
            time.sleep(0.1)
        else:
            self.hardware_interface = pybullet_interface.Interface(config=self.config,
                                                                   render=render,
                                                                   render_meshes=render_meshes,
                                                                   plane_tilt=plane_tilt)

        self.controller = controller.Controller(
            self.config, kinematics.four_legs_inverse_kinematics)
        self.state = robot_state.RobotState(
            height=-0.05)

    def slow_stand(self,
                   duration=1.0,
                   min_height=-0.07,
                   max_height=-0.11,
                   do_sleep=False):
        """
        Blocking slow stand up behavior.

        Args:
            duration: How long the slow stand takes in seconds. Increase to make it slower.
            do_sleep: Set to False if you want simulated robot to go faster than real time
        """
        for height in np.linspace(min_height, max_height, int(duration / self.config.dt)):
            print(height)
            ob = self.get_observation()
            self.step({'height': height})
            if self.run_on_robot:
                time.sleep(self.config.dt)
            else:
                if do_sleep:
                    time.sleep(self.config.dt)

    def start_trot(self):
        pupper_command = command.Command(self.config.default_z_ref)
        pupper_command.trot_event = True
        self.controller.run(self.state, pupper_command)

    def _update_actions(self, action):
        """
        Update the command and config based on the given action dictionary.
        """

        # Convert to defaultdict(int) makes non existent keys return 0
        action = defaultdict(int, action)
        #action = defaultdict(float, {'x_velocity': -1.0, 'y_velocity': -0.02977926219788244, 'yaw_rate': 0.0854983323126207, 'height': -0.11068185047760919, 'pitch': -0.008044656932101048, 'com_x_shift': -0.0004617729765720894, 'z_clearance': 0.08629007873657413, 'alpha': 0.4622902730448037, 'beta': 0.5406641271611506, 'delta_y': 0.11221057680972596})
        self.command = command.Command(self.config.default_z_ref)
        # Update actions or use default value if no value provided
        x_vel = action['x_velocity'] or 0.0
        y_vel = action['y_velocity'] or 0.0
        self.command.horizontal_velocity = np.array((x_vel, y_vel))
        self.command.yaw_rate = action['yaw_rate'] or 0.0
        self.command.height = action['height'] or self.config.default_z_ref
        self.command.pitch = action['pitch'] or 0.0
        self.config.x_shift = action['com_x_shift'] or self.config.x_shift

        self.config.z_clearance = action['z_clearance'] or self.config.z_clearance
        self.config.alpha = action['alpha'] or self.config.alpha
        self.config.beta = action['beta'] or self.config.beta
        self.config.overlap_time = action['overlap_time'] or self.config.overlap_time
        self.config.swing_time = action['swing_time'] or self.config.swing_time
        self.config.delta_x = action['delta_x'] or self.config.delta_x
        self.config.delta_y = action['delta_y'] or self.config.delta_y

        # Clip actions to reasonable values
        self.command.horizontal_velocity = np.clip(self.command.horizontal_velocity,
                                                   (self.config.min_x_velocity,
                                                    self.config.min_y_velocity),
                                                   (self.config.max_x_velocity, self.config.max_y_velocity))
        self.command.yaw_rate = np.clip(
            self.command.yaw_rate,
            self.config.min_yaw_rate,
            self.config.max_yaw_rate)
        self.command.height = np.clip(
            self.command.height,
            self.config.min_height,
            self.config.max_height)
        self.command.pitch = np.clip(self.command.pitch,
                                     self.config.min_pitch,
                                     self.config.max_pitch)
        self.config.x_shift = np.clip(self.config.x_shift,
                                      self.config.min_x_shift,
                                      self.config.max_x_shift)
        self.config.z_clearance = np.clip(self.config.z_clearance, 0.10, 0.065)
        self.config.alpha = np.clip(self.config.alpha, 0.3, 0.7)
        self.config.beta = np.clip(self.config.beta, 0.3, 0.7)
        self.config.overlap_time = np.clip(self.config.overlap_time, 0.01, 0.1)
        self.config.swing_time = np.clip(self.config.swing_time, 0.01, 0.1)
        self.config.delta_x = np.clip(self.config.delta_x, 0.05, 0.15)
        self.config.delta_y = np.clip(self.config.delta_y, 0.05, 0.15)

    def step(self, action):
        """
        Make pupper controller take one action (push one timestep forward).

        Args:
            action: Dictionary containing actions. Available keys are:
                x_velocity
                y_velocity
                yaw_rate
                height
                pitch
                com_x_shift
        """
        self._update_actions(action)
        self.controller.run(self.state, self.command)
        self.hardware_interface.set_cartesian_positions(
            self.state.final_foot_locations)
        return self.get_observation()

    def reset(self):
        # TODO figure out how to do a slow stand on real robot, but in sim doing 1) slow stand for realistic mode 2) instantaenous stand for training mode
        self.hardware_interface.deactivate()
        self.hardware_interface.activate()
        return self.get_observation()

    def shutdown(self):  # TODO LATER
        pass
        # self.hardware_interface.shutdown()

    def get_observation(self):
        """
        TODO: Add: Body angular velocity, linear acceleration (need to check data), joint velocities
        """
        # reads up to 1024 bytes # TODO fix hardware interface code
        self.hardware_interface.read_incoming_data()
        base_roll_pitch = np.array(
            [self.hardware_interface.robot_state.roll,
             self.hardware_interface.robot_state.pitch])
        joint_positions = self.hardware_interface.robot_state.position
        return np.concatenate((base_roll_pitch, joint_positions))

    def body_velocity(self):
        """
        TODO: put this function in the respective hardware interfaces
        Note: Can use this on real robot https://github.com/erwincoumans/motion_imitation/blob/master/mpc_controller/com_velocity_estimator.py
        """
        if self.run_on_robot:
            raise NotImplementedError
        else:
            (linear, angular) = self.hardware_interface._bullet_client.getBaseVelocity(
                self.hardware_interface.robot_id)
            return linear
        
    def angular_velocity(self):
        """
        TODO: put this function in the respective hardware interfaces
        Note: Can use this on real robot https://github.com/erwincoumans/motion_imitation/blob/master/mpc_controller/com_velocity_estimator.py
        """
        if self.run_on_robot:
            raise NotImplementedError
        else:
            (linear, angular) = self.hardware_interface._bullet_client.getBaseVelocity(
                self.hardware_interface.robot_id)
            return angular
