#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "April 1, 2023"
__version__   = "0.0.1"
__status__    = "Production"

# TODO: Have the gazebo interface structures available in python using Boost

import time
import threading

import numpy as np
import socket
import sys
import pyshmxtreme.SHMSegment as shmx
import ctypes
import posix_ipc

# class ConstantParameters(ctypes.Structure):
#     _fields_ = [
#         ('robot_name', ctypes.c_char_p),
#         ('world_name', ctypes.c_char_p),
#         ('dir_name', ctypes.c_char_p),
#         ('num_joints',    ctypes.c_int),
#         ('num_contact_sensors',    ctypes.c_int),
#         ('max_contacts_per_sensor',    ctypes.c_int)
#     ]

class ModelParameters(ctypes.Structure):
    _fields_ = [
        ('operating_mode',    ctypes.c_int),
        ('state_update_rate', ctypes.c_double)]

class WorldParameters(ctypes.Structure):
    _fields_ = [
        ('step_size',             ctypes.c_double),
        ('real_time_update_rate', ctypes.c_double)]

class RobotInterface(object):
    TORQUE_MODE = 0
    VELOCITY_PID_MODE = 1
    POSITION_PID_MODE = 2
    DIRECT_FORCE_MODE = 3

    def __init__(self, robot_name, num_joints, num_contact_sensors=None):
        """Interface between Gazebo (C++) and Python using shared NumPy memory blocks
        Args:

        robot_name: string, robot name defined in gazebo world file
        num_joints: int, total number of joints in the robot
        num_contact_sensors: int, total number of contact sensors in the robot's SDF file
        max_contacts_per_sensor: int, total allowable contacts per contact sensor in SDF
            Note: if not defined default is 10 in SDF 1.7v
        dir_n: string, directory name where shared memory blocks are created and accessed
        world_name: string, world name defined in gazebo world file
        """
        self.robot_name = robot_name
        self.world_name = "world"
        self.dir = "/tmp/"

        self.num_joints = num_joints
        self.num_contact_sensors = num_contact_sensors or 1
        # self.max_contacts_per_sensor = max_contacts_per_sensor*50  
        # # TODO: Varaible length of contact measurements taken
        self._max_timeouts = 10

        self._initialize_shared_memory()
        self._initialize_clients()

    def _initialize_shared_memory(self):
        self._world_parameters_shm = shmx.SHMSegment(robot_name='GAZ', seg_name='WORLD_PARAMS', init=False)
        self._world_parameters_shm.add_blocks(name='data', data=np.array(WorldParameters))

        self._model_parameters_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='MODEL_PARAMS', init=False)
        self._model_parameters_shm.add_blocks(name='data', data=np.array(ModelParameters))

        self._joint_states_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='STATES', init=False)
        self._joint_states_shm.add_blocks(name='time', data=np.zeros((1,1)))
        self._joint_states_shm.add_blocks(name='position', data=np.zeros((self.num_joints,1)))
        self._joint_states_shm.add_blocks(name='velocity', data=np.zeros((self.num_joints,1)))
        self._joint_states_shm.add_blocks(name='force', data=np.zeros((self.num_joints,1)))

        self._joint_force_commands_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='FORCE_COMMS', init=False)
        self._joint_force_commands_shm.add_blocks(name='data', data=np.zeros((self.num_joints,1)))

        self._position_pid_gains_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='PID_GAINS', init=False)
        self._position_pid_gains_shm.add_blocks(name='data', data=np.zeros((3*self.num_joints,1)))
        
        self._joint_position_commands_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='POS_COMMS', init=False)
        self._joint_position_commands_shm.add_blocks(name='data', data=np.zeros((self.num_joints,1)))

        self._joint_limits_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='JOINT_LIMITS', init=False)
        self._joint_limits_shm.add_blocks(name='data', data=np.zeros((2*self.num_joints,1)))

        self._effort_limits_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='EFFORT_LIMITS', init=False)
        self._effort_limits_shm.add_blocks(name='data', data=np.zeros((2*self.num_joints,1)))

        self._body_pose_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='BODY_POSE', init=False)
        self._body_pose_shm.add_blocks(name='time', data=np.zeros((1,1)))
        self._body_pose_shm.add_blocks(name='position', data=np.zeros((3,1)))
        self._body_pose_shm.add_blocks(name='quaternion', data=np.zeros((4,1)))
        self._body_pose_shm.add_blocks(name='euler_angles', data=np.zeros((3,1)))
        self._body_pose_shm.add_blocks(name='velocity', data=np.zeros((3,1)))

        self._imu_states_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='IMU_STATES', init=False)
        self._imu_states_shm.add_blocks(name='time', data=np.zeros((1,1)))
        self._imu_states_shm.add_blocks(name='accel', data=np.zeros((3,1)))
        self._imu_states_shm.add_blocks(name='ang_rate', data=np.zeros((3,1)))

        self._limb_contacts_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='LIMB_CONTACTS', init=False)
        # timestamp (1) + position (3) + normal (3) + depth (1) + wrench force (3) + wrench torque (3)
        # This is multiplied by the amount of contacts per sensor on addition there is a variable number of measurements per update
        # so it is additionally multiplied by 50 to provide enough of a buffer
        # self._limb_contacts_shm.add_blocks(name='all_data', data=np.zeros((14*self.max_contacts_per_sensor+1, self.num_contact_sensors)))
        self._limb_contacts_shm.add_blocks(name='on', data=np.zeros((self.num_contact_sensors, 1)))

        self._body_force_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='BODY_FORCE', init=False)
        self._body_force_shm.add_blocks(name='force', data=np.zeros((3,1)))

        self._body_torque_shm = shmx.SHMSegment(robot_name=self.robot_name, seg_name='BODY_TORQUE', init=False)
        self._body_torque_shm.add_blocks(name='torque', data=np.zeros((3,1)))

        try:
            self._world_parameters_shm.connect_segment()
            self._model_parameters_shm.connect_segment()
            self._joint_states_shm.connect_segment()
            self._joint_force_commands_shm.connect_segment()
            self._position_pid_gains_shm.connect_segment()
            self._joint_position_commands_shm.connect_segment()
            self._joint_limits_shm.connect_segment()
            self._effort_limits_shm.connect_segment()
            self._body_pose_shm.connect_segment()
            self._imu_states_shm.connect_segment()
            self._limb_contacts_shm.connect_segment()
            self._body_force_shm.connect_segment()
            self._body_torque_shm.connect_segment()


        except posix_ipc.ExistentialError as error:
            self._world_parameters_shm.initialize = True
            self._model_parameters_shm.initialize = True
            self._joint_states_shm.initialize = True
            self._joint_force_commands_shm.initialize = True
            self._position_pid_gains_shm.initialize = True
            self._joint_position_commands_shm.initialize = True
            self._joint_limits_shm.initialize = True
            self._effort_limits_shm.initialize = True
            self._body_pose_shm.initialize = True
            self._imu_states_shm.initialize = True
            self._limb_contacts_shm.initialize = True
            self._body_force_shm.initialize = True
            self._body_torque_shm.initialize = True

            self._world_parameters_shm.connect_segment()
            self._model_parameters_shm.connect_segment()
            self._joint_states_shm.connect_segment()
            self._joint_force_commands_shm.connect_segment()
            self._position_pid_gains_shm.connect_segment()
            self._joint_position_commands_shm.connect_segment()
            self._joint_limits_shm.connect_segment()
            self._effort_limits_shm.connect_segment()
            self._body_pose_shm.connect_segment()
            self._imu_states_shm.connect_segment()
            self._limb_contacts_shm.connect_segment()
            self._body_force_shm.connect_segment()
            self._body_torque_shm.connect_segment()

    def _initialize_clients(self):
        connected = False
        timeouts = 0
        while (not connected and timeouts < self._max_timeouts):
            try:
                self._world_address = self.dir + self.world_name
                self._world_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self._world_socket.connect(self._world_address)

                self._model_address = self.dir + self.robot_name
                self._model_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self._model_socket.connect(self._model_address)     
                
                connected = True

            except socket.error as error:
                timeouts = timeouts + 1
                time.sleep(3)
                print("GAZEBO NOT RUNNING! Timeouts remaining: ", self._max_timeouts-timeouts)

        if (not connected):
            raise Exception("Please start Gazebo before running again!")

    def get_current_position(self):
        return self._joint_states_shm.get()['position']

    def get_current_velocity(self):
        return self._joint_states_shm.get()['velocity']

    def get_current_force(self):
        return self._joint_states_shm.get()['force']

    def get_current_time(self):
        return self._joint_states_shm.get()['time']

    def get_body_position(self):
        return self._body_pose_shm.get()['position']

    def get_body_quaternion(self):
        return self._body_pose_shm.get()['quaternion']

    def get_body_euler_angles(self):
        return self._body_pose_shm.get()['euler_angles']

    def get_body_velocity(self):
        return self._body_pose_shm.get()['velocity']

    def get_imu_acceleration(self):
        return self._imu_states_shm.get()['accel']

    def get_imu_angular_rate(self):
        return self._imu_states_shm.get()['ang_rate']

    def get_limb_contacts(self):
        return self._limb_contacts_shm.get()['on']

    # def get_limb_wrench(self):
    #     return self._limb_contacts_shm.get()['sn_force'], self._limb_contacts_shm.get()['sn_torque']

    # def get_contacts_sensor_num(self):
    #     return self._limb_contacts_shm.get()['sensor_num']
    
    def get_contacts(self):
        return self._limb_contacts_shm.get()['all_data']

    # def get_contacts_normal(self):
    #     return self._limb_contacts_shm.get()['normal']

    # def get_contacts_depth(self):
    #     return self._limb_contacts_shm.get()['depth']

    # def get_contacts_position(self):
    #     return self._limb_contacts_shm.get()['wrench_force']

    # def get_contacts_position(self):
    #     return self._limb_contacts_shm.get()['wrench_torque']

    def set_command_force(self, force):
        # TODO: Change this so can just input array instead of a dict
        data = {}
        data['data'] = np.array(force).reshape((self.num_joints,1))
        self._joint_force_commands_shm.set(data)

    def set_command_positions(self, position):
        # TODO: Change this so can just input array instead of a dict
        data = {}
        data['data'] = np.array(position).reshape((self.num_joints,1))
        self._joint_position_commands_shm.set(data)

    def pause_physics(self):
        self._world_socket.send(b"pause_physics")
        self._world_socket.recv(1024)

    def unpause_physics(self):
        self._world_socket.send(b"unpause_physics")
        self._world_socket.recv(1024)

    def step_simulation(self):
        self._world_socket.send(b"step_simulation")
        self._world_socket.recv(1024)

    def reset_simulation(self, initial_pose=None):
        if initial_pose:
            self.set_command_positions(initial_pose)
        else:
            self.set_command_positions(np.zeros(self.num_joints))
        self._world_socket.send(b"reset_simulation")
        self._world_socket.recv(1024)

    def turn_on_ludicrous_mode(self):
        data = self._world_parameters_shm.get()
        stru = data['data'].ctypes.data_as(ctypes.POINTER(WorldParameters)).contents
        stru.real_time_update_rate = 0.0
        self._world_socket.send(b"update_world_parameters")
        self._world_socket.recv(1024)

    def turn_off_ludicrous_mode(self):
        data = self._world_parameters_shm.get()
        stru = data['data'].ctypes.data_as(ctypes.POINTER(WorldParameters)).contents
        stru.real_time_update_rate = 1000.0
        self._world_socket.send(b"update_world_parameters")
        self._world_socket.recv(1024)

    def set_real_time_update_rate(self, rate):
        data = self._world_parameters_shm.get()
        stru = data['data'].ctypes.data_as(ctypes.POINTER(WorldParameters)).contents
        stru.real_time_update_rate = rate
        self._world_socket.send(b"update_world_parameters")
        self._world_socket.recv(1024)

    def set_step_size(self, step_size):
        data = self._world_parameters_shm.get()
        stru = data['data'].ctypes.data_as(ctypes.POINTER(WorldParameters)).contents
        stru.step_size = step_size
        self._world_socket.send(b"update_world_parameters")
        self._world_socket.recv(1024)

    def set_all_position_pid_gains(self, p_gains, i_gains, d_gains):
        data = {}
        data['data'] = np.zeros((3*self.num_joints,1))
        data['data'][np.arange(0,3*self.num_joints,3)] = np.array(p_gains).reshape((self.num_joints,1))
        data['data'][np.arange(1,3*self.num_joints,3)] = np.array(i_gains).reshape((self.num_joints,1))
        data['data'][np.arange(2,3*self.num_joints,3)] = np.array(d_gains).reshape((self.num_joints,1))
        self._position_pid_gains_shm.set(data)
        self._model_socket.send(b"set_position_pid_gains")
        self._model_socket.recv(1024)

    def set_joint_position_pid_gains(self, joint_idx, p_gain, i_gain, d_gain):
        data = self._position_pid_gains_shm.get()
        data['data'][3*joint_idx] = p_gain
        data['data'][3*joint_idx+1] = i_gain
        data['data'][3*joint_idx+2] = d_gain
        self._position_pid_gains_shm.set(data)
        self._model_socket.send(b"set_position_pid_gains")
        self._model_socket.recv(1024)

    def set_operating_mode(self, mode):
        data = self._model_parameters_shm.get()
        stru = data['data'].ctypes.data_as(ctypes.POINTER(ModelParameters)).contents
        stru.operating_mode = np.int(mode)
        self._model_socket.send(b"update_model_parameters")
        self._model_socket.recv(1024)

    def set_joint_limits(self, lower_limits, upper_limits):
        data = {}
        data['data'] = np.zeros((2*self.num_joints,1))
        data['data'][np.arange(0,2*self.num_joints,2)] = np.array(lower_limits).reshape((self.num_joints,1))
        data['data'][np.arange(1,2*self.num_joints,2)] = np.array(upper_limits).reshape((self.num_joints,1))
        self._joint_limits_shm.set(data)
        self._model_socket.send(b"set_joint_limits")
        self._model_socket.recv(1024)

    def set_effort_limits(self, effor_limits):
        data = {}
        data['data'] = np.array(effor_limits).reshape((self.num_joints,1))
        self._effort_limits_shm.set(data)
        self._model_socket.send(b"set_effort_limits")
        self._model_socket.recv(1024)

    def set_body_force(self, force):
        data = {}
        data['force'] = force
        self._body_force_shm.set(data)    
        self._model_socket.send(b"set_body_force")
        self._model_socket.recv(1024)

    def set_body_torque(self, torque):
        data = {}
        data['torque'] = torque
        self._body_force_shm.set(data)    
        self._model_socket.send(b"set_body_torque")
        self._model_socket.recv(1024)
