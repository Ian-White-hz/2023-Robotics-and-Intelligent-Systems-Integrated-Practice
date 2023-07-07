from RoboticArm import RoboticArm
from Tool import pos2trans, change_base
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import numpy as np
import time


class RobotController:
    def __init__(self) -> None:

        self.robot = RoboticArm()
        self.theta = [0, 0, 0, 0, 0, 0, 0]
        
        # 抬起
        self.pose_0_1 = (-0.3, 0, -0.05, np.pi, 0, np.pi/2)
        # 对接前状态
        self.pose_0_2 = (0.6, 0, -0.05, np.pi, 0, -np.pi/2)
        # 对接状态
        self.pose_0_3 = (0.6, 0, 0, np.pi, 0, -np.pi/2)
        
        # 抬起
        self.pose_1_1 = (0, -0.6, 0.02, np.pi, 0, -np.pi/2)
        # 对接前状态
        self.pose_1_2 = (0.12, 0.4, -0.1, np.pi/2, -np.pi/2, 0)
        # 对接状态
        self.pose_1_3 = (0.1, 0.4, -0.1, np.pi/2, -np.pi/2, 0)
        
        self.offset = [0, 0, 0, 0, 0, 0, 0]
        
        self.reverse = False

        """
        Remote API
        """
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        self.defaultIdleFps = self.sim.getInt32Param(
            self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)

        self.client.setStepping(True)
        self.sim.startSimulation()

        self.joints = []
        self.links = []

        self.joints.append(self.sim.getObject('./L_Joint1'))
        self.joints.append(self.sim.getObject('./L_Joint2'))
        self.joints.append(self.sim.getObject('./L_Joint3'))
        self.joints.append(self.sim.getObject('./Joint4'))
        self.joints.append(self.sim.getObject('./R_Joint3'))
        self.joints.append(self.sim.getObject('./R_Joint2'))
        self.joints.append(self.sim.getObject('./R_Joint1'))

        self.links.append(self.sim.getObject('./L_Base'))
        self.links.append(self.sim.getObject('./L_Link1'))
        self.links.append(self.sim.getObject('./L_Link2'))
        self.links.append(self.sim.getObject('./L_Link3'))
        self.links.append(self.sim.getObject('./R_Link3'))
        self.links.append(self.sim.getObject('./R_Link2'))
        self.links.append(self.sim.getObject('./R_Link1'))
        self.links.append(self.sim.getObject('./R_Base'))

    def set_init(self):
        self.theta = [0, 0, 0, 0, 0, 0, 0]
        self.set_theta()
    
    def reverse_list(self):
        self.joints.reverse()
        self.links.reverse()

    def get_offset(self):
        for i in range(7):
            self.offset[i] += 2 * self.theta[i]
            
    def set_pose(self, pose: tuple, is_deg=False):
        """
        设置目标位姿
        """
        x, y, z, alpha, beta, gamma = pose
        T = pos2trans(x, y, z, alpha, beta, gamma, is_deg)
        self.set_state(T)
            
    def set_state(self, T):
        """
        设置目标位姿矩阵
        """
        if self.reverse:
            self.reverse_list()
            T = change_base(T)
        
        # 需要减去offset再比较 
        self.theta = self.robot.inverse_kinetics(T, [i - j for i, j in zip(self.theta, self.offset)])
        
        self.set_theta()
        
        if self.reverse:
            self.reverse_list()

    def set_theta(self):
        """
        设置关节角
        """
        for i in range(7):
            self.sim.setJointPosition(self.joints[i], self.theta[i] + self.offset[i])

    def reverse_kinetics(self):
        self.get_offset()
        self.reverse_list()
        self.sim.setObjectParent(self.links[0], -1, True)
        for i in range(7):
            self.sim.setObjectParent(self.joints[i], self.links[i], True)
            self.sim.setObjectParent(self.links[i+1], self.joints[i], True)
        self.reverse = not self.reverse
    
    # def set_traj(self):

    #     for i in range(100):

    #         T = pos2trans(x=0, y=0.3, z=0.0015*i, alpha=np.pi, beta=0, gamma=-np.pi/2, is_deg=False)

    #         self.theta = self.robot.inverse_kinetics(T, self.theta)

    #         for i in range(7):
    #             self.sim.setJointPosition(self.joints[i], self.theta[i])

    #         time.sleep(0.01)

    def shut_down(self):

        # Stop simulation
        self.sim.stopSimulation()

        # Restore the original idle loop frequency:
        self.sim.setInt32Param(self.sim.intparam_idle_fps, self.defaultIdleFps)


if __name__ == "__main__":

    np.set_printoptions(precision=2, linewidth=90)

    rc = RobotController()
    
    rc.set_init()
    
    rc.reverse_kinetics()

    rc.set_pose((-0.3, 0, 0, np.pi, 0, np.pi/2))
    
    for k in np.linspace(0, 1, 100):
        rc.set_pose((-0.3, 0, -0.05*k, np.pi, 0, np.pi/2))
        time.sleep(0.001)
    rc.set_pose((-0.3, 0, -0.05, np.pi, 0, np.pi/2))
    
    for k in np.linspace(0, 1, 100):
        rc.set_pose((-0.3, 0.3*k, -0.05*k, np.pi, 0, np.pi/2))
        time.sleep(0.001)
    rc.set_pose((-0.3, 0.3, -0.05, np.pi, 0, np.pi/2))
    
    for k in np.linspace(0, 1, 100):
        rc.set_pose((-0.3+0.9*k, 0.3, -0.05*k, np.pi, 0, np.pi/2))
        time.sleep(0.001)
    rc.set_pose((-0.6, 0.3, -0.05, np.pi, 0, np.pi/2))
    
    for k in np.linspace(0, 1, 100):
        rc.set_pose((0.6, 0.3*(1-k), -0.05, np.pi, 0, np.pi/2))
        time.sleep(0.001)
    rc.set_pose((0.6, 0, -0.05, np.pi, 0, -np.pi/2))
    
    for k in np.linspace(0, 1, 100):
        rc.set_pose((0.6, 0, -0.05*(1-k), np.pi, 0, np.pi/2))
        time.sleep(0.001)
    rc.set_pose((0.6, 0, 0, np.pi, 0, -np.pi/2))
    
    rc.set_pose(rc.pose_0_3)
    
    # rc.reverse_kinetics()
    
    # for k in np.linspace(0, 1, 100):
    #     rc.set_pose((0, -0.6, 0.05*k, np.pi, 0, -np.pi/2))
    #     time.sleep(0.001)
        
        
        
    # rc.set_state(rc.T_state_1_1)
    
    
    # rc.set_state(rc.T_state_1_2)
    # time.sleep(1)
    # rc.set_state(rc.T_state_1_3)
    # time.sleep(1)
    
    

    # rc.set_state(rc.T_state_2)

    # rc.reverse_kinetics()

    # rc.set_state(rc.T_state_0)
    
    # rc.shut_down()
