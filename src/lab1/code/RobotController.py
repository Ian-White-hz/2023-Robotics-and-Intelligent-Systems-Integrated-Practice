from RoboticArm import RoboticArm
from Tool import pos2trans, change_base
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import numpy as np
import time
import matplotlib.pyplot as plt


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

        self.theata_record = []
        self.time_record = []
        self.end_record = []

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
        self.theata_record.append(self.theta)
        self.end_record.append(self.robot.forward_kinetic(self.theta)[2,3])
        self.time_record.append(time.time())
    
    def set_traj_theta(self, pose_start, pose_end, T, resolution=100):
        
        x, y, z, alpha, beta, gamma = pose_start
        T_start = pos2trans(x, y, z, alpha, beta, gamma)
        x, y, z, alpha, beta, gamma = pose_end
        T_end = pos2trans(x, y, z, alpha, beta, gamma)
        
        if self.reverse:
            self.reverse_list()
            T_start = change_base(T_start)
            T_end = change_base(T_end)
        
        theta_start = self.robot.inverse_kinetics(T_start, [i - j for i, j in zip(self.theta, self.offset)])
        theta_end = self.robot.inverse_kinetics(T_end, [i - j for i, j in zip(self.theta, self.offset)])
        
        def theta_t(theta_start, theta_end, t, T):
            """
            linear function
            """
            # return theta_start*(1-t/T) + theta_end*t/T
            return theta_start + (theta_end - theta_start) / T ** 2 * t ** 2
        
        for t in np.linspace(0, T, resolution):

            self.theta = theta_t(theta_start, theta_end, t, T)
            self.set_theta()
            self.client.step() 
            time.sleep(T/resolution)

        # while (t := self.sim.getSimulationTime()) < T:
        #     self.theta = theta_t(theta_start, theta_end, t, T)
        #     self.set_theta()
        #     self.client.step()  # triggers next simulation step
        #     time.sleep(0.01)
        
        if self.reverse:
            self.reverse_list()

    def reverse_kinetics(self):
        self.get_offset()
        self.reverse_list()
        self.sim.setObjectParent(self.links[0], -1, True)
        for i in range(7):
            self.sim.setObjectParent(self.joints[i], self.links[i], True)
            self.sim.setObjectParent(self.links[i+1], self.joints[i], True)
        self.reverse = not self.reverse
    
    def set_traj_line(self, pose_start, pose_end ,T, resolution=100):
        for t in np.linspace(0, T, resolution):
            x = pose_start[0] + (pose_end[0] - pose_start[0]) / T * t
            y = pose_start[1] + (pose_end[1] - pose_start[1]) / T * t
            z = pose_start[2] + (pose_end[2] - pose_start[2]) / T * t
            alpha = pose_start[3] + (pose_end[3] - pose_start[3]) / T * t
            beta = pose_start[4] + (pose_end[4] - pose_start[4]) / T * t
            gamma = pose_start[5] + (pose_end[5] - pose_start[5]) / T * t
            self.set_pose((x, y, z, alpha, beta, gamma))
            time.sleep(T/resolution)
        
    
    def set_traj_circle(self, pose_start, pose_end,T, resolution=100):
        for t in np.linspace(0, T, resolution):
            x = pose_start[0] + (pose_end[0] - pose_start[0]) / T * t
            y = 0.4 * x / np.pi / 0.12 ** 2
            z = ( 0.12 ** 2 - x ** 2 ) ** 0.5
            alpha = pose_start[3] + (pose_end[3] - pose_start[3]) / T * t
            beta = pose_start[4] + (pose_end[4] - pose_start[4]) / T * t
            gamma = pose_start[5] + (pose_end[5] - pose_start[5]) / T * t
            self.set_pose((x, y, z, alpha, beta, gamma))
            time.sleep(T/resolution)

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
    # for k in np.linspace(0, 1, 100):
    #     rc.set_pose((-0.003*k, 0, -0.0005*k, np.pi, 0, np.pi/2))
    #     time.sleep(0.01)
    #rc.set_theta((-0.3, 0, , np.pi, 0, np.pi/2))
    rc.set_traj_theta((-0.3, 0, 0, np.pi, 0, np.pi/2), (0.6, 0, -0.05, np.pi, 0, -np.pi/2), 1)
    plt.subplot(1,2,1)
    plt.plot(rc.time_record, rc.theata_record)
 
    plt.title("theta")
    plt.subplot(1,2,2)
    plt.title("end")
    plt.plot(rc.time_record, rc.end_record)
    # rc.set_pose(rc.pose_0_3)
    plt.show()
    rc.reverse_kinetics()
    
    # for k in np.linspace(0, 1, 100):
    #     rc.set_pose((0, -0.6, 0.02*k, np.pi, 0, -np.pi/2))
    #     time.sleep(0.01)
    rc.set_traj_circle(rc.pose_1_1, rc.pose_1_2, 1)    
    # rc.set_state(rc.T_state_1_1)
    
    
    # rc.set_state(rc.T_state_1_2)
    # time.sleep(1)
    # rc.set_state(rc.T_state_1_3)
    # time.sleep(1)
    
    

    # rc.set_state(rc.T_state_2)

    # rc.reverse_kinetics()

    # rc.set_state(rc.T_state_0)
    
    # rc.shut_down()
