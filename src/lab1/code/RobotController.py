from RoboticArm import RoboticArm
from Tool import pos2trans, change_base
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import numpy as np
import time


class RobotController:
    def __init__(self) -> None:

        self.robot = RoboticArm()
        self.theta = [0, 0, 0, 0, 0, 0, 0]
        # self.theta = [np.pi/4, 0, np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4]

        # self.T_W = pos2trans(x=0.650, y=0, z=0.235, alpha=0, beta=0, gamma=0, is_deg=False)

        self.T_0 = self.robot.forward_kinetic(self.theta)
        self.T_1 = pos2trans(x=0, y=0.3, z=0.15, alpha=np.pi,
                             beta=0, gamma=-np.pi/2, is_deg=False)

        self.T = self.T_0

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

        for i in range(7):
            self.sim.setJointPosition(self.joints[i], self.theta[i])

    def set_state1(self):

        T = pos2trans(x=0, y=0.6, z=0, alpha=np.pi,
                      beta=0, gamma=-np.pi/2, is_deg=False)

        self.theta = self.robot.inverse_kinetics(T, self.theta)
        self.joints.reverse()
        self.links.reverse()

        for i in range(6):
            self.sim.setJointPosition(self.joints[i], self.theta[i])
        self.sim.setJointPosition(self.joints[6], self.theta[6]+np.pi)

        self.joints.reverse()
        self.links.reverse()

    def set_state2(self):

        T = pos2trans(x=0, y=0.3, z=0, alpha=np.pi,
                      beta=0, gamma=-np.pi/2, is_deg=False)

        self.theta = self.robot.inverse_kinetics(T, self.theta)

        self.joints.reverse()
        self.links.reverse()

        for i in range(6):
            self.sim.setJointPosition(self.joints[i], self.theta[i])
        self.sim.setJointPosition(self.joints[6], self.theta[6])

        self.joints.reverse()
        self.links.reverse()

    def set_state(self):

        T = pos2trans(x=0, y=0.6, z=0, alpha=np.pi,
                      beta=0, gamma=-np.pi/2, is_deg=False)
        self.theta = self.robot.inverse_kinetics(T, self.theta)
        T_new = change_base(T)
        self.theta = self.robot.inverse_kinetics(T_new, self.theta)
        for i in range(6):
            self.sim.setJointPosition(self.joints[i], self.theta[i])
        self.sim.setJointPosition(self.joints[6], self.theta[6])
    # def set_traj(self):

    #     for i in range(100):

    #         T = pos2trans(x=0, y=0.3, z=0.0015*i, alpha=np.pi, beta=0, gamma=-np.pi/2, is_deg=False)

    #         self.theta = self.robot.inverse_kinetics(T, self.theta)

    #         for i in range(7):
    #             self.sim.setJointPosition(self.joints[i], self.theta[i])

    #         time.sleep(0.01)
    def reverse_kinetics(self):
        self.joints.reverse()
        self.links.reverse()
        self.sim.setObjectParent(self.links[0], -1, True)
        for i in range(7):
            self.sim.setObjectParent(self.joints[i], self.links[i], True)
            self.sim.setObjectParent(self.links[i+1], self.joints[i], True)

    def shut_down(self):

        # Stop simulation
        self.sim.stopSimulation()

        # Restore the original idle loop frequency:
        self.sim.setInt32Param(self.sim.intparam_idle_fps, self.defaultIdleFps)


if __name__ == "__main__":

    np.set_printoptions(precision=2, linewidth=90)

    rc = RobotController()

    # rc.reverse_kinetics()

    # rc.set_state1()

    # rc.reverse_kinetics()

    # rc.set_state2()

    rc.set_state3()

    # rc.set_init()

    # rc.shut_down()
