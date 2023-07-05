import time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from RoboticArm import RoboticArm
from tool import pos2trans

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)

client.setStepping(True)
sim.startSimulation()

joint1 = sim.getObject('./L_Joint1')
joint2 = sim.getObject('./L_Joint2')
joint3 = sim.getObject('./L_Joint3')
joint4 = sim.getObject('./Joint4')
joint5 = sim.getObject('./R_Joint3')
joint6 = sim.getObject('./R_Joint2')
joint7 = sim.getObject('./R_Joint1')

joints = [None, joint1, joint2, joint3, joint4, joint5, joint6, joint7]
thetas = [None, np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4]

end = sim.getObject('./R_Base/Dummy')


for i in range(1, 8):
    
    sim.setJointPosition(joints[i], thetas[i])
    
x, y, z = sim.getObjectPosition(end, -1)
a, b, g = sim.getObjectOrientation(end, -1)

arm = RoboticArm()
arm.forward_kinetic(thetas)

print("CoppeliaSim:")
print(pos2trans(x, y, z, a, b, g, is_deg=False))

print("Python:")
print(arm.T_0_7)

# Change the parent
# sim.setObjectParent(int objectHandle, int parentObjectHandle, bool keepInPlace)

# Stop simulation
sim.stopSimulation()

# Restore the original idle loop frequency:
sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)




# while (t := sim.getSimulationTime()) < 3:
#     sim.setJointPosition(joint1, 2 * np.pi * t / 3)

#     # message = f'Simulation time: {t:.2f} s'
#     # print(message)
    
#     sim.addLog(sim.verbosity_scriptinfos, message)
#     client.step()  # triggers next simulation step
#     time.sleep(0.01)


