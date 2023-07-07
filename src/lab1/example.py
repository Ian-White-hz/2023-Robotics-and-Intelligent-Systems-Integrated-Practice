# Before running this example, make sure you have installed the following package.
# pip install coppeliasim-zmqremoteapi-client numpy
# You can find more information about ZeroMQ remote API
# in the file path <Coppeliasim_install_path>\programming\zmqRemoteApi
# or on https://github.com/CoppeliaRobotics/zmqRemoteApi
#
# You can get more API about coppleliasim on https://coppeliarobotics.com/helpFiles/en/apiFunctions.htm
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time


print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

# When simulation is not running, ZMQ message handling could be a bit
# slow, since the idle loop runs at 8 Hz by default. So let's make
# sure that the idle loop runs at full speed for this program:
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
print('Default idle loop frequency: ', defaultIdleFps)
sim.setInt32Param(sim.intparam_idle_fps, 0)
print('New idle loop frequency: ', sim.getInt32Param(sim.intparam_idle_fps))

# Run a simulation in stepping mode:
client.setStepping(True)
sim.startSimulation()

# Get object handle
l_joint1 = sim.getObject('./L_Joint1')
l_joint2 = sim.getObject('./L_Joint2')
l_joint3 = sim.getObject('./L_Joint3')

while (t := sim.getSimulationTime()) < 30:
    sim.setJointPosition(l_joint1, 2 * np.pi * t / 9)
    sim.setJointPosition(l_joint2, 2 * np.pi * t / 9)
    sim.setJointPosition(l_joint3, 2 * np.pi * t / 9)
    message = f'Simulation time: {t:.2f} s'
    print(message)
    sim.addLog(sim.verbosity_scriptinfos, message)
    client.step()  # triggers next simulation step
    time.sleep(0.01)

# Change the parent
# sim.setObjectParent(int objectHandle, int parentObjectHandle, bool keepInPlace)

# Stop simulation
sim.stopSimulation()

# Restore the original idle loop frequency:
sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

print('Program ended')
