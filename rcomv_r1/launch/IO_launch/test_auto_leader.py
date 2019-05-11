# Launch file script for n agents with IO_collision avoidance and msrpa
# This is for hardware with physical obstacles (i.e. gazebo = 0)

import sys
## Append easylaunch directory to path
# sys.path.append('/home/dasc/Downloads/MSR/launch/easylaunch')
sys.path.append('/media/james/Data/code2/easylaunch')

from math import pi, cos, sin
from random import randint, sample
import easylaunch as el
import random



# Launchfile object

launch = el.launchFile()


# Configurations for auto leader

flattened_trajectory_list = []
flattened_formation_list = []
time_duration_list = [5.0, 5.0, 5.0, 5.0]


for ii in range(4):
    flattened_trajectory_list += [7.0] + [random.random() for jj in range(7)]
    flattened_formation_list += [2.0] + [random.random() for kk in range(2)]




# Nodes

launch.node = []

node_1 = el.node(name="auto_leader", pkg="rcomv_r1", type="msrpa_auto_leader", output="screen", launch_prefix="xterm -e gdb -ex run --args")
node_1.param = {
    "number_of_configurations": "4",
}
node_1.rosparam ={
    "trajectory_types": "['circular', 'square', 'circular', 'square']",
    "flattened_trajectory_list": str(flattened_trajectory_list),
    "flattened_formation_list": str(flattened_formation_list),
    "time_duration_list": str(time_duration_list)
}

launch.node = [node_1]

# Write
launchfile_name = "./test_auto_leader.launch"
launch.write(filename=launchfile_name)