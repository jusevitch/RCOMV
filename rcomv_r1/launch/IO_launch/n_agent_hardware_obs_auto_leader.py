# Launch file script for n agents with IO_collision avoidance and msrpa
# This is for hardware with physical obstacles (i.e. gazebo = 0)

import sys
## Append easylaunch directory to path
sys.path.append('/home/dasc/Downloads/MSR/launch/easylaunch')
# sys.path.append('/media/james/Data/code2/easylaunch')

from math import pi, cos, sin
from random import randint, sample
import easylaunch as el

## Variables

common_namespace = "R" # Root of the namespace for all nodes

n = 6
k = 3
F = 1

eta = 10

xc = 0
yc = -1
wd = 0.09
mu2 = 0.5

Leng = 1
psi = 0.7854
V = .2  
startLIdx = 0

number_of_obstacles = 0

obstacle_radii = [0.25, 0.25]

rover_numbers = [2,3,4,5,6,7]

gdb_xterm_output = 1

# Configurations for auto leader

# Fast cw
# slow cw
# slow ccw
# fast ccw

time_duration_list = [30.0 for i in range(4)] # 30 seconds each

trajectory_types = ['circular' for i in range(4)]

# Configurations

trajectory_r = 1.25

phi01 = 0.0
wd1 = 0.17
trajectory1 = [7.0, 0.0, 0.0, 0.0, trajectory_r, wd1, phi01, 0.0] # Padded with 0.0 at end

phi02 = time_duration_list[0]*wd1 + phi01
wd2 = 0.09
trajectory2 = [7.0, 0.0, 0.0, 0.0, trajectory_r, wd2, phi02, 0.0]

phi03 = time_duration_list[1]*wd2 + phi02
wd3 = -0.09
trajectory3 = [7.0, 0.0, 0.0, 0.0, trajectory_r, wd3, phi03, 0.0]

phi04 = time_duration_list[2]*wd3 + phi03
wd4 = -0.17
trajectory4 = [7.0, 0.0, 0.0, 0.0, trajectory_r, wd4, phi04, 0.0]

list = [
    trajectory1,
    trajectory2,
    trajectory3,
    trajectory4
]

flattened_trajectory_list = []
for i in range(4):
    flattened_trajectory_list += list[i]

formation_r = 1.0
formation_angle = []

for ii in range(n):
    formation_angle.append(ii*2*pi/n + pi/2)


formation = [2.0, formation_r, n*1.0]
flattened_formation_list = []
for i in range(4):
    flattened_formation_list += formation


## Create launchFile object

launch = el.launchFile()

# Global args for launchFile

rover_args = {
    "ugv_name": "r1_sim",
    "world_name": "basic",
    "enable_logging": "true",
    "enable_ground_truth": "true",
    "n": str(n),
    "qi_x": "10",
    "qi_y": "0",
    "qi_theta": "1.5708",
    "qf_x": "-10",
    "qf_y": "0",
    "qf_theta": "1.5708",
    "poly_k": "60",
    "T": "30",
    "endless": "true",
    "b": "0.2",
    "k1": "0.5",
    "k2": "0.5",
    "k3": "2",
    "vmax": "0.5",
    "wmax": "1.3",
    "mu2": str(mu2),
    "wd": str(wd),
    "R": str(trajectory_r),
    "R1": "1",
    "R2": "1",
    "ds": "0.4",
    "dc":"0.5",
    "gazebo": "0",
    "xc": str(xc),
    "yc": str(yc),
    "obstacle_radii": str(obstacle_radii),
    "number_of_obstacles": str(number_of_obstacles),
    "path_type": "NaN"
}

# Why did I use str(0) instead of "0"? I have no idea...it's all the same.
msrpa_args = {
    "k": str(k),
    "F": str(F),
    "eta": str(eta),
    "radius": str(formation_r),
    "t0": str(0),
    "xc": str(xc),
    "yc": str(yc),
    "Rad": str(trajectory_r),
    "wd": str(wd),
    "phi0": str(0.0),
    "Leng": str(Leng),
    "psi": str(psi),
    "V": str(V),
    "startLIdx": str(startLIdx),
    "common_namespace": common_namespace,
    "is_malicious": "0"
}

# Merges the two dictionaries together (https://stackoverflow.com/a/26853961). 
# !!! msrpa_args will overwrite similar values in rover_args !!!
launch.arg = {**rover_args, **msrpa_args} 


## Include elements

launch.include = []

# include element - Gazebo empty_world.launch
# ew = el.include(file="$(find gazebo_ros)/launch/empty_world.launch")
# ew.arg = {
#     "world_name": "$(find rcomv_uav)/worlds/$(arg world_name).world",
#     "paused": "true"
# }
# launch.include.append(ew)

# include element - ugv. We'll copy this to make all the ugvs.

ugv = el.include(file="$(find rcomv_r1)/launch/IO_launch/ugv_with_IO_collision.launch")
ugv.defarg = [
    "enable_logging",
    "endless",
    "b",
    "k1",
    "k2",
    "k3",
    "vmax",
    "wmax",
    "mu2",
    "wd",
    "R",
    "R1",
    "R2",
    "n",
    "ds",
    "dc",
    "gazebo",
    "xc",
    "yc",
    "obstacle_radii",
    "number_of_obstacles",
    "path_type"
]

ugv.arg = {
    "rover_number": "1",
    "agent_index": "1",
    "ugv_name": "R1",
    "name_space": "R1",
    "x": "0",
    "y": "0",
    "z": "0",
    "yaw": "1.5708",
    "Ri": "0.0",
    "alphai": "0.0",
    "qi_x": "9.9662",
    "qi_y": "-0.053919",
    "qi_theta": "1.5498",
    "qf_x": "-9.9225",
    "qf_y": "-0.0078972",
    "qf_theta": "1.5918",
    "poly_k": "60",
    "T": "30",
    "gdb_xterm_output": str(gdb_xterm_output)
}

ugv_list = ugv.copy(n)

# Different args for the other UGVs

for j in range(n):
    temp_args = {
        "x": "0",
        "y": str(5*j - 5*n/2),
        "z": "0.1",
        "rover_number": str(rover_numbers[j]),
        "agent_index": str(j+1),
        "ugv_name": "R" + str(rover_numbers[j]),
        "name_space": "R" + str(rover_numbers[j]),
        "Ri": str(formation_r),
        "alphai": str(formation_angle[j])
    }
    ugv_list[j].arg.update(temp_args)

launch.include += ugv_list

# vicon_bridge node

# launch.include += [el.include(file="$(find vicon_bridge)/launch/vicon.launch")]


## Node elements

launch.node = []

# Switch node
# switch_node = el.node(name="switch_node", pkg="rcomv_r1", type="switch")
# launch.node.append(switch_node)

# Ugv nodes
builder_node = el.node(name="builder_node", pkg="state_graph_builder", type="builder")
builder_node.defparam = ["n", "gazebo"]
builder_node.param = {"base_ns": "/R"}
builder_node.rosparam = {"rover_numbers_list": str(rover_numbers)}
launch.node.append(builder_node)

# MS-RPA nodes
msrpa_node = el.node(name="msrpa_node", pkg="rcomv_r1", type="msrpa_node")
msrpa_node.defparam = [
    "n",
    "k",
    "F",
    "eta",
    "radius",
    "t0",
    "xc",
    "yc",
    "Rad",
    "wd",
    "phi0",
    "Leng",
    "psi",
    "V",
    "startLIdx",
    "common_namespace",
]

array_msrpa = msrpa_node.copy(n)

# The namespace must be set here.
for i in range(n):
    array_msrpa[i].ns = common_namespace + str(rover_numbers[i])

for i in range(n):
    array_msrpa[i].param = {
        "idx": str(i + 1),
        "role": str(2),
        "Rf": str(trajectory_r),
        "rover_number": str(rover_numbers[i]),
        "clear_params": "true"
    }

for i in range(n):
    in_neighbor_array = []
    for j in range(k):
        in_neighbor_array.append(rover_numbers[i-(j+1)]) # Should work with negative indices too.
    array_msrpa[i].rosparam = {
        "in_neighbors": str(in_neighbor_array)
    }
        

print([array_msrpa[i].param["rover_number"] for i in range(len(array_msrpa))])

# Set all agents' mode to non-malicious by default

for k in array_msrpa:
    k.param["is_malicious"] = str(0)

# Array of malicious agents
malicious = sample(range(0,n),F)
print(malicious)

for j in malicious:
    # array_msrpa[j].param["role"] = str(1) # j-1?
    array_msrpa[j].param["is_malicious"] = str(1)

leaders = [1,2,3]

for j in leaders:
    array_msrpa[j-1].param["role"] = str(3)
    array_msrpa[j-1].param["trajectory_type"] = "NaN"

# for j in [4,5,6]:
#     array_msrpa[j-1].output = "screen"
#     array_msrpa[j-1].launch_prefix = "xterm -e gdb -ex run --args"


launch.node += array_msrpa

# Debugging: Set leader (1) gdb debugging on
# launch.node[-n].output = "screen"
# launch.node[-n].launch_prefix = "xterm -e gdb -ex run --args"

# launch.node[-(n-1)].output = "screen"
# launch.node[-(n-1)].launch_prefix = "xterm -e gdb -ex run --args"

# launch.node[-(n-2)].output = "screen"
# launch.node[-(n-2)].launch_prefix = "xterm -e gdb -ex run --args"

launch.node[-1].output = "screen"
launch.node[-1].launch_prefix = "xterm -e gdb -ex run --args"

# Node for msrpa_auto_leader

node_auto = el.node(name="auto_leader", pkg="rcomv_r1", type="msrpa_auto_leader", output="screen", launch_prefix="xterm -e gdb -ex run --args")
node_auto.param = {
    "number_of_configurations": "4",
}
node_auto.rosparam ={
    "trajectory_types": str(trajectory_types),
    "flattened_trajectory_list": str(flattened_trajectory_list),
    "flattened_formation_list": str(flattened_formation_list),
    "time_duration_list": str(time_duration_list)
}

launch.node += [node_auto]


# Make all nodes have xterm and gdb output for debugging if gdb_xterm_output = True above
if gdb_xterm_output == 1:
    for i in launch.node:
        i.output = "screen"
        i.launch_prefix = "xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T " + str(i.name) + " -e gdb -ex run --args"

# Debugging: Set leader (1) gdb debugging on
# launch.node[-n].output = "screen"
# launch.node[-n].launch_prefix = "xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T  -e gdb -ex run --args"


## Write the file

# Command line option
if len(sys.argv) > 1:
    launchfile_name = sys.argv[1] # Must be 1. The 0th entry of the array will be "msrpa_only.py".
else:
    launchfile_name = "./" + str(n) + "_agent_hardware_obs_auto.launch"

launch.write(filename=launchfile_name)