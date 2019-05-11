# XML generator for msrpa with IO_control_collision nodes

import sys
## Append easylaunch directory to path
sys.path.append('/media/james/Data/code2/easylaunch')

from math import pi, cos, sin
import easylaunch as el


### TODO: MAKE THIS PROGRAM TAKE FILENAME COMMANDS FROM COMMAND LINE

## Variables
n = 10 # Number of agents

formation_r = 5
formation_angle = []
for i in range(n):
    formation_angle.append(i*2*pi/n)


## Create launchFile object
launch = el.launchFile()

# Args for launchFile
launch.arg = {
    "ugv_name": "r1_sim",
    "world_name": "basic",
    "enable_logging": "true",
    "enable_ground_truth": "true",
    "n": str(n),
    "path_type": "circular",
    "qi_x": "10",
    "qi_y": "0",
    "qi_theta": "1.5708",
    "qf_x": "-10",
    "qf_y": "0",
    "qf_theta": "1.5708",
    "poly_k": "60",
    "T": "30",
    "endless": "true",
    "b": "0.05",
    "k1": "0.5",
    "k2": "0.5",
    "vmax": "1.5",
    "wmax": "4",
    "wd": "0.09",
    "R": "10",
    "R1": "1",
    "R2": "1",
    "ds": "1",
    "dc":"3",
    "gazebo": "1"
}

# Args that I don't think are needed, but might be needed later
arg_debatable = {
    "wmsr_node": "false",
    "k": "7",
    "F": "1",    
} 

## Include elements

launch.include = []

# include element - empty_world.launch
ew = el.include(file="$(find gazebo_ros)/launch/empty_world.launch")
ew.arg = {
    "world_name": "$(find rcomv_uav)/worlds/$(arg world_name).world",
    "paused": "true"
}
launch.include.append(ew)

# include element - ugv. We'll copy this to make all the ugvs.

ugv = el.include(file="$(find rcomv_r1)/launch/IO_launch/ugv_with_IO_collision.launch")
ugv.defarg = [
    "enable_logging",
    "path_type",
    "endless",
    "b",
    "k1",
    "k2",
    "vmax",
    "wmax",
    "wd",
    "R",
    "R1",
    "R2",
    "n",
    "ds",
    "dc",
    "gazebo"
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
}

ugv_list = ugv.copy(n)

# Different args for the other UGVs

for j in range(0,n):
    temp_args = {
        "x": "0",
        "y": str(5*j - 5*n/2),
        "z": "0.1",
        "rover_number": str(j+1),
        "agent_index": str(j+1),
        "ugv_name": "R" + str(j+1),
        "name_space": "R" + str(j+1),
        "Ri": str(formation_r),
        "alphai": str(formation_angle[j])
    }
    ugv_list[j].arg.update(temp_args)

launch.include += ugv_list

## Node elements

launch.node = []
# switch_node = el.node(launch_prefix="xterm -e", name="switch_node", output="screen", pkg="rcomv_r1", type="switch")
# launch.node.append(switch_node)

builder_node = el.node(name="builder_node", output="screen", pkg="state_graph_builder", type="builder", launch_prefix="xterm -e gdb -ex run --args")
builder_node.defparam = ["n", "gazebo"]
builder_node.param = {"base_ns": "/R"}
launch.node.append(builder_node)

## Write the file

launch.write(filename="./n_agent_circletraj_circleform.launch")

