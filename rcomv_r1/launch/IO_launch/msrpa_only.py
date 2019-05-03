# XML generator for running ONLY msrpa nodes

import sys
## Append easylaunch directory to path
## !!! THE DIRECTORY MUST BE THE DIRECTORY TO THE EASYLAUNCH MODULE: https://github.com/jusevitch/easylaunch
sys.path.append('/media/james/Data/code2/easylaunch')

from math import pi, cos, sin
from random import randint, sample
import easylaunch as el


### TODO: MAKE THIS PROGRAM TAKE FILENAME COMMANDS FROM COMMAND LINE

## Variables

common_namespace = "R" # Root of the namespace for all nodes

n = 10
k = 7
F = 0

formation_r = 5
formation_angle = []

for i in range(n):
    formation_angle.append(i*2*pi/n)


eta = 10

## Create launchFile object
launch = el.launchFile()

    # nh_private_.param<double>("t0", t0, 0);
    # nh_private_.param<double>("xc", xc, 0);
    # nh_private_.param<double>("yc", yc, 0);
    # nh_private_.param<double>("Rad", Rad, 100);
    # nh_private_.param<double>("omegad", omegad, 0);
    # nh_private_.param<double>("phi0", phi0, 0);
    # nh_private_.param<double>("leng", leng, 10);
    # nh_private_.param<double>("psi", psi, 0);
    # nh_private_.param<double>("v", v, 0);
    # nh_private_.param<double>("start_L", start_L, 0);

# Args for launchFile
launch.arg = {
    "n": str(n),
    "k": str(k),
    "F": str(F),
    "eta": str(eta),
    "radius": str(formation_r),
    "t0": str(0),
    "xc": str(0),
    "yc": str(0),
    "Rad": str(10),
    "wd": str(0),
    "phi0": str(0),
    "leng": str(10),
    "psi": str(0),
    "v": str(0),
    "start_L": str(0),
    "common_namespace": common_namespace
}


## Include elements

launch.include = []

# include element - empty_world.launch
ew = el.include(file="$(find gazebo_ros)/launch/empty_world.launch")
ew.arg = {
    "world_name": "$(find rcomv_uav)/worlds/basic.world",
    "paused": "true"
}
launch.include.append(ew)


## Node elements

launch.node = []
switch_node = el.node(launch_prefix="xterm -e", name="switch_node", output="screen", pkg="rcomv_r1", type="switch")
launch.node.append(switch_node)

msrpa_node = el.node(launch_prefix="xterm -e gdb -ex run --args", name="msrpa_node", output="screen", pkg="rcomv_r1", type="msrpa_node")
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
    "leng",
    "psi",
    "v",
    "start_L",
    "common_namespace"
]

array_msrpa = msrpa_node.copy(n)

# The namespace must be set here.
for i in range(n):
    array_msrpa[i].ns = common_namespace + str(i+1)

for i in range(n):
    array_msrpa[i].param = {
        "idx": str(i + 1),
        "role": str(2)
    }

# Array of malicious agents
malicious = sample(range(1,n),F)

for j in malicious:
    array_msrpa[j].param["role"] = str(1)

leaders = [1]

for j in leaders:
    array_msrpa[j].param["role"] = str(3)


launch.node += array_msrpa

## Write the file

# Command line option
if len(sys.argv) > 1:
    launchfile_name = sys.argv[1] # Must be 1. The 0th entry of the array will be "msrpa_only.py".
else:
    launchfile_name = "./msrpa_only.launch"

launch.write(filename=launchfile_name)

