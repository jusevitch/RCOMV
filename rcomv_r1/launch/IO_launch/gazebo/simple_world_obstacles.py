### A file for generating simple launch files of the empty world with obstacles

import sys
## Append easylaunch directory to path
sys.path.append('/media/james/Data/code2/easylaunch')

from math import pi, cos, sin
import easylaunch as el


## Variables



## Create launchFile object
launch = el.launchFile()

# Args for launchFile


## Include elements

launch.include = []

# Include element: empty_world.launch
ew = el.include(file="$(find gazebo_ros)/launch/empty_world.launch")
ew.arg = {
    "world_name": "$(find rcomv_r1)/worlds/basic.world",
    "paused": "true"
}
launch.include.append(ew)

## Node elements

launch.node = []

# Trucks in a circle!
number_of_trucks = 2
radius = 40

truck = el.node(name="obs", pkg="gazebo_ros", type="spawn_model")
truck_array = truck.copy(number_of_trucks)

args_string = "-file $(find rcomv_r1)/models/pickup/model.sdf -sdf -model "

for i in range(number_of_trucks):
    truck_array[i].args = args_string + truck_array[i].name + " -x " + str(round(radius*cos(i*2*pi/number_of_trucks),2)) + " -y " + str(round(radius*sin(i*2*pi/number_of_trucks),2))

launch.node += truck_array

## Write the file
# Command line option
if len(sys.argv) > 1:
    launchfile_name = sys.argv[1] # Must be 1. The 0th entry of the array will be "msrpa_only.py".
else:
    launchfile_name = "./" + str(number_of_trucks) + "_obstacles_trucks.launch"

launch.write(filename=launchfile_name)