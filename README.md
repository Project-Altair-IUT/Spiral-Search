# Spiral-Search
ROS2 Package for simulating and executing square spiral search pattern to find points of interests, for URC Autonomous stack 

## How to install
Copy the package folder into the src folder of your ros2 workspace.
Package was made and developed for ros2 foxy, not sure if it works on ros2 humble.

## Relevant files for rover operation
For understanding the code, check out the following scripts
 - rover_spiral  (Subscribes to `rotation_vector` for yaw, `\gps\fix` for gps data | Publishes cmd_vel msg to `cmd_vel`)
 - move_rover    (Subscribes to `cmd_vel` topic and controls arduino board)
 - spiral_search (Provides a list of target gps to visit, publishes to topic `gps_tagets_list`)

## Files to run spiral search simulation
Simulation code runs on articubot_one package, check out josh newans github repository for the robot package
articubot one package for running simulation will be added soon.
Two types of spirals are simulated
 1) Square spiral
 2) Circular spiral

The package also provides a script for simulating the spiral using the articubot one robot urdf.
To run the simulation, first run the `spiral_search` node
Then run `gps_nav` node

A topic diagram explaining the inter node communication is attached below.
![Spiral_Search_ROS_Topography](https://github.com/user-attachments/assets/b8979b3e-3a82-4366-b63e-8912cec2351b)
