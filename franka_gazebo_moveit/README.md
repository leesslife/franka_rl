# Franka Emika Panda Gazebo Package with Moveit Integration

Clone & build. Dependencies: moveit, ros_control

Can run & control Gazebo Franka with either joint **position** control, or joint **effort** control. In both cases, the controllers are **joint trajectory controllers** 

To launch:

1. 
    a. run gazebo simulation with position controllers `roslaunch franka_gazebo_moveit simulator_position.launch`
    This should start an empty Gazebo world with Franka Panda with **RED** link0 and end effector
    b. run gazebo simulation with effort controllers `roslaunch franka_gazebo_moveit simulator_effort.launch`
    This should start an empty Gazebo world with Franka Panda with **GREEN** end effector


2. run moveit planning `roslaunch franka_gazebo_moveit gazebo_ctrl_gui.launch`
from rviz motion planning, plan & execute 


TODO:

- panda\_arm\_**hand** integration
- controllers partially tuned
- joint position control (w/o trajectory)
- custom motion control from source - example