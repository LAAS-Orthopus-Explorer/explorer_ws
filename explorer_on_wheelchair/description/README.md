# URDF  

## simulation.urdf.xacro

This URDF combine explorer_old_description, gripper_pincette_description, wheelchair_description, simulation.ros2_control and simulation.gazebo. It is the URDF used for the simulation.

# ros2_control

## simulation.ros2_control.xacro

The `ros2_control` tag specifies hardware configuration of the robot and allows to access and control the robot interfaces. 

# gazebo

## simulation.gazebo.xacro

Gazebo plugin parses the `ros2_control` tags and loads the appropriate hardware interfaces and controller manager
