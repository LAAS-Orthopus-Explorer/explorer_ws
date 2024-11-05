# Explorer on wheelchair

This package allows to simulate the explorer on a wheelchair using `ros2_control` and Gazebo

## View the URDF

To simply view the URDF in RViz, open a terminal and launch the `view_explorer_on_wheelchair.launch.py`.  

```
ros2 launch explorer_on_wheelchair view_explorer_on_wheelchair.launch.py
```

In RViz, change the fixed frame from "odom" to "chassis" to see the urdf appear.

With the `joint_state_publisher_gui` you can now change the position of every joint.


## Launch the simulation with position command only

To launch the simulation in Gazebo, open a terminal and launch the `simulation.launch.py`.  

```
ros2 launch explorer_on_wheelchair simulation_qp.launch.py
```

You can run the simulation with some arguments :

* `gui:=false` to deactivate RViz
* `spacenav:=false` to deactivate spacenav

To control the explorer you can use the GUI or a space mouse.
To control the wheelchair and the user's, you can use an xbox controller.

if the joint_state_broadcaster controller failed to activate, open a new terminal and run

```
ros2 control set_controller_state joint_state_broadcaster active
```