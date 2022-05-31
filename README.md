# Bachelor-E2205
Repository that organizes the work of group E2205

This repository consists of several packages, but some of them are only used as interfaces between different BlueROV2s.

"Bluerov2_communication"-package is used in parallell with QGroundControl for the standard BlueROV2 Heavy.

"Bluerov2_interface_bridge"-package is used to interface the controller_package with actuator drivers on a specific, modified BlueROV2 Heavy.

"Bluerov_interfaces"-package is a message interface used by the modified BlueROV2 Heavy, and is needed to build "Bluerov2_interface_bridge".



# Installation:

If cloning to another folder, remember to change the commands to match the correct folder.

Clone repository:
```
cd ~
mkdir -p bs_ws/src && cd bs_ws/src
git clone https://github.com/ErikRowe/Bachelor-E2205
```

Build and source in terminal.
```
cd ~/bs_ws
colcon build --packages-select controller_package && source install/setup.bash
```
To start the controller node use:
```
ros2 launch controller_package controller_launch.py
```

If using a joystick, start joy in a new terminal window:
```
ros2 run joy joy_node
```

If connecting with Basso, bluerov2_interface_bridge and bluerov_interfaces is required:
```
cd ~/bs_ws
colcon build --packages-select bluerov_interfaces && source install/setup.bash
colcon build --packages-select bluerov2_interface_bridge && source install/setup.bash
```

To start the interface bridge:
```
ros2 run bluerov2_interface_bridge bridge_node
```

Use params.yaml to change node parameters during runtime. Upload changes using:
```
ros2 param load /Control_Node ~/bs_ws/src/Bachelor-E2205/controller_package/params/params.yaml
```
