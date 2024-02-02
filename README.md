# RNMThu
Robotics and Navigation in Medicine Thursday

## Setup and Launch
- Clone the repository into your catkin workspace `src`, e.g. `~/catkin_ws/src/`.

### Installation
Building the catkin package
```bash
    cd <catkin_ws>
    catkin_make RNMThu
     . devel/setup.bash
```
To launch any launch file (e.g. `listen_for_point.launch`), run
```bash
roslaunch RNMThu listen_for_point.launch
```
To run an installed script, use
```
rosrun RNMThu listen_for_point.py
```

### Debugging Setup
For a manual run  without `catkin_make` (e.g. debugging):
- setup the panda simulation via:
`roslaunch franka_example_controllers joint_position_example_controller_sim.launch`
- start the python REPL from `RNMThu` to import the `rnm` packages
- or add the folder `RNMThu` to python package search path via
```python
import sys
sys.path.append(r'<catkin_ws>/src/RNMThu/src')
```

### Adding Python Scripts For Installation
See [http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile#Installing_scripts_and_exporting_modules]


