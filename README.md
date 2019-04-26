# Moveit Simple

## Installation
1. Clone repository into workspace [It doesn't have to be called moveit_simple_ws]
```
mkdir -p moveit_simple_ws/src && cd moveit_simple_ws/src
git clone -b kinetic-devel https://github.com/plusone-robotics/moveit_simple.git
```

2. Get Source Dependencies
```
cd moveit_simple_ws/src
wstool init . moveit_simple/.travis.rosinstall
```

3. Install Package Dependencies
```
cd moveit_simple_ws
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
4. Configure and Build
```
cd moveit_simple_ws
catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```
**NOTE:** To use gdb with this project set `-DCMAKE_BUILD_TYPE=Debug` instead.

5. Source the Workspace
```
cd moveit_simple_ws
source devel/setup.bash
```

**NOTE:** You will need to re-run step #5 in any new terminal


## Building and Running Tests

1.  Run the MoveIt Simple Tests
```
roscd moveit_simple
catkin run_tests --no-deps --this -i
```

1. Get test results:
```
cd moveit_simple_ws
catkin_test_results build/moveit_simple
```

### Run the Tests Using RViz as a Visualizer
1. Launch the RViz visualizer
```
roslaunch moveit_simple test_display.launch
```

2. Launch a Test. (Be sure to substitute `<Test Name>` with either `motoman_mh5` or `kuka_kr210`)
```
rostest moveit_simple <Test Name>_utest.launch -r
```

Eg:
```
rostest moveit_simple motoman_mh5_utest.launch -r
```

**NOTE:** Refresh the RobotModel by un-checking and re-checking the checkbox.
