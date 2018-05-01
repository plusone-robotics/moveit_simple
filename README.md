# Moveit Simple

## Installation
1. Clone repository into workspace [It doesn't have to be called moveit_simple_ws]
```
mkdir moveit_simple_ws && cd moveit_simple_ws
mkdir src && cd src
catkin_init_workspace
git clone -b kinetic-devel https://github.com/plusone-robotics/moveit_simple.git
```

2. Source Dependencies
```
wstool init . moveit_simple/.travis.rosinstall
cd ..
```

3. Package Dependencies
```
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

4. Build
```
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
or
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash
```

## Running Tests with Rviz Visualization
```
roslaunch moveit_simple test_display.launch
rostest moveit_simple <Test Name>_utest.launch -r

Example:
rostest moveit_simple motoman_mh5_utest.launch -r
```

Refresh the RobotModel by un-checking and re-checking the checkbox.