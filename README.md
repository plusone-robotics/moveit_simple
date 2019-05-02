# Moveit Simple

## Installation
1. Clone repository into workspace [It doesn't have to be called moveit_simple_ws]
```
export CATKIN_WS=$HOME/moveit_simple_ws/
mkdir -p $CATKIN_WS/src && cd $CATKIN_WS/src
git clone -b kinetic-devel https://github.com/plusone-robotics/moveit_simple.git
```

2. Get Source Dependencies
```
cd $CATKIN_WS/src && \
wstool init . moveit_simple/.travis.rosinstall
```

3. Install Package Dependencies
```
cd $CATKIN_WS && \
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
4. Configure and Build
```
cd $CATKIN_WS && \
catkin init -w . && \
catkin config --extend /opt/ros/$ROS_DISTRO \
              --blacklist \
                    motoman_bmda3_support \
                    motoman_csda10f_moveit_config \
                    motoman_csda10f_support \
                    motoman_epx_support \
                    motoman_experimental \
                    motoman_gp12_support \
                    motoman_gp7_support \
                    motoman_gp8_support \
                    motoman_mh12_support \
                    motoman_mh50_support \
                    motoman_motomini_support \
                    motoman_mpl80_moveit_config \
                    motoman_mpl_support \
                    moveit_ros_planning_interface \
                    moveit_experimental \
                    moveit_planners_chomp \
                    moveit_setup_assistant \
                    moveit_ros_planning_interface \
                    moveit_ros_manipulation \
                    moveit_ros_visualization \
                    motoman_driver \
                    moveit_chomp_optimizer_adapter \
                    moveit_ros_warehouse \
                    moveit_planners_ompl \
                    moveit_ros_perception \
              --cmake-args -DCMAKE_BUILD_TYPE=Release && \
catkin build
```
**NOTE:** To use gdb with this project set `-DCMAKE_BUILD_TYPE=Debug` instead.

5. Source the Workspace
```
source $CATKIN_WS/devel/setup.bash
```

**NOTE:** Whenever you open a new terminal session, you should re-run the following commands:

```
export CATKIN_WS=$HOME/moveit_simple_ws/
source $CATKIN_WS/devel/setup.bash
```

## Building and Running Tests

Run the following steps from anywhere within `$CATKIN_WS`. (eg. `cd $CATKIN_WS`)

1.  Run the MoveIt Simple Tests
```
catkin build moveit_simple --no-deps -i --catkin-make-args run_tests
```

1. Get test results:
```
catkin_test_results $CATKIN_WS/build/moveit_simple
```

### Run the Tests Using RViz as a Visualizer
Open two terminals and set your environment variables appropriately.

1. Launch the RViz visualizer in the first terminal
```
roslaunch moveit_simple test_display.launch
```

2. Launch a test in the second terminal. (Be sure to substitute `<Test Name>` with either `motoman_mh5` or `kuka_kr210`)
```
rostest moveit_simple <Test Name>_utest.launch -r
```

Eg:
```
rostest moveit_simple motoman_mh5_utest.launch -r
```

**NOTE:** Refresh the RobotModel by un-checking and re-checking the checkbox.
