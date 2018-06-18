# Rapyuta Cloud Bridge Analyzer

### Summary

This contains an implementation of the could bridge test.
This test measure the delay and count the number of message drop

## Installation
##### Prerequisite
- Ubuntu 16.04
- [ROS Kinetic desktop-full](http://wiki.ros.org/kinetic/Installation/Ubuntu)

##### Package build
- install dependency
```
git clone git@bitbucket.org:rapyutians/cloud_bridge.git
rosdep update
source /opt/ros/indigo/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y -r
```
- create workspace
```
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
cd src
ln -s <path to cloud bridge repository>/* .
```
- build package
```
cd catkin_ws
source /opt/ros/indigo/setup.bash
catkin build <pkg_name> # for instance rr_cloud_bridge_analyzer
```

## Test process
##### Prerequisite
- Two computers with this build package
- Rabbit MQ

#### Run test
##### PC1
1. sync two computer clock and record time offset
```
ntpdate -b 10.81.1.185(=ip of PC2)
ntpdate -q 10.81.1.185
```
2. record output time offset
3. run test node
```
cd catkin_ws
source ../devel/setup.bash
rosclean purge #not always required but it is better since bag file can be huge.
roslaunch rr_cloud_bridge_analyzer cloud_bridge_test.launch 
```
##### PC2
1. run test node
```
cd catkin_ws
source ../devel/setup.bash
rosclean purge
roslaunch rr_cloud_bridge_analyzer cloud_bridge_test.launch this_pc:=2 another_pc:=1
```
##### Analyze data
1. get bag file from both pc
2. git clone unstable repositories
```
git clone git@bitbucket.org:rapyutians/unstable.git
git checkout yu_cb_test #todo
```
3. build plotting tools
```
cd catkin_ws/src
ln -s <path to unstable repository>/* .
cd ..
catkin build rr_tools_plotting
```
4. plotting and output average and etc.
```
cd catkin_ws
source ../devel/setup.bash
python src/rr_cloud_bridge_analyzer/scripts/plotting_cb_test.py <bag file from PC1 or PC2>
```