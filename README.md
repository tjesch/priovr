priovr
======

ROS packages developed by ...

**Maintainers:** 
Thomas Esch
Francisco Suárez Ruiz, [http://www.romin.upm.es/fsuarez/](http://www.romin.upm.es/fsuarez/)

### Documentation

  * See the installation instructions below.
  * This repository.
  * Throughout the various files in the packages.
  * For questions, please use [http://answers.ros.org](http://answers.ros.org)

### Build Status
TODO

## Installation

### Basic Requirements

  1. Install [ROS Hydro](http://wiki.ros.org/hydro/Installation/Ubuntu) (**Desktop Install** Recommended)
  2. Add your user to the `dialout` group so you can access the `/dev/ttyACM0` port.
  
```
$ sudo apt-get install ros-hydro-desktop python-wstool
$ sudo adduser $(whoami) dialout
``` 

### Repository Installation

Go to your ROS working directory. e.g.
```
cd ~/catkin_ws/src
``` 
Use the `wstool` to install the repository
```
wstool init .
wstool merge https://raw.github.com/tjesch/priorvr/master/priorvr.rosinstall
wstool update
``` 
Install any missing dependencies using rosdep:
```
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro hydro
``` 
Now compile your ROS workspace. e.g.
```
cd ~/catkin_ws && catkin_make
``` 

### Testing Installation

Be sure to always source the appropriate ROS setup file, which for Hydro is done like so:
```
source /opt/ros/hydro/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try any of the `.launch` files in the package: (e.g. `human_rviz.launch`)
```
roslaunch priovr human_rviz.launch fake_state:=true
``` 

## Changelog
TODO

## Roadmap
### 0.1.0 (2013-07-14)
* Initial Release

## Tutorials
TODO
