# URDF2ROBCOGEN {#mainpage}

**Authors**: Jan Carius (jcarius@ethz.ch) and Ruben Grandia (rgrandia@ethz.ch), based on an earlier version by Diego Pardo (depardo@ethz.ch)<br>

**Maintainers**: Jan Carius (jcarius@ethz.ch) and Ruben Grandia (rgrandia@ethz.ch)

## What is this repository for?

This tool generates the [RobCoGen](https://robcogenteam.bitbucket.io/) description files (.kindsl & .dtdsl) from a [URDF](http://wiki.ros.org/urdf) description format.

## Building
[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_leggedrobotics/urdf2robcogen/master)](https://ci.leggedrobotics.com/job/github_leggedrobotics/job/urdf2robcogen/job/master/)

**Dependencies**

* [Eigen](http://eigen.tuxfamily.org)
* [Boost](https://www.boost.org/)
* [ros::urdf](http://wiki.ros.org/urdf)
* [kindr](https://github.com/ethz-asl/kindr) (a kinematics library for robotics)

**Build the executable**

We use [catkin](http://wiki.ros.org/catkin) as a build system.
To build the bare version use
```bash
catkin build urdf2robcogen
```

**Build the documentation**

We use `rosdoc_lite` with doxygen backend to build documentation for this catkin package.
```base
rosdoc_lite /path/to/package
```


## Usage

Two modes of operation are possible.

### A) Run as standalone application (plain URDF file)
You must have a URDF description of your robot as a plaintext xml file.
This should not contain [xacro](http://wiki.ros.org/xacro) macros, i.e., you must invoke the xacro command beforehand to produce the URDF file.

To generate the robcogen description, execute
```bash
./urdf2robcogen_script RobotName /path/to/description.urdf
```

The generated files are written into the directory where the executable was executed (i.e., the output of `pwd`).

### B) Using a launch file (xacro urdf file)
The urdf2robcogen.launch file converts a robot description on the parameter server.

```bash
roslaunch urdf2robcogen urdf2robcogen.launch robot_name:=myRobot description_name:=robot_description
```

See xacro_example.launch to see how to integrate this with generation of the robot description.

As a default, output files are placed in the /generated folder of this package.

## Main differences between URDF and RobCoGen/Kindsl ##

+ URDF:
  * Frame rotations are extrinsic (rotations about a fixed frame)
  * The axis of rotation of the joints is set with the vector property 'axis' of the joint element
  * There is no concept of *Frame*
  * Artificial links are used to define frames (a.k.a., link\_frames), they may or may not have inertia
  * Artificial Joints (usually fixed) are used to connect link\_frames to the real links
  * It is common to find URDF with 'virtual' links for compatibility with other tools
  * Virtual links might have incomplete information (i.e., inertial parameters)
  * The inertial parameters are expressed w.r.t a frame at the CoM of the link


+ RobCoGen:
  * Frame rotations are intrinsic (rotations about the current frame)
  * Frame rotation order is x,y,z
  * The axis of rotation of the joint is always Z
  * Links always carry inertia and must have a parent joint that is moving (i.e., not fixed)
  * Additional frames can be defined at any link
  * Inertial parameters are expressed w.r.t. the link frame (i.e., the corresponding joint frame)
  * It includes a special feature to set a different frame for expressing the inertial parameters (ref\_frame)


## Caveats
* Double-check the ordering of the joints/links in the generated files. They may differ from what you expect.
