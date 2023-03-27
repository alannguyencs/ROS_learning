# ROS Basic
## What is ROS?
How can you move the Turtlebot robot?
The easiest way is by executing an existing **ROS program** to control the robot. 
A ROS program is executed by using some special files called **launch files**.

## roslaunch
**roslaunch** is the command used to launch a ROS program. Its structure goes as follows: 
```apex
roslaunch <package_name> <launch_file>
roslaunch turtlebot_teleop keyboard_teleop.launch
```

## package
**package** ROS uses packages to organize its programs. You can think of a package as all 
the files that a specific ROS program contains; all its cpp files, python files, configuration files, 
compilation files, launch files, and parameters files.

All those files in the package are organized with the following structure:
+ launch folder: Contains launch files
+ src folder: Source files (cpp, python)
+ CMakeLists.txt: List of cmake rules for compilation
+ package.xml: Package information and dependencies

To go to any ROS package, ROS gives you a command named roscd: 
```apex
roscd <package_name>
roscd turtlebot_teleop
ls
```
It will take you to the path where the package package_name is located.

Notes:
+ Every ROS program that you want to execute is organized in a package.
+ Every ROS program that you create will have to be organized in a package.
+ Packages are the main organization system of ROS programs.


## launch file
Open the launch folder inside the turtlebot_teleop package and check the keyboard_teleop.launch file.
```apex
roscd turtlebot_teleop
cd launch
cat keyboard_teleop.launch
```

All launch files are contained within a **<launch> tag**. Inside that tag, you can see a **<node> tag**, 
where we specify the following parameters:
+ **pkg="package_name"** # Name of the package that contains the code of the ROS program to execute
+ **type="python_file_name.py"** # Name of the program file that we want to execute
+ **name="node_name"** # Name of the ROS node that will launch our Python file
+ **output="type_of_output"** # Through which channel you will print the output of the Python file

```html
<launch>
  <!-- turtlebot_teleop_key already has its own built in velocity smoother -->
  <node pkg="turtlebot_teleop" type="turtlebot_teleop_key.py" name="turtlebot_teleop_keyboard"  output="screen">
    <param name="scale_linear" value="0.5" type="double"/>
    <param name="scale_angular" value="1.5" type="double"/>
    <remap from="turtlebot_teleop_keyboard/cmd_vel" to="/cmd_vel"/>   <!-- cmd_vel_mux/input/teleop"/-->
  </node>
</launch>
```

## Create a ROS package
When we want to create packages, we need to work in a very specific ROS workspace, 
which is known as the **catkin workspace**. The catkin workspace is the directory in your 
hard disk where your **own ROS packages must reside** in order to be usable by ROS. 
Usually, the **catkin workspace directory** is called **catkin_ws**.

Go to the catkin_ws in your WebShell.
```apex
roscd
cd ..
pwd
```

Inside this workspace, there is a directory called src. This folder will contain all the packages created. 
Every time you want to create a new package, you have to be in this directory **(catkin_ws/src)**.

```apex
cd src
catkin_create_pkg <package_name> <package_dependecies>
catkin_create_pkg my_package rospy
```
The **package_name** is the name of the package you want to create, and the **package_dependencies** are the names of 
other ROS packages that your package depends on.

In order to check that our package has been created successfully, we can use some ROS commands related to packages. 
For example:
+ `rospack list` Gives you a list with all of the packages in your ROS system.
+ `rospack list | grep my_package` Filters, from all of the packages located in the ROS system, the package named my_package.
+ `roscd my_package ` Takes you to the location in the Hard Drive of the package, named my_package.

## My first ROS program
+ step 1: create a `src` directory in `my_package`
+ step 1b: create a script `simple.py` in `src`
+ step 2: create a `launch` directory inside the package named `my_package`
+ step 3: create a new launch file inside the launch directory `touch launch/my_package_launch_file.launch`
+ step 4: fill the launch file
+ step 5: execute the roslaunch command in the WebShell in order to launch your program

Sometimes ROS won't detect a new package when you have just created it, so you won't be able to do a roslaunch. 
In this case, you can force ROS to do a refresh of its package list with the command:
`rospack profile`

Python script `simple.py`
```apex
#! /usr/bin/env python

import rospy

rospy.init_node("ObiWan")
rate = rospy.Rate(2)               # We create a Rate object of 2Hz
while not rospy.is_shutdown():     # Endless loop until Ctrl + C
   print("Help me Obi-Wan Kenobi, you're my only hope")
   rate.sleep()                    # We sleep the needed time to maintain the Rate fixed above
    
# This program creates an endless loop that repeats itself 2 times per second (2Hz) until somebody presses Ctrl + C
# in the Shell
```


## Common Issues
A common issue when working with Python scripts is that the execution permission is not yet activated. 
We may check the permission by command `ls -la`. If it is the case, we simply add execution permission to the file:
`chmod +x name_of_file.py`.

## ROS Nodes
You've initiated a node in the previous code but... what's a node? ROS nodes are basically programs made in ROS. 
The ROS command to see what nodes are actually running in a computer is: `rosnode list`.

In order to see information about our node, we can use the next command: `rosnode info <node_name>`. 
This command will show us information about all the connections that our Node has.

## Compile a package
When you create a package, you will usually need to compile it in order to make it work. 
There are different methods that can be used to compile your ROS packages. 
For this course we will present you the most common one.

**catkin make** The command `catkin_make`  will compile your whole src directory, and it needs to be issued in your 
catkin_ws directory in order to work. This is MANDATORY. If you try to compile from another directory, it won't work.

After compiling, it's also very important to source your workspace: `source devel/setup.bash`. 
This will make sure that ROS will always get the latest changes done in your workspace.

Sometimes (for example, in large projects) you will not want to compile all of your packages, but just the one(s) 
where you've made changes. You can do this with the following command: `catkin_make --only-pkg-with-deps <package_name>`

## Parameter Server
A Parameter Server is a **dictionary** that ROS uses to store parameters. These parameters can be used by nodes at 
runtime and are normally used for static data, such as configuration parameters. 
+ `rosparam list` to get a list of these parameters
+ `rosparam get <parameter_name>` to get a value of a particular parameter
+ `rosparam set <parameter_name> <value>`  to set a value to a parameter


To get the value of the '/camera/imager_rate' parameter, and change it to '4.0,' you will have 
to do the following: 
```apex
rosparam get /camera/imager_rate
rosparam set /camera/imager_rate 4.0
rosparam get /camera/imager_rate
```

## Roscore
In order to have all of this working, we need to have a roscore running. 
The roscore is the main process that manages all of the ROS system. 
You always need to have a roscore running in order to work with ROS. 
The command that launches a roscore is: `roscore`
![image](../images/roscore.jpg)

## Environment Variables
ROS uses a set of Linux system environment variables in order to work properly. 
You can check these variables by typing: `export | grep ROS`. The output would be like this:

```apex
declare -x ROSLISP_PACKAGE_DIRECTORIES="/home/user/catkin_ws/devel/share/common-lisp"
declare -x ROS_DISTRO="noetic"
declare -x ROS_ETC_DIR="/opt/ros/noetic/etc/ros"
declare -x ROS_MASTER_URI="http://localhost:11311"
declare -x ROS_PACKAGE_PATH="/home/user/catkin_ws/src:/opt/ros/noetic/share:/opt/ros/noetic/stacks"
declare -x ROS_ROOT="/opt/ros/noetic/share/ros"
```
The most important variables are the ROS_MASTER_URI and the ROS_PACKAGE_PATH:
+ **ROS_MASTER_URI** -> Contains the url where the ROS Core is being executed. Usually, your own computer (localhost).
+ **ROS_PACKAGE_PATH** -> Contains the paths in your Hard Drive where ROS has packages in it.

## So now... what is ROS?
ROS is basically the framework that allows us to do all that we showed along this chapter. 
It provides the background to manage all these processes and communications between them... and much,
 much more!! In this tutorial you've just scratched the surface of ROS, the basic concepts. 
 ROS is an extremely powerful tool. If you dive into our courses you'll learn much more about 
 ROS and you'll find yourself able to do almost anything with your robots!
 
## Questions:
1. Gosh! You just created a package but ROS does not recognize it, what command can you use 
to tell ROS to refresh its database? 