# ROS Deconstruction
## Method
1. **DECONSTRUCTION**: we have identified the important parts of ROS that you must master in order to understand 80% of ROS programs. You will concentrate on learning these parts very deep.

1. **REMOVING**: we have removed many things that are not needed and just add noise to your learning.

1. **LEARNING**: we guide you step by step in a progressive manner through all those important parts, starting always from a robot that does things.

1. **PRACTICING**: we make you practice a lot on every step, always on a robot using our simulated robots.

## What do you need to learn to program a robot with ROS?
+ First, you'll need to learn some **Basic Concepts**. 
In the first unit of this course (the Basic Concepts chapter), 
you'll be introduced to some ROS Basic Concepts that you need 
to learn before you start working on more complex concepts.
+ Second, you'll need to know about **Topics**. ROS handles almost all its 
communications through topics. Even more complex communication systems, 
such as services or actions, rely, at the end, on topics. That's why 
they are so important! Through ROS topics, you will, for instance, 
be able to communicate with your robot in order **to make it move, to read your robot's sensor readings, and more**.
+ Third, you'll need to know about **Services**. Services allow you to code a 
specific functionality for your robot, and then provide for anyone to call it. 
For instance, you could create a service that makes your robot move for a 
specific period of time, and then stop. Services are a little bit more 
complex than topics since they are structured in two parts. 
On one side, you have the **Service Server**, which provides the functionality 
to anyone who wants to use it (call it). On the other side, you have the 
**Service Client**, which is the one who calls/requests the service functionality.
+ Fourth, you'll need to know about **Actions**.  Actions are similar to services, 
in the sense that they also allow you to code a functionality for your robot, 
and then provide it so that anyone can call it. The main difference between 
actions and services is that when you call a service, the robot has to wait 
until the service has ended before doing something else. On the other hand, 
when you call an action, your robot can still keep doing something else while 
performing the action.
+ Finally, you'll need to know how to use the **Debugging Tools**. If you are going 
to start working with robots, you need to know this one thing: you will have 
to deal with errors. Fortunately, ROS provides lots of tools that will help you 
to detect what's going on. In the following example, you will be introduced 
to what is likely to be the most important tool of all of them: **RViz**.

## Example commands:
**Topics**
+ To move `roslaunch publisher_example move.launch`
+ To stop `roslaunch publisher_example stop.launch`

Awesome, right? You'll probably have a lot of questions about how this 
whole process works... What you've actually done is to launch a **Publisher**, 
which is a ROS program that writes a **message** into a **topic**, in this case 
into the `/cmd_vel` topic. This topic is the one used to send velocity 
commands to the base of the robot. So, by sending a message through 
this topic, you've made the robot start moving.

**Services**
+ Service Server side: `roslaunch service_demo service_launch.launch`
+ Service Client side: `rosservice call /service_demo "{}"`

**Actions**
+ Start the action: `roslaunch action_demo action_launch.launch`
+ Call the action: `roslaunch action_demo_client client_launch.launch`

**RViZ**
+ start RViZ: `rosrun rviz rviz`
+ Change the Fixed Frame for the **base_link**.
+ Click the "Add" button below the Displays section. Select the RobotModel option.
+ Now, repeat the process and select the LaserScan option.
+ Select the /kobuki/laser/scan topic from the LaserScan options.
+ If you want to, you can also move the robot, for instance, 
using the programs you launched on Example 1.1


## Apply what you learnt to a Real Robot Project
As you master the different concepts of ROS, you will have to apply 
everything you learn during the course in a complete robot project. 
And I'm not talking about any kind of robot project, but a project based 
on a real robot which is located in our facilities in Barcelona.

For this purpose, we will be using the [Real Robot Lab](https://app.theconstructsim.com/RealRobot) tool. 
This amazing tool will allow you to remotely control and run your 
ROS programs, from any place of the world, in a real robot.

But that's not all! After you complete the real robot project, 
you will have the chance to do a live presentation of your project.

## Get a certificate
+ Pass all the course Quizzes
+ Successfully complete the live presentation of your project

