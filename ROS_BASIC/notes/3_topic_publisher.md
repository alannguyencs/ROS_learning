# Topics: Publishers
## Topic Publisher
To create a ROS package named **my_publisher_example_pkg**

1. create the package **my_publisher_example_pkg**
```commandline
cd ~/catkin_ws/src
catkin_create_pkg my_publisher_example_pkg rospy std_msgs
``` 
2. create **scripts**
```commandline
cd ~/catkin_ws/src/my_publisher_example_pkg
mkdir scripts
```
3. Inside the **scripts** folder, create the example Python script
```commandline
cd ~/catkin_ws/src/my_publisher_example_pkg/scripts
touch simple_topic_publisher.py
chmod +x simple_topic_publisher.py
```
Python script:
```python
"""
Creat a topic named /counter, 
and published through it as an integer that increases indefinitely.
"""
#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32 

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/counter', Int32, queue_size=1)
rate = rospy.Rate(2)
count = Int32()
count.data = 0

while not rospy.is_shutdown(): 
  pub.publish(count)
  count.data += 1
  rate.sleep()
```

4. to compile and source your workspace
```commandline
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

5. execute the script with the command rosrun
```commandline
rosrun my_publisher_example_pkg simple_topic_publisher.py
```

## Topic: query information
**Topic list**
```commandline
rostopic list
rostopic list | grep  '/counter'
```
**Topic info**
```commandline
rostopic info <topic_name>
rostopic info /counter
```

**Topic streaming data**
```commandline
rostopic echo <topic_name>
rostopic echo /counter
rostopic echo <topic_name> -n1
```

**Topic with full list of options**
```commandline
rostopic -h
```

## Messages
Topics handle information through messages. 

**message info**
```commandline
rosmsg show <message>
rosmsg show std_msgs/Int32
```
![text](../images/std_msgs.png)
The sample output shows that:
+ std_msgs: is a package
+ Int32: is a message
+ `count = Int32()` the message type is **std_msgs/Int32**
+ int32: datatype
+ data: variable name

## QUIZ
1. In the ROS Python API, which class is used to create a topic publisher? `rospy.Publisher`
2.  What does this command do - `rostopic echo /talker`? prints out the information on /talker
3. You have a publisher `pub`, which is connected to the topic `/cmd_vel`, 
how do you send a message `move` to the `/cmd_vel` topic? `pub.publish(move)`
4.  To get more information about a ROS message, you run? `rosmsg show`
5. How do you get more information about a topic "/chatter"? `rostopic info /chatter`


