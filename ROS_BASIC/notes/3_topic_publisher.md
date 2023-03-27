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




