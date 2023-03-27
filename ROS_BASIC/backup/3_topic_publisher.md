# Topics: Publishers
## Topic Publisher
To create a ROS package named **my_publisher_example_pkg**
+ create the package **my_publisher_example_pkg**
```commandline
cd ~/catkin_ws/src
catkin_create_pkg my_publisher_example_pkg rospy std_msgs
``` 
+ create a new folder inside your ROS package named **scripts**
```commandline
cd ~/catkin_ws/src/my_publisher_example_pkg
mkdir scripts
```
+ Inside the **scripts** folder, create the example Python script
```commandline
cd ~/catkin_ws/src/my_publisher_example_pkg/scripts
touch simple_topic_publisher.py
chmod +x simple_topic_publisher.py
```
+ Paste the following code into the script:
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

+ to compile and source your workspace
```commandline
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

+ execute the script with the command rosrun
```commandline
rosrun my_publisher_example_pkg simple_topic_publisher.py
```

A **topic** is like a **pipe**. 
**Nodes** use **topics** to **publish information** for other nodes so that they can communicate.

On your webshell, type rostopic list and check for a topic named /counter:
```commandline
rostopic list | grep  '/counter'
```
You can request information about a topic:
```commandline
rostopic info /counter
```

Now, type rostopic echo /counter and check the output of the topic in realtime.
You should see a succession of consecutive numbers:
```commandline
rostopic echo /counter
```

To get a **list of available topics** in a ROS system, you have to use the next command:
```commandline
rostopic list
```

To **read the information that is being published** in a topic, use the next command:
```commandline
rostopic echo <topic_name>
```

This command will start printing all of the information that is being published 
into the topic, which sometimes (ie: when there's a massive amount of information, 
or when messages have a very large structure) can be annoying. In this case, 
you can read just the last message published into a topic with the next command:
```commandline
rostopic echo <topic_name> -n1
```

To get information about a certain topic, use the next command:
```commandline
rostopic info <topic_name>
```

Finally, you can check the different options that rostopic command has by using the next command:
```commandline
rostopic -h
```

## Messages
As you may have noticed, topics handle information through messages. 
There are many different types of messages.
In the case of the code you executed before, the message type was an **std_msgs/Int32**, 
but ROS provides a lot of different messages. You can even create your own messages, 
but it is recommended to use ROS default messages when its possible.

Messages are defined in **.msg files**, which are located inside 
a **msg directory** of a package. To **get information about a message**, 
you use the next command:
```commandline
rosmsg show <message>
rosmsg show std_msgs/Int32
```
![text](images/std_msgs.png)
The sample output shows that:
+ std_msgs: is a package
+ Int32: is a message
+ `count = Int32()` the message type is **std_msgs/Int32**
+ int32: datatype
+ data: variable name

## Questions
1. In the ROS Python API, which class is used to create a topic publisher? `rospy.Publisher`
2.  What does this command do - `rostopic echo /talker`? prints out the information on /talker
3. You have a publisher `pub`, which is connected to the topic `/cmd_vel`, 
how do you send a message `move` to the `/cmd_vel` topic? `pub.publish(move)`
4.  To get more information about a ROS message, you run? `rosmsg show`
5. How do you get more information about a topic "/chatter"? `rostopic info /chatter`


