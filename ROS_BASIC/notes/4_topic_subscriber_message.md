# Understanding ROS Topics: Subscribers & Messages
## Topic Subscriber
You've learned that a topic is a channel where nodes can either write or read information. 
You've also seen that you can write into a topic using a publisher, so you may be thinking 
that there should also be some kind of similar tool to read information from a topic. 
And you're right! That's called a subscriber. A subscriber is a node that reads information 
from a topic.

For this and following examples, you should create a ROS package 
**named my_subscriber_example_pkg**.

First, create the package **my_subscriber_example_pkg**:
```commandline
cd ~/catkin_ws/src
catkin_create_pkg my_subscriber_example_pkg rospy std_msgs
```

Next, create a new folder inside your ROS package named **scripts**
```commandline
cd ~/catkin_ws/src/my_subscriber_example_pkg
mkdir scripts
```

Inside the scripts folder, create the example Python script named **simple_topic_subscriber.py**
```commandline
cd ~/catkin_ws/src/my_subscriber_example_pkg/scripts
touch simple_topic_subscriber.py
chmod +x simple_topic_subscriber.py
```

Python script:
```python
"""
You've basically created a subscriber node that listens to the /counter topic, 
and each time it reads something, it calls a function that does a print of the msg. 
Initially, nothing happened since nobody was publishing into the /counter topic, 
but when you executed the rostopic pub command, you published a message into 
the /counter topic, so the function has printed the number to the console and 
you could also see that message in the rostopic echo output.
"""
#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32 

def callback(msg): 
  print (msg.data)

rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('/counter', Int32, callback)
rospy.spin()
```

to compile and source your workspace
```commandline
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

execute the script with the command `rosrun`:
```commandline
rosrun my_subscriber_example_pkg simple_topic_subscriber.py
```

echo the subscriber:
```commandline
rostopic echo /counter
```

Let's then publish something into the topic and see what happens:
```commandline
rostopic pub <topic_name> <message_type> <value>
rostopic pub /counter std_msgs/Int32 5
```
Now check the output of the console where you did the `rostopic echo` again. 
You should see something like this: `data: 5`

## Prepare CMakeLists.txt and package.xml for custom Message compilation
Now you may be wondering... in case I need to publish some data that is not an Int32, 
which type of message should I use? You can use all ROS defined (**rosmsg list**) messages. 
But, in case none fit your needs, you can create a new one.

In order to create a new message, you will need to do the following steps:
1. Create a directory named 'msg' inside your package
1. Inside this directory, create a file named Name_of_your_message.msg 
(more information down)
1. Modify CMakeLists.txt file (more information down)
1. Modify package.xml file (more information down)
1. Compile
1. Use in code

For example, let's create a message that indicates age, with years, months, and days:
Create a directory msg in your package
```commandline
roscd <package_name>
mkdir msg
```
The Age.msg file must contain this:
```commandline
float32 years
float32 months
float32 days
```

In **CMakeLists.txt**, You will have to edit four functions:
1. find_package()
2. add_message_files()
3. generate_messages()
4. catkin_package()

### find_package()
If you open the CMakeLists.txt file in your IDE, you'll see that almost all of the 
file is commented. This includes some of the lines you will have to modify. Instead 
of copying and pasting the lines below, find the equivalents in the file and uncomment 
them, and then add the parts that are missing.
```commandline
find_package(catkin REQUIRED COMPONENTS
       rospy
       std_msgs
       message_generation   # Add message_generation here, after the other packages
)
```

### add_message_files()
This function includes all of the messages of this package (in the msg folder) to be compiled. 
The file should look like this.
```commandline
add_message_files(
      FILES
      Age.msg
    ) # Dont Forget to UNCOMENT the parenthesis and add_message_files TOO
```

###  generate_messages()
Here is where the packages needed for the messages compilation are imported.
```commandline
generate_messages(
      DEPENDENCIES
      std_msgs
) # Dont Forget to uncoment here TOO
```

### catkin_package()
State here all of the packages that will be needed by someone that executes something from your package. 
All of the packages stated here must be in the package.xml as exec_depend.
```commandline
catkin_package(
      CATKIN_DEPENDS rospy message_runtime   # This will NOT be the only thing here
)
```

Summarizing, this is the minimum expression of what is needed for the **CMakelists.txt** file to work:

Note: Keep in mind that the name of the package in the following example is **topic_ex**, 
so in your case, the name of the package may be different
```commandline
cmake_minimum_required(VERSION 2.8.3)
project(topic_ex)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Age.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

### Modify package.xml
Just add these 3 lines to the package.xml file.
```commandline
<build_depend>message_generation</build_depend> 

<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```

Then, the package.xml would be like this:
```commandline
<?xml version="1.0"?>
<package format="2">
  <name>topic_ex</name>
  <version>0.0.0</version>
  <description>The topic_ex package</description>


  <maintainer email="user@todo.todo">user</maintainer>

  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_export_depend>rospy</build_export_depend>
  <exec_depend>rospy</exec_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
  <export>

  </export>
</package>
```

Now you have to compile the msgs:
```commandline
roscd; cd ..
catkin_make
source devel/setup.bash
```

To verify that your message has been created successfully, 
type in your webshell `rosmsg show Age`. If the structure of the Age message appears, 
it will mean that your message has been created successfully and it's ready to be 
used in your ROS programs.
```commandline
rosmsg show Age
```
+
### Warning
There is an issue in ROS that could give you problems when importing msgs from 
the msg directory. If your package has the same name as the Python file that does 
the import of the msg, this will give an error saying that it doesn't find the msg 
element. This is due to the way Python works. Therefore, you have to be careful to 
not name the **Python file** exactly the same as its **parent package**.

