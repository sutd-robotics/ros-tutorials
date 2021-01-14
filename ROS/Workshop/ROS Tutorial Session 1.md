# ROS Tutorial Session 1

## What is ROS?

As you know, ROS is a **publisher-subscriber** framework for facilitating robotics applications. What does this mean, even?

> Imagine you're on an internet forum, with individuals publishing  posts into and subscribing to (and reading from) from the forum topics,  and you should get a pretty good idea of what's going on.

The ROS communication graph is made of ROS nodes (which can run robotics applications) communicating with each other:

- **ROS nodes** can be distributed across different systems, communicating via wired connections, wireless or the internet!
- **ROS nodes** can act as **Publishers** and/or **Subscribers**, and run programs that can react to messages set through ROS
- **Publishers** post **messages** onto **topics**
- **Subscribers** subscribe to **topics**, and receive **messages** as they are published
- All this activity is facilitated by the **ROS Master** (in the analogy, treat it as the forum site itself that hosts the nodes and topics, and mediates the messages!)

![1.1](../assets/1.1.png)

So for a practical example. Consider a simple image processing system implemented in ROS.

You have a camera node that **publishes** to an image_data **topic**, which is **subscribed** to by an image processing node and image display node.

![1.2](../assets/1.2.png)


## Initial Setup and Example Codes

1. Get the ROS examples and tutorial code

   ```bash
   $ cd ~
   $ git clone https://github.com/sutd-robotics/ros-tutorials/tree/noetic
   ```

2. source your ROS workspace with the following command

   ```bash
   $ source /opt/ros/noetic/setup.bash
   ```

  Or you could append it into your ~/.bashrc, so that each time you start a new terminal, you don't need to source it again

   ```shell
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

3. Create a new ROS workspace in the home directory

   ```bash
   $ cd ~
   $ mkdir -p ROS_examples/src
   ```

   `src` is the folder to store the source code for the ROS packages - these have been prepared for you.

4. Copy over the files under `ros-tutorials/ROS/Starter\ Code\ and\ Resources/workshop_examples/` into the `ROS_examples/src` folder

5. Navigate to the root directory of the new ROS workspace AKA `ROS_examples` NOT `ROS_examples/src`. Then start the build process by running:

   ```bash
   $ catkin_make
   ```

6. The ROS workspace is now full of files:

   ```bash
   ROS_examples
       >build
       >devel
       >src
   ```

   `src` - source code for the ROS packages within the ROS workspace

   `build` - contains the built binaries and packages

   `devel` - stores development files for setting up environments and workspace

## Setting up a ROS Workspace

1. Check the `ROS_HOSTNAME` and `ROS_MASTER_URI`

   ```bash
   $ echo $ROS_HOSTNAME
   $ echo $ROS_MASTER_URI
   ```

   The IP addresses should be set to that of the ROS master computer. In this example, the master computer is your own local computer. Thus it has to be set to localhost.

2. Set the `ROS_HOSTNAME` and `ROS_MASTER_URI` to localhost:

   ```shell
   $ export ROS_HOSTNAME=localhost
   $ export ROS_MASTER_URI=http://localhost:11311
   ```

3. Setup the workspace by sourcing the development setup file.

   ```shell
   $ cd ~/ROS_examples
   $ source devel/setup.bash
   ```

4. (Optional) Give all scripts inside the workspace execute authority. Sometimes this is not a given

   ```bash
   $ chmod -R +x ~/ROS_examples
   ```

   

## Running the ROS Workspace and Examples

### ROS Hello World

Running the ROS hello world. It just screams Hello World in the console.

```bash
$ rosrun basic_pub_sub hello_world 
```

This launches one node called `logger`:

```python
rospy.init_node('logger', anonymous = True)
```

Anonymous just allows the node to not clash with existing nodes named `logger`

Open `rqt_graph` to take look at the current node:

```shell
$ rosrun rqt_graph rqt_graph
```

There should only be one node called `logger` with a bunch of randomized numbers trailing it. This is because there is only one node screaming the log message "HELLO WORLD"

```python
log_str = "HELLO WORLD"
        
        # ROS log is usually shown on the Terminal screen
        rospy.loginfo(log_str)
```

Note that the log message is not actually being pushed to a real topic. Thus it is only for logging or debugging purposes.

### Basic Pub Sub

On one terminal run:

```bash
$ rosrun basic_pub_sub basic_pub
```

On another terminal run:

```bash
$ rosrun basic_pub_sub basic_sub
```

There should now be messages being sent from the publisher to the subscriber.

Open `rqt_graph` to take look at the current node:

```shell
$ rosrun rqt_graph rqt_graph
```

You should notice the `talker` node is publishing to the topic `chatter`

```python
pub = rospy.Publisher('chatter', String, queue_size = 10)
```

Meanwhile the `listener` is subscribed to the `chatter` topic which also invokes a callback to publish a log message with the message it receives.

```python
rospy.Subscriber("chatter", String, callback)
```

In fact you can directly publish to the topic.

1. Close the publisher terminal.

2. Publish directly to the topic using the `rostopic` pub command:

   ```bash
   $ rostopic pub /chatter std_msgs/String "data: BYE WORLD''" 
   ```

   Use tab complete to help you get the template message automatically!

   Add `-r <Hz>` to the end of the publish command to publish at a specific frequency.

3. Refresh the `rqt_graph` - the publisher node should be replaced with the `rostopic` node.

## Basic Pub-Sub Launch

Launch files are a more convenient way to package multiple packages and commands into a single file!

```shell
$ roslaunch basic_pub_sub basic_pub_sub_init.launch 
```

Full launch file:

```shell
<launch>
	<node name="talker" pkg="basic_pub_sub" type="basic_pub" output="screen" />
	<node name="listener" pkg="basic_pub_sub" type="basic_sub" output="screen" /> 	
</launch>
```

## Class Pub Sub

Class based implementation of a publisher! Also has a callback for when it receives messages on the same topic.

```shell
$ rosrun class_pub_sub class_pub_sub 
```

If you open `rqt_graph`, you will notice something quite interesting. The node is publishing it to itself! This is because it is both a publisher and subscriber.

```python
self.pub = rospy.Publisher("chatter", String, queue_size=10)
self.sub = rospy.Subscriber("chatter", String, self.msg_cb, queue_size=1)
```

## ROS Message Example

ROS messages are structured data that can be sent through topics. They act like C++ structs!

```shell
$ rosrun msg_example msg_example 
```

Check the message format under `src/msg_example_msgs/msg`

## ROS PARAM Example

It is also possible to allow users to configure ROS params during a package's run time. 

```shell
$ rosrun param_example param_example 
```

You will notice that the log is echoing the value of the parameter `info_param`. You can set this value by running the following command in another window:

```shell
$ rosparam set /info_param NEWTEXT
```


## ROSTOPIC Command

`rostopic` is a very powerful command for managing and debugging ROS topics.

- `rostopic list` - list all topics
- `rostopic echo <topic>` - echo what is being published on a topic
- `rostopic pub` - publish message

