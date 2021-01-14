# ROS Tutorial Session 2

## Recap from last week

- ROS uses a publisher-subscriber system
  - If any component of the system fails, the rest of the system can work (unless it's the ROS master)
  - If the ROS master fails, all components of the system will wait for the ROS master

- Can any of you think where ROS can be useful?
- Can any of you think why ROS is useful for making robust robotics?

- Setting up a ROS workspace
  - Setting `ROS_HOSTNAME` and `ROS_MASTER_URI`
  - Source the workspace
- `rosrun`
  - What is this?
  - How do we use it?
- `rqt_graph`
  - What is this for?
- `roslaunch`
  - When do we use this?
- `rostopic`
  - What is this useful for?
- `rosparam`
  - What do we use this for?


Remember:

- Plenty of opportunities for ROS Engineers (Singapore and Overseas)
- High paying jobs 5k - 8k SGD bracket depending on skill

What about companies that do not use ROS?

- ROS is used as yardstick for hiring managers to gauge an applicant's exposure to robotics, robotic systems and architecture

If you intend to do Robotics, Industry 4.0 or IoT, learning some ROS will not hurt you.

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

## ROS Service Example

ROS Service is a type of node that performs a function only when it is requested of it.

Start the ROS master:

```bash
$ roscore
```

Start the ROS server:

```bash
$ rosrun srv_example srv_example_server 
```

You will notice that the server will say "ready to add two ints"

Run the ROS Client:

```bash
$ rosrun srv_example srv_example_client 3 4
```

In this case the client is calling the server to add 2 integers and readback the output.

If you bringup the rostopic list you will notice that there are no topics.

If the server is closed and a request is sent, the rostopic will hang and wait for a server to come online.

### Service file

Access the service file

```bash
$ nano ~/ROS_examples/src/srv_example/AddTwoInts.srv
```

It looks very similar to a `rosmsg` file!

```bash
int64 A
int64 B
---
int64 Sum
```


## Making and building a rospy package

Pre-requisites:

- You have to have **created** and **sourced** your catkin workspace!

  - Create some workspace (example_ws)

  - Create a src directory inside that workspace

  - Go to the root of the workspace, and use `$ catkin_make`

  - Then `$ source devel/setup.bash`

### 1. Initialise your package

```shell
$ cd <your_workspace_directory>/src
$ catkin_create_pkg package_name rospy <any other dependencies, including standard ones!>

# Eg. catkin_create_pkg my_pkg rospy std_msgs
```

### 2. Populate src with your rospy scripts

Or, if you want to go modular, put them in sub-directories!

> **Example folder structure:**
>
> my_catkin_ws
>
> - src
>   - CMakeLists.txt
>   - **YOUR ROSPY PACKAGE**
>     - CMakeLists.txt (This is the one you edit)
>     - package.xml (This too!)
>     - **setup.py**
>     - src
>       - **PUT YOUR PYTHON SCRIPTS HERE**

### 3. Create setup.py

#### **Setup.py**

^ Super important! Put it in the **root** of your **package**!

Example setup.py:

```python
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    
    # State your package directories within /src here
    packages = ['package_1', 'package_2'],
    
    # Script locations
    scripts = ['scripts/script_name'],
    
    # root/src, basically
    package_dir = {'': 'src'},
    
    # Your Python dependencies (eg. 'serial')
    install_requires = ['python_module_1', 'python_module_2']
)

setup(**setup_args)
```

### 4. Write your package description (package.xml)

For more info, read the **catkin tutorial**

Example package.xml:

>  **NOTE:** Python dependencies are defined using the `<exec_depend>` tags. But using the name from the rosdistro_list!
>
> In so doing, it's slightly different from declaring Python dependencies in setup.py.
>
> Writing it this way allows catkin to install it for other people when they install via catkin

```python
<?xml version="1.0"?>
<package format="2">
  <name>basic_pub_sub</name>
  <version>0.0.0</version>
  <description>A minimal rospy basic pub-sub package!</description>

  <author email="methyldragon@gmail.com">methylDragon</author>
  <maintainer email="methyldragon@gmail.com">methylDragon</maintainer>
  <url type="website">http://github.com/methylDragon</url>

  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  
</package>
```
> - **Build Dependencies** specify which packages are needed to build this package. This is the case when any file from these packages is required at build time. This can be including headers from these packages at compilation time, linking against libraries from these packages or requiring any other resource at build time (especially when these packages are find_package()-ed in CMake). In a cross-compilation scenario build dependencies are for the targeted architecture.
>
> - **Build Export Dependencies** specify which packages are needed to build libraries against this package. This is the case when you transitively include their headers in public headers in this package (especially when these packages are declared as (CATKIN_)DEPENDS in catkin_package() in CMake).
>
> - **Execution Dependencies** specify which packages are needed to run code in this package. This is the case when you depend on shared libraries in this package (especially when these packages are declared as (CATKIN_)DEPENDS in catkin_package() in CMake).


### 5. Configure the build (CMakeLists.txt)

For more info, read the **catkin tutorial**

Example CMakeLists.txt:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(<package_name>)

find_package(catkin REQUIRED COMPONENTS
  rospy
  <other_dependencies>
)

# Enable python building
catkin_python_setup()

# Initialise the export variables
# Giving no arguments still initialises the variables (eg. CATKIN_PACKAGE_BIN_DESTINATION)
catkin_package()

# This is for installing SCRIPTS into the Install space
# Note: ONLY INSTALL THE EXECUTABLES YOU WANT TO BE ABLE TO ROSRUN!!
install(PROGRAMS
  <YOUR SOURCE_CODE DIRS HERE>
  <folders/SOURCE_CODE>
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 6. Build the package

- Go to the root of your workspace
- Run `catkin_make` and ensure no errors occured
- Source your workspace again `source devel/setup.bash`

### 7. Verify the package

Start a ROS master

```shell
$ roscore
```

Run your nodes

```shell
$ rosrun your_package_name node_name
```

#### **False Failures**

> You might find that the first time you run your package, the command will not autocomplete, because ROS takes awhile to find the package.
>
> To speed it along, either manually type out the rosrun command, or use `rospack profile` to rebuild the package tree.


### Activity!

In the package you just created, modify the `class_pub_sub` node:
- Instead of publishing and subscribing `std_msgs/String`, publish `msg_example_msgs/My_msg` type of message
- The `name` field of the `My_msg` will come from the param `/info_param`
- Remember to update your `package.xml` and `CMakeLists.txt` accordingly as wel!

> Hint: refer to the code in `param_example` and `msg_example`

