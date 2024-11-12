# ros2_MinimalMoveitThreadClass

This repository demonstrates how to implement a MoveIt node running in a parallel thread, which allows the node spinner to continue running. This enables the node to receive incoming messages from subscriber topics or service calls while executing motion planning tasks, allowing dynamic behavior modification even during execution.

## Overview

The package provides a minimal example of how to run a basic UR routine, while the Node spinner remains alive in the background.

## Usage

Launch the example node:
```bash
ros2 launch minimal_moveit_thread_class launch_ur_example.launch.py
```

This will start:
- A MoveIt instance for the UR robot
- The threaded MoveIt node
- RViz for visualization

## Installation

Create and setup a ROS 2 workspace:

```bash
mkdir -p ros2_ws/src
cd ros2_ws
cd ./src
git clone https://github.com/pep248/MinimalMoveitThreadClass.git
cd ..
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Note

If you are using this package inside WSL, it may complain about the OGRE library:

```log
terminate called after throwing an instance of 'Ogre::UnimplementedException'
what():  OGRE EXCEPTION(9:UnimplementedException):  in GL3PlusTextureGpu::copyTo at /build/ogre-next-UFfg83/ogre-next-2.2.5+dfsg3/RenderSystems/GL3Plus/src/OgreGL3PlusTextureGpu.cpp (line 677)
```

 To fix this, you can try the following:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### Dependencies
- MoveIt
- UR Robot packages
- RViz
- Universal_Robots_ROS2_GZ_Simulation

## Author
* Josep Rueda Collell: rueda_999@hotmail.com
