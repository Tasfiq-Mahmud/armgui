
# ArmGui

I have used ROS2 Humble and multiple python libraries. They are - tkinter,PIL,pybullet,threading,numpy,time etc


## Prerequisites

You need two more python libraries installed outside requirements.
They are numpy and pybullet 

```bash
  #installation
  pip install numpy
  pip install pybullet
```

    
## Run

1. First of all, you have to create a ros workspace and then in the src folder clone this repo.

```bash
  git clone https://github.com/Tasfiq-Mahmud/armgui.git
```

2. Now, you have to build the package.
Go to the root workspace directory and run

```bash
  colcon build
```

3. Source the workspace.

```bash
  source install/setup.bash #setup.zsh if you are using zsh
```

4. Now, run the program

```bash
  ros2 run armgui gui
```

5.(optional) If you want to see the urdf model in rviz. Then run

```bash
  ros2 launch armgui display.launch.py
```

