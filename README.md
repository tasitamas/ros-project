# ROS2 Turtlesim Project

This is a ros2 turtlesim project which draws the Koch curve or the Koch snowflake by given parameters.

## Table of Contents

- [Required installation](#required-installation)
- [Optional installation](#optional-installation)
- [Clone the project](#clone-the-project)
- [Usage](#usage)
- [Sources](#sources)

## Required installation
### Install git on your system
**Paste these commands into a terminal:**
```bash
sudo apt update
sudo apt install git
```

## Optional installation
### Install Terminator
Paste these commands into a terminal:
```bash
sudo apt update
sudo apt install terminator
```
## Clone the project

### Make sure to clone the project into your ros2 workspace!

```bash 
git clone https://github.com/tasitamas/ros-project.git ~/your/ros2/workspace
```

### Modify setup.py in your ros2 workspace

**Add this line:**

```py
'turtlesim_controller = ros2_course.turtlesim_controller:main'
```
## Usage

### Open two terminals or [terminator](#optional)

***On the first terminal or page navigate to your ros2 workspace**
```bash
cd ~/your/ros2/workspace
```

**Then copy these commands in order:**
```bash
colcon build --symlink-install
ros2 run ~/your/ros2/workspace/src turtlesim_controller
```

**On the second terminal or page run this command:**
```bash
ros2 run turtlesim turtlesim_node
```

## Sources
- [Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [Koch curve](https://en.wikipedia.org/wiki/Koch_snowflake)


