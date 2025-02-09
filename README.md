# Nav2 Task - ROS 2 Navigation Simulation

This repository contains a ROS 2 package for simulating a robot navigating in an environment using the ROS 2 Navigation Stack (Nav2). The robot is simulated in Gazebo and RViz, where it follows navigation commands to reach a specified target.

## Installation

### Clone the Package

1. Clone the repository into the `src` directory of your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/AniketSingh1207734/Nav2_task.git
    ```

2. Navigate to the root of your workspace:
    ```bash
    cd ~/ros2_ws
    ```

3. Build the workspace:
    ```bash
    colcon build
    ```

## Running the Simulation

### 1. Source the Workspace

After building the package, open a new terminal and source your workspace:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```
### 2. Launch Gazebo with the Robot

Launch Gazebo with the robot in a world (e.g., obstacle.world):

```bash
ros2 launch my_robot launch_sim.launch.py world:=./ros2_ws/src/my_robot/worlds/obstacle.world
```

### 3. Launch the Navigation Stack

In another terminal, source your workspace and launch the Nav2 navigation stack:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot nav2_launch.launch.py
```

### 4. Set the 2D Pose Estimate

In RViz, set the estimated 2D pose of the robot to help the navigation stack localize the robot in the environment.

### 5. Command the Robot to Navigate

Open a new terminal, source your workspace, and run the robot navigator node to send navigation goals:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot robot_navigator
```

You can now give the robot a target position, and it will move in both Gazebo and RViz to reach the destination.

