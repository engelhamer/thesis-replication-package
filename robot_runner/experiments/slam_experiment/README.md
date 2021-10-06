# Experiment guide

## Prerequisites
### Remote PC
- Configure the remote PC for ROS Noetic according to the [TurtleBot3 manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)
- Generate a public key to use for establishing an SSH connection to the TurtleBot
- Install the custom profiler packages in your catkin workspace
  - [ros1-ina219-profiler-service](https://github.com/engelhamer/thesis-replication-package/tree/master/ros_packages/ros1-ina219-profiler-service)
  - [ros1-resource-profiler-service](https://github.com/engelhamer/thesis-replication-package/tree/master/ros_packages/ros1-resource-profiler-service)

### TurtleBot
- Assemble and configure the robot for ROS Noetic according to the [TurtleBot3 manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup)
  - Use `ubuntu` as the username
- Install the SLAM packages under evaluation
  - [Cartographer installation guide](https://google-cartographer.readthedocs.io/en/latest/) (be sure to install the package _on the robot itself_!)
  - `sudo apt install ros-noetic-gmapping`
  - `sudo apt install ros-noetic-hector-mapping`
  - `sudo apt install ros-noetic-slam-karto`
- Install the custom profiler packages in your catkin workspace
  - [ros1-ina219-profiler-service](https://github.com/engelhamer/thesis-replication-package/tree/master/ros_packages/ros1-ina219-profiler-service) (see the README there as well)
  - [ros1-resource-profiler-service](https://github.com/engelhamer/thesis-replication-package/tree/master/ros_packages/ros1-resource-profiler-service) (see the README there as well)
- Add the previously generated public key from the remote PC to the list of trusted SSH connections
- Set the static IP address of the TurtleBot to `192.168.1.222` (or update the address in the [experiment configuration](https://github.com/engelhamer/thesis-replication-package/blob/master/robot_runner/experiments/slam_experiment/config.py))
- Clone the replication package repository (or at least the path `thesis-replication-package/robot_runner/experiments/slam_experiment/turtlebot`) into the home folder of the TurtleBot

## Running the experiment
The below steps have to be performed / checked before any sequence of experiment runs.

- Update any local paths that may not be appropriate for your system in the [experiment configuration](https://github.com/engelhamer/thesis-replication-package/blob/master/robot_runner/experiments/slam_experiment/config.py)
- Connect the remote PC and TurtleBot to the same local network
- Synchronise the clock of the remote PC and the Raspberry Pi on the TurtleBot, e.g. using [chrony](https://askubuntu.com/questions/787855/how-to-use-chrony-to-synchronize-timestamp-on-two-computers)
- Start the profiler services on the TurtleBot using the [launch file](https://github.com/engelhamer/thesis-replication-package/blob/master/robot_runner/experiments/profilers.launch)
- Start the experiment with `python3.8 robot-runner/ experiments/slam_experiment/config.py` (from the [robot_runner](https://github.com/engelhamer/thesis-replication-package/tree/master/robot_runner) folder)
