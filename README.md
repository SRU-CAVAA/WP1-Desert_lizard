# Enabling robot autonomy through self-regulatory dynamics

From weaving spiders to hibernating mammals and migratory birds, nature presents numerous examples of organisms exhibiting extraordinary autonomous behaviors that ensure their self-maintenance. However, physiological needs often interact and compete with each other, which demands super-autonomous organisms to orchestrate them as a complex set of internal needs rather than as isolated subsystems. This paper presents a synthetic agent equipped with a brain-based neural mass model replicating fundamental self-regulatory behaviors observed in desert lizards. Our results show this agent not only autonomously regulates its internal temperature by navigating to areas with optimal environmental conditions but also harmonizes this process with other internal needs, such as energy, hydration, security, and mating. Using game theory metrics, we observe that such a biomimetic agent outperforms an interoceptive-agnostic agent in efficiency, fairness, and stability. Together, our results suggest that grounding robot behavior in biological processes of self-regulation provides an excellent approach for addressing trade-off situations in the physical deployments of autonomous robots.

## Repository


This repository includes the source code for the IROS2023 manuscript 'Enabling robot autonomy through biomimetic self-regulatory dynamics', part of the workshop [Learning Robot Super Autonomy](https://wp.nyu.edu/workshopiros2023superautonomy/).


The repository is specifically design to be run on a Windows-WSL(Ubuntu22.04) environment with [ROS2(iron)](https://docs.ros.org/en/iron/index.html). The simulation requires the installation [Webotst](https://cyberbotics.com/) + the [webots_ros2 package](https://github.com/cyberbotics/webots_ros2).


## Getting started

Follow the following steps to run the experiment:
1. Build the repository as a ROS2 workspace
2. source install/local_setup.bash every terminal you open.
3. ros2 launch webots_pkg epuck_launch.py
4. ros2 run pseudo_supervisor pseudo_supervisor
5. ros2 run webots_pkg random_walker

