# e-MDB configuration files for launching experiments

This repository is devoted to configuration files, in yaml format, for experiments. These files specify which simulated worlds need to be used, what the objectives will be, what is the initial configuration of long-term memory (LTM) in each experiment, and more.

There is an example experiment configuration, called default_experiment.yaml.

For more information about the cognitive architecture design, you can visit the [emdb_core](https://github.com/GII/emdb_core?tab=readme-ov-file#design) repository or the [PILLAR Robots official website](https://pillar-robots.eu/).

## Table of Contents

- **[Dependencies](#dependencies)**
- **[Installation](#installation)**
- **[Configurate an experiment](#configurate-an-experiment)**
- **[Execution](#execution)**
- **[Results](#results)**

## Dependencies

These are the dependencies required to use this repository:

- ROS 2 Humble

## Installation

To install this package, it's necessary to clone this repository in a ROS workspace and build it with colcon.

```
colcon build --symlink-install
```
This respository only includes the package in which the configuration of the experiments is stores. To get full functionality, it's required to add to the ROS workspace the packages that makes the architecture and the interface that connects the architecture with a real or a simulated environment. Therefore, to use the first version implemented by GII, these repositories need to be cloned into the workspace:
- [_emdb_core_](https://github.com/GII/emdb_cognitive_nodes_gii). Core of the cognitive architecture.
- [_emdb_cognitive_nodes_gii_](https://github.com/GII/emdb_cognitive_nodes_gii). Reference implementation for the main cognitive nodes.
- [_emdb_cognitive_processes_gii_](https://github.com/GII/emdb_cognitive_processes_gii). Reference implementation for the main cognitive processes.
- [_emdb_discrete_event_simulator_gii_](https://github.com/GII/emdb_discrete_event_simulator_gii). Implementation of a discrete event simulator used in many experiments.
- [_emdb_experiments_gii_](https://github.com/GII/emdb_experiments_gii). Configuration files for experiments.

In these repositories is included an example experiment with the discrete event simulator in which the Policies, the Goal and the World Model are defined in the beginning, the objective being to create the corresponding PNodes and CNodes, which allow the Goal to be achieved effectively by the simulated robot. 

The Goal, called ObjectInBoxStandalone, consists of introducing a cylinder into a box correctly. For that, the robot can use, in a World Model called GripperAndLowFriction, the following policies:
- Grasp object: use one of the two grippers to grasp an object
- Grasp object with two hands: use both arms to grasp an object between their ends
- Change hands: move object from one gripper to the other 
- Sweep object: sweep an object to the central line of the table
- Ask nicely: ask experimenter, simulated in this case, to bring something to within reach
- Put object with robot: deposit an object to close to the robot base
- Put object in box: place an object in a receptacle
- Throw: throw an object to a position
  
The reward obtained could be 0.2, 0.3 or 0.6 if the robot with its action improves its situation to get the final goal. Finally, when the cylinder is introduced into the box, the reward obtained is 1.0. Thus, at the end of the experiment, seven PNodes and CNodes should be created, one per policy, except Put object with robot, which doesn't lead to any reward.

## Configurate an experiment

Several packages can be configured in these files, so the specific configuration is explained in the repositories mentioned in the [installation](https://github.com/GII/emdb_experiments_gii/edit/main/README.md#installation) section.

## Execution

To execute the example experiment or another launch file, it's essential to source the ROS workspace:
```
source install/setup.bash
```
Afterwards, the experiment can be launched:
```
ros2 launch core example_launch.py
```
Once executed, it is possible to see the logs in the terminal, being able to follow the behavior of the experiment in real time.

## Results

Executing the example experiment, it will create two files by default: goodness.txt and pnodes_success.txt. 

In the first one, it is possible to observe important information, such as the policy executed and the reward obtained per iteration. It is possible to observe the learning process by seeing this file in real time with the following command:
```
tail -f goodness.txt
```
| Iteration | Goal                     | World                     | Reward | Policy                | Sensorial changes | C-nodes |
|-----------|--------------------------|---------------------------|--------|-----------------------|-------------------|---------|
| 1416      | object_in_box_standalone | GRIPPER_AND_LOW_FRICTION  | 0.3    | sweep_object          | True              | 7       |
| 1417      | object_in_box_standalone | GRIPPER_AND_LOW_FRICTION  | 0.6    | grasp_with_two_hands  | True              | 7       |
| 1418      | object_in_box_standalone | GRIPPER_AND_LOW_FRICTION  | 1.0    | put_object_in_box     | True              | 7       |

In the second file, it's possible to see the activation of the PNodes and if it was a point (True) or an anti-point (False).

When the execution is finished, it's possible to obtain statistics about reward and PNodes activations per 100 iterations by using the scripts available in the scripts directory of the core package (emdb_core/core/scripts):
```
python3 $ROS_WORKSPACE/src/emdb_core/core/scripts/generate_grouped_statistics -n 100 -f goodness.txt > goodness_grouped_statistics.csv

python3 $ROS_WORKSPACE/src/emdb_core/core/scripts/generate_grouped_success_statistics -n 100 -f pnodes_success.txt > pnodes_grouped_statistics.csv
```
To use these scripts it's necessary to have installed python-magic 0.4.27 dependency.

By plotting the data of these final files, it is possible to obtain a visual interpretation of the learning of the cognitive architecture.
