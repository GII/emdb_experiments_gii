# OSCAR experiment

This experiment uses an OSCAR robot in the gazebo simulator. The task consists in placing a cylinder in a box, these elements are placed in random positions and may be out of reach of the robot.

![Baxter robot](images/OSCAR.png)

As in the default experiment, this experiment tests the behavior of the Long-Term Memory (LTM) of the architecture. There is a single World Model (GripperAndLowFriction), and no Needs nor Drives are used.

The Policy nodes that the robot can use are the following ones:

- **Grasp right:** Use the right gripper to grasp an object
- **Grasp left:** Use the left gripper to grasp an object
- **Change hands:** Move an object from one gripper to the other 
- **Press button:** Ask experimenter, simulated in this case, to bring something to within reach
- **Place object right:** Deposit an object in a receptacle using the right arm
- **Place object left:** Deposit an object in a receptacle using the left arm

The goal node ObjectInBoxStandalone provides reward for placing the cylinder inside the box. Partial rewards are given as the robot nears the final objective: 0.25 for approaching the cylinder with the button, 0.5 for grasping the cylinder, 0.75 for changing the cylinder to the appropriate hand and 1.0 for placing the cylinder in the box.    

A P-Node and C-Node pair is created each time a reward is obtained. Thus, at the end of the experiment, seven pairs should be created, one per Policy, except *Put object with robot*, which doesn't lead to any reward.

We will have this structure for each Policy:

![CNode](images/CNode_v2.png)

So, as we can see, the Policies activation depends exclusively on their P-Nodes activation. Thus, during the experiment, points and anti-point will be added to the P-Nodes each time a Policy is executed, improving the calculation of activations and, in consequence, the decision-making of the cognitive architecture.

You can find more conceptual information about the e-MDB architecture in the 5.1 Deliverable of the WP5 of the PILLAR Robots project.

## Installation

To install this experiment, it's necessary to create a ROS workspace and clone the following GitHub meta repository:

[wp5_gii meta repository](https://github.com/pillar-robots/wp5_gii)

```bash
mkdir -p ~/eMDB_ws/src

cd ~/eMDB_ws/src

git clone --recursive https://github.com/pillar-robots/wp5_gii.git
```

The next step is to compile the ROS workspace:

```bash
colcon build --symlink-install
```

```{note}
Remember to install the dependencies!

- ROS 2 Humble
- Numpy 1.24.3
- Sklearn 1.4.2
- Tensorflow 2.15.0
- Yamlloader 1.3.2

*Other versions could work, but the indicated ones have proven to be functional.*
```

Aditionally, the OSCAR simulator must be installed. The instructions can be found [here](https://github.com/efallash/oscar/tree/humble_gazebo_classic).

## Execution

To execute the experiment or another launch file, it's essential to source the ROS workspace:

```bash
source install/setup.bash
```

Then, OSCAR simulation must be launched:

```bash
ros2 launch oscar_bringup oscar_launch.py
```

Afterwards, the experiment can be launched:

```bash
ros2 launch experiments oscar_launch.py
```

Once executed, it is possible to see the logs in the terminal, being able to follow the behavior of the experiment in real time.

## Results

Executing the example experiment, it will create two files by default: **goodness.txt** and **pnodes_success.txt**.

In the first one, it is possible to observe important information, such as the policy executed and the reward obtained per iteration. It is possible to observe the learning process by seeing this file in real time with the following command:

```bash
tail -f goodness.txt
```

| Iteration | Goal                     | World                     | Reward | Policy                | Sensorial changes | C-nodes |
|-----------|--------------------------|---------------------------|--------|-----------------------|-------------------|---------|
| 1415      | object_in_box_standalone | GRIPPER_AND_LOW_FRICTION  | 0.2    | press_button          | True              | 6       |
| 1416      | object_in_box_standalone | GRIPPER_AND_LOW_FRICTION  | 0.5    | grasp_right           | True              | 6       |
| 1417      | object_in_box_standalone | GRIPPER_AND_LOW_FRICTION  | 1.0    | place_object_right    | True              | 6       |

In the second file, it's possible to see an activation historical of the P-Nodes and if it was a point (True) or an anti-point (False).

When the execution is finished, it's possible to obtain statistics about reward and P-Nodes activations per 100 iterations by using the scripts available in the scripts directory of the core package (~/eMDB_ws/src/wp5_gii/emdb_core/core/scripts):

```bash
python3 ~/eMDB_ws/src/wp5_gii/emdb_core/core/scripts/generate_grouped_statistics -n 100 -f goodness.txt > goodness_grouped_statistics.csv

python3 ~/eMDB_ws/src/wp5_gii/emdb_core/core/scripts/generate_grouped_success_statistics -n 100 -f pnodes_success.txt > pnodes_grouped_statistics.csv
```

```{note}
To use these scripts it's necessary to have installed **python-magic 0.4.27** dependency.
```

By plotting the data of these final files, it is possible to obtain a visual interpretation of the learning of the cognitive architecture. 










