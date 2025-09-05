# PutObjectInBox Experiment

This experiment uses a Discrete Event Simulator that simulates, without physics, the Baxter robot, which has two arms. 

The task consists in placing a cylinder in a box. These elements are placed in random positions and may be out of reach of the robot.


<div style="width:100%; margin:auto; text-align:center;">

![Baxter robot](images/P1030749-2.png)

*Baxter robot*
</div>

## Experiment description

In this experiment is tested the behavior of the Long-Term Memory (LTM), without any motivational system. There are no Desires (Needs & Missions) and Drives, and the Policies, Goal, and World Model are defined in the beginning. The objective is to create the corresponding P-Nodes and C-Nodes, which allow the simulated robot to achieve the Goal effectively, choosing the correct Policy in each situation.

The World Model node, called GripperAndLowFriction, defines the behavior of the simulator, but it hasn't effect on the operation of the cognitive architecture, because it 
acts as a dummy node that always has 1.0 activation. That is, changing the WorldModel affects the behavior of the simulated environment and, therefore, the effect of the policies in it, but without a direct influence on the cognitive architecture.

The Goal node, called ObjectInBoxStandalone, consists on introducing a cylinder into a box correctly. It can give several values of reward: 0.2 if the policy executed slightly approached the robot to the final objective of putting the cylinder in the box; 0.3 or 0.6 if the robot approached to it more closely; or 1.0 if the robot reached the objective. This node, as the World Model, has always 1.0 activation.

The perceptions received by the cognitive architecture are the following ones:

- **Boxes**: data about the boxes in the table.
    - **Distance**: distance from the robot to the box.
    - **Angle**: angle from the robot to the box.
    - **Diameter**: diameter of the box.

- **Cylinders**: data about the cylinders in the table.
    - **Distance**: distance from the robot to the cylinder.
    - **Angle**: angle from the robot to the cylinder.
    - **Diameter**: diameter of the cylinder.
- **Ball in left hand**: indicates if the robot has the cylinder in the left gripper.
- **Ball in right hand**: indicates if the robot has the cylinder in the right gripper.

The policies that the robot can use to solve the task are the following ones:

- **Grasp object:** Use one of the two grippers to grasp an object.
- **Grasp object with two hands:** Use both arms to grasp an object between their ends.
- **Change hands:** Move an object from one gripper to the other.
- **Sweep object:** Sweep an object to the central line of the table.
- **Ask nicely:** Ask experimenter, simulated in this case, to bring something to within reach.
- **Put object with robot:** Deposit an object close to the robot base.
- **Put object in box:** Place an object in a receptacle.
- **Throw:** Throw an object to a position.

During the execution of the experiment, a P-Node and C-Node pair is created each time a reward is obtained. Thus, at the end of the experiment, seven pairs should be created, one per Policy, except *Put object with robot*, which doesn't lead to any reward.

We will have this structure for each Policy:

<div style="width:100%; margin:auto; text-align:center;">

![CNode](images/CNode_v2.png)

*Contextual structure*
</div>

So, as we can see, the Policies activation depends exclusively on their P-Nodes activation. Thus, during the experiment, points and anti-points will be added to the P-Nodes each time a Policy is executed, improving the calculation of activations and, in consequence, the decision-making of the cognitive architecture.

## Execution

The experiment is launched using the following command:

```bash
ros2 launch experiments put_object_in_box_launch.py
```

This experiment is configured through the [put_object_in_box_experiment.yaml](https://github.com/pillar-robots/emdb_experiments_gii/blob/main/experiments/experiments/put_object_in_box_experiment.yaml) file, which you can find in the *experiments* package of this repository.

Once executed, it is possible to see the logs in the terminal, being able to follow the behavior of the experiment in real time.

<!-- ## Results

Executing the example experiment, it will create two files by default: **goodness.txt** and **pnodes_success.txt**.

In the first one, it is possible to observe important information, such as the policy executed and the reward obtained per iteration. It is possible to observe the learning process by seeing this file in real time with the following command:

```bash
tail -f goodness.txt
```

| Iteration | Goal                     | World                     | Reward | Policy                | Sensorial changes | C-nodes |
|-----------|--------------------------|---------------------------|--------|-----------------------|-------------------|---------|
| 1416      | object_in_box_standalone | GRIPPER_AND_LOW_FRICTION  | 0.3    | sweep_object          | True              | 7       |
| 1417      | object_in_box_standalone | GRIPPER_AND_LOW_FRICTION  | 0.6    | grasp_with_two_hands  | True              | 7       |
| 1418      | object_in_box_standalone | GRIPPER_AND_LOW_FRICTION  | 1.0    | put_object_in_box     | True              | 7       |

In the second file, it's possible to see an activation historical of the P-Nodes and if it was a point (True) or an anti-point (False).

When the execution is finished, it's possible to obtain statistics about reward and P-Nodes activations per 100 iterations by using the scripts available in the scripts directory of the core package (~/eMDB_ws/src/wp5_gii/emdb_core/core/scripts):

```bash
python3 ~/eMDB_ws/src/wp5_gii/emdb_core/core/scripts/generate_grouped_statistics -n 100 -f goodness.txt > goodness_grouped_statistics.csv

python3 ~/eMDB_ws/src/wp5_gii/emdb_core/core/scripts/generate_grouped_success_statistics -n 100 -f pnodes_success.txt > pnodes_grouped_statistics.csv
```

```{note}
To use these scripts it's necessary to have installed **python-magic 0.4.27** dependency.
```

By plotting the data of these final files, it is possible to obtain a visual interpretation of the learning of the cognitive architecture. -->










