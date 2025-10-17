# FruitShop Experiment

This experiment demonstrates autonomous sub-goal generation for lifelong learning in robots using the e-MDB cognitive architecture. The experiment is developed in a discrete event simulator that simulates a TIAGo ++ robot that must classify fruits based on their weight through a multi-step process.

<div style="width:100%; margin:auto; text-align:center;">

![TIAGo robot experimental setup](images/ExperimentalSetup.jpeg)

*TIAGo robot experimental setup*
</div>

## Experiment Description

This experiment tests the autonomous generation of sub-goals using both top-down and bottom-up approaches within the e-MDB cognitive architecture. The robot must learn to complete a complex multi-step task without explicit intermediate rewards, relying on autonomous sub-goal discovery to break down the main objective into manageable components.

### Task Overview

The final objective is to classify fruits as accepted or rejected based on their weight. The complete task involves:

1. **Collection**: Pick fruits from a designated collection area at the far edge of the table
2. **Weighing**: Place the fruit on a scale within a separate weighing area
3. **Classification**: Determine if the fruit is valid based on weight readings
4. **Sorting**: Deposit valid fruits in the right basket, invalid ones in the left basket

### Environment Setup

**Physical Elements:**
- Fruits initially located in a collection area at the table's far edge
- Weighing scale in a separate area
- Two baskets for fruit sorting (right for valid, left for invalid)
- Button with toggle light (irrelevant to main task but provides environmental effects)

**Sensory Input:**

The robot receives a 10-dimensional perceptual state (all normalized in the range 0-1).

- **Fruits**: data about the closest fruit in the table.
    - **Distance**: distance from the robot to the fruit.
    - **Angle**: angle from the robot to the fruit.
    - **Dim max**: maximum dimension of the fruit.

- **Scales**: data about the weighing scale.
    - **Distance**: distance from the robot to the scale.
    - **Angle**: angle from the robot to the scale.
    - **Active**: whether the scale has a fruit on it or not.
    - **State**: state of the scale (0: inactive, 1: valid fruit detected, 2: invalid fruit detected).
- **Fruit in left hand**: indicates if the robot has a fruit in the left gripper.
- **Fruit in right hand**: indicates if the robot has a fruit in the right gripper.
- **Button light**: indicates the on/off (0/1) status of the button light.

**Available policies**

The robot has access to 8 predefined policies:

- **Pick fruit:** Grasp the closest fruit on the table.
- **Test fruit:** Place grasped fruit on the scale for weight evaluation.
- **Place fruit:** Leave fruit in table center for pickup with other arm (large fruits, dim_max >= 0.085m).
- **Change hands:** Move fruit from one gripper to another (for small fruits, dim_max < 0.085m).
- **Accept fruit:** Place fruit in the acceptance basket (right side).
- **Discard fruit:** Place fruit in the rejection basket (left side).
- **Ask nicely**: Request experimenter to provide more fruits when collection area is empty.
- **Press button**: Toggle the button light on/off.

## Sub-Goal Generation Approaches

The experiment implements two complementary approaches for autonomous sub-goal generation: Top-Down and Bottom-Up approaches. These work by using the intrinsic motivations explained in the [cognitive nodes repository.](https://docs.pillar-robots.eu/en/latest/concepts/concepts.html#intrinsic-motivations) 


### Top-Down Approach

This approach allows creating chains of sub-goals starting from the knowledge obtained about how to reach a final goal. This is done by creating goals that aim to reach the states that correspond to previously learned P-Nodes (the starting states to reach the goal), so that the final goal can be achieved. This is performed with the internal effectance intrinsic motivation. 

### Bottom-Up Approach

This approach uses the latent knowledge accumulated in the perceptual classes of P-Nodes and goals to search for new paths to achieve goals by reusing experiences learned when in different domains or while pursuing a different goal. The prospection intrinsic motivation allows this behavior in the architecture. 

## Learning Curriculum

To simulate lifelong learning with knowledge reuse, the experiment follows a structured curriculum:

### Stage 1: First exploration
The purpose is exploring the environment and discovering basic interactions, such as picking fruits, or change it from one hand
to another.

The scale is not active at this stage.

### Stage 2: Placement Learning
The purpose is placing fruits in the center of the table, reusing the skills learned in the previous stage.

This knowledge will be reused in the next stages. The scale is still inactive.

### Stage 3: Second exploration
The purpose is exploring again, but the scale is now active, allowing the robot to learn how to use it.

### Stage 4: Classification Task
The purpose is the final classification task, where the robot must classify fruits based on their weight, using the skills learned in the previous stages.


## Execution

The experiment can be executed using the launch file contained in the experiments package:
```bash
ros2 launch emdb_experiments fruit_shop_launch.py experiment_file:=fruit_shop_experiment.yaml
```

The experiment file can be selected from the ones available in the experiments package of this repository:

- fruit_shop_experiment.yaml: Default experiment
- fruit_shop_llm_experiment.yaml: Includes the LLM exploration policy. 
- fruit_shop_progress_experiment.yaml: Provides intermediate rewards to the architecture. Used as benchmark.
- fruit_shop_top_down_experiment.yaml: Uses only the Top-Down subgoal generation approach.

Configuration parameters can be adjusted through the corresponding YAML configuration file to modify learning parameters, curriculum timing, and neural network architectures.