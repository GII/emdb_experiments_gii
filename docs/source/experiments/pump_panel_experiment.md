# Pump panel Experiment
(Work in progress)

This experiment demonstrates the usage of a LLM-based exploration strategy to achieve goals in an environment where it is not feasible to achieve goals by means of random exploration. It is developed in a discrete event simulator where the robot has to operate an industrial panel to start a pump system.

<div style="width:100%; margin:auto; text-align:center;">

![Pump panel experimental setup](images/pump_panel.png)

*Pump panel experimental setup*
</div> 

## Experiment Description

The former features a deliberately challenging setup, introducing several distractors and elements that hinder the robot's progress. As a result the task is much harder to solve through random exploration. Therefore, using an LLM as a cultural repository becomes crucial, as it helps to guide the robot toward the goal by drawing on cultural knowledge about the world.

### Task Overview

The final objective is to start the pump, to do that the robot has to follow a specific sequence:

1. **Step 1**: Enable panel functionality (On Button)
2. **Step 2**: Open the three valves (V1, V2, V3)
3. **Step 3**: Start the pump (Start button)

In this process, the robot can be distracted by trying to press or grasp useless objects or by moving the tools. Also, the robot could press the Off button, which turns off the panel and its functionality, or the Emergency button, which closes the valves and disables the functionality of the panel until it is pressed again.

### Environment Setup

**Physical Elements:**

- Buttons: On, Off, Start, Emergency, Mode, Test Light, Valve 1, Valve 2, Valve 3. These cause an effect in the panel whenever pressed.
- Objects: Two tools that the robot can grasp and place into a delivery area. 
- Dials: Voltage and output flow indicators. 
 
**Sensory Input:**

The robot receives a 15-dimensional perceptual state (all normalized in the range 0–1).  

- **Discharge light**: indicates whether the discharge light is active (1 if on, 0 if off).  
- **Emergency button**: indicates whether the emergency button is pressed (1 if activated, 0 if not).  
- **Mode selector**: operational mode of the system (1 if manual, 0 if automatic).  
- **Off button light**: indicates the off button status (1 if lit, 0 if off).  
- **On button light**: indicates the on button status (1 if lit, 0 if off).  
- **Output flow dial**: continuous value from 0.0 to 1.0 representing the output flow level.  
- **Start button**: encodes the state of the start button (0 if off, 0.5 if light on but pump not started, 1.0 if pump started).  
- **System backup light**: indicates whether the backup light is active (1 if on, 0 if off).  
- **Test light**: indicates whether the test light is active (1 if on, 0 if off).  
- **Tool 1**: indicates the state of tool 1 (0 if stored, 0.5 if grasped, 1.0 if delivered).  
- **Tool 2**: indicates the state of tool 2 (0 if stored, 0.5 if grasped, 1.0 if delivered).  
- **Valve 1 (V1)**: indicates the valve status (0 if light off, 0.5 if closed, 1.0 if open).  
- **Valve 2 (V2)**: indicates the valve status (0 if light off, 0.5 if closed, 1.0 if open).  
- **Valve 3 (V3)**: indicates the valve status (0 if light off, 0.5 if closed, 1.0 if open).  
- **Voltage dial**: continuous value from 0.0 to 1.0 representing the system voltage.  




**Available policies**

The robot has access to 4 predefined policies:

- **Press object**: Pushes the specified object.  
- **Grasp object**: Picks up the specified object.  
- **Deliver object**: Places a grasped tool at the delivery area.  
- **Store object**: Places a grasped tool into its appropriate box.  


## LLM Exploration Policy

This policy leverages a large language model as a knowledge injection mechanism to guide the robot when it lacks sufficient information to decide the next action. Instead of relying on random exploration, the LLM interprets the human’s stated purpose and proposes exploratory actions aligned with it. This allows the robot to acquire relevant knowledge more efficiently while grounding new goals and actions through its sensorimotor interaction with the environment.

The process begins with a human-provided purpose, expressed as a natural language input. This input, together with the robot’s current perceptions, the set of available policies, and the most recent episodes (perceptions, actions, and rewards), is formulated into a query for the LLM. The LLM responds with the next policy to execute. The robot carries out this policy, and if the execution yields a positive reward, the system updates its long-term memory by creating new goals and contexts that consolidate the usefulness of the chosen policy. In this way, the cultural knowledge embedded in the LLM helps the robot to focus its exploration. This enables the progressive building of grounded knowledge that aligns human purposes with the architecture’s predefined missions.

<div style="width:100%; margin:auto; text-align:center;">

![LLM Exploration Policy](images/eMDB_LLM_def.png)

*LLM Exploration Policy*
</div> 



## Execution

The experiment can be executed using the launch file contained in the experiments package:

```bash
ros2 launch emdb_experiments pump_panel_launch.py experiment_file:=pump_panel_llm_experiment.yaml
```

The experiment file can be selected from the ones available in the experiments package of this repository:

- pump_panel_experiment.yaml: Experiment without the LLM exploration policy. 
- pump_panel_llm_experiment.yaml: Includes the LLM exploration policy. 

Configuration parameters can be adjusted through the corresponding YAML configuration file to modify the model used, initial prompt and other learning parameters. 