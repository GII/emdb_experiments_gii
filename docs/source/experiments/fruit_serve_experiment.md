# Fruit Serve Experiment

In this experiment a simulated TIAGo robot serves a client: it takes a fruit from a shelf, weighs it on a scale placed on a table, and puts it in the delivery box. It is simulated with a discrete event simulator (the `fruit_serve_simulator` executable of the *simulators* package, which is the one started by the launch file).

## Experiment description

The robot has two hands and it faces either a table or a shelf, and it can turn around to face the other one. At the start of every trial there are between 2 and 4 fruits (apples or kiwis) on the shelf, with random positions and sizes. With a probability of 50%, one more fruit is placed in one of five fixed places: two positions on the table, the delivery box, the client or the scale. A scale is placed on the table, on the left side of the robot, and the delivery box is on the right side.

The task is to serve a fruit that has been weighed: the robot has to pick a fruit from the shelf, face the table, weigh it with the left hand (the one closest to the scale), pick it again from the scale, hold it with the right hand (the one closest to the box) and place it in the box. A hand is chosen automatically when picking: the right hand if the fruit is on the right side of the robot, and the left hand otherwise. A fruit can be moved to the other hand if it is small enough (`dim_max` of at most 0.085).

A trial ends when a fruit has been placed in the box or when the number of iterations of a trial (40 in the main file) is reached. In both cases the world is reset to a new random state: the position of the scale and of the fruits, whether the robot faces the table or the shelf (50% each) and whether the robot starts holding a fruit (50%). If it does, the fruit is the first one generated and, with another 50% of probability, it has already been weighed.

### Motivational system

The motivational system of *fruit_serve_experiment.yaml* has five purposes:

- **Mission**: *client_served_mission* (weight 1.0, terminal). Its Drive (*client_served_drive*) is a `DriveExponential` with `min_eval` 0.8 that uses the *client_served_goal* sensor of the simulator. The Goal of the Mission is not defined in the file; the default class of the Goals is `GoalLearnedSpace`.
- **Needs**:
    - *novelty_need* (0.1), *effectance_need* (0.25, internal effectance), *external_effects_need* (0.3, external effectance) and *prospection_need* (1.0). See the [intrinsic motivations](https://docs.pillar-robots.eu/en/latest/concepts/concepts.html#intrinsic-motivations) in the documentation of the cognitive nodes.

### World Model

The file defines one World Model, *FRUIT_SERVE* (the base `WorldModel` class), which is linked to the C-Nodes of the Needs.

### Perceptions

The perceptions received by the cognitive architecture are the following ones (all of them are processed by `FruitShopPerception`):

- **Fruits**: data about the closest visible fruit, with **distance**, **angle** and **dim_max** (the largest dimension), normalized. If the robot faces the table, the visible fruits are those on the table, on the scale and in the hands; if it faces the shelf, those on the shelf and in the hands. If there is no visible fruit, the distance is 1.9, the angle 1.4 and the dim_max 0.1 (before normalizing).
- **Scales**: data about the scale, with **distance**, **angle**, **state** and **active**. The distance and the angle are those of the limits of the field of view (1.9 and 1.4) when the robot faces the shelf. The state is normalized with three states (`n_states: 3`); the simulator always publishes state 0. The scale is active while a fruit is on it.
- **Fruit in left hand** and **Fruit in right hand**: whether the robot holds a fruit with each hand.
- **Holding weighed fruit**: whether the fruit in the hand has been weighed.
- **Facing table** and **Facing shelf**: where the robot is facing. Exactly one of them is true.

### Policies

The following policies are executed by the simulator:

- **Pick fruit:** Take a fruit with the hand that corresponds to its side. The robot has to have its hands free. When facing the table, a fruit on the scale is picked first (and it is considered weighed once picked); otherwise the closest fruit on the table. When facing the shelf, the closest fruit on the shelf.
- **Place fruit:** Leave the held fruit at a fixed position in front of the robot, on the side opposite to the hand that held it. The fruit is left on the table if the robot faces the table, and on the shelf otherwise. It stops being weighed.
- **Place in box:** Put the held fruit in the delivery box. The fruit has to be weighed, held with the right hand, and the robot has to face the table.
- **Change hands:** Move the held fruit to the other hand, if its `dim_max` is at most 0.085.
- **Give fruit:** Leave the held fruit with the client.
- **Weigh fruit:** Leave the held fruit on the scale. The fruit must not be weighed already, held with the left hand, and the robot has to face the table.
- **Discard fruit:** Remove the held fruit, which has to be weighed.
- **Turn around:** Face the other side (table or shelf).

The remaining policies belong to the motivational system: *novelty_policy* (random policy, which excludes *effectance_policy*, *external_effect_policy* and *prospection_policy*), *effectance_policy*, *external_effect_policy* and *prospection_policy*.

### Rewards

The simulator publishes one sensor, *client_served_goal*, which is 1.0 once a fruit has been placed in the box and 0.0 otherwise. It goes back to 0.0 when the robot picks a fruit again or the world is reset.

### Stages

The `Stages` section of the file lists three stages that start at iterations 1000, 3000 and 5000. The simulator reads them, but none of its behaviors depends on the stage in the current version.

## Execution

The experiment is launched using the following command:

```bash
ros2 launch experiments fruit_serve_launch.py
```

The launch file starts the commander, the LTM and the simulator, and it has these arguments:

| Argument             | Default                       | Description                                              |
|----------------------|-------------------------------|----------------------------------------------------------|
| `log_level`          | `info`                        | Logging level.                                           |
| `random_seed`        | `0`                           | Seed of the random number generators.                    |
| `experiment_file`    | `fruit_serve_experiment.yaml` | Experiment file, from the `experiments` folder of `experiment_package`. It is read by the simulator, which sends it to the commander. |
| `experiment_package` | `experiments`                 | Package of the experiment file.                          |
| `config_file`        | `commander.yaml`              | Commander configuration file.                            |
| `config_package`     | `core`                        | Package of the commander configuration file.             |

The experiment is configured through the [fruit_serve_experiment.yaml](https://github.com/pillar-robots/emdb_experiments_gii/blob/main/experiments/experiments/fruit_serve_experiment.yaml) file, in the *experiments* package of this repository. The main loop is a `MainLoop` with 15000 iterations, trials of 40 iterations, subgoals and softmax policy selection (temperature 0.3), and it stops the system when it finishes. The parameters that can be set in these files are described in the [configuration page](https://docs.pillar-robots.eu/projects/emdb_core/en/latest/core/configuration.html) of the core documentation.

The *experiments* package also includes these variants, which are selected with the `experiment_file` argument:

- ***fruit_serve_llm_experiment.yaml***: 10000 iterations and trials of 20 iterations. The novelty Need is replaced by an LLM exploration Need (*llm_exploration_need*, weight 0.1, with `DriveLLMExploration`), and the novelty policy by *llm_exploration_policy* (`PolicyLLMExplorationFruitServe`). The policy asks a language model (`llama3.3:latest`, served at the `client_host` address of the file, with temperature 0.1 and up to 4 episodes sent) to choose one of the policies of the simulator, given a description of the perceptions, of the policies and of the goal of serving a weighed fruit. The model server must be reachable from the machine that runs the experiment. The P-Nodes and Goals use a `NormalCentroidPointBasedSpace` of 50000 points, and the Goals have a history of 300 samples and `min_confidence` 0.94.
- ***fruit_serve_progress_experiment.yaml***: the Mission (*progress_client_served_mission*) uses a gradual reward, the sensor *progress_client_served_goal*, instead of the binary one. The only Need is the novelty one: the effectance, external effects and prospection Needs are removed. The value of the sensor is:

    | Value | Situation |
    |-------|-----------|
    | 0.0   | None of the following. |
    | 0.125 | Facing the shelf with fruits on it, and no fruit in the hands or on the scale. |
    | 0.25  | Holding an unweighed fruit (and no fruit on the scale), facing the shelf. |
    | 0.375 | Same, facing the table, with the right hand. |
    | 0.5   | Same, facing the table, with the left hand. |
    | 0.625 | Holding a weighed fruit and facing the shelf, or a fruit is on the scale (and the robot does not hold a weighed fruit). |
    | 0.75  | Holding a weighed fruit, facing the table, with the left hand. |
    | 0.875 | Holding a weighed fruit, facing the table, with the right hand. |
    | 1.0   | A fruit has been placed in the box. |

Once executed, it is possible to see the logs in the terminal, being able to follow the behavior of the experiment in real time. The files listed in the `Files` section of the YAML (for example *goodness.txt* and *pnodes_success.txt*) are written by the main loop in the current directory.
