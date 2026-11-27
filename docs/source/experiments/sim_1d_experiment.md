# 1D Simulator Experiment

In this experiment an agent moves along a line to reach a target. It is the simplest scenario of the repository: a one-dimensional slider with a single continuous action. It is simulated with the `simple_1d_sim` executable of the *sim_1d_emdb* package, which is the one started by the launch file. The experiment file included in the *experiments* package is *sim_1d_experiment_policy_learning.yaml*.

## Experiment description

The agent is a scalar position in the range from 0 to 1000. The target is another position in the same range. At every world reset, both positions are drawn at random with a uniform distribution. The agent has a single action, a velocity between -150 and 150 (the actuation bounds of the file), that is added to its position; the resulting position is kept within the range. The target is reached when the distance between the agent and the target is lower than 50.

The agent receives two sensors, published every 0.01 s:

- ***dist_to_target***: signed distance from the agent to the target (target position minus agent position), between -1000 and 1000.
- ***reached_target***: 1.0 if the target is reached and 0.0 otherwise.

A trial ends when the number of iterations of a trial (2 in the file) is reached, and the world is reset to a new random state. The file sets 500 iterations in total.

### Motivational system

The motivational system of *sim_1d_experiment_policy_learning.yaml* has four purposes:

- **Mission**: *reached_target_need* (weight 1.0, terminal). Its Drive (*reached_target_drive*) is a `DriveExponential` with `min_eval` 0.8 that uses the *reached_target* sensor of the simulator.
- **Needs**:
    - *novelty_need* (0.1) and *effectance_need* (0.25, internal effectance with `min_confidence` 0.84). See the [intrinsic motivations](https://docs.pillar-robots.eu/en/latest/concepts/concepts.html#intrinsic-motivations) in the documentation of the cognitive nodes.
    - *model_creation_need* (1.0), whose Drive (`ModelCreationDrive`) is linked to the *model_creation_policy* (`ModelCreationPolicy`). They are used to create the models that are missing.

The Goals of the novelty, effectance and model creation Needs are dummy Goals. In the novelty C-Node, exploration is done with a Utility Model (*novelty_exploration*, `NoveltyUtilityModel`, with `trace_length` 20, 50 `max_iterations` and 100 candidate actions) linked to the novelty C-Node; there is no novelty policy. The only policy defined in the file apart from *model_creation_policy* is *effectance_policy*.

### Perceptions

The file defines one Perception, ***dist_to_target***, processed by `Sim2DDistancesPerception` of the *sim_2d_emdb* package. The signed distance is normalized between -1000 and 1000.

### Learned models

The file does not define the World Models, Utility Models or Policies of the Mission: the default classes of the `Connectors` section are used for the nodes created during the execution. They are:

| Node            | Default class                             | Main parameters                                                                 |
|-----------------|-------------------------------------------|---------------------------------------------------------------------------------|
| Space           | `ANNSpace`                                | |
| P-Node          | `PNode` with `ANNSpace`                   | `history_size` 500.                                                             |
| Goal            | `GoalMotiven` with `ANNSpace`             | `history_size` 300, `min_confidence` 0.94.                                      |
| World Model     | `WorldModelLearned`                       | `main_size` 2000, `train_sample` 200, `train_split` 0.8, `validation_split` 0.1, `secondary_size` 50, `retrain` true. |
| Utility Model   | `QUtilityModel`                           | Hidden layers of 256 and 128 units, `learning_rate` 0.001, `discount_factor` 0.9, `reward_factor` 10.0, `candidate_actions` 100 (latin generation), `epochs` 100. |
| Policy          | `PolicyLearned`                           | `obs_dim` 1, `buffer_size` 100000, `batch_size` 100, `gradient_steps` 100, `learning_rate` 0.001. |

The `Globals` section sets the actuation configuration: the actuator `agent` with the parameter `velocity`.

### Rewards

The simulator publishes the *reached_target* sensor, which is 1.0 while the distance to the target is lower than 50 and 0.0 otherwise. It is the input of the *reached_target_drive*.

## Execution

The experiment is launched using the following command:

```bash
ros2 launch experiments sim1d_launch.py
```

The launch file starts the commander, the LTM and the simulator, and it has these arguments:

| Argument             | Default                                 | Description                                              |
|----------------------|-----------------------------------------|----------------------------------------------------------|
| `log_level`          | `info`                                  | Logging level.                                           |
| `random_seed`        | `0`                                     | Seed of the random number generators.                    |
| `experiment_file`    | `sim_1d_experiment_policy_learning.yaml`| Experiment file, from the `experiments` folder of `experiment_package`. It is read by the simulator, which sends it to the commander. |
| `experiment_package` | `experiments`                           | Package of the experiment file.                          |
| `config_file`        | `commander_threaded.yaml`               | Commander configuration file.                            |
| `config_package`     | `core`                                  | Package of the commander configuration file.             |
| `visualize`          | `true`                                  | Show the matplotlib window of the simulator.             |

The window shows the agent, the target and the success zone. It needs a graphical matplotlib backend: the simulator uses a non-interactive backend unless the `MPLBACKEND` environment variable is set, for example `MPLBACKEND=TkAgg`. Use `visualize:=false` to run without it.

The experiment is configured through the [sim_1d_experiment_policy_learning.yaml](https://github.com/pillar-robots/emdb_experiments_gii/blob/main/experiments/experiments/sim_1d_experiment_policy_learning.yaml) file, in the *experiments* package of this repository. The main loop is a `MainLoopLight`, with softmax selection disabled, and it stops the system when it finishes. The parameters that can be set in these files are described in the [configuration page](https://docs.pillar-robots.eu/projects/emdb_core/en/latest/core/configuration.html) of the core documentation.

Once executed, it is possible to see the logs in the terminal, being able to follow the behavior of the experiment in real time. The files listed in the `Files` section of the YAML (*goodness.txt*, *pnodes_content.txt*, *trials.txt*, *dataset.csv*, *world_model_success.txt* and the *models_save* files) are written in the current directory.
