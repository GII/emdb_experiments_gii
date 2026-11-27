# Bartender Experiment

In this experiment a simulated robot acts as a bartender: it prepares drinks, serves them to clients and returns the used glass to the preparation table. It is simulated with a discrete event simulator (the `bartender_rl_simulator` executable of the *simulators* package, which is the one started by the launch file).

## Experiment description

The robot has two tables, a preparation table and a serving table, and it can be at only one of them at a time. There are three bottles (drink types 1, 2 and 3) and one glass. A client arrives with a preference (a drink type and whether the drink must be shaken), which the robot does not know until it asks. The task is to prepare the drink the client wants, serve it, and bring the used glass back to the preparation table, where it is cleaned.

The simulator has three clients, with fixed preferences:

| Client | Drink type | Shaken |
|--------|------------|--------|
| 1      | 1          | No     |
| 2      | 2          | Yes    |
| 3      | 3          | No     |

In the single client mode only client 1 appears, with drink type 2 and shaken. A new client is drawn at every world reset.

A trial ends when a correct drink has been served and the used glass has been returned to the preparation table, or when the number of iterations of a trial (40 in the main file) is reached. In both cases the world is reset to a new random state, generated according to the current curriculum stage (see the Stages section below).

### Motivational system

The motivational system of *bartender_experiment.yaml* has seven purposes:

- **Missions**:
    - *return_the_glass_mission* (weight 1.0, terminal): its Drive (*return_the_glass_drive*) uses the *return_the_glass* sensor of the simulator.
    - *serve_the_drink_mission* (weight 0.8, not terminal): its Drive (*serve_the_drink_drive*) uses the *serve_the_drink* sensor of the simulator.

  Both Drives are `DriveExponential` with `min_eval` 0.8. The Goals of the two Missions are not defined in the file: they are learned during the execution (`GoalLearnedSpace`, with `min_confidence` 0.9).
- **Needs**:
    - *client_present_need* (weight 1.0) and *get_response_need* (weight 1.0). They have dummy Drives. Their C-Nodes are linked to the *ask_nicely* and *save_preference* policies, respectively (see below).
    - *novelty_need* (0.1), *effectance_need* (0.25, internal effectance) and *prospection_need* (1.0). See the [intrinsic motivations](https://docs.pillar-robots.eu/en/latest/concepts/concepts.html#intrinsic-motivations) in the documentation of the cognitive nodes.

### World Model

The file defines one World Model, *BARTENDER* (`BarEmpty`). It is active (1.0) while the client in the perception is not known, and inactive once the client has been registered. When the *save_preference* policy registers a client, it creates a new World Model for it (`ClientInBar`, named `client_<id>`) with the preference of that client. That World Model is active only when its client is the one in the perception, and it publishes the preference. The `bottles` perception uses that value to select the bottle that the robot sees.

### Perceptions

The perceptions received by the cognitive architecture are the following ones:

- **Client**: the client in front of the bar.
    - **Id**: id of the client, normalized.
    - **Preference**: drink type that the client wants, normalized. It is 0 until the robot asks for it with *ask_nicely*.
- **Glass**: data about the glass.
    - **Distance** and **angle**: from the robot to the glass, normalized.
    - **State**: whether the glass has a drink.
    - **Was used**: whether the drink has been served and the glass has not been cleaned yet.
    - **Drink type**: type of the drink in the glass, normalized.
    - **Is shaken**: whether the drink has been shaken.
- **Bottles**: data about one bottle, with **distance**, **angle** and **drink type**. The simulator publishes all the bottles; the perception keeps the one that the robot holds in its right hand or, if there is none, the one of the preference published by the active World Model, or the first one otherwise.
- **Robot position**: 0 at the preparation table and 0.95 at the serving table.
- **Glass in left hand** and **Bottle in right hand**: whether the robot is holding something with each hand.

### Policies

The following policies are executed by the simulator:

- **Pick glass:** Take the glass with the left hand, if it is on the table where the robot is.
- **Pick bottle:** Take with the right hand the bottle that appears in the `bottles` perception, if it is on the table where the robot is.
- **Place bottle:** Leave the bottle on the table where the robot is.
- **Prepare drink:** Fill the empty glass with the drink of the bottle. Requires the glass in the left hand and the bottle in the right hand.
- **Shake glass:** Shake the glass held in the left hand, if it has a drink.
- **Change position:** Move the robot to the other table.
- **Place glass:** Leave the glass on the table where the robot is. At the serving table, if the drink is the one the client wants (type and shake), the drink is served and the glass becomes used. At the preparation table, the glass is cleaned: this is how a wrong drink is discarded and how a used glass is returned.
- **Ask nicely:** Ask the client for its preference, which then appears in the *client* perception.

*pick_bottle* also sends the current perception to the simulator, so that it can know which bottle was selected. *save_preference* is not executed by the simulator: it registers the client and its preference in the *BARTENDER* World Model, and creates the World Model of the client.

The remaining policies belong to the motivational system: *novelty_policy* (random policy, which excludes *ask_nicely*, *save_preference* and the policies of the other Needs), *effectance_policy* and *prospection_policy*.

### Rewards

The simulator publishes two binary sensors, one per Mission:

- ***serve_the_drink***: 1.0 once the correct drink has been served (the glass is used), and 0.0 otherwise.
- ***return_the_glass***: 0.0 while the glass is used, and 1.0 otherwise. That is, it is 1.0 again once the used glass has been cleaned at the preparation table.

The simulator also publishes a *finished_signal*, which is true when a correct drink has been served and the used glass has been cleaned. The main loop of this experiment (`BartenderExperimentMainLoop`) uses it to reset the world.

### Stages

The `Stages` section of the file sets the curriculum of the simulator, which determines the probability of each element of the initial state of a trial (bottles on one table or the other, bottle or glass already in the hand, empty or filled glass, correct or wrong drink, used glass). The stage changes when the iteration reaches the number given in the file:

| Stage  | Starts at iteration | Curriculum | Initial states                                                                 |
|--------|---------------------|------------|--------------------------------------------------------------------------------|
| stage0 | 0                   | help       | Biased towards states close to the solution (e.g. the correct bottle is often already grasped). |
| stage1 | 60000               | balanced   | Intermediate probabilities.                                                    |
| stage2 | 90000               | benchmark  | The glass is empty in most of the trials and nothing is grasped.               |

The main file has 140000 iterations in total and three stages, so the benchmark stage lasts 50000 iterations. The probabilities of each curriculum are in the `BartenderSim` class of the simulator.

## Execution

The experiment is launched using the following command:

```bash
ros2 launch experiments bartender_launch.py
```

The launch file starts the commander, the LTM and the simulator, and it has these arguments:

| Argument             | Default                     | Description                                              |
|----------------------|-----------------------------|----------------------------------------------------------|
| `log_level`          | `info`                      | Logging level.                                           |
| `random_seed`        | `0`                         | Seed of the random number generators.                    |
| `experiment_file`    | `bartender_experiment.yaml` | Experiment file, from the `experiments` folder of `experiment_package`. It is read by the simulator, which sends it to the commander. |
| `experiment_package` | `experiments`               | Package of the experiment file.                          |
| `config_file`        | `commander.yaml`            | Commander configuration file.                            |
| `config_package`     | `core`                      | Package of the commander configuration file.             |
| `single_client`      | `False`                     | Passed to the simulator. If `True`, only client 1 appears. |

The `single_client` argument and the YAML file are independent: to run the single client variant, set both, for example:

```bash
ros2 launch experiments bartender_launch.py experiment_file:=bartender_experiment_single_client.yaml single_client:=True
```

The experiment is configured through the [bartender_experiment.yaml](https://github.com/pillar-robots/emdb_experiments_gii/blob/main/experiments/experiments/bartender_experiment.yaml) file, in the *experiments* package of this repository. The parameters that can be set in these files are described in the [configuration page](https://docs.pillar-robots.eu/projects/emdb_core/en/latest/core/configuration.html) of the core documentation.

The *experiments* package also includes these variants:

- ***bartender_experiment_single_client.yaml***: same agent configuration, but 60000 iterations, the stages start at iterations 0, 20000 and 30000, and the periodic saving of the P-Nodes and Goals content every 5000 iterations. It is meant to be used with `single_client:=True`.
- ***bartender_experiment_client_stages.yaml***: 180000 iterations, stages at 0, 45000 and 70000, and a `ClientStages` section that sets which clients appear: from iteration 0, clients 1 and 2; from 120000, only client 3; from 150000, the three clients. It also adds an eighth purpose, the Need *knowledge_reuse* (drive, goal, P-Node, C-Node and `knowledge_reuse_policy`), excluded from the *novelty_policy*, and `preactive_factor: 0.5` in the C-Node connector. The `ClientStages` are ignored when `single_client` is `True`.
- ***bartender_exp/***: 17 files, *bartender_experiment_0.yaml* to *bartender_experiment_16.yaml*, of 60000 iterations each (stage0 from 0, stage1 at 10000 or 20000, i.e. after one third or two thirds of the first 30000 iterations, and stage2 at 30000). They only differ from each other in the start of stage1 and in four parameters of the `ANNSpace` of the P-Nodes (`sampled_points`, `train_every`, `min_warmup_samples` and `min_samples_per_class`), whose values for each file are given in the first line of the file. Files 0 to 15 are the conditions of a factorial design (named `DOE1` in those comments), and 16 is the baseline (`BASELINE-A`), with the values of the main file.

Once executed, it is possible to see the logs in the terminal, being able to follow the behavior of the experiment in real time. The files listed in the `Files` section of the YAML (for example *goodness.txt* and *pnodes_success.txt*) are written by the main loop in the current directory, with a number before the extension.
