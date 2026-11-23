# ur5_mujoco_emdb: e-MDB node of the MuJoCo UR5e simulator

The e-MDB node of the MuJoCo UR5e simulator (`sim_ur5`), with its launch file and the scripts to run the experiments
and look at their results. The simulation, a UR5e arm on a table with a 5 cm ball (and a box), is the *ur5_mujoco*
package, in the `physics_simulators` folder of
[emdb_discrete_event_simulator_gii](https://github.com/GII/emdb_discrete_event_simulator_gii); its README describes
the scenes. It runs on ROS 2 Jazzy.

The gripper is chosen by the `Robot: gripper` entry of the `SimulatedUR5` section of the experiment yaml:

| `gripper` | Scenario of *ur5_mujoco* | Grasp |
|---|---|---|
| `none` (default) | `UR5Scenario` (`ur5e_ball_box_scene.xml`) | Magnet: the ball sticks to the flange within 7 cm |
| `robotiq_2f85` | `UR5GripperScenario` (`ur5e_2f85_ball_box_scene.xml`) | Robotiq 2F-85 with a grasp macro (close, check with a short lift, reopen) |
| `onrobot_2fg7` | `UR5TwoFG7Scenario` (`ur5e_2fg7_ball_box_scene.xml`) | OnRobot 2FG7, no macro, smooth motion: the learner has to close, lift and reopen by itself |

The experiment that uses it is `ur5_2fg7_lift_pl_only_experiment.yaml` of the *experiments* package (2FG7, grasp
and lift the ball 5 cm), started with its `ur5_launch.py`.

## Setup

The Python packages that Ubuntu 24.04 does not ship can go in a virtual environment that also sees the system
packages (numpy must stay below 2 for ROS 2 Jazzy):

```bash
python3 -m venv --system-site-packages ~/venvs/emdb
echo "numpy<2" > ~/venvs/emdb/constraints.txt
~/venvs/emdb/bin/pip install -c ~/venvs/emdb/constraints.txt torch --index-url https://download.pytorch.org/whl/cpu
~/venvs/emdb/bin/pip install -c ~/venvs/emdb/constraints.txt "stable-baselines3>=2.3" "gymnasium>=0.29" \
    "mujoco>=3.3" ruckig yamlloader
sudo apt install ffmpeg   # optional: videos of the trials
```

`ruckig` is the trajectory generator of the 2FG7 scene; torch, stable-baselines3 and gymnasium are used by the
learned policies of e-MDB (`PolicyLearned`). Build from the colcon workspace that holds the e-MDB repositories:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to core cognitive_nodes cognitive_processes dummy_nodes \
    simulators sim_2d_emdb experiments ur5_mujoco ur5_mujoco_emdb
```

Per-machine settings can go in `<workspace>/emdb_env.sh`, which `scripts/run_ur5.sh` sources (instead of ROS and
the workspace) when it exists. For example:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/<workspace>/install/setup.bash"
export PYTHONPATH="$HOME/venvs/emdb/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=50              # keep e-MDB runs apart from other ROS 2 systems on the network
export OMP_NUM_THREADS=1             # torch on CPU: one thread per process (e-MDB runs many small networks)
```

## Running an experiment

```bash
scripts/run_ur5.sh ur5_2fg7_lift_pl_only_experiment.yaml 1                 # seed 1, viewer open
VISUALIZE=False scripts/run_ur5.sh ur5_2fg7_lift_pl_only_experiment.yaml 2
VIDEO_EVERY=50 scripts/run_ur5.sh ur5_2fg7_lift_pl_only_experiment.yaml 1  # an mp4 of one trial in 50
scripts/run_ur5.sh ur5_2fg7_lift_pl_only_experiment.yaml 1 realtime:=true sim_speed:=1.0   # watch at real speed
```

Each run gets its own folder, `<workspace>/results/<yaml name>/seed<N>_<date-time>/` (`EMDB_RESULTS` changes the
results folder), with `launch.log`, the e-MDB files (`trials_0.txt`, `goodness_0.txt`, ...), the logs of the
learned policies (`logs/policy_*/`: `traces.csv`, `turns.csv`, `progress.csv`, and the saved model
`sac_model.zip` with `sac_model.json`) and the videos (`videos/trial_XXXXX.mp4`). Use seeds 1, 2, 3...: seed 0
means "random from the clock" (the seed used is printed in the log).

Scripts to look at the runs (with no argument they take the newest run):

| Script | What |
|---|---|
| `scripts/progress.py [RUN_DIR]` | Progress of a run: pace, estimated end, success rate, actions per success, policy learning |
| `scripts/plot_run.py [RUN_DIR]` | One run in detail: `<run>/run_summary.png` (successes over time and who reached them, success rate, actions per trial, SAC training, frames of a recorded trial) |
| `scripts/compare_runs.py [RUN_DIR ...]` | Several runs side by side: a text summary per run and one figure |

The simulation runs as fast as it can (about 0.02 s of wall clock per action), so the architecture sets the pace,
and with the viewer each motion flashes by. `realtime:=true sim_speed:=1.0` paces it to the wall clock; videos
(`VIDEO_EVERY=N`) always play at real speed, one frame per 1/30 s of simulated time, whatever the clock. Video
recording works with or without the viewer: the offscreen recorder starts before the viewer window opens (in the
other order its EGL context cannot be made current). If it cannot record, the log shows
`Video recording disabled: ...` and the run goes on without video.

## Simulator node

`sim_ur5` (launch file `launch/ur5_sim_launch.py` to start it alone) reads the `SimulatedUR5` section of the
experiment yaml (`Robot`, `Actuation`, `Perceptions`) and its `Control` section (the action and world reset
services), and publishes the perceptions on `/mdb/ur5/sensor/<name>`. Parameters:

| Parameter | Default | What |
|---|---|---|
| `config_file` | | Experiment yaml |
| `random_seed` | 0 | Seed of the ball placement (0: from the clock) |
| `visualize` | True | MuJoCo viewer |
| `realtime`, `sim_speed` | False, 2.0 | Physics paced to the wall clock, at that speed |
| `spawn_box` | True | False parks the box off the scene |
| `perception_rate` | 20.0 | Hz of the perception topics |
| `video_dir`, `video_every` | "", 1 | 2FG7 scene: record one trial in `video_every` to `video_dir` |
| `model_path` | "" | Scene file (default: the one of the gripper) |
| `standalone` | False | Do not load the experiment in the commander (to test the node alone) |

`Robot` has the `gripper` and, optionally, `hover_z`; with the 2F-85 `verify_lift`; with the 2FG7 `grip_force`,
`grip_speed`, `lift_height`, `tool_speed`, `tool_accel` and `blend_time` (see the README of *ur5_mujoco*). The
perceptions of the 2FG7 experiment are the ball relative to the pinch point (`ball_x`, `dist_gripper_ball`,
`gripper_ball_dz`), plus two readings that the real robot provides: `gripper_height` (pinch point height, from the
arm's kinematics) and `gripper_width` (aperture, as OnRobot's driver reports it). `ball_lifted` (1 when both pads
touch the ball and its bottom is 5 cm above the table) is the input of the goal; `grip_force` and `ball` are published for
watching.
