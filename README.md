# e-MDB configuration files for launching experiments

In this repository, you can find experiments developed by the GII that use the e-MDB cognitive architecture:

- **Put object in box**: Experiment implemented using a Discrete Event Simulator that simulates, without physics, the Baxter robot, which has two arms.
- **Fruit shop** and **Fruit serve**: Experiments with a Discrete Event Simulator of a TIAGo++ robot.
- **Bartender**: Experiment in which a simulated robot has to serve drinks to its clients.
- **OSCAR**: Experiments using two robotic arms in Gazebo, one of them with the alignment of the robot purposes.
- **2D and 1D simulators**: Experiments with low-level actions, which learn models or policies.
- **Pump panel**: Experiment with the LLM exploration policy, which is a work in progress.

The *experiments* ROS package includes the YAML files that configure the experiments and the launch files to run them. For example:

```
ros2 launch experiments put_object_in_box_launch.py
```

The rest of the packages connect the architecture with some of the simulators and robots of the experiments.

You can find information and tutorials about this repository and the others that are part of the e-MDB cognitive architecture software in the [PILLAR Robots official documentation](https://docs.pillar-robots.eu/en/latest/)

