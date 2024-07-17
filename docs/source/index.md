This is part of the e-MDB architecture documentation. Main page [here.](https://docs.pillar-robots.eu/en/latest/)


# e-MDB Experiments implemented by the GII

In this [repository](https://github.com/pillar-robots/emdb_experiments_gii), you can find experiments developed by the GII that use the e-MDB cognitive architecture. At this moment, two experiments are implemented:

- [Default experiment](experiments/default_experiment.md): Experiment implemented using a Discrete Event Simulator that simulates, without physics, the Baxter robot, which has two arms.

- [Oscar experiment](experiments/oscar_experiment.md): Experiment using two robotic arms in Gazebo.

You can find more information about each experiment in its respective section.

The *experiments* ROS package includes the YAML files that configure the experiments and the launch files to run them.

```{toctree}
:hidden:
:caption: GII Experiments
:glob:

experiments/*

```

```{toctree}
:hidden:
:caption: API Documentation
:glob:

api_documentation/*

```