This is part of the e-MDB architecture documentation. Main page [here.](https://docs.pillar-robots.eu/en/latest/)


# e-MDB experiments implemented by the GII

In this [repository](https://github.com/pillar-robots/emdb_experiments_gii), you can find experiments developed by the GII that use the e-MDB cognitive architecture. At this moment, two experiments are implemented:

- [PutObjectInBox Experiment](experiments/put_object_in_box_experiment.md): Experiment in which is tested the behavior of the Long-Term Memory (LTM) of the e-MDB cognitive architecture, without any motivational system. It is simulated with a discrete event simulator.

- [OSCAR Experiment](experiments/oscar_experiment.md): The same experiment as the PutObjectInBox one, but being simulated with the OSCAR, a two arms robot in Gazebo.

- **FruitShop Experiment (Work In Progress)**: Experiment in which is tested the behavior of the motivational system and the implemented intrinsic motivations.

The *experiments* ROS package includes the YAML files that configure the experiments and the launch files to run them.

In this repository are also included packages that allow the communication between the cognitive architecture and some of the simulators used in the experiments (OSCAR, for the moment). Their API documentation can be found in its respective section.

```{toctree}
:hidden:
:caption: GII Experiments
:glob:

experiments/put_object_in_box_experiment.md
experiments/oscar_experiment.md

```

```{toctree}
:hidden:
:caption: API Documentation
:glob:

api_documentation/oscar_emdb.rst

```