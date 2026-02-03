This is part of the e-MDB architecture documentation. Main page [here.](https://docs.pillar-robots.eu/en/latest/)


# e-MDB experiments implemented by the GII

In this [repository](https://github.com/pillar-robots/emdb_experiments_gii), you can find experiments developed by the GII that use the e-MDB cognitive architecture. The following experiments are implemented:

- [PutObjectInBox Experiment](experiments/put_object_in_box_experiment.md): Experiment that demonstrates the operation of the Long-Term Memory (LTM) of the cognitive architecture. It uses a simple motivational structure with predefined goals. It is simulated with a discrete event simulator.

- [OSCAR Experiment](experiments/oscar_experiment.md): Two experiments were carried out with this simulation. The first one is a manipulation experiment simulated in Gazebo with the OSCAR, a dual-armed robot. It also focuses on the operation of the Long-Term Memory (LTM). The second experimnt validates the alignment engine and automated mission and drive generation.

- [FruitShop Experiment](experiments/fruit_shop_experiment.md): Experiment in which the behavior of the motivational system and the implemented intrinsic motivations are tested. A bridge to execute this experiment with a real TIAGo++ robot is also provided. -->

<!-- - [PumpPanel Experiment](experiments/pump_panel_experiment.md): Experiment that demonstrates the use of the LLM exploration policy to solve a complex multi-step task. -->

2D Manipulation Experiment (Work in progress): Experiment that demonstrates low-level actuation, and learning of deliverative models.

The *experiments* ROS package includes the YAML files that configure the experiments and the launch files to run them.

In this repository are also included packages that allow the communication between the cognitive architecture and some of the simulators used in the experiments (OSCAR, for the moment). Their API documentation can be found in its respective section.

```{toctree}
:hidden:
:caption: GII Experiments
:glob:

experiments/put_object_in_box_experiment.md
experiments/oscar_experiment.md
experiments/fruit_shop_experiment.md
experiments/manipulation_2d.md

```

```{toctree}
:hidden:
:caption: API Documentation
:glob:

api_documentation/oscar_emdb.rst
api_documentation/sim_2d_emdb.rst

```