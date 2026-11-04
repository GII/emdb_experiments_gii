This is part of the e-MDB architecture documentation. Main page [here.](https://docs.pillar-robots.eu/en/latest/)


# e-MDB experiments implemented by the GII

In this [repository](https://github.com/pillar-robots/emdb_experiments_gii), you can find experiments developed by the GII that use the e-MDB cognitive architecture. The following experiments are implemented:

- [PutObjectInBox Experiment](experiments/put_object_in_box_experiment.md): Experiment that demonstrates the operation of the Long-Term Memory (LTM) of the cognitive architecture. It uses a motivational structure with a Mission and four intrinsic Needs. It is simulated with a discrete event simulator.

- [OSCAR Experiment](experiments/oscar_experiment.md): Two experiments were carried out with this simulation. The first one is a manipulation experiment simulated in Gazebo with the OSCAR, a dual-armed robot. It also focuses on the operation of the Long-Term Memory (LTM). The second experiment validates the alignment engine and automated mission and drive generation.

- [FruitShop Experiment](experiments/fruit_shop_experiment.md): Experiment in which the behavior of the motivational system and the implemented intrinsic motivations are tested. A bridge for a real TIAGo++ robot (a work in progress, see the *tiago_emdb* package) is also included.

- [2D Manipulation Experiment](experiments/manipulation_2d.md): Experiment that demonstrates low-level actuation, and learning of deliberative models.
<!-- - [PumpPanel Experiment](experiments/pump_panel_experiment.md): Experiment that demonstrates the use of the LLM exploration policy to solve a complex multi-step task. -->

The *experiments* ROS package includes the YAML files that configure the experiments and the launch files to run them. The parameters of these files are described in the [configuration page](https://docs.pillar-robots.eu/projects/emdb_core/en/latest/core/configuration.html) of the core documentation.

The repository also contains experiments that do not have a page yet: the bartender experiment, the fruit serve experiment and the 1D simulator experiment.

In this repository are also included packages that allow the communication between the cognitive architecture and some of the simulators used in the experiments (OSCAR and the 2D simulator) and a real robot (TIAGo++). Their API documentation can be found in its respective section. The *sim_1d_emdb* and *bartender_experiment* packages, which connect the architecture with the 1D simulator and the bartender experiment, do not have API documentation yet.

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
api_documentation/tiago_emdb.rst

```