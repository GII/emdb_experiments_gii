This is part of the e-MDB architecture documentation. Main page [here.](https://docs.pillar-robots.eu/en/latest/)


# e-MDB experiments implemented by the GII

In this [repository](https://github.com/pillar-robots/emdb_experiments_gii), you can find experiments developed by the GII that use the e-MDB cognitive architecture. The following experiments are implemented:

- [PutObjectInBox Experiment](experiments/put_object_in_box_experiment.md): Experiment that demonstrates the operation of the Long-Term Memory (LTM) of the cognitive architecture. It uses a motivational structure with a Mission and four intrinsic Needs. It is simulated with a discrete event simulator.

- [OSCAR Experiment](experiments/oscar_experiment.md): Two experiments were carried out with this simulation. The first one is a manipulation experiment simulated in Gazebo with the OSCAR, a dual-armed robot. It also focuses on the operation of the Long-Term Memory (LTM). The second experiment validates the alignment engine and automated mission and drive generation.

- [FruitShop Experiment](experiments/fruit_shop_experiment.md): Experiment in which the behavior of the motivational system and the implemented intrinsic motivations are tested. A bridge for a real TIAGo++ robot (a work in progress, see the *tiago_emdb* package) is also included.

- [2D Manipulation Experiment](experiments/manipulation_2d.md): Experiment that demonstrates low-level actuation, and learning of deliberative models.
- [Bartender Experiment](experiments/bartender_experiment.md): Experiment in which a simulated robot prepares and serves drinks to clients with different preferences, and returns the used glass. It is simulated with a discrete event simulator and uses a curriculum of three stages.

- [Fruit Serve Experiment](experiments/fruit_serve_experiment.md): Experiment in which a simulated TIAGo robot takes a fruit from a shelf, weighs it on a scale and puts it in a delivery box. It is simulated with a discrete event simulator and has variants with a gradual reward and with LLM exploration.

- [1D Simulator Experiment](experiments/sim_1d_experiment.md): Experiment in which an agent moves along a line to reach a target, with a single continuous action. It is simulated with the *sim_1d_emdb* package.

<!-- - [PumpPanel Experiment](experiments/pump_panel_experiment.md): Experiment that demonstrates the use of the LLM exploration policy to solve a complex multi-step task. -->

The *experiments* ROS package includes the YAML files that configure the experiments and the launch files to run them. The parameters of these files are described in the [configuration page](https://docs.pillar-robots.eu/projects/emdb_core/en/latest/core/configuration.html) of the core documentation.

In this repository are also included packages that allow the communication between the cognitive architecture and some of the simulators used in the experiments (OSCAR, the 2D simulator and the 1D simulator) and a real robot (TIAGo++). Their API documentation can be found in its respective section.

```{toctree}
:hidden:
:caption: GII Experiments
:glob:

experiments/put_object_in_box_experiment.md
experiments/oscar_experiment.md
experiments/fruit_shop_experiment.md
experiments/manipulation_2d.md
experiments/bartender_experiment.md
experiments/fruit_serve_experiment.md
experiments/sim_1d_experiment.md

```

```{toctree}
:hidden:
:caption: API Documentation
:glob:

api_documentation/oscar_emdb.rst
api_documentation/sim_2d_emdb.rst
api_documentation/tiago_emdb.rst
api_documentation/bartender_experiment.rst
api_documentation/sim_1d_emdb.rst

```