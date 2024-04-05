# e-MDB configuration files for launching experiments

Note:

***WORK IN PROGRESS***

The original repository has been split in 5 and we are refactoring everything, please, be patient while we move and rename everything.

These are the cognitive architecture repositories for PILLAR and their content:

- _emdb_core_. Essential elements of the cognitive architecture. These are necessary to run an experiment using the cognitive architecture.
- _emdb_cognitive_nodes_gii_. Reference implementation for the main cognitive nodes.
- _emdb_cognitive_processes_gii_. Reference implementation for the main cognitive processes.
- _emdb_discrete_event_simulator_gii_. Implementation of a discrete event simulator used in many experiments.
- _emdb_experiments_gii_. Configuration files for experiments.

This repository is devoted to configuration files, in yaml format, for experiments. These files specify which simulated worlds need to be used, what the objectives will be, and what is the initial configuration of long-term memory (LTM) in each experiment. ROS 2 launch files for experiments are stored here too.
