=======================
OSCAR API Documentation
=======================

This API documentation provides a detailed description of all the components that make up the OSCAR 
robot interface for the e-MDB Cognitive Architecture. OSCAR is a robot simulated in Gazebo that enables robotic 
manipulation and sensory perception for cognitive learning and exploration.

++++++++++++++
OSCAR Server
++++++++++++++

Python script which implements the interface between the OSCAR robot and the e-MDB Cognitive Architecture.
It provides high-level manipulation capabilities like grasping or placing objects.
This module also manages the environment state, perceptions, and reward functions, enabling robots to
interact with objects in a simulated environment.

.. automodule:: oscar_emdb.oscar_emdb_server
    :members:
    :show-inheritance:


++++++++++++++++
OSCAR Perception
++++++++++++++++

Python script which implements the perception system for the OSCAR robot. It processes raw
sensory data from cameras and grippers in the Gazebo simulation, transforming them into high-level representations.
This module handles visual object recognition, position calculation, and tactile feedback processing,
enabling robots to perceive and understand their environment.

.. automodule:: oscar_perception.oscar_perception
    :members:
    :show-inheritance:
