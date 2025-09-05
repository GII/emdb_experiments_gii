# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import sys
import os

# Add all necessary paths for documentation
sys.path.insert(0, os.path.abspath("../.."))  # Root of the project
sys.path.insert(0, os.path.abspath("../../oscar_emdb"))
sys.path.insert(0, os.path.abspath("../../oscar_perception"))
sys.path.insert(0, os.path.abspath("../../sim_2d_emdb"))
sys.path.insert(0, os.path.abspath("../../sim_2d_emdb/sim_2d_emdb"))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "e-MDB Experiments implemented by the GII"
copyright = "2025, GII"
author = "GII"
release = "Apache-2.0"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    # "sphinx.ext.autosummary",
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
    "myst_parser",
]

templates_path = ["_templates"]
exclude_patterns = []

# Mock modules that might not be available during documentation build
autodoc_mock_imports = [
    "rclpy",
    "std_msgs", 
    "tensorflow",
    "core_interfaces",
    "cognitive_node_interfaces",
    "cognitive_nodes",
    "oscar_interfaces",
    "oscar_emdb_interfaces",
    "core",
    "yamlloader",
    "simulators",
    "ros2_numpy",
    "cv_bridge",
    "cv2", 
    "cameratransform",
    "gazebo_msgs",
    "skimage",
    "numpy",
    "threading",
    "rcl_interfaces",
    "sensor_msgs",
    "control_msgs",
    "trajectory_msgs",
    "geometry_msgs",
    "yaml",
    "os",
    "enum",
    "rclpy.node",
    "rclpy.callback_groups",
    "rclpy.executors",
    "rclpy.task",
    "rclpy.logging",
    "rclpy.action",
    "simulators.scenarios_2D", 
    "tiago_moveit_py_interfaces", 
    "stereo_location_interfaces", 
    "tts_msgs"
]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
