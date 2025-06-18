# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import sys
import os

# Set up mock imports BEFORE adding paths
import unittest.mock
import importlib.util

# Create mocks for problematic modules
MOCK_MODULES = [
    "rclpy", "std_msgs", "tensorflow", "core_interfaces", 
    "cognitive_node_interfaces", "cognitive_nodes", "oscar_interfaces",
    "oscar_emdb_interfaces", "core", "yamlloader", "simulators",
    "ros2_numpy", "cv_bridge", "cv2", "cameratransform", "gazebo_msgs", 
    "skimage", "core.service_client", "core.utils", "simulators.scenarios_2D",
    "tensorflow.keras", "tensorflow.keras.models", "tensorflow.keras.layers"
]

for mod_name in MOCK_MODULES:
    sys.modules[mod_name] = unittest.mock.MagicMock()

# Now add paths to find the actual modules we want to document
sys.path.insert(0, os.path.abspath("../.."))  # Root of the project
sys.path.insert(0, os.path.abspath("../../oscar_emdb"))
sys.path.insert(0, os.path.abspath("../../oscar_perception"))
sys.path.insert(0, os.path.abspath("../../sim_2d_emdb"))
sys.path.insert(0, os.path.abspath("../../sim_2d_emdb/sim_2d_emdb"))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "e-MDB Experiments implemented by the GII"
copyright = "2024, GII"
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

# These are still needed for autodoc itself
autodoc_mock_imports = MOCK_MODULES

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
