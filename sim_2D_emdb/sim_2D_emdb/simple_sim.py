from enum import Enum
import importlib
import os.path
import math
import yaml
import yamlloader
import numpy
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from core.service_client import ServiceClient

from core_interfaces.srv import LoadConfig
from core.utils import class_from_classname



class Sim2DSimple(Node):
    def __init__(self):
        pass

def main():
    print('Hi from sim_2D_emdb.')


if __name__ == '__main__':
    main()
