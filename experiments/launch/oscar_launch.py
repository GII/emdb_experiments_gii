from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    FindExecutable,
    PathJoinSubstitution,
    Command,
)


def launch_setup(context: LaunchContext, *args, **kwargs):

    logger = LaunchConfiguration("log_level")
    random_seed = LaunchConfiguration("random_seed")
    experiment_file = LaunchConfiguration("experiment_file")
    experiment_package = LaunchConfiguration("experiment_package")
    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")

    core_node = Node(
        package="core",
        executable="commander",
        output="screen",
        arguments=["--ros-args", "--log-level", logger],
        parameters=[{"random_seed": random_seed}],
    )

    ltm_node = Node(
        package="core",
        executable="ltm",
        output="screen",
        arguments=["0", "--ros-args", "--log-level", logger],
    )

    oscar_perception_node = Node(
        package="oscar_perception",
        executable="oscar_perception_services",
        output="screen",
        arguments=["--ros-args", "--log-level", logger],
    )

    oscar_node = Node(
        package="oscar_emdb",
        executable="oscar_emdb_server",
        output="screen",
        parameters=[
            {
                "random_seed": random_seed,
                "config_file": PathJoinSubstitution(
                    [FindPackageShare(experiment_package), "experiments", experiment_file]
                ),
            }
        ],
    )

    config_service_call = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " ",
                "service call",
                " ",
                "commander/load_config",
                " ",
                "core_interfaces/srv/LoadConfig",
                " ",
                '"{file:',
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(config_package), "config", config_file]
                ),
                '}"',
            ]
        ],
        shell=True,
    )

    nodes_to_start = [config_service_call, core_node, ltm_node, oscar_node, oscar_perception_node]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "random_seed",
            default_value="0",
            description="The seed to the random numbers generator",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "experiment_file",
            default_value="oscar_experiment.yaml",
            description="The file that loads the experiment config",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="commander.yaml",
            description="The file that loads the commander config",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "config_package",
            default_value="core",
            description="Package where the config file is located",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "experiment_package",
            default_value="experiments",
            description="Package where the experiment file is located",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
