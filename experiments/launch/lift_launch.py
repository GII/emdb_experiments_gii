from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.substitutions import (
    LaunchConfiguration,
    FindExecutable,
    PathJoinSubstitution,
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

    # The bridge plays the "simulator" role for the architecture: it hosts the
    # executed_action / world_reset services and triggers commander/load_experiment.
    # The actual physics lives in emdb_simulator's scene_loader, launched
    # separately in the TFM venv (see lift_experiment.yaml header).
    bridge_node = Node(
        package="mujoco_emdb_sim",
        executable="sim_bridge",
        output="screen",
        arguments=["--ros-args", "--log-level", logger],
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

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=core_node,
            on_exit=[Shutdown()],
        )
    )

    return [config_service_call, core_node, ltm_node, bridge_node, shutdown_on_exit]


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            "log_level", default_value=["info"], description="Logging level"),
        DeclareLaunchArgument(
            "random_seed", default_value="0",
            description="The seed to the random numbers generator"),
        DeclareLaunchArgument(
            "experiment_file", default_value="lift_experiment.yaml",
            description="The file that loads the experiment config"),
        DeclareLaunchArgument(
            "config_file", default_value="commander_threaded.yaml",
            description="The file that loads the commander config"),
        DeclareLaunchArgument(
            "config_package", default_value="core",
            description="Package where the config file is located"),
        DeclareLaunchArgument(
            "experiment_package", default_value="experiments",
            description="Package where the experiment file is located"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
