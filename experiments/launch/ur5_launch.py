from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    FindExecutable,
    PathJoinSubstitution,
    Command,
)


def launch_setup(context: LaunchContext, *args, **kwargs):

    logger = LaunchConfiguration("log_level")
    random_seed = LaunchConfiguration("random_seed")
    visualize = LaunchConfiguration("visualize")
    experiment_file = LaunchConfiguration("experiment_file")
    experiment_package = LaunchConfiguration("experiment_package")
    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")
    realtime = LaunchConfiguration("realtime")
    sim_speed = LaunchConfiguration("sim_speed")
    spawn_box = LaunchConfiguration("spawn_box")
    perception_rate = LaunchConfiguration("perception_rate")
    video_dir = LaunchConfiguration("video_dir")
    video_every = LaunchConfiguration("video_every")

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

    simulator_node = Node(
        package="ur5_mujoco_emdb",
        executable="sim_ur5",
        output="screen",
        arguments=["--ros-args", "--log-level", logger],
        parameters=[
            {
                "random_seed": random_seed,
                "config_file": PathJoinSubstitution(
                    [FindPackageShare(experiment_package), "experiments", experiment_file]
                ),
                "visualize": visualize,
                "realtime": realtime,
                "sim_speed": sim_speed,
                "spawn_box": spawn_box,
                "perception_rate": ParameterValue(perception_rate, value_type=float),
                "video_dir": ParameterValue(video_dir, value_type=str),
                "video_every": ParameterValue(video_every, value_type=int),
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
            target_action=core_node,  # Nodo que supervisar
            on_exit=[Shutdown()],  # Acción: Cerrar todos los nodos
        )
    )

    nodes_to_start = [config_service_call, core_node, ltm_node, simulator_node, shutdown_on_exit]

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
            default_value="ur5_2fg7_lift_pl_only_experiment.yaml",
            description="The file that loads the experiment config",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="commander_threaded.yaml",
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

    declared_arguments.append(
        DeclareLaunchArgument(
            "visualize",
            default_value="True",
            description="Whether to visualize the simulation or not",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "realtime",
            default_value="False",
            description="Pace the simulation to wall clock for smooth viewing",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_speed",
            default_value="2.0",
            description="Playback speed when realtime is on (2.0 = 2x)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_box",
            default_value="True",
            description="Whether the target box is present. False parks it "
                        "off-scene (invisible, inert) for a ball-only setup.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "perception_rate",
            default_value="20.0",
            description="Hz at which the simulator publishes its perceptions. Every perception read waits for a "
                        "fresh message of each topic, so a higher rate shortens each architecture step; "
                        "~200 messages/s in total is fine (topics x rate).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "video_dir",
            default_value="",
            description="2FG7 scenario: folder for mp4 videos of the trials (empty: no video). One frame per "
                        "1/30 s of simulated time, so the videos play at real speed whatever the clock.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "video_every",
            default_value="1",
            description="2FG7 scenario: record one trial out of this many",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )