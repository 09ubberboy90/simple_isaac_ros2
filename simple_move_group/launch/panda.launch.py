import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_core.webots_launcher import WebotsLauncher
from launch.actions import ExecuteProcess
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('webots_driver')
    robot_description_config = xacro.process_file(
        os.path.join(
            pkg_share,
            "urdf",
            "panda_webots.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    ros2_control_params = os.path.join(pkg_share, 'config', 'panda_ros_controllers.yaml')

    # The WebotsLauncher is a Webots custom action that allows you to start a Webots simulation instance.
    # It searches for the Webots installation in the path specified by the `WEBOTS_HOME` environment variable and default installation paths.
    # The accepted arguments are:
    # - `world` (str): Path to the world to launch.
    # - `gui` (bool): Whether to display GUI or not.
    # - `mode` (str): Can be `pause`, `realtime`, or `fast`.

    webots = WebotsLauncher(
        world=os.path.join(pkg_share, 'worlds', 'panda.wbt'),
        mode="realtime",
        gui="True"
    )

    # The node which interacts with a robot in the Webots simulation is located in the `webots_ros2_driver` package under name `driver`.
    # It is necessary to run such a node for each robot in the simulation.
    # Typically, we provide it the `robot_description` parameters from a URDF file and `ros2_control_params` from the `ros2_control` configuration file.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )    
    webots_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        parameters=[
            robot_description,
            ros2_control_params
        ],
    )


    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_control_params],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )


    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 control load_start_controller {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Standard ROS 2 launch description
    return launch.LaunchDescription([

        # Start the Webots node
        webots,

        # Start the Webots robot driver
        webots_robot_driver,

        # Start the robot_state_publisher
        robot_state_publisher,
        # ros2_control_node,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
