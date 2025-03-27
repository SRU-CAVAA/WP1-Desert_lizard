import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node

###################################
# This ros2 launch file launches the webots simulation engine.
# The open_arena.wbt world file does not include the robot yet, they will be spawned by a Supervisor node.
# Still, this launch file also includes the controllers that the robot will use later on.
# This launch file also enables the webots ros2 supervisor functions, though the Supervisor node will be runned later on.

def get_ros2_control_spawners(*args):
    # Declare here all nodes that must be restarted at simulation reset
    ros_control_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffdrive_controller']
    )
    return [
        ros_control_node
    ]


def generate_launch_description():
    package_dir = get_package_share_directory('webots_pkg')
    robot_description_path = os.path.join(package_dir, 'resource', 'webots_epuck.urdf')
    robot_description_path_predator = os.path.join(package_dir, 'resource', 'webots_epuck_predator.urdf')
    robot_description_path_peer = os.path.join(package_dir, 'resource', 'webots_epuck_peer.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'open_arena.wbt'),
        ros2_supervisor=True
    )

    epuck_agent = WebotsController(
        robot_name='epuck_agent',
        parameters=[
            {'robot_description': robot_description_path}
        ],
        # Every time one resets the simulation the controller is automatically respawned
        respawn=True
    )

    epuck_predator = WebotsController(
        robot_name='epuck_predator',
        parameters=[
            {'robot_description': robot_description_path_predator}
        ],
        # Every time one resets the simulation the controller is automatically respawned
        respawn=True
    )

    epuck_peer = WebotsController(
        robot_name='epuck_peer',
        parameters=[
            {'robot_description': robot_description_path_peer}
        ],
        # Every time one resets the simulation the controller is automatically respawned
        respawn=True
    )

    allostatic_model_node = Node(
        package='webots_pkg',
        executable='allostatic_model',
        output='screen',
        )

    gradients_node = Node(
        package='webots_pkg',
        executable='gradients',
        output='screen',
        )

    navigation_node = Node(
        package='webots_pkg',
        executable='robot_navigation',
        output='screen',
        )

    data_gathering_node = Node(
        package='webots_pkg',
        executable='data_gathering',
        output='screen',
        )

    supervisor_node = Node(
        package='webots_pkg',
        executable='supervisor',
        output='screen',
        )

    # Declare the reset handler that respawns nodes when robot_driver exits
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=epuck_agent,
            on_exit=get_ros2_control_spawners,
        )
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        epuck_agent,
        epuck_predator,
        epuck_peer,
        allostatic_model_node,
        gradients_node,
        navigation_node,
        supervisor_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
