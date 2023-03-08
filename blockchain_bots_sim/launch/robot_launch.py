import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('blockchain_bots_sim')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'robot_1.urdf')).read_text()
    robot_description2 = pathlib.Path(os.path.join(package_dir, 'resource', 'robot_2.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    ros2_supervisor = Ros2SupervisorLauncher()


    path_planner = Node(
        output ='screen',
        package='sawtooth_bridge',
        executable='service',
    )


    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'robot_1'},
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    my_robot_driver2 = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'robot_2'},
        parameters=[
            {'robot_description': robot_description2},
        ]
    )

    path_planner = Node(
        output ='screen',
        package='blockchain_bots_control',
        executable='blockchain_bots_control_test',
    )
    # obstacle_avoider1 = Node(
    #     output ='screen',
    #     package='blockchain_bots_sim',
    #     executable='obstacle_avoider',
    # )

    # obstacle_avoider2 = Node(
    #     package='blockchain_bots_sim',
    #     executable='obstacle_avoider',
    # )

    return LaunchDescription([
        webots,
        ros2_supervisor,
        my_robot_driver,
        my_robot_driver2,
        path_planner,
        # obstacle_avoider1,
        # obstacle_avoider2,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])