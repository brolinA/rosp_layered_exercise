import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('rosp_layered_exercise'),
        'config',
        'turtlesim_control_params.yaml'
        )
    
    launch_turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim_node",
        output="screen",
    )

    kill_turtle = TimerAction(
        period=0.5,
        actions= [
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='ros2'),
                    " service call ",
                    "/kill ",
                    "turtlesim/srv/Kill ",
                    "'name: turtle1'",
                ]],
                shell=True
            )
        ]
    )

    spawn_new_turtle = TimerAction(
        period=1.0,
        actions= [
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='ros2'),
                    " service call ",
                    "/spawn ",
                    "turtlesim/srv/Spawn ",
                    "'{x: 1.0, y: 1.0, theta: 0, name: 'turtle'}'",
                ]],
                shell=True
            )
        ]
    )

    execute_goal = TimerAction(
        period=1.5,
        actions= [
            Node(
                package="rosp_layered_exercise",
                executable="rosp_layered_exercise",
                name="layered_exercise_node",
                output="screen",
                remappings=[
                ('/pose', '/turtle/pose'),
                ('/cmd_vel', '/turtle/cmd_vel'),
                ],
                parameters=[config]
            )
        ]
    )
    return LaunchDescription([
        launch_turtlesim,
        kill_turtle,
        spawn_new_turtle,
        execute_goal
    ])