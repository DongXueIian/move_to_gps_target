import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node,PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
from pathlib import Path


def generate_launch_description():
    move_to_gps_target_dir = get_package_share_directory('move_to_gps_target')
    ld = LaunchDescription()
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            f'{Path(move_to_gps_target_dir) / "rviz2" / "test_nav2_apm_mppi.rviz"}',
        ],
        parameters=[{'use_sim_time': True}],
    )


    not_height_tf_transform_cmd=Node(
            package='move_to_gps_target',
            executable='gztf_filter_not_height',
            name='not_height_tf_transform',
            output='screen',
            respawn=False,
    )

    # 构建 Python 脚本的完整路径
    apm_controller_script = Path(move_to_gps_target_dir) / "launch" / "apm_keyborad_controller.py"

    # 启动新的命令行窗口并执行 Python 脚本
    apm_controller_process = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', f'python3 {apm_controller_script}; exec bash'],
        output='screen'
    )


    ld.add_action(rviz)
    # ld.add_action(not_height_tf_transform_cmd)
    ld.add_action(apm_controller_process)

    return ld