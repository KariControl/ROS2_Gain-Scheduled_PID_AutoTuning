from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name = 'gain_scheduled_control'

def generate_launch_description():
    ld = LaunchDescription()
    node = Node(
        package=pkg_name,
        executable='gain_scheduled_control',
        name='gain_scheduled_control',
        output='screen',
        parameters=[{
        'kp': 0.766527,  # 比例ゲイン 
        'ki': 0.00,  # 積分ゲイン
        'dt': 0.01,  # サンプリング時間
        'set_poin': 0.0,  # 目標値
        'wP0': 1.0,  # kp係数1
        'wP1':0.0,  # kp係数2
        'wP2': 0.0,  # kp係数3
        'wI0': 0.0,  # ki係数1
        'wI1': 0.0,  # ki係数2
        'wI2': 0.0,  # ki係数3
        }]
    )
    ld.add_action(node)
    return ld