from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    detect_colors_node = Node(
        package='task2_vision',
        executable='detect_colors',
        name='detect_colors_node',
        output='screen'
    )

    visual_servo_planner_node = Node(
        package='task2_vision',
        executable='visual_servo_planner',
        name='visual_servo_planner',
        output='screen',
        parameters=[{
            'object_timeout': 1.0,
            'command_cooldown': 1.0,
            'center_tolerance_x': 40,
            'center_tolerance_y': 40,
            'touch_area_threshold': 18000,
        }]
    )

    mg400_executor_node = Node(
        package='task2_vision',
        executable='mg400_executor',
        name='mg400_executor',
        output='screen',
        parameters=[{
            'command_cooldown': 1.0,
            'shift_step': 0.01,
            'approach_step': 0.02,
            'touch_z': 0.02,
            'observe_x': 0.0,
            'observe_y': 0.0,
            'observe_z': 0.20,
            'action_name': '/mg400/mov_l',
        }]
    )

    return LaunchDescription([
        detect_colors_node,
        visual_servo_planner_node,
        mg400_executor_node,
    ])