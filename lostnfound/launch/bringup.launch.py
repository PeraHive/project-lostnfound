import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'lostnfound'
    package_dir = get_package_share_directory(package_name)
    
    # Path to the wrapper script
    ardupilot_script = os.path.join(package_dir, 'scripts', 'start_ardupilot.sh')
    
    # ArduPilot SITL simulation using wrapper script
    ardupilot_process = ExecuteProcess(
        cmd=[ardupilot_script],
        output='screen',
        shell=True
    )
    
    # MAVROS node with proper parameters
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        # namespace='/uav1',
        # name='mavros',
        parameters=[{
            'fcu_url': 'udp://:14551@127.0.0.1:14551',
            'gcs_url': 'udp://:14550@127.0.0.1:14550',
            # 'tgt_system': 1,
            # 'tgt_component': 1,
            # 'fcu_protocol': 'v2.0',
        }],
        output='screen',
        respawn=True,
        respawn_delay=5
    )
    
    # Foxglove Bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'max_qos_depth': 10,
            'num_threads': 4,
        }]
    )
    
    # Lost and Found Node
    # lost_found_node = Node(
    #     package='lostnfound',
    #     executable='lost_found_node',
    #     name='lost_found_node',
    #     output='screen'
    # )
    
    # Event handler to shutdown everything when ArduPilot exits
    ardupilot_exit_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=ardupilot_process,
            on_exit=[
                EmitEvent(event=Shutdown(reason='ArduPilot simulation ended'))
            ]
        )
    )

    return LaunchDescription([
        ardupilot_process,
        ardupilot_exit_event_handler,
        mavros_node,
        foxglove_bridge,
        # lost_found_node,
    ])