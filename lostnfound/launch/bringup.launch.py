import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    EmitEvent,
    GroupAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_name = 'lostnfound'
    package_dir = get_package_share_directory(package_name)

    # ---------- Launch arguments ----------
    sim = LaunchConfiguration('sim')  # "true" or "false"
    mavros_ns = LaunchConfiguration('mavros_ns')
    # SITL/UDP (sim) URLs
    sitl_fcu_url = LaunchConfiguration('sitl_fcu_url')
    sitl_gcs_url = LaunchConfiguration('sitl_gcs_url')
    # Hardware serial (real) URLs
    hw_fcu_url = LaunchConfiguration('hw_fcu_url')
    hw_gcs_url = LaunchConfiguration('hw_gcs_url')
    hw_tgt_system = LaunchConfiguration('tgt_system')
    hw_tgt_component = LaunchConfiguration('tgt_component')

    declare_args = [
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='If "true", run ArduPilot SITL and connect via UDP. If "false", connect to real FCU via serial.'
        ),
        DeclareLaunchArgument(
            'mavros_ns', default_value='/mavros',
            description='Namespace for MAVROS node.'
        ),
        # --- SIM defaults (same as your snippet) ---
        DeclareLaunchArgument(
            'sitl_fcu_url', default_value='udp://:14551@127.0.0.1:14551',
            description='FCU URL for MAVROS when using SITL.'
        ),
        DeclareLaunchArgument(
            'sitl_gcs_url', default_value='udp://:14550@127.0.0.1:14550',
            description='GCS URL for MAVROS when using SITL.'
        ),
        # --- REAL hardware defaults (matches your ros2 run example) ---
        DeclareLaunchArgument(
            'hw_fcu_url', default_value='serial:///dev/ttyUSB0:115200',
            description='Serial FCU URL for real hardware (e.g., serial:///dev/ttyUSB0:115200).'
        ),
        DeclareLaunchArgument(
            'hw_gcs_url', default_value='udp://@0.0.0.0:14550',
            description='GCS URL for real hardware.'
        ),
        DeclareLaunchArgument('tgt_system', default_value='1', description='Target MAVLink system id'),
        DeclareLaunchArgument('tgt_component', default_value='1', description='Target MAVLink component id'),
    ]

    # ---------- Common nodes ----------
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

    # Launch service
    launch_service = Node(
        package='bringup',
        executable='launch_sequence',
        name='launch_sequence',
        namespace='',  # keep global
        output='screen',
        parameters=[{
            'mavros_ns': mavros_ns,
            'altitude_reach_threshold': 0.3,   # meters
            'altitude_wait_timeout_s': 20      # seconds
        }]
    )

    # ---------- SIM group (SITL + MAVROS over UDP) ----------
    ardupilot_script = os.path.join(package_dir, 'scripts', 'start_ardupilot.sh')
    ardupilot_process = ExecuteProcess(
        cmd=[ardupilot_script],
        output='screen',
        shell=True
    )
    sim_mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=mavros_ns,
        output='screen',
        respawn=True,
        respawn_delay=5,
        parameters=[{
            'fcu_url': sitl_fcu_url,
            'gcs_url': sitl_gcs_url,
            # Uncomment if you want to force protocol:
            # 'fcu_protocol': 'v2.0',
        }]
    )
    ardupilot_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=ardupilot_process,
            on_exit=[EmitEvent(event=Shutdown(reason='ArduPilot simulation ended'))]
        )
    )

    # RSSI simulator
    rssi_sim = Node(
        package="lostnfound",
        executable="rssi_sim",
        name="rssi_sim",
        output="screen",
        parameters=[{
            "mode": "noisy",          # "ideal" or "noisy"
            "publish_rate_hz": 1.0
        }]
    )

    sim_group = GroupAction(
        condition=IfCondition(sim),
        actions=[
            ardupilot_process,
            ardupilot_exit_handler,
            sim_mavros_node,
            rssi_sim,
        ]
    )

    # ---------- REAL group (MAVROS over serial, no SITL) ----------
    real_mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=mavros_ns,
        output='screen',
        respawn=True,
        respawn_delay=5,
        parameters=[{
            'fcu_url': hw_fcu_url,      # e.g., serial:///dev/ttyUSB0:115200
            'gcs_url': hw_gcs_url,             # e.g., udp://@0.0.0.0:14550
            'tgt_system': hw_tgt_system,
            'tgt_component': hw_tgt_component,
            # 'fcu_protocol': 'v2.0',
        }],
    )
    real_group = GroupAction(
        condition=UnlessCondition(sim),
        actions=[
            real_mavros_node
        ]
    )

    return LaunchDescription(
        declare_args + [
            # Conditional groups
            sim_group,
            real_group,

            # Common nodes
            foxglove_bridge,
            launch_service,
            
        ]
    )
