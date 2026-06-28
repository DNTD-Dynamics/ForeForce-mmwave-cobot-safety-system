"""
dntd_mmwave_launch.py
DNTD Dynamics — mmWave Safety System Launch File

Single sensor (development):
  ros2 launch dntd_mmwave dntd_mmwave_launch.py

Three sensors (production 360° array):
  ros2 launch dntd_mmwave dntd_mmwave_launch.py sensor_count:=3

Each sensor driver gets its own namespace (/dntd/sensor_0, /dntd/sensor_1, ...)
The safety node subscribes to all active sensor namespaces.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


# ---------------------------------------------------------------------------
# Port assignments per sensor index
# Adjust if your udev rules assign different names
# ---------------------------------------------------------------------------
def sensor_ports(index: int) -> tuple[str, str]:
    """Returns (cli_port, data_port) for sensor at given index."""
    base = index * 2
    return f"/dev/ttyUSB{base}", f"/dev/ttyUSB{base + 1}"


def sensor_config_file(index: int) -> str:
    """Returns config file path for sensor at given index."""
    _REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) 
    configs = [
        os.path.join(_REPO_ROOT, 'configs', 'profile_AOP.cfg'),
        os.path.join(_REPO_ROOT, 'configs', 'profile_AOP_sensor1.cfg'),
        os.path.join(_REPO_ROOT, 'configs', 'profile_AOP_sensor2.cfg'),
    ]
    
    return configs[min(index, len(configs) - 1)]


# ---------------------------------------------------------------------------
# Launch description builder
# ---------------------------------------------------------------------------

def generate_launch_description():
    # Declare arguments
    sensor_count_arg = DeclareLaunchArgument(
        'sensor_count',
        default_value='1',
        description='Number of mmWave sensors (1, 2, or 3)',
    )
    safety_config_arg = DeclareLaunchArgument(
        'safety_config',
        default_value=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'configs', 'dntd_mmwave_config.yaml'),
        description='Path to safety node YAML config',
    )

    return LaunchDescription([
        sensor_count_arg,
        safety_config_arg,
        OpaqueFunction(function=_launch_nodes),
    ])


def _launch_nodes(context, *args, **kwargs):
    sensor_count = int(LaunchConfiguration('sensor_count').perform(context))
    safety_config = LaunchConfiguration('safety_config').perform(context)

    nodes = []

    # --- One driver node per sensor ---
    for i in range(sensor_count):
        cli_port, data_port = sensor_ports(i)
        cfg_file            = sensor_config_file(i)
        namespace           = f"/dntd/sensor_{i}"
        frame_id            = f"mmwave_sensor_{i}"

        driver = Node(
            package    = 'dntd_mmwave',
            executable = 'dntd_mmwave_driver_node',
            name       = f'mmwave_driver_{i}',
            namespace  = namespace,
            parameters = [{
                'cli_port':        cli_port,
                'cli_baud':        115200,
                'data_port':       data_port,
                'data_baud':       921600,
                'config_file':     cfg_file,
                'sensor_frame_id': frame_id,
                'sensor_model':    'iwr6843aop',
                'publish_hz':      10.0,
                'send_config':     True,
                'config_retry':    True,
            }],
            output = 'screen',
        )
        nodes.append(driver)

    # --- Safety node (subscribes to all sensor namespaces) ---
    # Build the list of raw_points topics for this sensor count
    raw_topics = [
        f"/dntd/sensor_{i}/mmwave/raw_points"
        for i in range(sensor_count)
    ]

    safety = Node(
        package    = 'dntd_mmwave',
        executable = 'dntd_mmwave_safety_node',
        name       = 'mmwave_safety',
        namespace  = '/dntd',
        parameters = [
            safety_config,
            # Override topic list based on actual sensor count
            {'raw_point_topics': raw_topics},
        ],
        output = 'screen',
        # Remappings for single-sensor convenience
        remappings = [
            # When sensor_count=1, remap the default topic directly
            ('/dntd/mmwave/raw_points', raw_topics[0]),
        ] if sensor_count == 1 else [],
    )
    nodes.append(safety)

    return nodes
