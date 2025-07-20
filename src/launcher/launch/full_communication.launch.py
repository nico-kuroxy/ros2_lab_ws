##################################################################################################
# # DEPENDENCIES
# Python libraries
# ROS2 libraries
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
##################################################################################################

##################################################################################################
# Generate the launch description.
def generate_launch_description():

    # Setup the rosbridge node.
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[],
        output='screen'
    )

    # Setup the cloudflare tunnel.
    cloudflared_tunnel = ExecuteProcess(
        cmd=['cloudflared', 'tunnel', 'run', 'laboratory'],
        output='screen'
    )

    # Start all communication processes.
    return LaunchDescription([
        rosbridge_node,
        cloudflared_tunnel
    ])

##################################################################################################