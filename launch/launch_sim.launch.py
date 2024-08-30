import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the ros_gz_sim launch file

    package_name='my_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gz_sim_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    # Include the gz_sim launch file with the empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={'gz_args': '~/maze.sdf'}.items(),

    )

    # Spawn the robot with initial pose adjustment

    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            # '-file', urdf_file,
            '-topic', '/robot_description',
            '-name', 'diffbot',
            '-x', '0',
            '-y', '0',
            '-z', '0.25',  # Adjust Z position to lift the robot
        ]
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/diffbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/diffbot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/model/diffbot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/world/empty/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
                   ],
        remappings=[
            ('/model/diffbot/tf', '/tf'),
            # ('/world/empty/model/diffbot/link/base_link/sensor/laser/scan', '/scan')
            # ('/world/empty/pose/info', '/tf')
            # ('/model/diffbot/tf_static', '/tf_static')
        ],
        parameters=[
            {'qos_overrides./model/diffbot.subscriber.reliability': 'reliable'},
            # {'gz_frame_id./model/diffbot/cmd_vel': 'custom_cmd_vel_frame'},
            # {'gz_frame_id./model/diffbot/odometry': 'custom_odometry_frame'},
            # {'gz_frame_id./model/diffbot/tf': '/tf'},
        ],
        # parameters=[{'qos_overrides./model/diffbot.subscriber.reliability': 'reliable'}],
        output='screen'
    )


    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',  # Adjust if your joystick is on a different device
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    config = os.path.join(
        get_package_share_directory('my_bot'),
        'config',
        'teleop_twist_joy.yaml'
        )

    # Node to run the teleop_twist_joy node with custom configuration
    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config],
            # remappings=[('/cmd_vel', '/cmd_vel_joy')], #'model/diffbot/cmd_vel')],
            output='screen'
        )
    
    
    lwheelpub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_left_wheel',
        arguments=['0', '0.175', '0', '0', '0', '-1.57', 'base_link', 'left_wheel'],
    )

    # Static transform publisher for right wheel
    rwheelpub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_right_wheel',
        arguments=['0', '-0.175', '0', '0', '0', '1.57', 'base_link', 'right_wheel']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz21',
    )

    # map_params = config = os.path.join(
    #     get_package_share_directory('my_bot'),
    #     'config',
    #     'mapper_params_online_async.yaml'
    #     )

    # map = Node(
    #     package='slam_toolbox',
    #     executable='online_async_launch.py',
    #     name='slam_toolbox',
    #     parameters=[{'use_sim_time': True, 'params_file': '~/ros2_ws/src/my_bot/config/mapper_params_online_async.yaml'}],
    #     output='screen'
    # )

    # Include the SLAM Toolbox launch file
    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(get_package_share_directory('my_bot'), 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    localize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(get_package_share_directory('my_bot'), 'config', 'mapper_params_online_async2.yaml')
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        # rsp,
        # gazebo,
        # spawn_robot,
        # bridge,
        joy_node,
        teleop_twist_joy_node,
        # lwheelpub,
        # rwheelpub,
        # rviz,
        slam_toolbox,
        # localize
    ])
