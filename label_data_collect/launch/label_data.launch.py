from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction

import os
from ament_index_python.packages import get_package_share_directory
import yaml



def generate_launch_description():

    aruco_params_file = os.path.join(
        get_package_share_directory('label_data_collect'),
        'config',
        'aruco_parameters.yaml'
    )

    with open(aruco_params_file, 'r') as file:
        config = yaml.safe_load(file)

    config = config["/aruco_node"]["ros__parameters"]
    
    # get param
    output_dir_arg = DeclareLaunchArgument(
        name='output_dir',
        default_value=config['output_dir'],
        description='the path of the output data',
    )
    image_topic_arg = DeclareLaunchArgument(
        name='image_topic',
        default_value=config['image_topic'],
        description='Name of the image RGB topic to subscribe to',
    )

    use_depth_input_arg = DeclareLaunchArgument(
        name='use_depth_input',
        default_value=str(config['use_depth_input']),
        description='Use depth input for pose estimation',
        choices=['true', 'false', 'True', 'False']
    )

    depth_image_topic_arg = DeclareLaunchArgument(
        name='depth_image_topic',
        default_value=config['depth_image_topic'],
        description='Name of the depth image topic to subscribe to',
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        name='camera_info_topic',
        default_value=config['camera_info_topic'],
        description='Name of the camera info topic to subscribe to',
    )

    camera_frame_arg = DeclareLaunchArgument(
        name='camera_frame',
        default_value=config['camera_frame'],
        description='Name of the camera frame where the estimated pose will be',
    )


    # launch realsense camera node
    cam_feed_launch_file = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    camera_feed_depth_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_feed_launch_file),
        launch_arguments={
            "pointcloud.enable": "false",
            "enable_rgbd": "true",
            "enable_sync": "true",
            "enable_color": "true",
            "enable_depth": "true",

            "depth_module.profile":"1280x720x15" ,
            "rgb_camera.profile":"1280x720x15",
            "align_depth.enable": "true"
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_depth_input'))
    )

    camera_feed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_feed_launch_file),
        launch_arguments={
            "pointcloud.enable": "true",
            "enable_color": "true",
            "rgb_camera.profile":"1280x720x30" 
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_depth_input'))
    )

    rviz_file = PathJoinSubstitution([
        FindPackageShare('label_data_collect'),
        'rviz',
        'cam_detect.rviz'
    ])
    print("rviz_file:",rviz_file)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )


    aruco_node = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[Node(
                    package='label_data_collect',
                    executable='aruco_node.py',
                    parameters=[{
                        "output_dir":LaunchConfiguration('output_dir'),
                        "image_topic": LaunchConfiguration('image_topic'),
                        "use_depth_input": LaunchConfiguration('use_depth_input'),
                        "depth_image_topic": LaunchConfiguration('depth_image_topic'),
                        "camera_info_topic": LaunchConfiguration('camera_info_topic'),
                        "camera_frame": LaunchConfiguration('camera_frame')
                    }],
                    output='screen',
                    emulate_tty=True
                )]
    )

    
    return LaunchDescription([
        # Arguments
        output_dir_arg,
        image_topic_arg,
        use_depth_input_arg,
        depth_image_topic_arg,
        camera_info_topic_arg,
        camera_frame_arg,

        # Nodes
        camera_feed_depth_node,
        camera_feed_node,
        #rviz2_node,
        aruco_node
        
        #rviz2_node
    ])