#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # Camera drive node
    camera_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('usb_cam'),
                'launch',
                'camera.launch.py'
            ])
        ])
    )
    
    # First image viewer node
    image_viewer_1 = Node(
        package = 'image_tools',
        executable = 'showimage',
        name = 'image_viwever_1',
        remappings=[
            ('image', 'camera1/image_raw')
        ]
    )

    # Gray scale convter  node
    gray_scale_convter = Node(
        package = 'smarter_image_processing',
        executable = 'gray_scale_converter',
        name = 'gray_scale_converter',
        remappings=[
            ('image', 'camera1/image_raw')
        ]
    )
    
    # Second image viewer node
    image_viewer_2 = Node(
        package = 'image_tools',
        executable = 'showimage',
        name = 'image_viwever_2',
        remappings=[
            ('image', 'camera1/gray_raw')
        ]
    )
    
    return LaunchDescription([
        camera_drive,
        image_viewer_1,
        gray_scale_convter,
        image_viewer_2
    ])
