from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    name = LaunchConfiguration('name')
    image_dir = LaunchConfiguration('image_dir')

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='fs_backend'),
        DeclareLaunchArgument('image_dir', default_value='[]'),  # YAML Array als String
        DeclareLaunchArgument('kernel_size_laplacian', default_value='5'),
        DeclareLaunchArgument('num_pyr_lvl', default_value='3'),
        DeclareLaunchArgument('sharpness_patch_size_y', default_value='10'),
        DeclareLaunchArgument('sharpness_patch_size_x', default_value='10'),
        DeclareLaunchArgument('img_resize', default_value='1.0'),
        DeclareLaunchArgument('z_spacing', default_value='1.0'),
        DeclareLaunchArgument('output_path_sharpness', default_value=''),
        DeclareLaunchArgument('output_path_depth', default_value=''),

        Node(
            package='fs_backend',
            executable='cpu_node',
            name=name,
            parameters=[{
                # Hier bleibt es als YAML-String
                'image_dir': image_dir,
                'kernel_size_laplacian': LaunchConfiguration('kernel_size_laplacian'),
                'num_pyr_lvl': LaunchConfiguration('num_pyr_lvl'),
                'sharpness_patch_size_y': LaunchConfiguration('sharpness_patch_size_y'),
                'sharpness_patch_size_x': LaunchConfiguration('sharpness_patch_size_x'),
                'img_resize': LaunchConfiguration('img_resize'),
                'z_spacing': LaunchConfiguration('z_spacing'),
                'output_path_sharpness': LaunchConfiguration('output_path_sharpness'),
                'output_path_depth': LaunchConfiguration('output_path_depth'),
            }]
        )
    ])

