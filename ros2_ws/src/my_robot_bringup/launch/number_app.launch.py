from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Declare launch arguments.
    num2pub = LaunchConfiguration("num2pub")
    pub_freq = LaunchConfiguration("pub_freq")

    num2pub_launch_arg = DeclareLaunchArgument(
        "num2pub",
        default_value='1'
    )

    pub_freq_launch_arg = DeclareLaunchArgument(
        "pub_freq",
        default_value='1.0'
    )

    remap_number_topic = ("number", "my_number")

    # Set up nodes.
    number_publisher_node = Node(
        # package="ros2_app_py",
        package="ros2_app_cpp",
        executable="number_publisher",
        name="my_number_publisher",
        remappings=[
                remap_number_topic
        ],
        parameters=[
            {"num2pub": num2pub},
            {"pub_freq": pub_freq}
        ]
    )

    number_counter_node = Node(
        # package="ros2_app_py",
        package="ros2_app_cpp",
        executable="number_counter",
        name="my_number_counter",
        remappings=[
                remap_number_topic
        ]
    )

    ld = LaunchDescription([
        num2pub_launch_arg,
        pub_freq_launch_arg
    ])

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    return ld
