from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    deviceName = LaunchConfiguration('device_name')
    flAxisNum = LaunchConfiguration('front_left_axis_number')
    frAxisNum = LaunchConfiguration('front_right_axis_number')
    rlAxisNum = LaunchConfiguration('rear_left_axis_number')
    rrAxisNum = LaunchConfiguration('rear_right_axis_number')
    wheelBaseWidth = LaunchConfiguration('wheel_base_width')
    wheelBaseLength = LaunchConfiguration('wheel_base_length')
    wheelRadius = LaunchConfiguration('wheel_radius')
    invertRight = LaunchConfiguration('invert_right')

    deviceNameLaunchArg = DeclareLaunchArgument(
        'device_name',
        default_value = 'agv0'
    )

    flAxisNumLaunchArg = DeclareLaunchArgument(
        'front_left_axis_number',
        default_value = 0
    )

    frAxisNumLaunchArg = DeclareLaunchArgument(
        'front_right_axis_number',
        default_value = 1
    )

    rlAxisNumLaunchArg = DeclareLaunchArgument(
        'rear_left_axis_number',
        default_value = 2
    )

    rrAxisNumLaunchArg = DeclareLaunchArgument(
        'rear_right_axis_number',
        default_value = 3
    )

    wheelBaseWidthLaunchArg = DeclareLaunchArgument(
        'wheel_base_width',
        default_value = 1
    )

    wheelBaseLengthLaunchArg = DeclareLaunchArgument(
        'wheel_base_length',
        default_value = 1
    )

    wheelRadiusLaunchArg = DeclareLaunchArgument(
        'wheel_radius',
        default_value = 1
    )

    invertRightLaunchArg = DeclareLaunchArgument(
        'invert_right',
        default_value = True
    )

    inverseKinematicNode = Node(
        package="mecanum_kinematics",
        executable="inverse",
        namespace=deviceName,
        name="inverse_kinematics",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"wheel_base_width": wheelBaseWidth},
            {"wheel_base_length": wheelBaseLength},
            {"wheel_radius": wheelRadius},
            {"invert_right": invertRight}
        ],
        remappings=[
            ("kinematics/inverse/output/velocity/front/left", "/" + deviceName + "/axis" + flAxisNum + "/input/velocity"),
            ("kinematics/inverse/output/velocity/front/right", "/" + deviceName + "/axis" + frAxisNum + "/input/velocity"),
            ("kinematics/inverse/output/velocity/rear/left", "/" + deviceName + "/axis" + rlAxisNum + "/input/velocity"),
            ("kinematics/inverse/output/velocity/rear/right", "/" + deviceName + "/axis" + rrAxisNum + "/input/velocity"),
            ("kinematics/inverse/input/velocity", "/cmd_vel")
        ]
    )

    forwardKinematicNode = Node(
        package="mecanum_kinematics",
        executable="forward",
        namespace=deviceName,
        name="forward_kinematics",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"wheel_base_width": wheelBaseWidth},
            {"wheel_base_length": wheelBaseLength},
            {"wheel_radius": wheelRadius},
            {"invert_right": invertRight},
            {"update_rate_ms": 100}
        ],
        remappings=[
            ("kinematics/forward/input/status/front/left", "/" + deviceName + "/axis" + flAxisNum + "/output/status"),
            ("kinematics/forward/input/status/front/right", "/" + deviceName + "/axis" + frAxisNum + "/output/status"),
            ("kinematics/forward/input/status/rear/left", "/" + deviceName + "/axis" + rlAxisNum + "/output/status"),
            ("kinematics/forward/input/status/rear/right", "/" + deviceName + "/axis" + rrAxisNum + "/output/status"),
            ("kinematics/forward/output/odometry", "/odom")
        ]
    )

    return LaunchDescription([
        deviceNameLaunchArg,
        flAxisNumLaunchArg,
        frAxisNumLaunchArg,
        rlAxisNumLaunchArg,
        rrAxisNumLaunchArg,
        wheelBaseWidthLaunchArg,
        wheelBaseLengthLaunchArg,
        wheelRadiusLaunchArg,
        invertRightLaunchArg,
        inverseKinematicNode,
        forwardKinematicNode
    ])