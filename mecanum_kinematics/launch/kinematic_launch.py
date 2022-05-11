
from launch import LaunchDescription
from launch_ros.actions import Node

device_name = "agv0"
front_left_axis_number = "0"
front_right_axis_number = "1"
rear_left_axis_number = "2"
rear_right_axis_number = "3"

wheelBaseWidth = 0.27178
wheelBaseLength = 0.181356
wheelRadius = 0.04064
invertRight = True

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mecanum_kinematics",
            executable="inverse",
            namespace=device_name,
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
                ("kinematics/inverse/output/velocity/front/left", "/" + device_name + "/axis" + front_left_axis_number + "/input/velocity"),
                ("kinematics/inverse/output/velocity/front/right", "/" + device_name + "/axis" + front_right_axis_number + "/input/velocity"),
                ("kinematics/inverse/output/velocity/rear/left", "/" + device_name + "/axis" + rear_left_axis_number + "/input/velocity"),
                ("kinematics/inverse/output/velocity/rear/right", "/" + device_name + "/axis" + rear_right_axis_number + "/input/velocity"),
                ("kinematics/inverse/input/velocity", "/cmd_vel")
            ]
        ),
        Node(
            package="mecanum_kinematics",
            executable="forward",
            namespace=device_name,
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
                ("kinematics/forward/input/status/front/left", "/" + device_name + "/axis" + front_left_axis_number + "/output/status"),
                ("kinematics/forward/input/status/front/right", "/" + device_name + "/axis" + front_right_axis_number + "/output/status"),
                ("kinematics/forward/input/status/rear/left", "/" + device_name + "/axis" + rear_left_axis_number + "/output/status"),
                ("kinematics/forward/input/status/rear/right", "/" + device_name + "/axis" + rear_right_axis_number + "/output/status"),
                ("kinematics/forward/output/odometry", "/odom")
            ]
        )
    ])