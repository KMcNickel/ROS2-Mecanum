
from launch import LaunchDescription
from launch_ros.actions import Node

device_name = "agv0"
front_left_axis_number = "0"
front_right_axis_number = "1"
rear_left_axis_number = "2"
rear_right_axis_number = "3"

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
                {"wheel_base_width": 0.27178},
                {"wheel_base_length": 0.181356},
                {"wheel_radius": 0.04064},
                {"invert_right": True}
            ],
            remappings=[
                ("kinematics/inverse/output/velocity/front/left", "/" + device_name + "/axis" + front_left_axis_number + "/input/velocity"),
                ("kinematics/inverse/output/velocity/front/right", "/" + device_name + "/axis" + front_right_axis_number + "/input/velocity"),
                ("kinematics/inverse/output/velocity/rear/left", "/" + device_name + "/axis" + rear_left_axis_number + "/input/velocity"),
                ("kinematics/inverse/output/velocity/rear/right", "/" + device_name + "/axis" + rear_right_axis_number + "/input/velocity"),
                ("kinematics/inverse/input/velocity", "/cmd_vel")
            ]
        )
    ])