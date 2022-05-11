#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <thread>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "odrive_interface/msg/axis_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#define RADIANS_PER_CIRCLE 6.28319

using std::placeholders::_1;

class forwardKinematicCalculator : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit forwardKinematicCalculator()
        : LifecycleNode("forwardKinematicCalculator")
        {
            RCLCPP_INFO(rclcpp::get_logger("Constructor"), "Node Created");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuring...");

            this->declare_parameter<std::double_t>("wheel_base_width", 1);
            this->declare_parameter<std::double_t>("wheel_base_length", 1);
            this->declare_parameter<std::double_t>("wheel_radius", 1);
            this->declare_parameter<bool>("invert_right", true);
            this->declare_parameter<bool>("update_rate_ms", 100);

            this->get_parameter("wheel_base_width", wheelBaseWidth);
            this->get_parameter("wheel_base_length", wheelBaseLength);
            this->get_parameter("wheel_radius", wheelRadius);
            this->get_parameter("invert_right", invertRight);
            this->get_parameter("update_rate_ms", updateRateMs);

            createInterfaces();

            odomUpdateTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(updateRateMs),
                    std::bind(&forwardKinematicCalculator::calculateOdometry, this));

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuration completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activating...");

            odometryPublisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivating...");

            odometryPublisher->on_deactivate();

            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_cleanup"), "Cleaning Up...");

            resetVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("on_cleanup"), "Cleanup completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_shutdown"), "Shutting Down...");

            resetVariables();

            RCLCPP_INFO(rclcpp::get_logger("on_shutdown"), "Shut down completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        ~forwardKinematicCalculator()
        {
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutting Down");

            resetVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutdown complete. Goodbye!");
        }

    private:
        void createInterfaces()
        {
            odometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>("kinematics/forward/output/odometry", 10);

            for(int i = 0; i < 4; i++)
            {
                auto callback =
                [this, i](const odrive_interface::msg::AxisStatus & message) 
                {
                    wheelVelocities[i] = message.current_velocity;
                    if((i == 1 || i == 3) && invertRight)
                        wheelVelocities[i] *= -1;
                };

                AxisStatus[i] = this->create_subscription<odrive_interface::msg::AxisStatus>(axisSubscriptionTopics[i], 50, callback);
            }
        }

        void resetVariables()
        {
            odometryPublisher.reset();

            AxisStatus[0].reset();
            AxisStatus[1].reset();
            AxisStatus[2].reset();
            AxisStatus[3].reset();

            odomUpdateTimer.reset();

            wheelBaseWidth = 1;
            wheelBaseLength = 1;
            wheelRadius = 1;
            invertRight = true;
            updateRateMs = 100;

            wheelVelocities[0] = 0;
            wheelVelocities[1] = 0;
            wheelVelocities[2] = 0;
            wheelVelocities[3] = 0;
        }

        void calculateOdometry()
        {
            nav_msgs::msg::Odometry odom;

            if(this->get_current_state().id() != 3)     //3 is active
                return;

            odom.header.stamp = this->now();
            odom.header.frame_id = "odom";

            odom.pose.pose.position.x = (wheelVelocities[0] + wheelVelocities[1] + wheelVelocities[2] + wheelVelocities[3]) *
                    (wheelRadius / 4) * RADIANS_PER_CIRCLE * (updateRateMs / 1000);
            odom.pose.pose.position.y = (-wheelVelocities[0] + wheelVelocities[1] + wheelVelocities[2] - wheelVelocities[3]) *
                    (wheelRadius / 4) * RADIANS_PER_CIRCLE * (updateRateMs / 1000);
            odom.pose.pose.position.z = 0;

            double theta = (-wheelVelocities[0] + wheelVelocities[1] - wheelVelocities[2] + wheelVelocities[3]) *
                    (wheelRadius / (4 * ((wheelBaseWidth / 2) + (wheelBaseLength / 2)))) * RADIANS_PER_CIRCLE * (updateRateMs / 1000);
            odom.pose.pose.orientation.set__z(theta);
        }

        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;

        rclcpp::Subscription<odrive_interface::msg::AxisStatus>::SharedPtr AxisStatus[4];
        std::string axisSubscriptionTopics[4] = 
        {
            "kinematics/forward/input/status/front/left",
            "kinematics/forward/input/status/front/right",
            "kinematics/forward/input/status/rear/left",
            "kinematics/forward/input/status/rear/right"
        };

        rclcpp::TimerBase::SharedPtr odomUpdateTimer;

        double_t wheelBaseWidth;
        double_t wheelBaseLength;
        double_t wheelRadius;
        bool invertRight;
        int32_t updateRateMs;

        float wheelVelocities[4];
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<forwardKinematicCalculator> lc_node =
        std::make_shared<forwardKinematicCalculator>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}