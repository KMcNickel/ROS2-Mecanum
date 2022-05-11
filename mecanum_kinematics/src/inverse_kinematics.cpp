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

#include "odrive_interface/msg/input_velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define RADIANS_PER_CIRCLE 6.28319

using std::placeholders::_1;

class inverseKinematicCalculator : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit inverseKinematicCalculator()
        : LifecycleNode("inverseKinematicCalculator")
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

            this->get_parameter("wheel_base_width", wheelBaseWidth);
            this->get_parameter("wheel_base_length", wheelBaseLength);
            this->get_parameter("wheel_radius", wheelRadius);
            this->get_parameter("invert_right", invertRight);

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Wheel Base Width: %f", wheelBaseWidth);
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Wheel Base Length: %f", wheelBaseLength);
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Wheel Radius: %f", wheelRadius);
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Invert Right Motors: %d", invertRight);

            createInterfaces();

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuration completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activating...");

            wheelFrontLeftVelocityPublisher->on_activate();
            wheelFrontRightVelocityPublisher->on_activate();
            wheelRearLeftVelocityPublisher->on_activate();
            wheelRearRightVelocityPublisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivating...");

            wheelFrontLeftVelocityPublisher->on_deactivate();
            wheelFrontRightVelocityPublisher->on_deactivate();
            wheelRearLeftVelocityPublisher->on_deactivate();
            wheelRearRightVelocityPublisher->on_deactivate();

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

        ~inverseKinematicCalculator()
        {
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutting Down");

            resetVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutdown complete. Goodbye!");
        }

    private:
        void createInterfaces()
        {
            wheelFrontLeftVelocityPublisher = this->create_publisher<odrive_interface::msg::InputVelocity>("kinematics/inverse/output/velocity/front/left", 10);
            wheelFrontRightVelocityPublisher = this->create_publisher<odrive_interface::msg::InputVelocity>("kinematics/inverse/output/velocity/front/right", 10);
            wheelRearLeftVelocityPublisher = this->create_publisher<odrive_interface::msg::InputVelocity>("kinematics/inverse/output/velocity/rear/left", 10);
            wheelRearRightVelocityPublisher = this->create_publisher<odrive_interface::msg::InputVelocity>("kinematics/inverse/output/velocity/rear/right", 10);

            incomingTwistMessageSubscription = this->create_subscription<geometry_msgs::msg::Twist>(
                "kinematics/inverse/input/velocity", 50, std::bind(&inverseKinematicCalculator::calculateTargetVelocities, this, _1));
        }

        void resetVariables()
        {
            wheelFrontLeftVelocityPublisher.reset();
            wheelFrontRightVelocityPublisher.reset();
            wheelRearLeftVelocityPublisher.reset();
            wheelRearRightVelocityPublisher.reset();

            incomingTwistMessageSubscription.reset();

            wheelBaseWidth = 1;
            wheelBaseLength = 1;
            wheelRadius = 1;
            invertRight = true;
        }

        void calculateTargetVelocities(const geometry_msgs::msg::Twist & message)
        {
            odrive_interface::msg::InputVelocity velocities[4];
            double reciprocalRadius = 1 / wheelRadius;
            double wheelSeperation = (wheelBaseWidth / 2) + (wheelBaseLength / 2);

            RCLCPP_DEBUG(rclcpp::get_logger("calculateTargetVelocities"),
                    "Incoming twist message:\n\tLinear - X: %f, Y: %f, Z: %f\n\tAngular - X: %f, Y: %f, Z: %f",
                    message.linear.x, message.linear.y, message.linear.z, message.angular.x, message.angular.y, message.angular.z);

            //Front Left
            velocities[0].input_velocity = (reciprocalRadius * (message.linear.x - message.linear.y - wheelSeperation * message.angular.z)) / RADIANS_PER_CIRCLE;
            //Front Right
            velocities[1].input_velocity = (reciprocalRadius * (message.linear.x + message.linear.y + wheelSeperation * message.angular.z)) / RADIANS_PER_CIRCLE;
            if(invertRight) velocities[1].input_velocity *= -1;
            //Rear Left
            velocities[2].input_velocity = (reciprocalRadius * (message.linear.x + message.linear.y - wheelSeperation * message.angular.z)) / RADIANS_PER_CIRCLE;
            //Rear Right
            velocities[3].input_velocity = (reciprocalRadius * (message.linear.x - message.linear.y + wheelSeperation * message.angular.z)) / RADIANS_PER_CIRCLE;
            if(invertRight) velocities[3].input_velocity *= -1;

            RCLCPP_DEBUG(rclcpp::get_logger("calculateTargetVelocities"), "Command Velocities: FL: %f, FR: %f, RL: %f, RR: %f",
                    velocities[0], velocities[1], velocities[2], velocities[3]);

            wheelFrontLeftVelocityPublisher->publish(velocities[0]);
            wheelFrontRightVelocityPublisher->publish(velocities[1]);
            wheelRearLeftVelocityPublisher->publish(velocities[2]);
            wheelRearRightVelocityPublisher->publish(velocities[3]);
        }

        rclcpp_lifecycle::LifecyclePublisher<odrive_interface::msg::InputVelocity>::SharedPtr wheelFrontLeftVelocityPublisher;
        rclcpp_lifecycle::LifecyclePublisher<odrive_interface::msg::InputVelocity>::SharedPtr wheelFrontRightVelocityPublisher;
        rclcpp_lifecycle::LifecyclePublisher<odrive_interface::msg::InputVelocity>::SharedPtr wheelRearLeftVelocityPublisher;
        rclcpp_lifecycle::LifecyclePublisher<odrive_interface::msg::InputVelocity>::SharedPtr wheelRearRightVelocityPublisher;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr incomingTwistMessageSubscription;

        double_t wheelBaseWidth;
        double_t wheelBaseLength;
        double_t wheelRadius;
        bool invertRight;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<inverseKinematicCalculator> lc_node =
        std::make_shared<inverseKinematicCalculator>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}