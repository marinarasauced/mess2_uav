
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol_local.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>

#include "mess2_plugins/utils.hpp"

using PoseStamped = geometry_msgs::msg::PoseStamped;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using CommandBool = mavros_msgs::srv::CommandBool;
using CommandTOLLocal = mavros_msgs::srv::CommandTOLLocal;
using SetMode = mavros_msgs::srv::SetMode;

using namespace mess2_plugins;
namespace mess2_nodes
{
class UAVOffboardNode : public rclcpp::Node
{
public:
    explicit UAVOffboardNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("uav_offboard", options)
    {
        using namespace std::placeholders;

        this->declare_parameter("agent_name", "agent");
        this->get_parameter("agent_name", agent_name_);
        _vicon_topic = get_vicon_topic(agent_name_);
        _vision_pose_topic = "vision_pose/pose";

        _vision_pose_publisher = this->create_publisher<PoseStamped>(
            _vision_pose_topic,
            10
        );

        _vicon_subscription = this->create_subscription<TransformStamped>(
            _vicon_topic,
            10,
            std::bind(&UAVOffboardNode::_vicon_callback, this, _1)
        );

        _arming_client = this->create_client<CommandBool>("cmd/arming");
        _set_mode_client = this->create_client<SetMode>("set_mode");
        _takeoff_local_client = this->create_client<CommandTOLLocal>("cmd/takeoff_local");
        _land_local_client = this->create_client<CommandTOLLocal>("cmd/land_local");

        // set mode to land since after communication loss will default to previous
        (void) set_mode("LAND");
        (void) set_mode("OFFBOARD");
        (void) set_arming(1);
    }

    void set_arming(bool arm)
    {
        if (!_arming_client->wait_for_service(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "service not available.");
            return;
        }

        auto request = std::make_shared<CommandBool::Request>();
        request->value = arm;

        auto result_future = _arming_client->async_send_request(request);

        try
        {
            auto result = result_future.get();
            if (result->success)
            {
                RCLCPP_INFO(this->get_logger(), "arming command succeeded.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "arming command failed.");
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(this->get_logger(), "service call failed: %s", e.what());
        }
    }

    void set_mode(const std::string & mode)
    {
        if (!_set_mode_client->wait_for_service(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "service not available.");
            return;
        }

        auto request = std::make_shared<SetMode::Request>();
        request->custom_mode = mode;

        auto result_future = _set_mode_client->async_send_request(request);

        try
        {
            auto result = rclcpp::spin_until_future_complete(shared_from_this(), result_future);
            if (result == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result_future.get();
                if (response->mode_sent)
                {
                    RCLCPP_INFO(this->get_logger(), "mode set to '%s'.", mode.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "failed to set mode to '%s'.", mode.c_str());
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "service call failed.");
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(this->get_logger(), "service call failed: %s", e.what());
        }
    }

private:
    std::string agent_name_;
    std::string _vicon_topic;
    std::string _vision_pose_topic;

    // TransformStamped global_;

    rclcpp::Publisher<PoseStamped>::SharedPtr _vision_pose_publisher;
    rclcpp::Subscription<TransformStamped>::SharedPtr _vicon_subscription;
    rclcpp::Client<CommandBool>::SharedPtr _arming_client;
    rclcpp::Client<SetMode>::SharedPtr _set_mode_client;
    rclcpp::Client<CommandTOLLocal>::SharedPtr _takeoff_local_client;
    rclcpp::Client<CommandTOLLocal>::SharedPtr _land_local_client;

    void _vicon_callback(const TransformStamped::SharedPtr msg)
    {
        // global_.header = msg->header;
        // global_.transform = msg->transform;

        PoseStamped msg_;
        msg_.header = msg->header;
        msg_.pose.position.x = msg->transform.translation.x;
        msg_.pose.position.y = msg->transform.translation.y;
        msg_.pose.position.z = -msg->transform.translation.z;  // invert z bc mavros uses FRD
        msg_.pose.orientation = msg->transform.rotation;
        _vision_pose_publisher->publish(msg_);
    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(mess2_nodes::UAVOffboardNode)
