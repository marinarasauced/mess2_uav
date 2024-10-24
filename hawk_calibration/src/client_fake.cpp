
#include <cmath>
#include <cstdint>
#include <functional>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "mess2_msgs/action/uav_calibrate.hpp"
#include "mess2_plugins/rotation.hpp"
#include "mess2_plugins/utils.hpp"

using TransformStamped = geometry_msgs::msg::TransformStamped;
using Action = mess2_msgs::action::UAVCalibrate;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

std::string get_executable_directory() {
    char buf[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len != -1) {
        buf[len] = '\0';
        std::string path(buf);
        size_t pos = path.find_last_of('/');
        return (pos != std::string::npos) ? path.substr(0, pos) : ".";
    }
    return ".";
}

void create_directories(const std::string &path) {
    std::string cmd = "mkdir -p " + path;
    system(cmd.c_str()); 
}

using namespace mess2_plugins;
namespace mess2_nodes
{
class UAVCalibrationClientFake : public rclcpp::Node
{
public:
    explicit UAVCalibrationClientFake(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("uav_calibration_client", options)
    {
        this->declare_parameter("num_measurements", 1000);
        this->declare_parameter("agent_name", "agent");
        this->get_parameter("agent_name", agent_name_);
        _vicon_topic = get_vicon_topic(agent_name_);

        this->_transform_publisher = this->create_publisher<TransformStamped>(_vicon_topic, 10);

        this->_transform_timer = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&UAVCalibrationClientFake::_publish_random_transform, this)
        );

        this->_calibration_client = rclcpp_action::create_client<Action>(
            this,
            "calibrate_uav"
        );

        this->_calibration_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&UAVCalibrationClientFake::send_goal, this)
        );
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->_calibration_timer->cancel();

        if (!this->_calibration_client->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "action server not available after waiting");
        rclcpp::shutdown();
        }

        auto goal_msg = Action::Goal();
        this->get_parameter("num_measurements", goal_msg.num_measurements);

        RCLCPP_INFO(this->get_logger(), "sending goal");

        auto send_goal_options = rclcpp_action::Client<Action>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
        std::bind(&UAVCalibrationClientFake::_goal_response_callback, this, _1);
        
        send_goal_options.feedback_callback =
        std::bind(&UAVCalibrationClientFake::_feedback_callback, this, _1, _2);
        
        send_goal_options.result_callback =
        std::bind(&UAVCalibrationClientFake::_result_callback, this, _1);
        
        this->_calibration_client->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Action>::SharedPtr _calibration_client;
    rclcpp::TimerBase::SharedPtr _calibration_timer;

    rclcpp::Publisher<TransformStamped>::SharedPtr _transform_publisher;
    rclcpp::TimerBase::SharedPtr _transform_timer;
    std::string _vicon_topic;
    std::string agent_name_;

    void _goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "goal accepted by server, waiting for result");
        }
    }

    void _feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Action::Feedback> feedback)
    {
        auto progress = feedback->progress;
        RCLCPP_INFO(this->get_logger(), "progress: %.2f%%", progress);
    }

    void _result_callback(const GoalHandle::WrappedResult &result) 
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "unknown result code");
                return;
        }

        YAML::Node yaml_node;
        yaml_node["x"] = result.result->quat_diff.x;
        yaml_node["y"] = result.result->quat_diff.y;
        yaml_node["z"] = result.result->quat_diff.z;
        yaml_node["w"] = result.result->quat_diff.w;

        std::string exe_dir = get_executable_directory();
        std::string output_dir = exe_dir + "/../../../agents/" + agent_name_;
        create_directories(output_dir);

        std::string write_path = output_dir + "/calibration.yaml";
        std::ofstream fout(write_path);
        fout << yaml_node;
        fout.close();

        std::stringstream ss;
        ss << "quat_diff:\n";
        ss << "\tx: " << result.result->quat_diff.x << "\n";
        ss << "\ty: " << result.result->quat_diff.y << "\n";
        ss << "\tz: " << result.result->quat_diff.z << "\n";
        ss << "\tw: " << result.result->quat_diff.w << "\n";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }

    void _publish_random_transform()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-1.0, 1.0);

        auto transform_msg = TransformStamped();
        transform_msg.header.stamp = this->get_clock()->now();
        transform_msg.header.frame_id = "world";
        transform_msg.child_frame_id = "random_frame";

        transform_msg.transform.translation.x = dis(gen);
        transform_msg.transform.translation.y = dis(gen);
        transform_msg.transform.translation.z = dis(gen);

        transform_msg.transform.rotation.x = dis(gen);
        transform_msg.transform.rotation.y = dis(gen);
        transform_msg.transform.rotation.z = dis(gen);
        transform_msg.transform.rotation.w = dis(gen);

        transform_msg.transform.rotation = normalize_quat(transform_msg.transform.rotation);

        _transform_publisher->publish(transform_msg);
    }
};  
}

RCLCPP_COMPONENTS_REGISTER_NODE(mess2_nodes::UAVCalibrationClientFake)
