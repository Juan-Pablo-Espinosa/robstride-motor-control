// gr0x_motor_node.cpp — ROS 2 Jazzy
// Bridges MotorBus (200Hz CAN loop) to ROS 2 JointState topics.
// Subscribe: /joint_commands  (sensor_msgs/JointState)
// Publish:   /joint_states    (sensor_msgs/JointState) at 50Hz
// The library handles all clamping, ramping, safety, and recovery.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "motor_bus.hpp"
#include "motor_config.hpp"
#include "socketcan_transport.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <map>

struct JointDef {
    uint8_t     motor_id;
    MotorConfig config;
    float       kp = 100.0f;
    float       kd = 5.0f;
};

class GR0XMotorNode : public rclcpp::Node {
public:
    GR0XMotorNode()
    : Node("gr0x_motor_node"),
      transport_("can1"),
      bus_(transport_, 200)
    {
        // Load joint map from YAML param
        std::string config_path = declare_parameter<std::string>(
            "config", "/home/gr0x-pi/gr0x-motor/config/joints.yaml");

        loadJoints(config_path);

        transport_.open();
        for (auto& [name, jd] : joints_)
            bus_.addMotor(jd.motor_id, jd.config);

        bus_.enableAll();
        bus_.start();

        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_commands", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    auto it = joints_.find(msg->name[i]);
                    if (it == joints_.end()) continue;
                    MotorTarget t{};
                    t.angle    = (i < msg->position.size()) ? msg->position[i] : 0.0f;
                    t.velocity = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0f;
                    t.torque   = (i < msg->effort.size())   ? msg->effort[i]   : 0.0f;
                    t.kp       = it->second.kp;
                    t.kd       = it->second.kd;
                    bus_.setTarget(it->second.motor_id, t);
                }
            });

        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),   // 50Hz
            [this]() {
                auto states = bus_.getAllStates();
                auto msg = sensor_msgs::msg::JointState();
                msg.header.stamp = now();
                for (auto& [name, jd] : joints_) {
                    auto it = states.find(jd.motor_id);
                    if (it == states.end()) continue;
                    msg.name.push_back(name);
                    msg.position.push_back(it->second.angle);
                    msg.velocity.push_back(it->second.velocity);
                    msg.effort.push_back(it->second.torque);
                }
                pub_->publish(msg);
            });

        pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        RCLCPP_INFO(get_logger(), "GR-0X motor node ready — %zu joints", joints_.size());
    }

    ~GR0XMotorNode() {
        bus_.stop();
        transport_.close();
    }

private:
    void loadJoints(const std::string& path) {
        YAML::Node cfg = YAML::LoadFile(path);
        for (auto node : cfg["joints"]) {
            JointDef jd;
            jd.motor_id              = node["id"].as<int>();
            jd.config.model          = static_cast<MotorModel>(node["model"].as<int>());
            jd.config.joint_name     = node["name"].as<std::string>();
            jd.config.min_angle      = node["min_angle"].as<float>();
            jd.config.max_angle      = node["max_angle"].as<float>();
            jd.config.max_torque     = node["max_torque"].as<float>();
            jd.config.max_velocity   = node["max_velocity"].as<float>();
            jd.config.max_acceleration = node["max_acceleration"].as<float>(-1.0f);
            jd.config.invert_direction = node["invert"].as<bool>(false);
            jd.kp = node["kp"].as<float>();
            jd.kd = node["kd"].as<float>();
            jd.config.resolve();
            joints_[jd.config.joint_name] = jd;
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu joints from %s", joints_.size(), path.c_str());
    }

    SocketCANTransport transport_;
    MotorBus            bus_;
    std::map<std::string, JointDef> joints_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr     pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GR0XMotorNode>());
    rclcpp::shutdown();
    return 0;
}
