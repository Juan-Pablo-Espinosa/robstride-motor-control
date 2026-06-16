// gr0x_motor_node.cpp — ROS 2 Jazzy
// Two-bus motor node for GR-0X humanoid robot.
// Each leg runs on an independent CAN bus and MotorBus instance.
// Buses can be individually enabled/disabled via joints.yaml.
//
// Subscribe: /joint_commands  (sensor_msgs/JointState)
// Publish:   /joint_states    (sensor_msgs/JointState) at 50Hz

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "motor_bus.hpp"
#include "motor_config.hpp"
#include "socketcan_transport.hpp"

#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <vector>
#include <map>

// One joint definition loaded from YAML
struct JointDef {
    uint8_t     motor_id;
    MotorConfig config;
    float       kp = 100.0f;
    float       kd = 5.0f;
};

// One leg = one CAN bus + one MotorBus + its joints
struct Leg {
    std::string                      name;
    bool                             enabled = false;
    std::unique_ptr<SocketCANTransport> transport;
    std::unique_ptr<MotorBus>           bus;
    std::map<std::string, JointDef>     joints;  // joint_name -> JointDef
};

class GR0XMotorNode : public rclcpp::Node {
public:
    GR0XMotorNode() : Node("gr0x_motor_node") {

        std::string config_path = declare_parameter<std::string>(
            "config", "/home/gr0x-pi/gr0x-motor/config/joints.yaml");

        YAML::Node cfg = YAML::LoadFile(config_path);

        // Load each leg section
        for (const std::string leg_name : {"left_leg", "right_leg"}) {
            if (!cfg[leg_name]) continue;
            auto leg_node = cfg[leg_name];

            auto leg = std::make_unique<Leg>();
            leg->name    = leg_name;
            leg->enabled = leg_node["enabled"].as<bool>(false);

            if (!leg->enabled) {
                RCLCPP_INFO(get_logger(), "%s disabled in config — skipping",
                            leg_name.c_str());
                legs_.push_back(std::move(leg));
                continue;
            }

            std::string can_iface = leg_node["can_interface"].as<std::string>();
            leg->transport = std::make_unique<SocketCANTransport>(can_iface);
            leg->bus       = std::make_unique<MotorBus>(*leg->transport, 200);

            // Load joints for this leg
            for (auto jnode : leg_node["joints"]) {
                JointDef jd;
                jd.motor_id                  = jnode["id"].as<int>();
                jd.config.model              = static_cast<MotorModel>(jnode["model"].as<int>());
                jd.config.joint_name         = jnode["name"].as<std::string>();
                jd.config.min_angle          = jnode["min_angle"].as<float>();
                jd.config.max_angle          = jnode["max_angle"].as<float>();
                jd.config.max_torque         = jnode["max_torque"].as<float>();
                jd.config.max_velocity       = jnode["max_velocity"].as<float>();
                jd.config.max_acceleration   = jnode["max_acceleration"].as<float>(-1.0f);
                jd.config.invert_direction   = jnode["invert"].as<bool>(false);
                jd.kp = jnode["kp"].as<float>();
                jd.kd = jnode["kd"].as<float>();
                jd.config.resolve();
                leg->joints[jd.config.joint_name] = jd;
            }

            // Open transport, add motors, enable, start
            leg->transport->open();
            for (auto& [jname, jd] : leg->joints)
                leg->bus->addMotor(jd.motor_id, jd.config);
            leg->bus->enableAll();
            leg->bus->start();

            RCLCPP_INFO(get_logger(), "%s ready on %s — %zu joints",
                        leg_name.c_str(), can_iface.c_str(), leg->joints.size());

            legs_.push_back(std::move(leg));
        }

        // Subscriber — receives joint commands from RL policy
        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_commands", 1,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    for (auto& leg : legs_) {
                        if (!leg->enabled) continue;
                        auto it = leg->joints.find(msg->name[i]);
                        if (it == leg->joints.end()) continue;
                        MotorTarget t{};
                        t.angle    = (i < msg->position.size()) ? msg->position[i] : 0.0f;
                        t.velocity = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0f;
                        t.torque   = (i < msg->effort.size())   ? msg->effort[i]   : 0.0f;
                        t.kp       = it->second.kp;
                        t.kd       = it->second.kd;
                        leg->bus->setTarget(it->second.motor_id, t);
                    }
                }
            });

        // Publisher — sends joint states to RL policy at 50Hz
        pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() {
                auto msg = sensor_msgs::msg::JointState();
                msg.header.stamp = now();
                for (auto& leg : legs_) {
                    if (!leg->enabled) continue;
                    auto states = leg->bus->getAllStates();
                    for (auto& [jname, jd] : leg->joints) {
                        auto it = states.find(jd.motor_id);
                        if (it == states.end()) continue;
                        msg.name.push_back(jname);
                        msg.position.push_back(it->second.angle);
                        msg.velocity.push_back(it->second.velocity);
                        msg.effort.push_back(it->second.torque);
                    }
                }
                pub_->publish(msg);
            });

            // Reconnect timer — every 2.5 seconds, try to re-enable lost motors
        reconnect_timer_ = create_wall_timer(
            std::chrono::milliseconds(2500),
            [this]() {
                for (auto& leg : legs_) {
                    if (!leg->enabled) continue;
                    for (auto& [jname, jd] : leg->joints) {
                        if (leg->bus->isFaulted(jd.motor_id)) {
                            RCLCPP_WARN(get_logger(),
                                "%s (id %d) faulted — attempting clearFault + re-enable",
                                jname.c_str(), jd.motor_id);
                            leg->bus->clearFault(jd.motor_id);
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            if (leg->bus->enable(jd.motor_id)) {
                                RCLCPP_INFO(get_logger(),
                                    "%s (id %d) re-enabled successfully",
                                    jname.c_str(), jd.motor_id);
                            } else {
                                RCLCPP_WARN(get_logger(),
                                    "%s (id %d) re-enable failed — motor may be off",
                                    jname.c_str(), jd.motor_id);
                            }
                        } else if (leg->bus->isStale(jd.motor_id)) {
                            RCLCPP_WARN(get_logger(),
                                "%s (id %d) stale — attempting re-enable",
                                jname.c_str(), jd.motor_id);
                            if (leg->bus->enable(jd.motor_id)) {
                                RCLCPP_INFO(get_logger(),
                                    "%s (id %d) recovered from stale",
                                    jname.c_str(), jd.motor_id);
                            }
                        }
                    }
                }
            });

        RCLCPP_INFO(get_logger(), "GR-0X motor node ready");
    }

    ~GR0XMotorNode() {
        for (auto& leg : legs_) {
            if (!leg->enabled) continue;
            leg->bus->stop();
            leg->transport->close();
        }
    }

private:
    std::vector<std::unique_ptr<Leg>> legs_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr     pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GR0XMotorNode>());
    rclcpp::shutdown();
    return 0;
}