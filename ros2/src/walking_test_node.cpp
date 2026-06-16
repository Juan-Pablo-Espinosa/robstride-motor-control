// walking_test_node.cpp — GR-0X Left Leg Walking Demo
// Simulates Jetson RL policy output for testing without the Jetson.
// Moves all 4 left leg joints in a coordinated slow walking gait.
//
// Sequence:
//   1. Move to idle position and hold 3 seconds
//   2. Walk for 10 cycles (~20 seconds)
//   3. Return to idle and hold
//   4. Shutdown
//
// Publish: /joint_commands (sensor_msgs/JointState) at 50Hz

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <vector>
#include <string>

class WalkingTestNode : public rclcpp::Node {
public:
    WalkingTestNode() : Node("walking_test_node"), cycle_(0.0), phase_(IDLE_START) {

        pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 1);

        // 50Hz timer — same rate as motor node publishes states
        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() { update(); });

        start_time_ = now();
        RCLCPP_INFO(get_logger(), "Walking test node started — moving to idle position...");
    }

private:
    // --- Gait parameters ---
    // All values in radians, tuned for a hanging leg (no ground contact)

    // Idle (neutral standing) angles from joints.yaml
    const float IDLE_HIP_PITCH  =  0.34f;
    const float IDLE_HIP_ROLL   =  0.0f;
    const float IDLE_KNEE_PITCH =  0.67f;
    const float IDLE_ANKLE_PITCH=  0.35f;

    // Walking amplitude per joint — conservative for first test
    const float AMP_HIP_PITCH   =  0.25f;  // swing forward/back
    const float AMP_HIP_ROLL    =  0.08f;  // lateral sway
    const float AMP_KNEE_PITCH  =  0.20f;  // knee flex
    const float AMP_ANKLE_PITCH =  0.10f;  // ankle push

    // Gait timing
    const float GAIT_PERIOD_S   =  2.0f;   // seconds per full stride
    const float IDLE_HOLD_S     =  3.0f;   // seconds to hold idle before walking
    const int   WALK_CYCLES     =  10;     // number of stride cycles
    const float RETURN_HOLD_S   =  3.0f;   // seconds to hold idle at end

    enum Phase { IDLE_START, WALKING, IDLE_END, DONE };

    void update() {
        auto elapsed = (now() - start_time_).seconds();
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();
        msg.name = {"left_hip_pitch", "left_hip_roll",
                    "left_knee_pitch", "left_ankle_pitch"};
        msg.velocity = {0.0, 0.0, 0.0, 0.0};
        msg.effort   = {0.0, 0.0, 0.0, 0.0};
        msg.position.resize(4);

        switch (phase_) {

        case IDLE_START:
            // Hold idle position
            msg.position = {IDLE_HIP_PITCH, IDLE_HIP_ROLL,
                            IDLE_KNEE_PITCH, IDLE_ANKLE_PITCH};
            if (elapsed > IDLE_HOLD_S) {
                phase_ = WALKING;
                walk_start_time_ = now();
                RCLCPP_INFO(get_logger(), "Starting walking gait...");
            }
            break;

        case WALKING: {
            float t = (now() - walk_start_time_).seconds();
            float total_walk_time = GAIT_PERIOD_S * WALK_CYCLES;

            // Phase angle for gait — one full cycle = 2*pi
            float phi = 2.0f * M_PI * t / GAIT_PERIOD_S;

            // Hip pitch — main forward/back swing
            // sin wave: forward on positive half, back on negative
            float hip_pitch = IDLE_HIP_PITCH + AMP_HIP_PITCH * std::sin(phi);

            // Hip roll — lateral sway, 90 degrees offset from hip pitch
            float hip_roll = IDLE_HIP_ROLL + AMP_HIP_ROLL * std::sin(phi + M_PI / 2.0f);

            // Knee pitch — flex on swing phase (when hip moves forward)
            // Always positive (knee only bends one way), extra flex during swing
            float knee_pitch = IDLE_KNEE_PITCH
                + AMP_KNEE_PITCH * (1.0f - std::cos(phi)) / 2.0f;

            // Ankle pitch — push off at end of stance, dorsiflexion during swing
            float ankle_pitch = IDLE_ANKLE_PITCH
                - AMP_ANKLE_PITCH * std::sin(phi);

            msg.position = {hip_pitch, hip_roll, knee_pitch, ankle_pitch};

            // Log progress every stride
            int current_cycle = static_cast<int>(t / GAIT_PERIOD_S);
            if (current_cycle > last_logged_cycle_) {
                last_logged_cycle_ = current_cycle;
                RCLCPP_INFO(get_logger(), "Stride %d / %d", current_cycle + 1, WALK_CYCLES);
            }

            if (t > total_walk_time) {
                phase_ = IDLE_END;
                idle_end_start_ = now();
                RCLCPP_INFO(get_logger(), "Walking complete — returning to idle...");
            }
            break;
        }

        case IDLE_END:
            // Return to idle and hold
            msg.position = {IDLE_HIP_PITCH, IDLE_HIP_ROLL,
                            IDLE_KNEE_PITCH, IDLE_ANKLE_PITCH};
            if ((now() - idle_end_start_).seconds() > RETURN_HOLD_S) {
                phase_ = DONE;
                RCLCPP_INFO(get_logger(), "Test complete. Ctrl+C to exit.");
            }
            break;

        case DONE:
            // Keep holding idle — don't stop publishing or motors go stale
            msg.position = {IDLE_HIP_PITCH, IDLE_HIP_ROLL,
                            IDLE_KNEE_PITCH, IDLE_ANKLE_PITCH};
            break;
        }

        pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    rclcpp::Time walk_start_time_;
    rclcpp::Time idle_end_start_;
    float cycle_;
    Phase phase_;
    int last_logged_cycle_ = -1;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkingTestNode>());
    rclcpp::shutdown();
    return 0;
}
