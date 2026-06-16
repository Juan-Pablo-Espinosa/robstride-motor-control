// walking_test_node.cpp — GR-0X Left Leg Walking Demo
// Plays back keyframe gait trajectory, interpolating smoothly between poses.
// Simulates Jetson RL policy output for hardware validation without the Jetson.
//
// Sequence:
//   1. Smooth transition from current idle to first keyframe (3 seconds)
//   2. Loop through all keyframes 5 times
//   3. Smooth return to idle position
//   4. Hold idle — Ctrl+C to exit
//
// Publish: /joint_commands (sensor_msgs/JointState) at 50Hz

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <vector>
#include <array>
#include <string>

using Pose = std::array<float, 4>;  // [hip_pitch, hip_roll, knee_pitch, ankle_pitch]

// Smooth interpolation — ease in/out using cosine
float smoothstep(float t) {
    t = std::max(0.0f, std::min(1.0f, t));
    return t * t * (3.0f - 2.0f * t);
}

float lerp(float a, float b, float t) {
    return a + (b - a) * smoothstep(t);
}

Pose lerpPose(const Pose& a, const Pose& b, float t) {
    return {lerp(a[0], b[0], t),
            lerp(a[1], b[1], t),
            lerp(a[2], b[2], t),
            lerp(a[3], b[3], t)};
}

class WalkingTestNode : public rclcpp::Node {
public:
    WalkingTestNode() : Node("walking_test_node") {

        pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 1);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),  // 50Hz
            [this]() { update(); });

        start_time_ = now();
        phase_start_ = now();
        RCLCPP_INFO(get_logger(), "Walking test — transitioning to start pose...");
    }

private:
    // --- Joint order: hip_pitch, hip_roll, knee_pitch, ankle_pitch ---

    // Idle position (from joints.yaml)
    const Pose IDLE = {0.34f, 0.0f, 0.67f, 0.35f};

    // Gait keyframes — verified against joint limits
    const std::vector<Pose> KEYFRAMES = {
        { 0.60f,  0.00f,  1.57f,  0.00f},  // 1: heel strike
        { 0.78f,  0.10f,  0.70f, -0.15f},  // 2: early stance
        { 0.60f,  0.20f,  0.50f, -0.25f},  // 3: mid stance
        {-0.20f,  0.10f,  0.20f,  0.25f},  // 4: late stance
        {-0.35f,  0.00f,  0.20f,  0.20f},  // 5: toe off (knee clamped to 0.2)
        {-0.45f, -0.10f,  0.78f,  0.60f},  // 6: early swing
        {-0.65f, -0.20f,  1.00f, -0.45f},  // 7: mid swing
        { 0.30f, -0.10f,  1.75f, -0.20f},  // 8: late swing
    };

    // Timing
    const float TRANSITION_S   = 3.0f;   // seconds to move from idle to first keyframe
    const float TIME_PER_POSE_S= 0.5f;   // seconds between keyframes (slow and smooth)
    const int   LOOP_COUNT     = 5;      // how many times to loop keyframes
    const float RETURN_S       = 3.0f;   // seconds to return to idle

    enum Phase { TRANSITION, WALKING, RETURNING, DONE };
    Phase phase_ = TRANSITION;

    void update() {
        float t = (now() - phase_start_).seconds();
        Pose cmd;

        switch (phase_) {

        case TRANSITION: {
            // Smooth move from idle to first keyframe
            float alpha = t / TRANSITION_S;
            cmd = lerpPose(IDLE, KEYFRAMES[0], alpha);
            if (alpha >= 1.0f) {
                phase_ = WALKING;
                phase_start_ = now();
                walk_keyframe_ = 0;
                walk_loop_ = 0;
                RCLCPP_INFO(get_logger(), "Starting gait — loop 1/%d", LOOP_COUNT);
            }
            break;
        }

        case WALKING: {
            // Interpolate between consecutive keyframes
            int n = KEYFRAMES.size();
            int from_idx = walk_keyframe_ % n;
            int to_idx   = (walk_keyframe_ + 1) % n;

            float alpha = t / TIME_PER_POSE_S;
            cmd = lerpPose(KEYFRAMES[from_idx], KEYFRAMES[to_idx], alpha);

            if (alpha >= 1.0f) {
                walk_keyframe_++;
                phase_start_ = now();

                // Completed one full loop
                if (walk_keyframe_ % n == 0) {
                    walk_loop_++;
                    if (walk_loop_ >= LOOP_COUNT) {
                        phase_ = RETURNING;
                        phase_start_ = now();
                        return_from_ = KEYFRAMES[0];
                        RCLCPP_INFO(get_logger(), "Gait complete — returning to idle...");
                    } else {
                        RCLCPP_INFO(get_logger(), "Loop %d/%d", walk_loop_ + 1, LOOP_COUNT);
                    }
                }
            }
            break;
        }

        case RETURNING: {
            // Smooth return to idle
            float alpha = t / RETURN_S;
            cmd = lerpPose(return_from_, IDLE, alpha);
            if (alpha >= 1.0f) {
                phase_ = DONE;
                RCLCPP_INFO(get_logger(), "Test complete. Ctrl+C to exit.");
            }
            break;
        }

        case DONE:
            cmd = IDLE;
            break;
        }

        // Publish
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();
        msg.name     = {"left_hip_pitch", "left_hip_roll",
                        "left_knee_pitch", "left_ankle_pitch"};
        msg.position = {cmd[0], cmd[1], cmd[2], cmd[3]};
        msg.velocity = {0.0, 0.0, 0.0, 0.0};
        msg.effort   = {0.0, 0.0, 0.0, 0.0};
        pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    rclcpp::Time phase_start_;

    int walk_keyframe_ = 0;
    int walk_loop_     = 0;
    Pose return_from_  = {};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkingTestNode>());
    rclcpp::shutdown();
    return 0;
}