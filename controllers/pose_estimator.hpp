#pragma once
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/msg/low_state.hpp>

// #include <unitree/idl/go2/LowState_.hpp>
#include <iostream>
#include <array>
#include <cmath>

// Quaternion to rotation matrix
inline void quaternionToRotationMatrix(const std::array<float, 4>& q, float R[3][3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2 * (y * y + z * z);
    R[0][1] = 2 * (x * y - z * w);
    R[0][2] = 2 * (x * z + y * w);
    R[1][0] = 2 * (x * y + z * w);
    R[1][1] = 1 - 2 * (x * x + z * z);
    R[1][2] = 2 * (y * z - x * w);
    R[2][0] = 2 * (x * z - y * w);
    R[2][1] = 2 * (y * z + x * w);
    R[2][2] = 1 - 2 * (x * x + y * y);
}

// Rotate vector from body to world frame
inline std::array<float, 3> bodyToWorld(const std::array<float, 3>& v, const std::array<float, 4>& q) {
    float R[3][3];
    quaternionToRotationMatrix(q, R);
    std::array<float, 3> result;
    for (int i = 0; i < 3; ++i) {
        result[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
    }
    return result;
}

class FloatingBasePoseEstimator {
public:
    FloatingBasePoseEstimator(float acc_alpha = 0.2f, float vel_alpha = 0.2f)
        : sub_("low_state"),
          position_{0.0f, 0.0f, 0.0f},
          velocity_{0.0f, 0.0f, 0.0f},
          filtered_acc_{0.0f, 0.0f, 0.0f},
          filtered_vel_{0.0f, 0.0f, 0.0f},
          last_time_(-1.0),
          acc_alpha_(acc_alpha),
          vel_alpha_(vel_alpha)
    {}

    // Get current orientation quaternion [w, x, y, z]
    std::array<float, 4> getOrientation() {
        auto state = sub_.read();
        if (state == nullptr) {
            std::cout << "state not available" << std::endl;
            return {0.0f, 0.0f, 0.0f, 1.0f};
        }
        return state->imu.quaternion;
    }

    // Get current angular velocity [wx, wy, wz]
    std::array<float, 3> getAngularVelocity() {
        auto state = sub_.read();
        if (state == nullptr) {
            std::cout << "state not available" << std::endl;
            return {0.0f, 0.0f, 0.0f};
        }
        return state->imu.gyroscope;
    }

    // Get current linear velocity [vx, vy, vz]
    std::array<float, 3> getLinearVelocity() {
        return filtered_vel_;
    }

    // Get current estimated position [x, y, z]
    std::array<float, 3> getPosition() {
        return position_;
    }

    // Update estimator (should be called periodically in main loop)
    void update(float current_time) {
        auto state = sub_.read();
        if (state == nullptr) {
            std::cout << "state not available" << std::endl;
            return;
        }
        if (last_time_ < 0) {
            last_time_ = current_time;
            return;
        }
        float dt = current_time - last_time_;
        last_time_ = current_time;

        // Get acceleration in body frame [ax, ay, az], unit: m/s^2
        auto acc_body = state->imu.accelerometer;

        // Get orientation
        auto quat = state->imu.quaternion;

        // Transform acceleration to world frame
        auto acc_world = bodyToWorld(acc_body, quat);

        // Gravity compensation (assuming z is up)
        constexpr float g = 9.81f;
        acc_world[2] -= g;

        // Low-pass filter for acceleration
        for (int i = 0; i < 3; ++i) {
            filtered_acc_[i] = acc_alpha_ * acc_world[i] + (1.0f - acc_alpha_) * filtered_acc_[i];
        }

        // Integrate acceleration to get velocity, then filter velocity
        for (int i = 0; i < 3; ++i) {
            velocity_[i] += filtered_acc_[i] * dt;
            filtered_vel_[i] = vel_alpha_ * velocity_[i] + (1.0f - vel_alpha_) * filtered_vel_[i];
            position_[i] += filtered_vel_[i] * dt;
        }
    }

private:
    unitree::robot::ChannelSubscriber<unitree::robot::msg::LowState> sub_;
    std::array<float, 3> position_;
    std::array<float, 3> velocity_;
    std::array<float, 3> filtered_acc_;
    std::array<float, 3> filtered_vel_;
    float last_time_;
    float acc_alpha_;
    float vel_alpha_;
}; 