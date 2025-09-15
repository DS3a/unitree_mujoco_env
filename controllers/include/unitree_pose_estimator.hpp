#pragma once
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <iostream>
#include <array>
#include <cmath>
#include "pose_estimator_interface.hpp"

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

class UnitreePoseEstimator : public PoseEstimatorInterface {
public:
    unitree_go::msg::dds_::LowState_ *state;
    UnitreePoseEstimator(float acc_alpha = 0.2f, float vel_alpha = 0.2f)
        : 
        // sub_("rt/low_state"),
          position_{0.0f, 0.0f, 1.0f}, // should be {0.0f, 0.0f, 1.1f} at the start of simulation but when the robot drops down its at ca. 0.7
          velocity_{0.0f, 0.0f, 0.0f},
          filtered_acc_{0.0f, 0.0f, 0.0f},
          filtered_vel_{0.0f, 0.0f, 0.0f},
          last_time_(-1.0),
          acc_alpha_(acc_alpha),
          vel_alpha_(vel_alpha),
          initialized_(false)
    {}

    // Get current orientation quaternion [w, x, y, z]
    std::array<float, 4> getOrientation() override {
        // unitree_go::msg::dds_::LowState_ *state;
        // sub_.Recv(state);
        if (state == nullptr) {
            std::cout << "[Unitree Pose Estimator] State not available" << std::endl;
            return {1.0f, 0.0f, 0.0f, 0.0f}; // Identity quaternion
        }
        
        // Check if IMU data is valid
        auto quat = std::array<float, 4>{
            state->imu_state().quaternion()[0],
            state->imu_state().quaternion()[1], 
            state->imu_state().quaternion()[2],
            state->imu_state().quaternion()[3]
        };
        
        // Normalize quaternion to ensure it's valid
        float norm = std::sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
        if (norm > 0.0f) {
            for (int i = 0; i < 4; ++i) {
                quat[i] /= norm;
            }
        }
        
        return quat;
    }

    // Get current angular velocity [wx, wy, wz]
    std::array<float, 3> getAngularVelocity() override {
        // auto state = sub_.read();
        if (state == nullptr) {
            std::cout << "[Unitree Pose Estimator] State not available" << std::endl;
            return {0.0f, 0.0f, 0.0f};
        }
        return {state->imu_state().gyroscope()[0], 
                state->imu_state().gyroscope()[1], 
                state->imu_state().gyroscope()[2]};
    }

    // Get current linear velocity [vx, vy, vz]
    std::array<float, 3> getLinearVelocity() override {
        return filtered_vel_;
    }

    // Get current estimated position [x, y, z]
    std::array<float, 3> getPosition() override {
        return position_;
    }

    // Check if estimator is initialized
    bool isInitialized() const override {
        return initialized_;
    }

    void update(float current_time) override {
        // This method is not used in this implementation, use update_est instead
        std::cout << "[Unitree Pose Estimator] Use update_est instead of update" << std::endl;
    }

    // Update estimator (should be called periodically in main loop)
    void update_est(float current_time, unitree_go::msg::dds_::LowState_ *state_) {
        this->state = state_;
        // auto state = sub_.read();
        if (state == nullptr) {
            std::cout << "[Unitree Pose Estimator] State not available" << std::endl;
            return;
        }
        
        if (last_time_ < 0) {
            last_time_ = current_time;
            initialized_ = false;
            return;
        }
        
        float dt = current_time - last_time_;
        if (dt <= 0.0f) {
            return; // Invalid time step
        }
        
        // Limit dt to prevent large jumps
        if (dt > 0.1f) {
            dt = 0.1f;
        }
        
        last_time_ = current_time;

        // Get acceleration in body frame [ax, ay, az], unit: m/s^2
        auto acc_body = std::array<float, 3>{
            state->imu_state().accelerometer()[0],
            state->imu_state().accelerometer()[1], 
            state->imu_state().accelerometer()[2]
        };

        // Get orientation
        auto quat = getOrientation();

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
            std::cout << "currrent position "<< position_[i] << " currrent velocity: " <<  filtered_vel_[i] << std::endl;
            std::cout << dt << std::endl;

        }
        
        if (!initialized_) {
            initialized_ = true;
            std::cout << "[Unitree Pose Estimator] Initialized successfully" << std::endl;
        }
    }

    // Reset estimator state
    void reset() override {
        position_ = {0.0f, 0.0f, 0.7f};
        velocity_ = {0.0f, 0.0f, 0.0f};
        filtered_acc_ = {0.0f, 0.0f, 0.0f};
        filtered_vel_ = {0.0f, 0.0f, 0.0f};
        last_time_ = -1.0;
        initialized_ = false;
    }

private:
    // unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_> sub_;
    std::array<float, 3> position_;
    std::array<float, 3> velocity_;
    std::array<float, 3> filtered_acc_;
    std::array<float, 3> filtered_vel_;
    float last_time_;
    float acc_alpha_;
    float vel_alpha_;
    bool initialized_;
}; 