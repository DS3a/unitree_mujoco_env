#pragma once
#include <array>
#include <memory>

/**
 * @brief Generic pose estimator interface
 * 
 * This interface provides a generic way to estimate floating base pose
 * without depending on specific robot APIs. Implementations can be
 * created for different robot platforms (Unitree, Boston Dynamics, etc.)
 */
class PoseEstimatorInterface {
public:
    virtual ~PoseEstimatorInterface() = default;

    /**
     * @brief Get current orientation quaternion [w, x, y, z]
     * @return Quaternion as [w, x, y, z]
     */
    virtual std::array<float, 4> getOrientation() = 0;

    /**
     * @brief Get current angular velocity [wx, wy, wz]
     * @return Angular velocity in rad/s
     */
    virtual std::array<float, 3> getAngularVelocity() = 0;

    /**
     * @brief Get current linear velocity [vx, vy, vz]
     * @return Linear velocity in m/s
     */
    virtual std::array<float, 3> getLinearVelocity() = 0;

    /**
     * @brief Get current estimated position [x, y, z]
     * @return Position in meters
     */
    virtual std::array<float, 3> getPosition() = 0;

    /**
     * @brief Check if estimator is initialized
     * @return true if initialized, false otherwise
     */
    virtual bool isInitialized() const = 0;

    /**
     * @brief Update estimator (should be called periodically)
     * @param current_time Current time in seconds
     */
    virtual void update(float current_time) = 0;

    /**
     * @brief Reset estimator state
     */
    virtual void reset() = 0;
};

/**
 * @brief Factory function to create pose estimator
 * 
 * This function should be implemented by the specific robot platform
 * to create the appropriate pose estimator implementation.
 * 
 * @param acc_alpha Acceleration filter parameter (default: 0.2)
 * @param vel_alpha Velocity filter parameter (default: 0.2)
 * @return Shared pointer to pose estimator interface
 */
std::shared_ptr<PoseEstimatorInterface> createPoseEstimator(float acc_alpha = 0.2f, float vel_alpha = 0.2f);
