#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include "Array.hh"
#include "QuadProg++.hh"

#include "../whole_body_roller/include/constraint.hpp"
#include <Eigen/Dense>
#include "../whole_body_roller/include/roller.hpp"
#include "../whole_body_roller/include/dynamics.hpp"
#include "../whole_body_roller/include/task_space_constraints/frame_acceleration.hpp"
#include "../whole_body_roller/include/pose_estimator.hpp"

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);
#define PI 3.141591


enum JointIndex {
  // Right leg
  kRightHipYaw = 8,
  kRightHipRoll = 0,
  kRightHipPitch = 1,
  kRightKnee = 2,
  kRightAnkle = 11,
  // Left leg
  kLeftHipYaw = 7,
  kLeftHipRoll = 3,
  kLeftHipPitch = 4,
  kLeftKnee = 5,
  kLeftAnkle = 10,

  kWaistYaw = 6,

  kNotUsedJoint = 9,

  // Right arm
  kRightShoulderPitch = 12,
  kRightShoulderRoll = 13,
  kRightShoulderYaw = 14,
  kRightElbow = 15,
  // Left arm
  kLeftShoulderPitch = 16,
  kLeftShoulderRoll = 17,
  kLeftShoulderYaw = 18,
  kLeftElbow = 19,

};

const int H1_NUM_MOTOR = 27;

const std::string model_urdf_path = "/home/ds3a/dev/humanoid_wbc/unitree_ros/robots/h1_description/urdf/h1.urdf";

class Custom
{
public:
    Custom(){
        this->model = std::make_shared<pinocchio::Model>();
        pinocchio::urdf::buildModel(model_urdf_path, *(this->model));
        this->data = std::make_shared<pinocchio::Data>(*this->model);

        // --- Initialize WBC components ---
        // Initialize control decision variables (nv = velocity variables, nc = contact points)

        // Initialize Roller (main WBC controller)
        roller = std::make_shared<whole_body_roller::Roller>(dec_v, dynamics);
        
        // Initialize pose estimator for floating base
        pose_estimator = std::make_unique<FloatingBasePoseEstimator>(0.2f, 0.2f); // acc_alpha, vel_alpha

        // Here: keep both feet stationary  
        right_foot_constraint = std::make_shared<whole_body_roller::FrameAccelerationConstraint>(dynamics, "right_ankle_link");
        roller->add_constraint(right_foot_constraint->constraint, right_foot_constraint);
        
        left_foot_constraint = std::make_shared<whole_body_roller::FrameAccelerationConstraint>(dynamics, "left_ankle_link");
        roller->add_constraint(left_foot_constraint->constraint, left_foot_constraint);
        
        // Initialize additional constraint handlers for different tasks (here we only keep the double foot standing task)
        // Torso control - using "torso_link" from H1 URDF
        torso_constraint = std::make_shared<whole_body_roller::FrameAccelerationConstraint>(dynamics, "torso_link");
        roller->add_constraint(torso_constraint->constraint, torso_constraint);
        
        // Center of mass control - using "pelvis" as COM reference frame
        com_constraint = std::make_shared<whole_body_roller::FrameAccelerationConstraint>(dynamics, "pelvis");
        roller->add_constraint(com_constraint->constraint, com_constraint);
        
        // Right arm control - using "right_elbow_link" for arm end-effector
        right_arm_constraint = std::make_shared<whole_body_roller::FrameAccelerationConstraint>(dynamics, "right_elbow_link");
        roller->add_constraint(right_arm_constraint->constraint, right_arm_constraint);
        
        // Left arm control - using "left_elbow_link" for arm end-effector
        left_arm_constraint = std::make_shared<whole_body_roller::FrameAccelerationConstraint>(dynamics, "left_elbow_link");
        roller->add_constraint(left_arm_constraint->constraint, left_arm_constraint);

        // std::cout << "Model loaded successfully!" << std::endl;
    };
    ~Custom(){};
    void Init();
    void joint_position_control(JointIndex, double, double);
    void joint_torque_control(JointIndex, double);

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();

private:
    double stand_up_joint_pos[27] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double stand_down_joint_pos[27] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double dt = 0.002;
    unitree_go::msg::dds_::LowCmd_ low_cmd;     // default init
    unitree_go::msg::dds_::LowState_ low_state; // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;

    std::shared_ptr<pinocchio::Model> model;
    std::shared_ptr<pinocchio::Data> data;

    // --- WBC components ---
    std::shared_ptr<whole_body_roller::ControlDecisionVariables> dec_v;
    std::shared_ptr<whole_body_roller::Dynamics> dynamics;
    std::shared_ptr<whole_body_roller::Roller> roller;
    std::shared_ptr<whole_body_roller::FrameAccelerationConstraint> right_foot_constraint;
    std::shared_ptr<whole_body_roller::FrameAccelerationConstraint> left_foot_constraint;
    
    // TODO: Additional constraint handlers for different tasks 
    std::shared_ptr<whole_body_roller::FrameAccelerationConstraint> torso_constraint;
    std::shared_ptr<whole_body_roller::FrameAccelerationConstraint> com_constraint;
    std::shared_ptr<whole_body_roller::FrameAccelerationConstraint> right_arm_constraint;
    std::shared_ptr<whole_body_roller::FrameAccelerationConstraint> left_arm_constraint;
    
    std::unique_ptr<FloatingBasePoseEstimator> pose_estimator;
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Custom::Init()
{
    InitLowCmd();
    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, int(dt * 1000000), &Custom::LowCmdWrite, this);

    // std::cout << "time, right_sh_yaw, left_sh_yaw\n";
}

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < H1_NUM_MOTOR; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Custom::LowStateMessageHandler(const void *message)
{
    this->low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void Custom::joint_position_control(JointIndex j_id, double angle, double stiffness=100) {
    low_cmd.motor_cmd()[j_id].q() = angle;
    low_cmd.motor_cmd()[j_id].dq() = 0;
    low_cmd.motor_cmd()[j_id].kp() = stiffness;
    low_cmd.motor_cmd()[j_id].kd() = 2.5;
    low_cmd.motor_cmd()[j_id].tau() = 0;
}

void Custom::joint_torque_control(JointIndex j_id, double torque) {
    // low_cmd.motor_cmd()[j_id].q() = 0;
    // low_cmd.motor_cmd()[j_id].dq() = 0;
    low_cmd.motor_cmd()[j_id].kp() = 0;
    low_cmd.motor_cmd()[j_id].kd() = 0;
    low_cmd.motor_cmd()[j_id].tau() = torque;
}

void Custom::LowCmdWrite()
{
    // --- Step 1: Update pose estimation for floating base ---
    static double current_time = 0.0;
    pose_estimator->update(current_time);
    current_time += dt;
    
    // --- Step 2: Read joint states from low_state ---
    Eigen::VectorXd q(model->nq);   // Configuration vector (floating base + joints)
    Eigen::VectorXd dq(model->nv);  // Velocity vector (floating base + joints)
    
    // --- Step 3: Get floating base pose from pose estimator ---
    auto base_pos = pose_estimator->getPosition();      // [x, y, z]
    auto base_quat = pose_estimator->getOrientation();  // [w, x, y, z]
    auto base_lin_vel = pose_estimator->getLinearVelocity();  // [vx, vy, vz]
    auto base_ang_vel = pose_estimator->getAngularVelocity(); // [wx, wy, wz]
    
    // --- Step 4: Fill floating base states (first 7 elements for pose, next 6 for velocity) ---
    // Position [x, y, z]
    q[0] = base_pos[0]; q[1] = base_pos[1]; q[2] = base_pos[2];
    // Orientation [qw, qx, qy, qz]
    q[3] = base_quat[0]; q[4] = base_quat[1]; q[5] = base_quat[2]; q[6] = base_quat[3];
    
    // Linear velocity [vx, vy, vz]
    dq[0] = base_lin_vel[0]; dq[1] = base_lin_vel[1]; dq[2] = base_lin_vel[2];
    // Angular velocity [wx, wy, wz]
    dq[3] = base_ang_vel[0]; dq[4] = base_ang_vel[1]; dq[5] = base_ang_vel[2];
    
    // --- Step 5: Fill joint states (starting from index 7 for q, index 6 for dq) ---
    // Assuming first 27 joints are actuated, adjust as needed
    for (int i = 0; i < 27; ++i) {
        q[i + 7] = low_state.motor_state()[i].q();  // +7 for floating base (3 pos + 4 quat)
        dq[i + 6] = low_state.motor_state()[i].dq(); // +6 for floating base (3 lin vel + 3 ang vel)
    }
    
    // --- Step 6: Update joint states in dynamics model ---
    dynamics->update_joint_states(q, dq);
    
    // --- Step 7: Set task-space goals (modify setpoints in constraint handlers) ---
    // Only keep the double foot standing (stationary feet) task
    Eigen::VectorXd zero_acc = Eigen::VectorXd::Zero(6);
    right_foot_constraint->set_acceleration_target(zero_acc);
    left_foot_constraint->set_acceleration_target(zero_acc);
    
    // --- Step 8: Solve QP to get joint torques ---
    bool qp_success = roller->step(); // This calls solve_qp internally
    
    if (!qp_success) {
        std::cout << "QP solve failed! Using fallback control." << std::endl;
        // Fallback to simple position control if QP fails
        for (int i = 0; i < H1_NUM_MOTOR; i++) {
            low_cmd.motor_cmd()[i].q() = stand_up_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 100;
            low_cmd.motor_cmd()[i].kd() = 3.5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    } else {
        // --- Step 9: Write computed torques to low_cmd ---
        // dec_v->tau is a shared_ptr<Eigen::VectorXd> of size nv-6 (floating base not actuated)
        for (int i = 0; i < dec_v->tau->size(); ++i) {
            low_cmd.motor_cmd()[i].tau() = (*(dec_v->tau))[i];
            // Set position/velocity gains to zero for pure torque control
            low_cmd.motor_cmd()[i].q() = PosStopF;
            low_cmd.motor_cmd()[i].dq() = VelStopF;
            low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].kd() = 0;
        }
        
        // Set remaining motors to zero torque (if any)
        for (int i = dec_v->tau->size(); i < H1_NUM_MOTOR; ++i) {
            low_cmd.motor_cmd()[i].tau() = 0;
            low_cmd.motor_cmd()[i].q() = PosStopF;
            low_cmd.motor_cmd()[i].dq() = VelStopF;
            low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].kd() = 0;
        }
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }
    // std::cout << "Press enter to start";
    std::cin.get();
    Custom custom;
    custom.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
