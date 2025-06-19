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

        // std::cout << "Model loaded successfully!" << std::endl;
    };
    ~Custom(){};
    void Init();
    void joint_position_control(JointIndex, double, double);
    void joint_torque_control(JointIndex, double);
    void hold_joint_position(JointIndex, unitree_go::msg::dds_::LowState_);

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

void Custom::hold_joint_position(JointIndex j_id, unitree_go::msg::dds_::LowState_ capture) {
    low_cmd.motor_cmd()[j_id].q() = capture.motor_state()[j_id].q();
    low_cmd.motor_cmd()[j_id].dq() = 0;
    low_cmd.motor_cmd()[j_id].kp() = 100; // very stiff
    low_cmd.motor_cmd()[j_id].kd() = 50.5;
    low_cmd.motor_cmd()[j_id].tau() = 0;
}


void Custom::LowCmdWrite()
{

    unitree_go::msg::dds_::LowState_ state_capture = low_state;
    
    float phase = 1.0;
    for (int i = 0; i < H1_NUM_MOTOR; i++) {
            low_cmd.motor_cmd()[i].q() = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 15;
            low_cmd.motor_cmd()[i].kd() = 3.5;
            low_cmd.motor_cmd()[i].tau() = 0;
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
