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

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

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




class Custom
{
public:
    Custom(){};
    ~Custom(){};
    void Init();
    void joint_position_control(JointIndex, double, double);
    void joint_torque_control(JointIndex, double);
    void joint_pd_controller_step(JointIndex, double);

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();


private:
    double stand_up_joint_pos[19] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double stand_down_joint_pos[19] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};     // default init
    unitree_go::msg::dds_::LowState_ low_state{}; // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;
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

class PDController {
public:
    float kp=50;
    float kd=3.5;
    float setpoint_;
    float current_value_;
    bool prev_error = false;
    float previous_error = 0;
    float dt_;
    PDController(float dt) {
        dt_ = dt;
    }

    void set_setpoint(double setpoint) {
        setpoint_ = setpoint;
    }

    double step(double current_value) {
        current_value_ = current_value;
        double error = setpoint_ - current_value_;
        double d_error = 0;
        if (prev_error) {
            d_error += (error - previous_error) / dt_;
        } else {
            prev_error = true;
        }

        previous_error = error;

        return kp*error + kd*d_error;
    }
};

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

void Custom::joint_pd_controller_step(JointIndex j_id, double setpoint) {
    float kp = 40;
    float kd = 3.5;
    float current_position = this->low_state.motor_state()[j_id].q();
}



void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
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
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void Custom::LowCmdWrite()
{
    
    PDController elbow_pd(dt);

    runing_time += dt;
    if (runing_time < 3.0)
    {
        // Stand up in first 3 second

        // Total time for standing up or standing down is about 1.2s
        phase = tanh(runing_time / 1.2);
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = phase * 50.0 + (1 - phase) * 20.0;
            low_cmd.motor_cmd()[i].kd() = 3.5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }
    else if (runing_time < 5.0)
    {
        elbow_pd.set_setpoint(0);
        double left_elbow_angle = low_state.motor_state()[JointIndex::kLeftElbow].q();
        std::cout << runing_time << ", " << left_elbow_angle << std::endl;
        double torque = elbow_pd.step(left_elbow_angle);
        this->joint_torque_control(JointIndex::kLeftElbow, torque);
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
    std::cout << "time, elbow_angle\n";
    // std::cin.get();
    Custom custom;
    custom.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
