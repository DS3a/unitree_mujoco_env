#include <iostream>
#include <memory>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <roller.hpp>  // <- deine WBO-Implementierung
#include <roller.cpp>  // <- deine WBO-Implementierung
#include <dynamics.hpp>
#include <dynamics.cpp>
#include <control_decision_variables.hpp>  // falls benötigt
#include <control_decision_variables.cpp>  // falls benötigt
#include <dummy_constraint.hpp>

#include <helpers.hpp>


using namespace unitree::robot;
using namespace unitree::common;
using namespace whole_body_roller;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double dt = 0.002;
constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom {
public:
    Custom() = default;
    ~Custom() = default;

    void Init();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();

private:
    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    ThreadPtr lowCmdWriteThreadPtr;

    // WBO Controller Components
    std::shared_ptr<whole_body_roller::Roller> roller;
    std::shared_ptr<whole_body_roller::Dynamics> dynamics;
    std::shared_ptr<whole_body_roller::ControlDecisionVariables> dec_vars;
    pinocchio::Model pin_model;
    std::shared_ptr<pinocchio::Data> pin_data;

    std::atomic<bool> low_state_ready = false;

    double run_time = 0.0;
};

void Custom::Init() {
    // Init low command packet
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; ++i) {
        low_cmd.motor_cmd()[i].mode() = 0x01;
        low_cmd.motor_cmd()[i].q() = PosStopF;
        low_cmd.motor_cmd()[i].dq() = VelStopF;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }

    // Publisher + Subscriber Init
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    // Load URDF & initialize Pinocchio
    std::string urdf_path = "/home/linus-schmueser/unitree_mujoco_env/robot_urdfs/h1_description/urdf/h1.urdf";  // passe Pfad ggf. an
    pinocchio::urdf::buildModel(urdf_path, pin_model);
    pin_data = std::make_shared<pinocchio::Data>(pin_model);
    std::shared_ptr<pinocchio::Model> pin_model_ptr = std::make_shared<pinocchio::Model>(pin_model);


    // WBO-Strukturen initialisieren
    dec_vars = std::make_shared<whole_body_roller::ControlDecisionVariables>(pin_model.nv, 4);
    dynamics = std::make_shared<whole_body_roller::Dynamics>(4, pin_model_ptr, dec_vars);
    roller = std::make_shared<whole_body_roller::Roller>(dec_vars, dynamics);

    
    dynamics->joint_positions_ = Eigen::VectorXd::Zero(pin_model.nq);
    dynamics->joint_velocities_ = Eigen::VectorXd::Zero(pin_model.nv);

    // Constraints hinzufügen (hier Beispiel — implementiere sie ggf.)

    auto dummy_constraint = std::make_shared<DummyConstraint>(dec_vars, 1, constraint_type_t::EQUALITY);
    auto dummy_handler = std::make_shared<DummyConstraintHandler>(dummy_constraint);
    roller->add_constraint(dummy_constraint, dummy_handler);


    // Start wiederholenden Thread
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("write_lowcmd", UT_CPU_ID_NONE, int(dt * 1e6), &Custom::LowCmdWrite, this);
}

void Custom::LowStateMessageHandler(const void *msg) {
    low_state = *(unitree_go::msg::dds_::LowState_ *)msg;
    std::cout << "[DEBUG] Received low_state, q[15] = " << low_state.motor_state()[15].q() << std::endl;
    low_state_ready = true;
}

void Custom::LowCmdWrite() {
    std::cout << "[DEBUG] In LowCmdWrite()" << std::endl;
    run_time += dt;

    if (!low_state_ready) {
    return;
    }

    
    // Set dynamics states
    for (int i = 0; i < pin_model.nq; ++i) {
        dynamics->joint_positions_(i) = low_state.motor_state()[i].q(); //low.state.motor_state whats the sdk index as input 
        dynamics->joint_velocities_(i) = low_state.motor_state()[i].dq();
    }

    // Run controller step
    if (roller->step()) {
        for (int i = 0; i < pin_model.nv; ++i) {
            low_cmd.motor_cmd()[i].q() = PosStopF;
            low_cmd.motor_cmd()[i].dq() = VelStopF;
            low_cmd.motor_cmd()[i].kp() = 0; //low.state.motor_state whats the sdk index as input 
            low_cmd.motor_cmd()[i].kd() = 0; //low.state.motor_state whats the sdk index as input 
            low_cmd.motor_cmd()[i].tau() =(*roller->joint_torques)(sdk_to_urdf_Index[i]); //roller->joint_torques are indexed with urdf indexes, i assume
        }
    }

    /*
   for (int i = 15; i < 20; ++i) {
    low_cmd.motor_cmd()[urdf_to_sdk_Index[i]].mode() = 0x01;     // torque mode
    low_cmd.motor_cmd()[urdf_to_sdk_Index[i]].tau() = 3.0f;      // fester Testwert
    // nicht setzen: q(), dq(), kp(), kd()
    }
    */

    // CRC32 + send
    extern uint32_t crc32_core(uint32_t *ptr, uint32_t len);  // ggf. global definieren
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, const char **argv) {
    if (argc < 2) {
        ChannelFactory::Instance()->Init(1, "lo");
    } else {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }

    std::cout << "[INFO] Starting Whole Body Controller..." << std::endl;
    Custom custom;
    custom.Init();

    while (true) {
        sleep(10);
    }

    return 0;
}


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