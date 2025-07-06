//
// Created by redwan on 7/5/25.
//
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/blackboard.h>
#include <nlohmann/json.hpp>
#include "common.hpp"
#include <deque>



using namespace BT;

class VelocityControlNode : public SyncActionNode {
public:
    VelocityControlNode(const std::string& name, const NodeConfiguration& config, const MAV_FUNC& mavlink_sender)
        : SyncActionNode(name, config), mavlink_sender_(mavlink_sender) {}

    static PortsList providedPorts() {
        return {
            InputPort<float>("gain"),
            InputPort<VelocityCommand>("velocity_cmd"),
            OutputPort<VelocityCommand>("control_cmd")
            };
    }

    NodeStatus tick() override {
        VelocityCommand cmd;
        float gain;
        // Get the velocity command from the blackboard
        getInput<VelocityCommand>("velocity_cmd", cmd);
        // Get the gain from the blackboard
        if (!getInput<float>("gain", gain)) {
            std::cerr << "Failed to get gain from blackboard\n";
            return NodeStatus::FAILURE;
        }
        std::cout << "\r   ";
        float total_vel = 0.0f;
        for(int i = 0; i < 4; ++i) {
            cmd_.values[i] = gain * cmd_.values[i] + ( 1.0 - gain) * cmd.values[i]; // Apply gain to each velocity component
//            std::cout << cmd_.values[i] << "\t";
            printf("% .4f\t", cmd_.values[i]);
            std::clamp(cmd_.values[0], -1.0f, 1.0f); // Clamp vx
            total_vel += std::abs(cmd_.values[i]);
        }
//        std::cout << std::endl;
        std::cout << "                                          " << std::flush;

        setOutput("control_cmd", cmd_); // Set the control command in the blackboard
        sendVelocityCommand(cmd_); // Send the velocity command via MAVLink
        return NodeStatus::SUCCESS;
    }

protected:
    void sendVelocityCommand(const VelocityCommand& cmd) {
        enable_manual_ = false;
        mavlink_message_t msg;
        mavlink_set_position_target_local_ned_t target;

        // Fill the SET_POSITION_TARGET_LOCAL_NED message
        target.time_boot_ms = 0; // Timestamp (not used)
        target.target_system = target_system;
        target.target_component = target_component;
        target.coordinate_frame = MAV_FRAME_LOCAL_NED;

        // Type mask: use velocity setpoints
        target.type_mask = 0b0000111111000111; // Ignore position and acceleration, use velocity

        // Position (ignored due to type mask)
        target.x = 0.0f;
        target.y = 0.0f;
        target.z = 0.0f;

        // Velocity setpoints (NED frame)
        target.vx = cmd.vx;  // North (forward)
        target.vy = cmd.vy;  // East (right)
        target.vz = cmd.vz;  // Down (negative up)

        // Acceleration (ignored)
        target.afx = 0.0f;
        target.afy = 0.0f;
        target.afz = 0.0f;

        // Yaw and yaw rate
        target.yaw = 0.0f;
        target.yaw_rate = cmd.yaw_rate;

        // Pack and send the message
        mavlink_msg_set_position_target_local_ned_encode(255, 0, &msg, &target);

        mavlink_sender_(msg);
    }




private:
    VelocityCommand cmd_;
    MAV_FUNC mavlink_sender_;
    uint8_t target_system = 1;
    uint8_t target_component = 1;
    bool enable_manual_ = true;
    bool first_manual_ = true;
};

class ManualControlNode : public StatefulActionNode {
public:
    ManualControlNode(const std::string& name, const NodeConfiguration& config, const MAV_FUNC& mavlink_sender)
        : StatefulActionNode(name, config), mavlink_sender_(mavlink_sender) {}

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus onStart() override {
        last_time_ = std::chrono::steady_clock::now();
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time_).count();
        double freq_z = 1.0 / (elapsed * 1e-6);
        if(!init_){
            freq_ = freq_z;
            init_ = true;
        }
        freq_ = alpha_ * freq_ + (1 - alpha_) * freq_z;
//        std::cout << "\r   [Manual Control Frequency]: " << freq_ << " Hz          " << freq_z << std::endl;
        manualControl();

        return NodeStatus::SUCCESS;
    }

    void onHalted() override {
        // Optionally send a stop command or perform cleanup
    }
private:
    std::chrono::steady_clock::time_point last_time_;
    double freq_ = 0.0;
    double alpha_ = 0.99;
    bool init_ = false;
    MAV_FUNC mavlink_sender_;

protected:
    void manualControl(){
        mavlink_message_t manual_msg;
        mavlink_msg_manual_control_pack(
                sysid, compid, &manual_msg,
                target_sysid,     // target system
                0,                // x (pitch input, -1000..1000)
                0,                // y (roll input, -1000..1000)
                500,              // z (throttle up = 1000)
                0,                // r (yaw, -1000..1000)
                0,                // buttons
                0,                // buttons2
                0,                // sld1
                0,                // sld2
                0,                // aux1
                0,                // aux2
                0,                // aux3
                0,                // aux4
                0,                // aux5
                0,                // aux6
                0                 // aux7
        );
        mavlink_sender_(manual_msg);
    }

};

