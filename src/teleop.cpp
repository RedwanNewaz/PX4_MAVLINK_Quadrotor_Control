//
// Created by redwan on 7/5/25.
//
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <regex>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/blackboard.h>
#include <nlohmann/json.hpp>
#include "common.hpp"
using namespace BT;

//param set COM_RC_IN_MODE 1
// param set COM_OBL_ACT 0

// Keyboard input node
class KeyboardInputNode : public SyncActionNode {
public:
    KeyboardInputNode(const std::string &name, const NodeConfiguration &config)
            : SyncActionNode(name, config) {
        setupTerminal();
    }

    ~KeyboardInputNode() {
        restoreTerminal();
    }

    static PortsList providedPorts() {
        return {OutputPort<std::string>("keyboard_cmd")};
    }

    NodeStatus tick() override {
        // Check for keyboard input
        nlohmann::json cmd_json;
        for(auto& key : {"w", "s", "a", "d", "q", "e", "j", "l", "x", "t", "g"}) {
            cmd_json[key] = 0.0f; // Initialize all keys to 0.0f
        }

        if (kbhit()) {
            char key = getchar();
            if(key == 27)
                return NodeStatus::FAILURE;
            else if(isValidKey(key)) {
                // Convert char to string for JSON key
                std::string keyStr(1, key);
                cmd_json[keyStr] = 1.0f; // Set the key value to 1.0f
            }
        }

        std::string cmd = cmd_json.dump(); // Convert JSON object to string
//        std::cout << "\r  [Command]: " << cmd << "\t" << std::endl;


        // Set the velocity command in the blackboard
        setOutput("keyboard_cmd", cmd);
        return NodeStatus::SUCCESS;
    }

private:
    struct termios old_termios;

protected:
    // Handle special keys like ESC (ASCII 27)
    bool isValidKey(int key) {
        // For printable characters
        if (key >= 32 && key <= 126) {
            std::regex validKeys("^[wsadqejlxtg]$");
            std::string keyStr(1, static_cast<char>(key));
            return std::regex_match(keyStr, validKeys);
        }
            // For special keys
        else if (key == 27) { // ESC
            return true;
        }
        return false;
    }

    void setupTerminal() {
        tcgetattr(STDIN_FILENO, &old_termios);
        struct termios new_termios = old_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    void restoreTerminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
    }

    bool kbhit() {
        fd_set readfds;
        struct timeval timeout;

        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        return select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout) > 0;
    }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class SendVelocityNode : public SyncActionNode {
public:
    SendVelocityNode(const std::string &name, const NodeConfiguration &config)
            : SyncActionNode(name, config) {
    }

    ~SendVelocityNode() {
    }

    static PortsList providedPorts() {
        return {InputPort<std::string>("keyboard_cmd"),
                OutputPort<VelocityCommand>("velocity_cmd")};
    }

    NodeStatus tick() override {
        std::string cmd_str;
        getInput<std::string>("keyboard_cmd", cmd_str);

        try {
            // Parse the JSON command string
            nlohmann::json cmd_json = nlohmann::json::parse(cmd_str);

            // Convert it to std::map<std::string, float>
            std::map<std::string, float> cmd_map;

            // Use get() to convert from JSON to map
            cmd_map = cmd_json.get<std::map<std::string, float>>();

            VelocityCommand vel_cmd;
            if(!processCommands(cmd_map, vel_cmd))
                return NodeStatus::FAILURE; // If special keys are detected, return FAILURE

            setOutput("velocity_cmd", vel_cmd); // Set the velocity command in the blackboard
            return NodeStatus::SUCCESS;
        }
        catch (const nlohmann::json::exception& e) {
            // Handle JSON parsing errors
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
            return NodeStatus::FAILURE;
        }
    }

private:
    bool processCommands(const std::map<std::string, float>& cmd_map, VelocityCommand& cmd) {
        for (const auto& [key, value] : cmd_map) {

            if (value == 0.0f) {
                continue; // Skip if the value is 0.0f
            }

            std::regex specialKeys("^[xtg]$");
            std::string keyStr(1, key[0]); // Convert single character key to string


            if(std::regex_match(keyStr, specialKeys))
                return false;

            if (key == "w") {
//                std::cout << "  [Forward]";
                cmd.vx = 1.0f; // Forward
            } else if (key == "s") {
//                std::cout << "  [Backward]";
                cmd.vx = -1.0f; // Backward
            } else if (key == "a") {
//                std::cout << "  [Left]";
                cmd.vy = -1.0f; // Left
            } else if (key == "d") {
//                std::cout << "  [Right]";
                cmd.vy = 1.0f; // Right
            } else if (key == "q") {
//                std::cout << "  [Up]";
                cmd.vz = 1.0f; // Up
            } else if (key == "e") {
//                std::cout << "  [Down]";
                cmd.vz = -1.0f; // Down
            } else if (key == "j") {
//                std::cout << "  [Yaw Left]";
                cmd.yaw_rate = -1.0f; // Yaw left
            } else if (key == "l") {
//                std::cout << "  [Yaw Right]";
                cmd.yaw_rate = 1.0f; // Yaw right
            } else if (key == "x") {
                std::cout << "  [Stop]";
            } else if (key == "t") {
                std::cout << "  [Takeoff]";
            } else if (key == "g") {
                std::cout << "  [Land]";
            } else if (key == "esc") {
                std::cout << "  [Exit Requested]";
            }
        }

//        std::cout << "                                          " << std::flush;
        return true;
    }
};

class SendSpecialCommandNode : public ThreadedAction {
public:

    SendSpecialCommandNode(const std::string &name, const NodeConfiguration &config, const MAV_FUNC& mavlink_sender)
            : ThreadedAction(name, config), mavlink_sender_(mavlink_sender) {
    }

    ~SendSpecialCommandNode() {
    }

    static PortsList providedPorts() {
        return {InputPort<std::string>("keyboard_cmd")};
    }

    NodeStatus tick() override {
        auto cmd = getInput<std::string>("keyboard_cmd");
        if (!cmd) {
            std::cerr << "Failed to get special command from blackboard\n";
            return NodeStatus::FAILURE;
        }

        // Parse the JSON command string
        nlohmann::json cmd_json;
        try {
            std::string cmd_str = cmd.value();
            if (cmd_str.empty() || cmd_str == "{}") {
                return NodeStatus::SUCCESS; // No command received
            }

            cmd_json = nlohmann::json::parse(cmd_str);

            // Process the commands
            for (const auto& [key, value] : cmd_json.items()) {
                if(value == 0.0f)
                    continue; // Skip if the value is 0.0f
                if (key == "t") {
                    std::cout << "\r \t\t\t\t\t\t [Takeoff Command Sent]";
                    takeoff();
                } else if (key == "g") {
                    std::cout << "\r \t\t\t\t\t\t [Land Command Sent]";
                    land();
                } else if (key == "x") {
//                    for (int i = 0; i < 10; ++i) {
//                        manualControl();
//                        usleep(10000); // 10ms interval
//                    }
                    switchGuideMode();
                    std::cout << "\r \t\t\t\t\t\t [Guided Mode Sent]";
                } else if (key == "esc") {
                    std::cout << "\r \t\t\t\t\t\t [Exit Requested]";
                    return NodeStatus::FAILURE; // Exit the tree
                }
            }

        } catch (const nlohmann::json::parse_error &e) {
            std::cerr << "JSON parse error: " << e.what() << "\n";
            std::cerr << "Command string was: " << cmd.value() << "\n";
            return NodeStatus::FAILURE;
        }

        return NodeStatus::SUCCESS;
    }
private:
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
    void takeoff(){
        // 1. ARM
        mavlink_message_t arm_msg;
        mavlink_msg_command_long_pack(
                sysid, compid, &arm_msg,
                target_sysid, target_compid,
                MAV_CMD_COMPONENT_ARM_DISARM,
                0, // confirmation
                1, // param1 = 1 to arm
                0, 0, 0, 0, 0, 0
        );
//    send_mavlink_message(sockfd, sim_addr, arm_msg);
        mavlink_sender_(arm_msg);
        std::cout << "✅ Sent ARM command\n";
        sleep(2);

        // 2. SET_MODE to AUTO.TAKEOFF
        mavlink_message_t mode_msg;
        uint32_t custom_mode =
                (PX4_CUSTOM_MAIN_MODE_AUTO << 16) |
                (PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF << 24);

        mavlink_msg_set_mode_pack(
                sysid, compid, &mode_msg,
                target_sysid,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode
        );
//    send_mavlink_message(sockfd, sim_addr, mode_msg);
        mavlink_sender_(mode_msg);
        std::cout << "✅ Sent AUTO.TAKEOFF mode\n";

    }

    void switchGuideMode() {
        // 2. SET_MODE to GUIDED
        uint32_t custom_mode = (PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16);

        mavlink_message_t mode_msg;
        mavlink_msg_set_mode_pack(
                sysid, compid, &mode_msg,
                target_sysid,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode
        );
        mavlink_sender_(mode_msg);
        std::cout << "✅ Sent GUIDED mode\n";
    }

    void land() {
        // 3. LAND command
        mavlink_message_t land_msg;
        mavlink_msg_command_long_pack(
                sysid, compid, &land_msg,
                target_sysid, target_compid,
                MAV_CMD_NAV_LAND,
                0,
                0, 0, 0, 0, 0, 0, 0
        );
        mavlink_sender_(land_msg);
        std::cout << "✅ Sent LAND command\n";
    }
};