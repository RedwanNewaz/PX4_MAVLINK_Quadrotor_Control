#include <iostream>
#include <functional>
#include "src/common.hpp"
#include "src/teleop.cpp"
#include "src/control.cpp"



int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <behavior_tree_xml>\n";
        return 1;
    }
    // Set up UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in sim_addr{};
    sim_addr.sin_family = AF_INET;
    sim_addr.sin_port = htons(18570); // 🔧 PX4 MAVLink UDP port (from your log)
    inet_pton(AF_INET, "127.0.0.1", &sim_addr.sin_addr);



    std::function<void(const mavlink_message_t&)> mavlink_sender = [&](const mavlink_message_t& msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        size_t len = mavlink_msg_to_send_buffer(buf, &msg);
        sendto(sockfd, buf, len, 0, (sockaddr*)&sim_addr, sizeof(sim_addr));
    };



    // Create the behavior tree factory
    BehaviorTreeFactory factory;

    // Register custom nodes
    factory.registerNodeType<KeyboardInputNode>("KeyboardInput");
    factory.registerNodeType<SendVelocityNode>("SendVelocity");
    factory.registerNodeType<SendSpecialCommandNode>("SendSpecialCommand", mavlink_sender);
    factory.registerNodeType<VelocityControlNode>("VelocityControl", mavlink_sender);
    factory.registerNodeType<ManualControlNode>("ManualControl", mavlink_sender);

    // Create the behavior tree
    auto tree = factory.createTreeFromFile(argv[1]);

    std::cout << "Quadrotor Teleoperation Started\n";
    std::cout << "Controls:\n";
    std::cout << "  W/S: Forward/Backward\n";
    std::cout << "  A/D: Left/Right\n";
    std::cout << "  Q/E: Up/Down\n";
    std::cout << "  J/L: Yaw Left/Right\n";
    std::cout << "  T: Takeoff\n";
    std::cout << "  G: Land\n";
    std::cout << "  X: Stop\n";
    std::cout << "  ESC: Exit\n\n";

    // Run the behavior tree in a loop
    while (true) {
        NodeStatus status = tree.tickWhileRunning();

        if (status == NodeStatus::FAILURE) {
            std::cout << "Exiting teleoperation\n";
            break;
        }

        // Small delay to prevent excessive CPU usage
//        usleep(50000); // 50ms delay (20Hz update rate)
//        usleep(50); // 50ms delay (20KHz update rate)
    }






    close(sockfd);
    return 0;
}
