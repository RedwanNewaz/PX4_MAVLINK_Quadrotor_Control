//
// Created by redwan on 7/5/25.
//

#ifndef QUADROTOR_TELEOPERATION_COMMON_HPP
#define QUADROTOR_TELEOPERATION_COMMON_HPP
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <array>
extern "C" {
#include <mavlink/common/mavlink.h>
}

// PX4 custom mode macros
#define PX4_CUSTOM_MAIN_MODE_AUTO          4
#define PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF   2
#define PX4_CUSTOM_MAIN_MODE_OFFBOARD 6
// MAVLink identifiers
const uint8_t sysid = 255; // ID of this GCS/program
const uint8_t compid = MAV_COMP_ID_MISSIONPLANNER;
const uint8_t target_sysid = 1;
const uint8_t target_compid = 1;

inline void send_mavlink_message(int sockfd, sockaddr_in& addr, const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sockfd, buf, len, 0, (sockaddr*)&addr, sizeof(addr));
}
using MAV_FUNC = std::function<void(const mavlink_message_t&)>;
// Structure to hold velocity commands
union VelocityCommand{
    struct {
        float vx = 0.0f;  // Forward/backward velocity
        float vy = 0.0f;  // Left/right velocity
        float vz = 0.0f;  // Up/down velocity
        float yaw_rate = 0.0f;  // Yaw rate
    };
    std::array<float, 4> values;
};
#endif //QUADROTOR_TELEOPERATION_COMMON_HPP
