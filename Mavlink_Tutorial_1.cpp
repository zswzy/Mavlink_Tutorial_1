// System
#include <iostream>
#include <time.h>

// Project header
#include "small_udp.h"

// MAVSDK
#include "mavsdk.h"

using namespace mavsdk;

int main()
{
    auto component_type = Mavsdk::ComponentType::Autopilot;
    auto config = Mavsdk::Configuration(component_type);
    auto mav = Mavsdk(config);
    std::cout << "Mavsdk version: " << mav.version() << std::endl;
    std::cout << "Sys ID: " << unsigned(config.get_system_id())  // use unsigned for uin8_t
        << ", Component ID: " << unsigned(config.get_component_id())
        << ", Always heartbeats: " << config.get_always_send_heartbeats()
        << std::endl;

    // Use my own udp library
    auto udp = UdpCom();
    int r = udp.Open("127.0.0.1", 14540, "127.0.0.1", 14550);
    std::cout << "UDP Open: " << r << std::endl;

    // Timing
    static time_t last_time = 0;
    time_t start_time = time(NULL);
    time_t current_time = time(NULL);

    // Heartbeat message
    mavlink_message_t msg_heartbeat;
    uint8_t len_heartbeat;
    uint8_t buf_heartbeat[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(config.get_system_id(), config.get_component_id(), &msg_heartbeat,
        MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
    len_heartbeat = mavlink_msg_to_send_buffer(buf_heartbeat, &msg_heartbeat);

    // position message
    mavlink_message_t msg_position;
    uint8_t len_position;
    uint8_t buf_position[MAVLINK_MAX_PACKET_LEN];
    int32_t lat = 48.708944E7 + (current_time - start_time);
    int32_t lon = 2.167528E7;
    mavlink_msg_global_position_int_pack(config.get_system_id(), config.get_component_id(), &msg_position,
        (current_time - start_time) * 1000, lat, lon, 100, 100, 0, 0, 0, 0);
    len_position = mavlink_msg_to_send_buffer(buf_position, &msg_position);


    // Whenever a second has passed, we send a message.
    while (true)
    {
        current_time = time(NULL);
        if (current_time - last_time >= 1) {
            udp.Send((char*)(buf_heartbeat), len_heartbeat);
            std::cout << "Heartbeat sent." << std::endl;

            udp.Send((char*)(buf_position), len_position);
            std::cout << "Position sent." << std::endl;

            last_time = current_time;
        }
    }

    return 0;
}


