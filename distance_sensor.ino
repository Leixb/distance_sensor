//include the mavlink library
//#include "./libraries/mavlink/include/mavlink.h"
//#include <mavlink.h>
#include "./libraries/mavlink.h"

//Baudrate
#define bRate 115200

void setup() {
    Serial.begin(bRate);
}

void loop() {
    command_heartbeat();
    uint16_t distance = read_distance();
    command_distance(distance, MAV_SENSOR_ROTATION_NONE);
}

/************************************************************
 * @brief Reads distance from sensor
 * @param NONE
 * @return distance
 *************************************************************/

uint16_t read_distance() {
    // placeholder
    return 100;
}

/************************************************************
 * @brief Sends a heartbeat message every second.
 * @param NONE
 * @return void
 *************************************************************/

void command_heartbeat() {

    //< ID 1 for this system
    int sysid = 1;                   
    //< The component sending the message.
    int compid = MAV_COMP_ID_MISSIONPLANNER;    

    // Define the system type, in this case ground control station
    uint8_t     system_type = MAV_TYPE_GCS;
    uint8_t     autopilot_type = MAV_AUTOPILOT_INVALID;

    uint8_t     system_mode     = 0;
    uint32_t    custom_mode     = 0;
    uint8_t     system_state    = 0;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_heartbeat_pack(sysid,compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message 
    delay(1000);
    Serial.write(buf, len);
}

/************************************************************
 * @brief Sends distance command
 * @param distance reading
 * @return void
 *************************************************************/

void command_distance(const uint16_t distance, const MAV_SENSOR_ORIENTATION sensor_rotation) {

    //TARGET DRONE
    uint8_t system_id = 1;    // Target drone id
    uint8_t component_id = 0; // Target component, 0 = all

    uint16_t seq = 0; // Sequence is always set to 0
    uint8_t frame = MAV_FRAME_GLOBAL; // Set target frame to global default

    const uint32_t    time_boot_ms        = 0;                  // Time since system boot
    const uint16_t    min_distance        = 50;                 // Minimum distance the sensor can measure in centimeters
    const uint16_t    max_distance        = 500;                // Maximum distance the sensor can measure in centimeters

    uint16_t    current_distance    = distance;                 // Current distance reading

    const uint8_t     type = MAV_DISTANCE_SENSOR_ULTRASOUND;    // Type from MAV_DISTANCE_SENSOR enum.
    const uint8_t     id   = 0;                                 // Onboard ID of the sensor

    uint8_t     orientation = sensor_rotation;                  // Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.

    const uint8_t     covariance  = 0;                          // Measurement covariance in centimeters, 0 for unknown / invalid readings

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_distance_sensor_pack(system_id, component_id, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes) 
    delay(1000);
    Serial.write(buf, len);
}
