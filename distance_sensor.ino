//include the mavlink library
#include "./libraries/mavlink.h"

//Baudrate
#define bRate 115200

// TODO: 4 different ultrasonic sensor pins
const uint8_t EchoPin = 5;
const uint8_t TriggerPin = 6;
const uint8_t LedPin = 13;

//TODO: heartbeat needed?

void setup() {
    // TODO: initialize missing analog ports for reading
    pinMode(LedPin, OUTPUT);
    pinMode(TriggerPin, OUTPUT);
    pinMode(EchoPin, INPUT);
    Serial.begin(bRate);
}

void loop() {
    /*command_heartbeat();*/
    uint16_t distance = ping(TriggerPin, EchoPin);
    command_distance(distance, MAV_SENSOR_ROTATION_NONE); 
    // TODO: send distance for all 4 orientations
    delay(1000);
}

/************************************************************
 * @brief Sends a heartbeat message every second.
 * @param NONE
 * @return void
 *************************************************************/

void command_heartbeat() {

    //< ID 1 for this system
    int system_id = 1;
    //< The component sending the message.
    int component_id = MAV_COMP_ID_PERIPHERAL;

    // Define the system type, in this case ground control station
    uint8_t     type            = MAV_TYPE_GCS;
    uint8_t     autopilot       = MAV_AUTOPILOT_INVALID;

    uint8_t     base_mode       = 0;
    uint32_t    custom_mode     = 0;
    uint8_t     system_status   = 0;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot, \
            base_mode, custom_mode, system_status);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message 
    Serial.write(buf, len);
}

/************************************************************
 * @brief Sends distance command
 * @param1 Distance reading
 * @param2 Sensor orientation
 * @return void
 *************************************************************/

void command_distance(const uint16_t& current_distance, const MAV_SENSOR_ORIENTATION& orientation) {

    // Target drone id
    const uint8_t   system_id       = 1;
    // Target component
    const uint8_t   component_id    = MAV_COMP_ID_PATHPLANNER;

    // Time since system boot
    const uint32_t  time_boot_ms    = millis();
    // Minimum distance the sensor can measure in centimeters
    const uint16_t  min_distance    = 50;
    // Maximum distance the sensor can measure in centimeters
    const uint16_t  max_distance    = 500;

    // @param1 current_distance = Distance reading in centimetres

    // type and id are IGNORED by pixhawk
    // Type from MAV_DISTANCE_SENSOR enum.
    const uint8_t   type            = MAV_DISTANCE_SENSOR_ULTRASOUND;
    // Onboard ID of the sensor
    const uint8_t   id              = 0;

    // @param2 orientation = Direction the sensor faces MAV_SENSOR_ORIENTATION

    // covariance is ignored by pihawk
    // Measurement covariance in centimeters, 0 for unknown / invalid readings
    const uint8_t   covariance      = 0;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_distance_sensor_pack(system_id, component_id, &msg, \
            time_boot_ms, min_distance, max_distance, \
            current_distance, type, id, orientation, covariance);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    Serial.write(buf, len);
}

/************************************************************
 * @brief  Reads ultrasound sensor distance
 * @param1 Trigger PIN
 * @param2 Echo PIN
 * @return distance (cm)
 *************************************************************/

uint16_t ping(const uint8_t& TriggerPin, const uint8_t& EchoPin) {
    unsigned long duration;

    // To generate a clean pulse we put LOW during 4us
    digitalWrite(TriggerPin, LOW);
    delayMicroseconds(4);
    // Generate Trigger of 10us
    digitalWrite(TriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TriggerPin, LOW);

    // Time between pulses in ms
    duration = pulseIn(EchoPin, HIGH);

    return (duration * 5)/292; // Convert distance to cm (10/292/2)
}
