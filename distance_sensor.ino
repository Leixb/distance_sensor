// Include the mavlink library
#include "./libraries/mavlink.h"

// Baudrate
#define bRate 115200

// ID of this system
#define SYSTEM_ID 125
// ID of this component
#define COMPONENT_ID MAV_COMP_ID_PERIPHERAL

const uint8_t EchoPin[]                     = {2};//,                        4,                          6,                           8};
const uint8_t TriggerPin[]                  = {3};//,                        5,                          7,                           9};
const MAV_SENSOR_ORIENTATION Orientation[]  = {MAV_SENSOR_ROTATION_NONE, MAV_SENSOR_ROTATION_YAW_90, MAV_SENSOR_ROTATION_YAW_180, MAV_SENSOR_ROTATION_YAW_270};
const uint8_t LedPin = 13;

const uint8_t EchoPin_size = sizeof(EchoPin) / sizeof(EchoPin[0]);
const uint8_t TriggerPin_size = sizeof(TriggerPin) / sizeof(TriggerPin[0]);
const uint8_t Orientation_size = sizeof(Orientation) / sizeof(Orientation[0]);

bool LedStatus = 0;

void setup() {
    pinMode(LedPin, OUTPUT);
    for (uint8_t i = 0; i < EchoPin_size and i < TriggerPin_size; ++i) {
        pinMode(TriggerPin[i], OUTPUT);
        pinMode(EchoPin[i], INPUT);
    }
    Serial.begin(bRate);
}

// TODO: heartbeat needed?

void loop() {
    send_heartbeat();
    for (uint8_t i = 0; i < EchoPin_size and i < TriggerPin_size and i < Orientation_size; ++i) {
        uint16_t distance = ping(TriggerPin[i], EchoPin[i]);
        send_distance(distance, Orientation[i]); 
        delay(50); // 10 meters -> 30 ms
        LedStatus^=1;
        digitalWrite(LedPin, ((LedStatus)? LOW : HIGH));
    }
    delay(800); // 50ms*4 + 300ms = 1.0s
}

/************************************************************
 * @brief Sends a heartbeat message
 * @param NONE
 * @return void
 *************************************************************/


void send_heartbeat() {

    // Define the system type
    uint8_t     type            = MAV_TYPE_GCS;
    uint8_t     autopilot       = MAV_AUTOPILOT_INVALID;

    uint8_t     base_mode       = 0;
    uint32_t    custom_mode     = 0;
    uint8_t     system_status   = 0;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, type, autopilot, \
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

void send_distance(const uint16_t& current_distance, const MAV_SENSOR_ORIENTATION& orientation) {

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
    mavlink_msg_distance_sensor_pack(SYSTEM_ID, COMPONENT_ID, &msg, \
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
