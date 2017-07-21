//include the mavlink library
#include "./libraries/mavlink.h"

//Baudrate
#define bRate 115200

const int EchoPin = 5;
const int TriggerPin = 6;
const int LedPin = 13;

//TODO: sysid and compid

void setup() {
    // TODO: initialize analgo ports for reading
    pinMode(LedPin, OUTPUT);
    pinMode(TriggerPin, OUTPUT);
    pinMode(EchoPin, INPUT);
    Serial.begin(bRate);
}

void loop() {
    /*command_heartbeat();*/
    uint16_t distance = ping(TriggerPin, EchoPin);
    command_distance(distance, MAV_SENSOR_ROTATION_NONE); // TODO: send distance for all 4 orientations
    delay(1000);
}

/************************************************************
 * @brief Reads distance from sensor
 * @param NONE
 * @return distance
 *************************************************************/

uint16_t read_distance() {

    // TODO: implementation

    return 100;
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
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot, base_mode, custom_mode, system_status);

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

void command_distance(const uint16_t distance, const MAV_SENSOR_ORIENTATION sensor_rotation) {

    //TARGET DRONE
    uint8_t system_id = 1;    // Target drone id
    uint8_t component_id = 0; // Target component, 0 = all

    const uint32_t    time_boot_ms        = millis();           // Time since system boot
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
    Serial.write(buf, len);
}

int ping(int TriggerPin, int EchoPin) {
    long duration, distanceCm;

    digitalWrite(TriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
    delayMicroseconds(4);
    digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
    delayMicroseconds(10);
    digitalWrite(TriggerPin, LOW);

    duration = pulseIn(EchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos

    distanceCm = duration * 10 / 292/ 2;   //convertimos a distancia, en cm
    return distanceCm;
}
