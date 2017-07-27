#include "./libraries/mavlink.h"
#include <iostream>
using namespace std;

const char distance_sensor_msg[] = "\xfd\ //magic
                                    \x0b\ // lenght
                                    \x00\ // comp flags
                                    \x00  // incomp flags
                                    \x01\ // seq
                                    \x7d\ // sysid
                                    \x9e\ // compid
                                    \x84\x00\x00 // msg id
                                    \x00\x00\x00\x00 // time boot ms
                                    \x00\x32 // min dist
                                    \x01\xf4 // max dist
                                    \x00\x64\ // current dist
                                    \x01 // type
                                    \x54 // id
                                    \xb0" // orientation
                                    ;  

const char heartbeat_msg[] = "\xfd\
                              \x09\x00\x00\x00\x7d\x9e\
                              \x00\x00\x00\x00\x00\x00\x00\x06\x08\x00\x00\x03\xc2"; 

int main () {

    mavlink_heartbeat_t         heartbeat;
    mavlink_msg_heartbeat_decode(&heartbeat_msg,&heartbeat);

    cout << "mavlink_version = " << heartbeat.mavlink_version << endl;
    cout << "custom_mode = " << heartbeat.custom_mode << endl;
    cout << "autopilot = " << heartbeat.autopilot << endl;
    cout << "type = " << heartbeat.type << endl;
    cout << "system_status = " << heartbeat.system_status << endl;
    cout << "base_mode = " << heartbeat.base_mode << endl;

    mavlink_distance_sensor_t    distance_sensor;
    mavlink_msg_heartbeat_decode(&distance_sensor_msg,&distance_sensor);

    cout << "time_boot_ms = " << distance_sensor.time_boot_ms << endl;
    cout << "min_distance = " << distance_sensor.min_distance << endl;
    cout << "max_distance = " << distance_sensor.max_distance << endl;
    cout << "current_distance = " << distance_sensor.current_distance << endl;
    cout << "type = " << distance_sensor.type << endl;
    cout << "id = " << distance_sensor.id << endl;
    cout << "orientation = " << distance_sensor.orientation << endl;
    cout << "covariance = " << distance_sensor.covariance << endl;

}
