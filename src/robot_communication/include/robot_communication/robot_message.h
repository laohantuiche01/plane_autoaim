#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

#include <cstdint>

#define AIM_DATA_RECV_ID 0x81
#define CHASSIS_CONTROL_RECV_ID 0x82
#define REFEREE_RECV_ID 0x83
#define IMU_SEND_ID 0x11
#define TYRE_SPEED_SEND_ID 0x12
#define REFEREE_SEND_ID 0x13
#define GIMBAL_AND_CONFIG_SEND_ID 0x14

message_data aim_data_t {
    uint8_t success;
    float pitch;
    float yaw;
    float distance;
    float w_pitch;
    float w_yaw;
    float jmp_time;
    uint8_t target_rate;
    uint8_t target_number;
};

message_data chassis_control_data_t {
    int16_t forward_speed;
    int16_t panning_speed;
    int16_t rotate_speed;
};

message_data referee_data_r_t {
    uint16_t data_length;
    uint8_t referee_recv_data[2043];
};


message_data referee_data_s_t {
    uint16_t data_length;
    uint8_t referee_send_data[2043];
};

message_data imu_data_t {
    float x_gyro;
    float y_gyro;
    float z_gyro;
    float x_accel;
    float y_accel;
    float z_accel;
    float X;
    float Y;
    float Z;
    float W;
};

message_data tyre_speed_data_t {
    float x_pos;
    float y_pos;
    float robot_yaw_angle;
    float forward_speed;
    float panning_speed;
    float rotate_speed;
};

message_data gimbal_and_config_data_t {
    uint8_t mode;
    float roll;
    float pitch;
    float yaw;
};

message_data record_data_t {
    uint8_t record;
};

message_data nuc_receive_data_t {
    aim_data_t aim_data_received;
    chassis_control_data_t chassis_data_received;
    referee_data_r_t referee_data_received;
};

message_data nuc_transmit_data_t {
    imu_data_t imu_data_send;
    tyre_speed_data_t tyre_speed_data_send;
    referee_data_s_t referee_data_send;
    gimbal_and_config_data_t robot_gimbal_data_send;
};

#endif

