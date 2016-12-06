#ifndef COMMON_JAUS_HPP
#define COMMON_JAUS_HPP

const int JAUS_DRIVE_MESSAGE_HEADER_SIZE = 4;
const int JAUS_DRIVE_MESSAGE_PAYLOAD_SIZE = 6;
const int JAUS_DRIVE_MESSAGE_FOOTER_SIZE = 1;
const int JAUS_DRIVE_MESSAGE_TOTAL_SIZE = 11;

const int MOTOR_CONTROLLER_ID = 0x05;

const uint8_t DLE_BYTE = 0xAB;
const uint8_t ETX_BYTE = 0xCD;

const int SET_SPEED_CMD = 0x99;
const int SET_ACTUATORS_CMD = 0xA5;

const int WHEEL_LEFT_INDEX = 4;
const int WHEEL_RIGHT_INDEX = 7;

const int ARM_TURNTABLE_INDEX = 4;
const int ARM_SHOULDER_INDEX = 5;
const int ARM_ELBOW_INDEX = 6;
const int ARM_WRIST_INDEX = 7;
const int ARM_CLAWROT_INDEX = 8;
const int ARM_CLAWGRIP_INDEX = 9;

#endif
