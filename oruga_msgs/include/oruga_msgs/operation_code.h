#ifndef OPERATION_CODE_H
#define OPERATION_CODE_H

namespace operation_code {

// Relative moves
const uint8_t INCREMENT_RIGHT    = 0x01;  // = 1
const uint8_t INCREMENT_LEFT     = 0x02;  // = 2 
const uint8_t INC_RIGTH_DEC_LEFT = 0x03;  // = 3
const uint8_t INC_LEFT_DEC_RIGHT = 0x04;  // = 4
const uint8_t INCREMENT_FORWARD  = 0x05;  // = 5
const uint8_t INCREMENT_BACKWARD = 0x06;  // = 6

// Absolute moves
const uint8_t ABS_BOTH_MOTORS = 0xF1;  // = 241
const uint8_t ABS_RIGHT_MOTOR = 0xF2;  // = 242
const uint8_t ABS_LEFT_MOTOR  = 0xF3;  // = 243
const uint8_t ABS_R_L_MOTORS  = 0xF4;  // = 244

// Sensors & actuators
const uint8_t REQUEST_SENSOR  = 0x20;  // = 32
const uint8_t RESPONSE_SENSOR = 0x21;  // = 33

const uint8_t SWITCH_LEDS     = 0x22;  // = 34
const uint8_t SWITCH_LIGHT    = 0x23;  // = 35

// Ping
const uint8_t REQUEST_PING  = 0x25;  // = 37
const uint8_t RESPONSE_PING = 0x26;  // = 38

}

#endif  // OPERATION_CODE_H
