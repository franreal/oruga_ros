uint8 code
uint8[] value

# Operation codes in base 10:

## Relative moves
uint8 INCREMENT_RIGHT    = 1
uint8 INCREMENT_LEFT     = 2
uint8 INC_RIGTH_DEC_LEFT = 3
uint8 INC_LEFT_DEC_RIGHT = 4
uint8 INCREMENT_FORWARD  = 5
uint8 INCREMENT_BACKWARD = 6

## Absolute moves
uint8 ABS_BOTH_MOTORS = 241
uint8 ABS_RIGHT_MOTOR = 242
uint8 ABS_LEFT_MOTOR  = 243
uint8 ABS_R_L_MOTORS  = 244

## Sensors & actuators
uint8 REQUEST_SENSOR  = 32
uint8 RESPONSE_SENSOR = 33

uint8 SWITCH_LEDS     = 34

