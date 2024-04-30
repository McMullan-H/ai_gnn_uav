# States of the remote agents

uint64 timestamp

uint8[4] armed
uint8 SYS_DISARMED = 1
uint8 SYS_ARMED = 2

uint8[4] in_position
uint8 POS_ADJUSTING = 1
uint8 POS_READY = 2

uint8[4] battery_state
uint8 BAT_WARN = 1
uint8 BAT_OK = 2
