# TOPICS serial_tx serial_rx

uint8 DEV_ID_TELEM1 = 0
uint8 DEV_ID_TELEM2 = 1
uint8 DEV_ID_GPS1 = 2
uint8 DEV_ID_GPS2 = 3
uint8 DEV_ID_SHELL = 10
uint8 DEV_ID_SERIAL_0 = 100
uint8 DEV_ID_SERIAL_1 = 101
uint8 DEV_ID_SERIAL_2 = 102
uint8 DEV_ID_SERIAL_3 = 103
uint8 DEV_ID_SERIAL_4 = 104
uint8 DEV_ID_SERIAL_5 = 105
uint8 DEV_ID_SERIAL_6 = 106
uint8 DEV_ID_SERIAL_7 = 107
uint8 DEV_ID_SERIAL_8 = 108
uint8 DEV_ID_SERIAL_9 = 109

uint8 CTRL_FLAG_REPLY = 1
uint8 CTRL_FLAG_RESPOND = 2
uint8 CTRL_FLAG_EXCLUSIVE = 4
uint8 CTRL_FLAG_BLOCKING = 8
uint8 CTRL_FLAG_MULTI = 16

uint8 DATA_SIZE = 70

uint64 timestamp	# Time since startup (ns)
uint8 dev_id		# Protocol device ID
uint8 ctrl_flag		# Control flags
uint16 timeout		# Timeout for reply (ms)
uint32 baud			# Baudrate in bits/s
uint8 count			# Data length
uint8[70] data		# Data field
uint8 target_system
uint8 target_component
