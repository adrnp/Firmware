uint8 HUNT_STATE_OFF = 0		# hunt has been turned off
uint8 HUNT_STATE_START = 1		# hunt just started, need to go to starting position
uint8 HUNT_STATE_WAIT = 2		# waiting for next command from tracking
uint8 HUNT_STATE_MOVE = 3		# executing a move command from tracking
uint8 HUNT_STATE_ROTATE = 4		# executing a rotate command from tracking

uint64 timestamp			# the current timestamp
uint8 hunt_mode_state		# which mode the hunter is currently in
