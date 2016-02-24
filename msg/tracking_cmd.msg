uint8 HUNT_CMD_TRAVEL = 1
uint8 HUNT_CMD_ROTATE = 2
uint8 HUNT_CMD_FINISH = 3

uint64 timestamp	# timestamp of the command
uint8 cmd_type 		# type of command
uint16 cmd_id		# id of the command, an incremental number sent from tracking (mainly for double checking 
float32 north			# north distance or angle or rotation, depending on command type 
float32 east			# east distance 
float32 yaw_angle		# a third parameter, possible desired yaw to maintain during flight... 
float32 altitude		# altitude in m AMSL