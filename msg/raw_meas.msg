uint8 RAW_MEAS_MAX_SATELLITES = 20

uint8 count			# Number of satellites of raw measurements
float64[20] cpMes	# the carrier phase measurement [cycles]
float64[20] prMes	# psuedorange measurement [m]
float32[20] doMes	# dopper measurement [Hz]
uint8[20] sv		# space vehicle number
int8[20] mesQI		# nav measurement quality indicator
int8[20] cno 		# signal strength C/N0 [dbHz]
uint8[20] lli		# los of lock indicator