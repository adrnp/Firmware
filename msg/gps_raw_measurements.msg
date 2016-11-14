uint8 GPS_RAW_MEASUREMENTS_MAX_SATELLITES = 20

uint32 tow		# gps time of week of the measurement [ms]
uint16 wn 		# gps week number
int8 nobs		# number of valid observations
uint32[20] psuedorange	# psuedorange measurement for this satellite [cm]
int32[20] carrier_i		# carrier phase whole cycles [cycles] 
uint8[20] carrier_f		# carrier phase fractional part [cycles / 256]
uint8[20] cn0			# carrier-to-Noise density [dB Hz * 4] 
uint16[20] lock			# lock indicator. This value changes whenever a satellite signal has lost and regained lock, indicating that the carrier phase ambiguity may have changed.
uint8[20] prn			# PRN code of the satellite (may be off by 1)