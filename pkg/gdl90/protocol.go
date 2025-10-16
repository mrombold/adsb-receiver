package gdl90


// Message IDs
const (
	MsgHeartbeat          = 0x00
	MsgOwnshipReport      = 0x0A
	MsgOwnshipGeoAltitude = 0x0B
	MsgTrafficReport      = 0x14
	MsgUplinkData         = 0x07
	MsgStratuxHeartbeat   = 0xCC // For Garmin Pilot compatibility
)

// prepareMessage adds CRC and frame delimiters
func prepareMessage(msg []byte) []byte {
	// Calculate CRC
	crc := computeCRC(msg)
	msg = append(msg, byte(crc&0xFF), byte(crc>>8))
	
	// Escape special characters and add frame delimiters
	escaped := []byte{0x7E} // Frame start
	for _, b := range msg {
		if b == 0x7E || b == 0x7D {
			escaped = append(escaped, 0x7D, b^0x20)
		} else {
			escaped = append(escaped, b)
		}
	}
	escaped = append(escaped, 0x7E) // Frame end
	return escaped
}

// computeCRC calculates the GDL90 CRC
func computeCRC(msg []byte) uint16 {
	crc := uint16(0)
	for _, b := range msg {
		crc = crcTable[crc>>8] ^ (crc << 8) ^ uint16(b)
	}
	return crc
}

// MakeHeartbeat creates a standard GDL90 heartbeat message
func MakeHeartbeat(hasGPS, hasAHRS bool) []byte {
	msg := make([]byte, 7)
	msg[0] = MsgHeartbeat
	
	// Status byte 1
	if hasGPS {
		msg[1] |= 0x80 // GPS position valid
	}
	
	// Status byte 2
	if hasAHRS {
		msg[2] |= 0x01 // AHRS valid
	}
	
	// Timestamp (placeholder - can add actual GPS time later)
	msg[3] = 0
	msg[4] = 0
	
	// Message counts (placeholder)
	msg[5] = 0
	msg[6] = 0
	
	return prepareMessage(msg)
}

// MakeStratuxHeartbeat creates a Stratux identification heartbeat
func MakeStratuxHeartbeat(hasGPS, hasAHRS bool) []byte {
	msg := []byte{MsgStratuxHeartbeat, 0}
	
	if hasGPS {
		msg[1] |= 0x02
	}
	if hasAHRS {
		msg[1] |= 0x01
	}
	
	// Protocol version (1)
	msg[1] |= (1 << 2)
	
	return prepareMessage(msg)
}


// MakeTrafficReport creates a traffic report message
func MakeTrafficReport(icao uint32, lat, lon float64, altitude int, track, speed int) []byte {
	msg := make([]byte, 28)
	msg[0] = MsgTrafficReport
	
	// Traffic alert status (0 = no alert) and address type (0 = ADS-B with ICAO)
	msg[1] = 0x00
	
	// ICAO address (24 bits)
	msg[2] = byte(icao >> 16)
	msg[3] = byte(icao >> 8)
	msg[4] = byte(icao)
	
	// Latitude (encoded as integer)
	latInt := int32(lat * 8388608.0 / 180.0)
	msg[5] = byte(latInt >> 16)
	msg[6] = byte(latInt >> 8)
	msg[7] = byte(latInt)
	
	// Longitude (encoded as integer)
	lonInt := int32(lon * 8388608.0 / 180.0)
	msg[8] = byte(lonInt >> 16)   // FIXED: was msg[9]
	msg[9] = byte(lonInt >> 8)    // FIXED: was msg[10]
	msg[10] = byte(lonInt)        // FIXED: was msg[11]
	
	// Altitude: (alt/25) + 40 offset
	var alt int16
	if altitude < -1000 || altitude > 101350 {
		alt = 0x0FFF
	} else {
		alt = int16((altitude / 25) + 40)
	}
	msg[11] = byte((alt & 0xFF0) >> 4)
	msg[12] = byte((alt & 0x00F) << 4)
	
	// Miscellaneous byte (lower 4 bits of msg[12])
	// 0x01 = true track valid, 0x08 = airborne
	msg[12] = msg[12] | 0x09
	
	// NIC (upper 4 bits) and NACp (lower 4 bits)
	msg[13] = 0x88  // NIC=8, NACp=8
	
	// Horizontal velocity (12-bit, in knots)
	speedEncoded := uint16(speed) & 0x0FFF
	msg[14] = byte((speedEncoded & 0xFF0) >> 4)
	msg[15] = byte((speedEncoded & 0x00F) << 4)
	
	// Vertical velocity (12-bit, 64 fpm resolution) - set to 0
	msg[15] = msg[15] | 0x08  // 0x800 >> 8
	msg[16] = 0x00
	
	// Track (degrees / 1.4)
	trackEncoded := uint8(float64(track) * 256.0 / 360.0)
	msg[17] = trackEncoded
	
	// Emitter category (1 = light aircraft)
	msg[18] = 1
	
	// Callsign (8 bytes, space-padded)
	for i := 19; i < 27; i++ {
		msg[i] = ' '
	}
	
	// Priority/emergency status
	msg[27] = 0
	
	return prepareMessage(msg)
}

// CRC lookup table
var crcTable = [256]uint16{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
}


// MakeOwnshipReport creates an ownship position report
func MakeOwnshipReport(lat, lon float64, altitude int, track, speed int) []byte {
	msg := make([]byte, 28)
	msg[0] = MsgOwnshipReport
	
	// Status byte (airborne with valid position)
	msg[1] = 0x09 // 0x08 = airborne, 0x01 = GPS valid
	
	// Address type (1 = non-ICAO)
	msg[2] = 1
	
	// Address (can be zeros for ownship)
	msg[3] = 0
	msg[4] = 0
	msg[5] = 0
	
	// Latitude (encoded as integer)
	latInt := int32(lat * 8388608.0 / 180.0)
	msg[6] = byte(latInt >> 16)
	msg[7] = byte(latInt >> 8)
	msg[8] = byte(latInt)
	
	// Longitude (encoded as integer)
	lonInt := int32(lon * 8388608.0 / 180.0)
	msg[9] = byte(lonInt >> 16)
	msg[10] = byte(lonInt >> 8)
	msg[11] = byte(lonInt)
	
	// Altitude 
	altEncoded := int16((altitude / 25) + 40)  // CORRECT
	msg[12] = byte(altEncoded >> 4)
	msg[13] = byte((altEncoded&0x0F)<<4) | 0x09 // 0x09 = airborne + valid altitude
	
	// Misc byte
	msg[14] = 0x01 // NIC = 1
	
	// NACp
	msg[15] = 8
	
	// Horizontal velocity
	speedEncoded := uint16(speed)
	msg[16] = byte(speedEncoded >> 4)
	msg[17] = byte((speedEncoded & 0x0F) << 4)
	
	// Vertical velocity (0 for stationary)
	msg[18] = 0x80
	
	// Track/heading
	trackEncoded := uint8(float64(track) * 256.0 / 360.0)
	msg[19] = trackEncoded
	
	// Emitter category (1 = light aircraft)
	msg[20] = 1
	
	// Callsign (8 bytes)
	copy(msg[21:28], []byte("OWNSHIP"))
	
	return prepareMessage(msg)
}

// MakeOwnshipGeoAltitude creates ownship geometric altitude message
func MakeOwnshipGeoAltitude(altitude int) []byte {
	msg := make([]byte, 5)
	msg[0] = MsgOwnshipGeoAltitude
	
	// Altitude in 5-foot increments, offset by 5000 feet
	altEncoded := int16((altitude + 5000) / 5)
	msg[1] = byte(altEncoded >> 8)
	msg[2] = byte(altEncoded)
	
	// Vertical metrics (0x0001 = valid)
	msg[3] = 0x00
	msg[4] = 0x01
	
	return prepareMessage(msg)
}
