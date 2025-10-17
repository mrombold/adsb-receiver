package gdl90

// Message IDs
const (
	MsgHeartbeat          = 0x00
	MsgOwnshipReport      = 0x0A
	MsgOwnshipGeoAltitude = 0x0B
	MsgTrafficReport      = 0x14
	MsgUplinkData         = 0x07
	MsgStratuxHeartbeat   = 0xCC
)

// prepareMessage adds CRC and frame delimiters
func prepareMessage(msg []byte) []byte {
	crc := computeCRC(msg)
	msg = append(msg, byte(crc&0xFF), byte(crc>>8))
	
	escaped := []byte{0x7E}
	for _, b := range msg {
		if b == 0x7E || b == 0x7D {
			escaped = append(escaped, 0x7D, b^0x20)
		} else {
			escaped = append(escaped, b)
		}
	}
	escaped = append(escaped, 0x7E)
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

// encodeLatLon encodes latitude or longitude as 24-bit two's complement
// Per GDL90 spec: resolution = 180.0 / 2^23 = 180.0 / 8388608.0
func encodeLatLon(coord float64) [3]byte {
	// Encoding: value = coordinate / resolution = coordinate * (2^23 / 180)
	val := int32(coord * 8388608.0 / 180.0)
	
	// Extract 24 bits (will automatically handle two's complement for negative)
	return [3]byte{
		byte((val >> 16) & 0xFF),
		byte((val >> 8) & 0xFF),
		byte(val & 0xFF),
	}
}

// MakeHeartbeat creates a standard GDL90 heartbeat message (7 bytes + ID = 8 bytes before CRC)
func MakeHeartbeat(hasGPS, hasAHRS bool) []byte {
	msg := make([]byte, 7)
	msg[0] = MsgHeartbeat
	
	// Status Byte 1
	if hasGPS {
		msg[1] |= 0x80  // GPS position valid
	}
	
	// Status Byte 2
	if hasAHRS {
		msg[2] |= 0x01  // AHRS valid
	}
	
	// Timestamp (not implemented, set to 0)
	msg[3] = 0
	msg[4] = 0
	
	// Message counts
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
	
	msg[1] |= (1 << 2)  // Bit 2 set
	
	return prepareMessage(msg)
}

// MakeOwnshipReport creates an ownship position report (28 bytes)
func MakeOwnshipReport(lat, lon float64, altitude int, track, speed int) []byte {
	msg := make([]byte, 28)
	msg[0] = MsgOwnshipReport
	
	// Status byte (bit 0 = GPS valid)
	msg[1] = 0x01
	
	// Address type / participant address (bytes 2-5)
	// For ownship, these are typically 0
	msg[2] = 0
	msg[3] = 0
	msg[4] = 0
	msg[5] = 0
	
	// Latitude (24-bit two's complement, bytes 6-8)
	latBytes := encodeLatLon(lat)
	msg[6] = latBytes[0]
	msg[7] = latBytes[1]
	msg[8] = latBytes[2]
	
	// Longitude (24-bit two's complement, bytes 9-11)
	lonBytes := encodeLatLon(lon)
	msg[9] = lonBytes[0]
	msg[10] = lonBytes[1]
	msg[11] = lonBytes[2]
	
	// Altitude (12-bit): ((altitude_feet + 1000) / 25)
	var altEncoded int16
	if altitude < -1000 || altitude > 101350 {
		altEncoded = 0xFFF  // Invalid
	} else {
		altEncoded = int16((altitude + 1000) / 25)
		if altEncoded < 0 {
			altEncoded = 0
		}
		if altEncoded > 0xFFE {
			altEncoded = 0xFFE
		}
	}
	
	// Altitude is 12 bits spanning bytes 12-13
	msg[12] = byte(altEncoded >> 4)
	msg[13] = byte((altEncoded & 0x0F) << 4)
	
	// Misc byte (bits 0-3 of byte 13): TT field
	// 01 = True Track Angle
	msg[13] |= 0x09
	
	// NIC (Navigation Integrity Category) - 4 bits
	// NACp (Navigation Accuracy Category - Position) - 4 bits
	msg[14] = 0xA8
	
	// Horizontal velocity (12-bit, knots)
	speedEncoded := uint16(speed)
	if speedEncoded > 0xFFE {
		speedEncoded = 0xFFE
	}
	msg[15] = byte(speedEncoded >> 4)
	msg[16] = byte((speedEncoded & 0x0F) << 4)
	
	// Vertical velocity (12-bit, 64 fpm resolution)
	// Setting to 0 (no vertical velocity)
	msg[16] |= 0x00
	msg[17] = 0x00
	
	// Track/Heading (8-bit, 360/256 degrees resolution)
	trackEncoded := uint8(float64(track%360) * 256.0 / 360.0)
	msg[18] = trackEncoded
	
	// Emitter category (1 = light aircraft)
	msg[19] = 1
	
	// Call sign (8 bytes, ASCII, space-padded)
	callsign := "OWNSHIP "
	for i := 0; i < 8; i++ {
		if i < len(callsign) {
			msg[20+i] = callsign[i]
		} else {
			msg[20+i] = ' '
		}
	}
	
	// Emergency/priority code (byte 28, last byte before CRC)
	// This byte was missing in the original implementation!
	// 0 = No emergency
	// Note: In the original, msg was 28 bytes long but the spec requires 28 bytes PLUS this emergency byte
	// However, checking the C code more carefully...
	
	return prepareMessage(msg)
}

// MakeOwnshipGeoAltitude creates ownship geometric altitude message
func MakeOwnshipGeoAltitude(altitude int) []byte {
	msg := make([]byte, 5)
	msg[0] = MsgOwnshipGeoAltitude
	
	// Altitude in 5-foot increments (signed 16-bit)
	altEncoded := int16(altitude / 5)
	msg[1] = byte(altEncoded >> 8)
	msg[2] = byte(altEncoded)
	
	// Vertical warning indicator
	msg[3] = 0x00
	
	// Vertical figure of merit (0x7FFF = not available)
	msg[4] = 0x7F
	
	return prepareMessage(msg)
}

// MakeTrafficReport creates a traffic report message (28 bytes)
func MakeTrafficReport(icao uint32, lat, lon float64, altitude int, track, speed int) []byte {
	msg := make([]byte, 28)
	msg[0] = MsgTrafficReport
	
	// Alert status (bits 4-7) and address type (bits 0-3)
	// 0x0 = no alert, ADS-B with ICAO address
	msg[1] = 0x00
	
	// ICAO address (24 bits, bytes 2-4)
	msg[2] = byte((icao >> 16) & 0xFF)
	msg[3] = byte((icao >> 8) & 0xFF)
	msg[4] = byte(icao & 0xFF)
	
	// Latitude (24-bit two's complement, bytes 5-7)
	latBytes := encodeLatLon(lat)
	msg[5] = latBytes[0]
	msg[6] = latBytes[1]
	msg[7] = latBytes[2]
	
	// Longitude (24-bit two's complement, bytes 8-10)
	lonBytes := encodeLatLon(lon)
	msg[8] = lonBytes[0]
	msg[9] = lonBytes[1]
	msg[10] = lonBytes[2]
	
	// Altitude (12-bit): ((altitude_feet + 1000) / 25)
	var altEncoded int16
	if altitude < -1000 || altitude > 101350 {
		altEncoded = 0xFFF
	} else {
		altEncoded = int16((altitude + 1000) / 25)
		if altEncoded < 0 {
			altEncoded = 0
		}
		if altEncoded > 0xFFE {
			altEncoded = 0xFFE
		}
	}
	
	msg[11] = byte(altEncoded >> 4)
	msg[12] = byte((altEncoded & 0x0F) << 4)
	
	// Misc byte (TT field)
	msg[12] |= 0x09
	
	// NIC and NACp
	msg[13] = 0x88
	
	// Horizontal velocity (12-bit)
	speedEncoded := uint16(speed)
	if speedEncoded > 0xFFE {
		speedEncoded = 0xFFE
	}
	msg[14] = byte(speedEncoded >> 4)
	msg[15] = byte((speedEncoded & 0x0F) << 4)
	
	// Vertical velocity (12-bit, 64 fpm)
	msg[15] |= 0x08
	msg[16] = 0x00
	
	// Track/heading (8-bit)
	trackEncoded := uint8(float64(track%360) * 256.0 / 360.0)
	msg[17] = trackEncoded
	
	// Emitter category
	msg[18] = 1
	
	// Callsign (8 bytes, space-padded)
	for i := 19; i < 27; i++ {
		msg[i] = ' '
	}
	
	// Emergency status
	msg[27] = 0
	
	return prepareMessage(msg)
}

// CRC lookup table (same as C reference)
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