package gdl90

import (
	"encoding/binary"
	"time"
	"adsb-receiver/pkg/types"
	"strconv"
)


// Message IDs
const (
	MsgHeartbeat          = 0x00
	MsgOwnshipReport      = 0x0A
	MsgOwnshipGeoAltitude = 0x0B
	MsgTrafficReport      = 0x14
	MsgUplinkData         = 0x07
	MsgStratuxHeartbeat   = 0xCC
	MsgAHRS               = 0x4C  // Stratux message
	SubIDAHRS             = 0x45  // AHRS message
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
// Per GDL90 spec: resolution = 180.0 / 2^23
func encodeLatLon(coord float64) [3]byte {
	// Scale factor: 2^23 / 180 = 46603.377778
	val := int32(coord * 46603.377778)
	
	return [3]byte{
		byte((val >> 16) & 0xFF),
		byte((val >> 8) & 0xFF),
		byte(val & 0xFF),
	}
}


// MakeHeartbeat creates a standard GDL90 heartbeat message (7 bytes total)
func MakeHeartbeat(hasGPS, hasAHRS bool) []byte {
    msg := make([]byte, 7)
    msg[0] = MsgHeartbeat
    
    // Status Byte 1 (byte 1)
    if hasGPS {
        msg[1] |= 0x80  // Bit 7: GPS position valid
    }
    msg[1] |= 0x01      // Bit 0: UAT Initialized
    
    // Status Byte 2 (byte 2)
    msg[2] |= 0x01      // Bit 0: UTC OK
    
    // Timestamp (seconds since midnight UTC)
    now := time.Now().UTC()
    secondsSinceMidnight := uint32(now.Hour()*3600 + now.Minute()*60 + now.Second())
    
    // Bit 16 of timestamp goes into bit 7 of Status Byte 2
    if secondsSinceMidnight & 0x10000 != 0 {
        msg[2] |= 0x80
    }
    
    // Bits 15-0 of timestamp go into bytes 3-4 (little-endian)
    binary.LittleEndian.PutUint16(msg[3:5], uint16(secondsSinceMidnight&0xFFFF))
    
    // Message counts (bytes 5-6)
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
	
	msg[1] |= (1 << 2)  // Bit 2 set  (protocol version)
	
	return prepareMessage(msg)
}


// MakeStratuxStatus creates the extended Stratux status message
func MakeStratuxStatus(hasGPS, hasAHRS bool) []byte {
	msg := make([]byte, 29)
	msg[0] = 'S'
	msg[1] = 'X'
	msg[2] = 1  // Message version
	msg[3] = 1  // Version code
	
	// Version string
	copy(msg[4:8], []byte{9, 9, 9, 9})
	
	// Hardware revision
	msg[8] = 0xFF
	msg[9] = 0xFF
	msg[10] = 0xFF
	msg[11] = 0xFF
	
	// Status flags
	if hasAHRS {
		msg[12] |= 0x01
		msg[13] |= (1 << 2)
	}

	if hasGPS {
		msg[13] |= (1<<7)
	}

	// not sure what byte 14 does
	// byte 15 is num radios and imu hardware

	// byte 16 is number of satellites locked
	msg[16] = byte(5)

	// byte 17 is number of satellites tracked
	msg[17] = byte(5)

	// byte 18,19 is number of uat traffic targets

	// byte 20,21 is number of 1090es traffic targets

	// byte 22,23 is number of uat messages per minute

	// byte 24,25 is number of 1090es messages per minute

	// byte 26,27 is cpu temperature uint16

	// byte 28 is number of ADS-B towers seen

	// remainder of bytes are list of ADS-B towers lat/lon as 24 bit encoded values (exceeds initial buffer but that's ok for some reason)

	return prepareMessage(msg)
}


// MakeAHRS creates a ForeFlight AHRS message (0x65 0x01)
// roll, pitch: in degrees (will be encoded as 0.1° units, range ±180°)
// heading: in degrees (0-359.9, will be encoded as 0.1° units)
// ias, tas: in knots (use 0x7FFF for invalid/unavailable)
//
// Per ForeFlight spec:
// - Positive roll = right wing down
// - Positive pitch = nose up
// - Heading: 0=North, 90=East, 180=South, 270=West
// - MSB of heading = 1 if magnetic heading
// - Send at 5 Hz for best results
func MakeAHRS(roll, pitch, heading float64) []byte {
	msg := make([]byte, 24)
	msg[0] = MsgAHRS          // 0x4C
	msg[1] = SubIDAHRS        // 0x45
	msg[2] = 0x01
	msg[3] = 0x01
	
	// Clamp roll and pitch to ±180°
	if roll > 180.0 {
		roll = 180.0
	} else if roll < -180.0 {
		roll = -180.0
	}
	
	if pitch > 180.0 {
		pitch = 180.0
	} else if pitch < -180.0 {
		pitch = -180.0
	}
	
	// Normalize heading to 0-360
	for heading < 0 {
		heading += 360.0
	}
	for heading >= 360.0 {
		heading -= 360.0
	}
	
	// Encode roll: signed int16, units of 0.1°
	rollEncoded := int16(roll * 10.0)
	msg[4] = byte((rollEncoded)>>8 & 0xFF)
	msg[5] = byte(rollEncoded & 0xFF)

		
	// Encode pitch: signed int16, units of 0.1°
	pitchEncoded := int16(pitch * 10.0)
	msg[6] = byte((pitchEncoded)>>8 & 0xFF)
	msg[7] = byte(pitchEncoded & 0xFF)
	
	// Encode heading: unsigned int16, units of 0.1°
	// Set MSB = 1 for magnetic heading (we're using true heading, so MSB = 0)
	headingEncoded := uint16(heading * 10.0)
	msg[8] = byte((headingEncoded)>>8 & 0xFF)
	msg[9] = byte(headingEncoded & 0xFF)
	




	slip_skid := uint16(0)
	yaw_rate := uint16(0)
	g := uint16(0)
	airspeed := uint16(0)
	palt := uint16(0)
	vs := uint16(0)

	// Slip/skid.
	msg[10] = byte((slip_skid >> 8) & 0xFF)
	msg[11] = byte(slip_skid & 0xFF)

	// Yaw rate.
	msg[12] = byte((yaw_rate >> 8) & 0xFF)
	msg[13] = byte(yaw_rate & 0xFF)

	// "G".
	msg[14] = byte((g >> 8) & 0xFF)
	msg[15] = byte(g & 0xFF)

	// Indicated Airspeed
	msg[16] = byte((airspeed >> 8) & 0xFF)
	msg[17] = byte(airspeed & 0xFF)

	// Pressure Altitude
	msg[18] = byte((palt >> 8) & 0xFF)
	msg[19] = byte(palt & 0xFF)

	// Vertical Speed
	msg[20] = byte((vs >> 8) & 0xFF)
	msg[21] = byte(vs & 0xFF)

	// Reserved
	msg[22] = 0x7F
	msg[23] = 0xFF

	return prepareMessage(msg)
}


// MakeOwnshipReport creates an ownship position report (28 bytes)
// lat, lon in degrees, altitude in ft (pressure alt), track in degrees from GPS, speed in knots from GPS
func MakeOwnshipReport(lat, lon float64, altitude, track, speed, vertvel int) []byte {
	msg := make([]byte, 28)
	msg[0] = MsgOwnshipReport
	
	// Status byte
	msg[1] = 0x00 // No alert, ADS-B with ICAO address
	
	// Address type (0 = ADS-B with ICAO address)
	msg[1] |= 0x00
	
	// ICAO address (3 bytes) - N8200X = 0xAB3349, N6296E = 0xA83ADB
	msg[2] = 0xAB
	msg[3] = 0x33
	msg[4] = 0x49
	
	// Latitude (3 bytes, signed)
	latBytes := encodeLatLon(lat)
	copy(msg[5:8], latBytes[:])
	
	// Longitude (3 bytes, signed)
	lonBytes := encodeLatLon(lon)
	copy(msg[8:11], lonBytes[:])
	
	// PRESSURE Altitude (12 bits) - encoded as (alt + 1000) / 25  
	altEncoded := uint16((altitude + 1000) / 25)
	msg[11] = byte((altEncoded >> 4) & 0xFF)
	msg[12] = byte((altEncoded & 0x0F) << 4)
	
	// Type of data in "tt" field, updated or extrap, and ground/airborne
	msg[12] |= 0x09 // 0b1001 = airborne, "tt"=true track angle
	
	// NIC
	msg[13] = 0x88 // NIC=8, NACp=8 <0.1nm, <0.05nm
	
	// Horizontal velocity (12 bits unsigned) - speed in knots
	speedEncoded := uint16(speed)
	msg[14] = byte((speedEncoded >> 4) & 0xFF)
	msg[15] = byte((speedEncoded & 0x0F) << 4)
	

	// Vertical velocity (12 bits signed) - in units of 64 fpm
	// 0x800 for no vertical rate info available
	// Field spans: upper 4 bits of msg[15] and all 8 bits of msg[16]

	// Encode as signed 12-bit value in units of 64 fpm
	vvEncoded := int16(vertvel / 64)
	
	// Clamp to valid range: -510 to +509 (0xE02 to 0x1FD in the spec)
	if vvEncoded > 509 {
		vvEncoded = 509  // 0x1FD
	} else if vvEncoded < -510 {
		vvEncoded = -510 // 0xE02
	}
			
	// Upper 4 bits go into lower nibble of msg[15]
	msg[15] |= byte((vvEncoded >> 8) & 0x0F)
	
	// Lower 8 bits go into msg[16]
	msg[16] = byte(vvEncoded & 0xFF)


	// Track (8 bits) - track/1.40625   (track / (360/256))
	trackEncoded := uint8(float64(track) / 1.40625)
	msg[17] = trackEncoded
	
	// Emitter category (8 = light aircraft)
	msg[18] = 0x01
	
	// Call sign (8 bytes) - "N12345  "
	copy(msg[19:27], "N8200X  ")
	
	// Emergency/priority code
	msg[27] = 0x00
	
	return prepareMessage(msg)
}


// MakeOwnshipGeoAltitude creates ownship geometric altitude message (eight above WGS-84 ellipsoid)
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
	// 0x0A = 10 meters
	msg[4] = 0x0A
	
	return prepareMessage(msg)
}


// MakeTrafficReport creates a traffic report message (28 bytes)
func MakeTrafficReport(ac types.Aircraft) []byte {
	msg := make([]byte, 28)
	msg[0] = MsgTrafficReport
	
	// Alert status (bits 4-7) and address type (bits 0-3)
	// 0x0 = no alert, ADS-B with ICAO address
	// Byte 1: Alert status (bits 4-7) and address type (bits 0-3)
	// Bits 4-7: Traffic Alert Status (s)
	msg[1] = (0x0 << 4) & 0xF0
	msg[1] |= 0 & 0x0F
	
	// ICAO address (24 bits, bytes 2-4)
	icao := parseICAO(ac.ICAO)
	msg[2] = byte((icao >> 16) & 0xFF)
	msg[3] = byte((icao >> 8) & 0xFF)
	msg[4] = byte(icao & 0xFF)
	
	// Latitude (24-bit two's complement, bytes 5-7)
	latBytes := encodeLatLon(ac.Lat)
	msg[5] = latBytes[0]
	msg[6] = latBytes[1]
	msg[7] = latBytes[2]
	
	// Longitude (24-bit two's complement, bytes 8-10)
	lonBytes := encodeLatLon(ac.Lon)
	msg[8] = lonBytes[0]
	msg[9] = lonBytes[1]
	msg[10] = lonBytes[2]
	
	// Altitude (12-bit): ((altitude_feet + 1000) / 25)
	var altEncoded int16
	if ac.Altitude < -1000 || ac.Altitude > 101350 {
		altEncoded = 0xFFF
	} else {
		altEncoded = int16((ac.Altitude + 1000) / 25)
		if altEncoded < 0 {
			altEncoded = 0
		}
		if altEncoded > 0xFFE {
			altEncoded = 0xFFE
		}
	}
	
	msg[11] = byte((altEncoded >> 4) & 0xFF)
	msg[12] = byte((altEncoded & 0x0F) << 4)
	
	// Msg 12 lower nibble
	// Bit 3: 1=Airborne, 0=Ground
	// Bit 2: 1=Extrapolated, 0=Updated
	// Bit 1, 0: 00 = invalid, 01 = true trak, 10=hdg mag, 11=hdg true
	if !ac.OnGround {
    	msg[12] |= (1 << 3)
	}
	msg[12] |= 0 << 2  // report is updated
	msg[12] |= 0b0001  // ground track / heading from ADS-B are true track
	

	// NIC and NAC not ouput from ADS-B dump1090.  Assume value is 9 for each
	// Byte 13 (bits 4-7): NIC - Navigation Integrity Category (i)
	msg[13] = (9 << 4) & 0xF0
	
	// Byte 13 (bits 0-3): NACp - Navigation Accuracy Category for Position (a)
	msg[13] |= 9 & 0x0F

	
	// Horizontal velocity (12-bit)
	speedEncoded := uint16(ac.Speed)
	if speedEncoded > 0xFFE {
		speedEncoded = 0xFFE
	}
	msg[14] = byte(speedEncoded >> 4)
	msg[15] = byte((speedEncoded & 0x0F) << 4)
	
	// Vertical velocity (12 bits signed) - in units of 64 fpm
	// 0x800 for no vertical rate info available
	// Field spans: upper 4 bits of msg[15] and all 8 bits of msg[16]

	if !ac.HasVertVel { 
		msg[15] |= 0x08
		msg[16] = 0x00
	} else {
		// Encode as signed 12-bit value in units of 64 fpm
		vvEncoded := int16(ac.VertVel / 64)
		
		// Clamp to valid range: -510 to +509 (0xE02 to 0x1FD in the spec)
		if vvEncoded > 509 {
			vvEncoded = 509  // 0x1FD
		} else if vvEncoded < -510 {
			vvEncoded = -510 // 0xE02
		}
				
		// Upper 4 bits go into lower nibble of msg[15]
		msg[15] |= byte((vvEncoded >> 8) & 0x0F)
		
		// Lower 8 bits go into msg[16]
		msg[16] = byte(vvEncoded & 0xFF)
	}
	
	// Track (8 bits) - track/1.40625   (track / (360/256))
	trackEncoded := uint8(float64(ac.Track) / 1.40625)
	msg[17] = trackEncoded
	

	// Byte 18: Emitter category (ee)
	msg[18] = 0x01  // dump1090 doesn't provide emitter category
	
	// Callsign (8 bytes, space-padded)
	cs := ac.Callsign
	if len(cs) > 8 {
		cs = cs[:8]
	}
	for len(cs) < 8 {
		cs += " "
	}
	copy(msg[19:27], cs)
	
    // Byte 27: Emergency/Priority code
    if ac.Emergency {
        switch ac.Squawk {
        case "7700":
            msg[27] = 1  // General emergency
        case "7600":
            msg[27] = 4  // No communications
        case "7500":
            msg[27] = 5  // Unlawful interference
        default:
            msg[27] = 1  // General emergency
        }
    } else {
        msg[27] = 0  // No emergency
    }
	
	return prepareMessage(msg)
}


// MakeUplinkData wraps a raw UAT uplink frame in GDL90 format
// This passes through FIS-B weather data as Stratux does
func MakeUplinkData(uatFrame []byte) []byte {
	// GDL90 Message ID 0x07: Uplink Data
	// Format: [0x07][raw UAT frame data]
	msg := make([]byte, 1+3+len(uatFrame))
	msg[0] = 0x07 // Message Type: Uplink Data
	msg[1] = 0xFF // Time of reception, 0xFFFFFF if invalid/not available
	msg[2] = 0xFF //
	msg[3] = 0xFF //
	// Copy raw UAT frame directly
	copy(msg[4:], uatFrame)
	
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


// parseICAO converts hex string ICAO address to uint32
// Example: "AB4549" -> 0xAB4549 (11223369 decimal)
func parseICAO(icaoHex string) uint32 {
    if icao, err := strconv.ParseUint(icaoHex, 16, 32); err == nil {
        return uint32(icao)
    }
    return 0
}