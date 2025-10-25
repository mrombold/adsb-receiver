package adsb1090

import (
	"bufio"
	"log"
	"net"
	"strconv"
	"strings"
	"time"
	"adsb-receiver/pkg/types"

)

// Aircraft represents an aircraft from ADS-B
// found in ../types/aircraft.go

// Client connects to dump1090 and reads BaseStation messages
type Client struct {
	addr string
}

// NewClient creates a new dump1090 client
func NewClient(addr string) *Client {
	return &Client{addr: addr}
}

// Connect and read messages, sending aircraft updates to the channel
func (c *Client) Read(updates chan<- types.Aircraft) error {
	for {
		conn, err := net.Dial("tcp", c.addr)
		if err != nil {
			time.Sleep(5 * time.Second)
			continue
		}
		
		log.Printf("Connected to dump1090 at %s", c.addr)
		
		scanner := bufio.NewScanner(conn)
		for scanner.Scan() {
			line := scanner.Text()
			if ac, ok := c.parseBaseStation(line); ok {
				updates <- ac
			}
		}
		
		if err := scanner.Err(); err != nil {
			log.Printf("dump1090 read error: %v", err)
		}
		
		conn.Close()
		time.Sleep(2 * time.Second)
	}
}

// parseBaseStation parses a BaseStation format message
// Fields (0-indexed):
//   0: MSG
//   1: Transmission type (1-8)
//   4: ICAO hex
//   10: Callsign
//   11: Altitude
//   12: Ground speed
//   13: Track
//   14: Latitude
//   15: Longitude
//   16: Vertical rate
//   17: Squawk
//   18: Alert (squawk change)
//   19: Emergency
//   20: SPI (Special Position Indicator) "Ident"
//   21: is_on_ground
func (c *Client) parseBaseStation(line string) (types.Aircraft, bool) {
	fields := strings.Split(line, ",")
	
	if len(fields) < 11 || fields[0] != "MSG" {
		return types.Aircraft{}, false
	}
	
	msgType := fields[1]
	icao := strings.ToUpper(strings.TrimSpace(fields[4]))

	
	if icao == "" {
		return types.Aircraft{}, false
	}
	
	ac := types.Aircraft{
		ICAO:      icao,
		Timestamp: time.Now(),
	}

	switch msgType {

	case "1":  
		if fields[10] != "" {
			ac.Callsign = strings.ToUpper(strings.TrimSpace(fields[10]))
			ac.HasCallsign = true
		}
		return ac, true

	case "2":		
		// Altitude
		if fields[11] != "" {
			if alt, err := strconv.Atoi(fields[11]); err == nil {
				ac.Altitude = alt
				ac.HasAltitude = true
			}
		}

		// Speed (sometimes present)
		if fields[12] != "" {
			if spd, err := strconv.Atoi(fields[12]); err == nil {
				ac.Speed = spd
				ac.HasSpeed = true
			}
		}
		
		// Track (sometimes present)
		if fields[13] != "" {
			if trk, err := strconv.Atoi(fields[13]); err == nil {
				ac.Track = trk
				ac.HasTrack = true
			}
		}
		// Position
		if fields[14] != "" && fields[15] != "" {
			if lat, err := strconv.ParseFloat(fields[14], 64); err == nil {
				if lon, err := strconv.ParseFloat(fields[15], 64); err == nil {
					ac.Lat = lat
					ac.Lon = lon
					ac.HasPosition = true
				}
			}
		}
				
		// On Ground
		if fields[21] != "" && fields[21] != "0" {
			ac.OnGround = (fields[21] == "1" || fields[21] == "-1")
			ac.HasOnGround = true
		}
		return ac, true

	case "3":		
		// Altitude
		if fields[11] != "" {
			if alt, err := strconv.Atoi(fields[11]); err == nil {
				ac.Altitude = alt
				ac.HasAltitude = true
			}
		}

		// Position
		if fields[14] != "" && fields[15] != "" {
			if lat, err := strconv.ParseFloat(fields[14], 64); err == nil {
				if lon, err := strconv.ParseFloat(fields[15], 64); err == nil {
					ac.Lat = lat
					ac.Lon = lon
					ac.HasPosition = true
				}
			}
		}
		
		// Alert (squawk change)
		if fields[18] != "" && fields[18] != "0" {
			ac.Alert = (fields[18] == "1" || fields[18] == "-1")
			ac.HasAlert = true
		}
		
		// Emergency
		if fields[19] != "" && fields[19] != "0" {
			ac.Emergency = (fields[19] == "1" || fields[19] == "-1")
			ac.HasEmergency = true
		}
		
		// SPI
		if fields[20] != "" && fields[20] != "0" {
			ac.SPI = (fields[20] == "1" || fields[20] == "-1")
			ac.HasSPI = true
		}
		
		// On Ground
		if fields[21] != "" && fields[21] != "0" {
			ac.OnGround = (fields[21] == "1" || fields[21] == "-1")
			ac.HasOnGround = true
		}
		return ac, true

	case "4":
		// Ground Speed
		if fields[12] != "" {
			if spd, err := strconv.Atoi(fields[12]); err == nil {
				ac.Speed = spd
				ac.HasSpeed = true
			}
		}
		
		// Track
		if fields[13] != "" {
			if trk, err := strconv.Atoi(fields[13]); err == nil {
				ac.Track = trk
				ac.HasTrack = true
			}
		}
		
		// Vertical rate
		if fields[16] != "" {
			if vr, err := strconv.Atoi(fields[16]); err == nil {
				ac.VertVel = vr
				ac.HasVertVel = true
			}
		}
		return ac, true

	case "5":
		// Altitude
		if fields[11] != "" {
			if alt, err := strconv.Atoi(fields[11]); err == nil {
				ac.Altitude = alt
				ac.HasAltitude = true
			}
		}
				
		// Alert
		if fields[18] != "" && fields[18] != "0" {
			ac.Alert = (fields[18] == "1" || fields[18] == "-1")
			ac.HasAlert = true
		}
				
		// SPI
		if fields[20] != "" && fields[20] != "0" {
			ac.SPI = (fields[20] == "1" || fields[20] == "-1")
			ac.HasSPI = true
		}

		// On Ground
		if fields[21] != "" && fields[21] != "0" {
			ac.OnGround = (fields[21] == "1" || fields[21] == "-1")
			ac.HasOnGround = true
		}
		return ac, true

	case "6":
		// Altitude
		if fields[11] != "" {
			if alt, err := strconv.Atoi(fields[11]); err == nil {
				ac.Altitude = alt
				ac.HasAltitude = true
			}
		}
		
		// Squawk
		if fields[17] != "" {
			ac.Squawk = strings.TrimSpace(fields[17])
			ac.HasSquawk = true
		}
		
		// Alert
		if fields[18] != "" && fields[18] != "0" {
			ac.Alert = (fields[18] == "1" || fields[18] == "-1")
			ac.HasAlert = true
		}
		
		// Emergency
		if fields[19] != "" && fields[19] != "0" {
			ac.Emergency = (fields[19] == "1" || fields[19] == "-1")
			ac.HasEmergency = true
		}
		
		// SPI
		if fields[20] != "" && fields[20] != "0" {
			ac.SPI = (fields[20] == "1" || fields[20] == "-1")
			ac.HasSPI = true
		}
		// On Ground
		if fields[21] != "" && fields[21] != "0" {
			ac.OnGround = (fields[21] == "1" || fields[21] == "-1")
			ac.HasOnGround = true
		}
		return ac, true

	case "7":
		// Altitude
		if fields[11] != "" {
			if alt, err := strconv.Atoi(fields[11]); err == nil {
				ac.Altitude = alt
				ac.HasAltitude = true
			}
		}
		
		// On Ground
		if fields[21] != "" && fields[21] != "0" {
			ac.OnGround = (fields[21] == "1" || fields[21] == "-1")
			ac.HasOnGround = true
		}
		return ac, true

	case "8":
		// On Ground
		if fields[21] != "" && fields[21] != "0" {
			ac.OnGround = (fields[21] == "1" || fields[21] == "-1")
			ac.HasOnGround = true
		}
		return ac, true
		
	default:
		return types.Aircraft{}, false
	}
	
}

