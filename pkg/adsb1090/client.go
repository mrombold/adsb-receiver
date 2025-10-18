package adsb1090

import (
	"bufio"
	"log"
	"net"
	"strconv"
	"strings"
	"time"
)

// Aircraft represents an aircraft from ADS-B
type Aircraft struct {
	ICAO      string
	Callsign  string
	Altitude  int
	Lat       float64
	Lon       float64
	Track     int
	Speed     int
	Timestamp time.Time
	
	// Track which fields are valid in this update
	HasCallsign bool
	HasPosition bool
	HasAltitude bool
	HasSpeed    bool
	HasTrack    bool
}

// Client connects to dump1090 and reads BaseStation messages
type Client struct {
	addr string
}

// NewClient creates a new dump1090 client
func NewClient(addr string) *Client {
	return &Client{addr: addr}
}

// Connect and read messages, sending aircraft updates to the channel
func (c *Client) Read(updates chan<- Aircraft) error {
	for {
		conn, err := net.Dial("tcp", c.addr)
		if err != nil {
			log.Printf("dump1090 connection failed: %v, retrying in 5s", err)
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
		log.Printf("Disconnected from dump1090, reconnecting...")
		time.Sleep(2 * time.Second)
	}
}

// parseBaseStation parses a BaseStation format message
// MSG,type,session,aircraft,icao,flight,date,time,date,time,callsign,altitude,speed,track,lat,lon,...
func (c *Client) parseBaseStation(line string) (Aircraft, bool) {
	fields := strings.Split(line, ",")
	
	if len(fields) < 11 || fields[0] != "MSG" {
		return Aircraft{}, false
	}
	
	msgType := fields[1]
	icao := strings.TrimSpace(fields[4])
	
	if icao == "" {
		return Aircraft{}, false
	}
	
	ac := Aircraft{
		ICAO:      icao,
		Timestamp: time.Now(),
	}
	
	// MSG Type 1: Callsign (identification)
	// Field 10 has callsign
	if msgType == "1" && len(fields) >= 11 {
		if fields[10] != "" {
			ac.Callsign = strings.TrimSpace(fields[10])
			ac.HasCallsign = true
		}
		return ac, true
	}
	
	// MSG Type 3: Position data with altitude
	// Fields: 10=callsign, 11=altitude, 14=lat, 15=lon
	if msgType == "3" && len(fields) >= 16 {
		// Callsign (sometimes present)
		if fields[10] != "" {
			ac.Callsign = strings.TrimSpace(fields[10])
			ac.HasCallsign = true
		}
		
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
		
		// Return if we got anything useful
		if ac.HasPosition || ac.HasAltitude {
			return ac, true
		}
	}
	
	// MSG Type 4: Velocity (altitude, speed, track)
	// Fields: 11=altitude, 12=speed, 13=track
	if msgType == "4" && len(fields) >= 14 {
		hasData := false
		
		// Altitude
		if fields[11] != "" {
			if alt, err := strconv.Atoi(fields[11]); err == nil {
				ac.Altitude = alt
				ac.HasAltitude = true
				hasData = true
			}
		}
		
		// Speed
		if fields[12] != "" {
			if spd, err := strconv.Atoi(fields[12]); err == nil {
				ac.Speed = spd
				ac.HasSpeed = true
				hasData = true
			}
		}
		
		// Track
		if fields[13] != "" {
			if trk, err := strconv.Atoi(fields[13]); err == nil {
				ac.Track = trk
				ac.HasTrack = true
				hasData = true
			}
		}
		
		if hasData {
			return ac, true
		}
	}
	
	// MSG Type 5: Altitude only
	if msgType == "5" && len(fields) >= 12 {
		if fields[11] != "" {
			if alt, err := strconv.Atoi(fields[11]); err == nil {
				ac.Altitude = alt
				ac.HasAltitude = true
				return ac, true
			}
		}
	}
	
	return Aircraft{}, false
}