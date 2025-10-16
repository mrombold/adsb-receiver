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
// Format: MSG,type,session,aircraft,icao,flight,date,time,date,time,callsign,altitude,speed,track,lat,lon,...
func (c *Client) parseBaseStation(line string) (Aircraft, bool) {
	fields := strings.Split(line, ",")
	
	// Need at least the basic fields
	if len(fields) < 11 || fields[0] != "MSG" {
		return Aircraft{}, false
	}
	
	msgType := fields[1]
	icao := strings.TrimSpace(fields[4])
	
	ac := Aircraft{
		ICAO:      icao,
		Timestamp: time.Now(),
	}
	
	// MSG type 3 has position data
	if msgType == "3" && len(fields) >= 16 {
		if fields[10] != "" {
			if callsign := strings.TrimSpace(fields[10]); callsign != "" {
				ac.Callsign = callsign
			}
		}
		
		if fields[11] != "" {
			ac.Altitude, _ = strconv.Atoi(fields[11])
		}
		
		if fields[14] != "" && fields[15] != "" {
			ac.Lat, _ = strconv.ParseFloat(fields[14], 64)
			ac.Lon, _ = strconv.ParseFloat(fields[15], 64)
		}
		
		if fields[12] != "" {
			ac.Speed, _ = strconv.Atoi(fields[12])
		}
		
		if fields[13] != "" {
			ac.Track, _ = strconv.Atoi(fields[13])
		}
		
		return ac, true
	}
	
	// MSG type 4 has altitude and speed
	if msgType == "4" && len(fields) >= 13 {
		if fields[11] != "" {
			ac.Altitude, _ = strconv.Atoi(fields[11])
		}
		
		if fields[12] != "" {
			ac.Speed, _ = strconv.Atoi(fields[12])
		}
		
		return ac, true
	}
	
	return Aircraft{}, false
}
