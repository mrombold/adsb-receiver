package gps

import (
	"bufio"
	"encoding/json"
	"log"
	"net"
	"time"
)

// Position represents GPS position data
type Position struct {
	Lat       float64
	Lon       float64
	Altitude  float64
	Track     float64
	Speed     float64
	Climb     float64
	Mode      int
	Timestamp time.Time
}

// TPVMessage is the JSON structure from gpsd
type TPVMessage struct {
	Class  string  `json:"class"`
	Mode   int     `json:"mode"`
	Time   string  `json:"time"`
	Lat    float64 `json:"lat"`
	Lon    float64 `json:"lon"`
	Alt    float64 `json:"alt"`
	Track  float64 `json:"track"`
	Speed  float64 `json:"speed"`
	Climb  float64 `json:"climb"`
}

// Client connects to gpsd
type Client struct {
	addr string
}

// NewClient creates a new gpsd client
func NewClient(addr string) *Client {
	return &Client{addr: addr}
}

// Read connects to gpsd and reads position updates
func (c *Client) Read(updates chan<- Position) error {
	for {
		conn, err := net.Dial("tcp", c.addr)
		if err != nil {
			log.Printf("gpsd connection failed: %v, retrying in 5s", err)
			time.Sleep(5 * time.Second)
			continue
		}
		
		log.Printf("Connected to gpsd at %s", c.addr)
		
		// Enable JSON streaming with watch command
		conn.Write([]byte("?WATCH={\"enable\":true,\"json\":true}\n"))
		
		scanner := bufio.NewScanner(conn)
		for scanner.Scan() {
			line := scanner.Bytes()
			
			var msg TPVMessage
			if err := json.Unmarshal(line, &msg); err != nil {
				continue
			}
			
			// Only process TPV (Time-Position-Velocity) messages with 3D fix
			if msg.Class == "TPV" && msg.Mode >= 2 {
				pos := Position{
					Lat:       msg.Lat,
					Lon:       msg.Lon,
					Altitude:  msg.Alt,
					Track:     msg.Track,
					Speed:     msg.Speed * 1.94384, // Convert m/s to knots
					Climb:     msg.Climb * 196.85,  // Convert m/s to ft/min
					Mode:      msg.Mode,
					Timestamp: time.Now(),
				}
				
				select {
				case updates <- pos:
				default:
					// Channel full, skip this update
				}
			}
		}
		
		if err := scanner.Err(); err != nil {
			log.Printf("gpsd read error: %v", err)
		}
		
		conn.Close()
		log.Printf("Disconnected from gpsd, reconnecting...")
		time.Sleep(2 * time.Second)
	}
}
