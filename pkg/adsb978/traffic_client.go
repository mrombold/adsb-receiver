package adsb978

import (
	"bufio"
	"encoding/json"
	"log"
	"net"
	"strings"
	"time"
	
	"adsb-receiver/pkg/types"
)

// TrafficClient reads JSON-formatted ADS-B traffic from dump978
type TrafficClient struct {
	addr       string
}

// dump978Message represents the JSON format from dump978
type dump978Message struct {
    Address              string  `json:"address"`
    AirgroundState       string  `json:"airground_state"`
    Callsign             string  `json:"callsign"`
    Emergency            string  `json:"emergency"`
    EmitterCategory      string  `json:"emitter_category"`
    GeometricAltitude    int     `json:"geometric_altitude"`
    GroundSpeed          float64 `json:"ground_speed"`
	Nic                  int     `json:"nic"`
	Nac                  int     `json:"nac_p"`
    PressureAltitude     int     `json:"pressure_altitude"`
    TrueTrack            float64 `json:"true_track"`
    VerticalVelocityGeometric    int     `json:"vertical_velocity_geometric"`
    VerticalVelocityBarometric   int     `json:"vertical_velocity_barometric"`
    Position             struct {
        Lat float64 `json:"lat"`
        Lon float64 `json:"lon"`
    } `json:"position"`
    NorthVelocity        int     `json:"north_velocity"`
    EastVelocity         int     `json:"east_velocity"`
    // Add other fields if you need them later
}



// NewTrafficClient creates a client for reading traffic from dump978 JSON port
func NewTrafficClient(addr string) *TrafficClient {
	return &TrafficClient{addr: addr}
}

// Read reads JSON-formatted traffic messages from dump978 and forwards them
func (c *TrafficClient) Read(updates chan<- types.Aircraft) error {
	for {
		conn, err := net.Dial("tcp", c.addr)
		if err != nil {
			time.Sleep(5 * time.Second)
			continue
		}

		log.Printf("Connected to dump978 traffic at %s", c.addr)

		scanner := bufio.NewScanner(conn)
		for scanner.Scan() {
			line := scanner.Text()
			if ac, ok := c.parseDumpJson(line); ok {
				updates <- ac
			}

		}

		if err := scanner.Err(); err != nil {
			log.Printf("Error reading from dump978 traffic: %v", err)
		}
		conn.Close()
		time.Sleep(2 * time.Second)
	}
}

func (c *TrafficClient) parseDumpJson(line string) (types.Aircraft, bool) {
	// Parse JSON message and place into variable msg which is dump978Message struct
	var msg dump978Message
	if err := json.Unmarshal([]byte(line), &msg); err != nil {
		log.Printf("Error parsing dump978 JSON: %v", err)
		return types.Aircraft{}, false
	}

	icao := strings.ToUpper(strings.TrimSpace(msg.Address))

	if icao == "" {
		return types.Aircraft{}, false
	}

	ac := types.Aircraft{
		ICAO:      icao,
		Timestamp: time.Now(),
	}

	// Callsign
	if msg.Callsign != "" {
		ac.Callsign = strings.TrimSpace(msg.Callsign)
		ac.HasCallsign = true
	}

	// Altitude - prefer pressure altitude for consistency with 1090ES
	// (GDL90 protocol uses pressure altitude)
	if msg.PressureAltitude != 0 {
		ac.Altitude = msg.PressureAltitude
		ac.HasAltitude = true
	} else if msg.GeometricAltitude != 0 {
		// Fallback to geometric if pressure not available
		ac.Altitude = msg.GeometricAltitude
		ac.HasAltitude = true
	}

	// Position - check if position object exists and has valid coordinates
	if msg.Position.Lat != 0 || msg.Position.Lon != 0 {
		ac.Lat = msg.Position.Lat
		ac.Lon = msg.Position.Lon
		ac.HasPosition = true
	}

	// Track (dump978 gives true_track in degrees)
	if msg.TrueTrack >= 0 {
		ac.Track = int(msg.TrueTrack)
		ac.HasTrack = true
	}

	// Speed (dump978 gives ground_speed in knots)
	if msg.GroundSpeed > 0 {
		ac.Speed = int(msg.GroundSpeed)
		ac.HasSpeed = true
	}

	// Vertical velocity - prefer barometric for consistency
	if msg.VerticalVelocityBarometric != 0 {
		ac.VertVel = msg.VerticalVelocityBarometric
		ac.HasVertVel = true
	} else if msg.VerticalVelocityGeometric != 0 {
		// Fallback to geometric if barometric not available
		ac.VertVel = msg.VerticalVelocityGeometric
		ac.HasVertVel = true
	}

	// OnGround status
	if msg.AirgroundState != "" {
		ac.OnGround = !(msg.AirgroundState == "airborne")
		ac.HasOnGround = true
	}

	// Emergency status
	if msg.Emergency != "" && msg.Emergency != "none" {
		ac.Emergency = true
		ac.HasEmergency = true
	}

	return ac, true

}