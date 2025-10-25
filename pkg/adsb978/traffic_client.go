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
	trafficOut chan<- types.Aircraft
}

// dump978Position represents the nested position object
type dump978Position struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

// dump978Message represents the actual JSON format from dump978
type dump978Message struct {
	Address                   string          `json:"address"`
	AddressQualifier          string          `json:"address_qualifier"`
	Callsign                  string          `json:"callsign"`
	Position                  dump978Position `json:"position"`
	GeometricAltitude         int             `json:"geometric_altitude"`
	PressureAltitude          int             `json:"pressure_altitude"`
	TrueTrack                 float64         `json:"true_track"`
	GroundSpeed               int             `json:"ground_speed"`
	VerticalVelocityBarometric int            `json:"vertical_velocity_barometric"`
	VerticalVelocityGeometric  int            `json:"vertical_velocity_geometric"`
	AirgroundState            string          `json:"airground_state"`
	Emergency                 string          `json:"emergency"`
}

// NewTrafficClient creates a client for reading traffic from dump978 JSON port
func NewTrafficClient(addr string, trafficOut chan<- types.Aircraft) *TrafficClient {
	return &TrafficClient{
		addr:       addr,
		trafficOut: trafficOut,
	}
}

// Read reads JSON-formatted traffic messages from dump978 and forwards them
func (c *TrafficClient) Read() error {
	for {
		conn, err := net.Dial("tcp", c.addr)
		if err != nil {
			log.Printf("dump978 traffic connection failed: %v, retrying in 5s", err)
			time.Sleep(5 * time.Second)
			continue
		}

		log.Printf("Connected to dump978 traffic at %s", c.addr)

		scanner := bufio.NewScanner(conn)
		for scanner.Scan() {
			line := scanner.Text()

			// Parse JSON message
			var msg dump978Message
			if err := json.Unmarshal([]byte(line), &msg); err != nil {
				log.Printf("Error parsing dump978 JSON: %v", err)
				continue
			}

			// Convert to our Aircraft structure
			ac := types.Aircraft{
				ICAO:      strings.ToUpper(msg.Address),
				Timestamp: time.Now(),
			}

			// Callsign
			if msg.Callsign != "" {
				ac.Callsign = strings.TrimSpace(msg.Callsign)
				ac.HasCallsign = true
			}

			// Position - check if position object exists and has valid coordinates
			if msg.Position.Lat != 0 || msg.Position.Lon != 0 {
				ac.Lat = msg.Position.Lat
				ac.Lon = msg.Position.Lon
				ac.HasPosition = true
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

			// Track (dump978 gives true_track in degrees)
			if msg.TrueTrack >= 0 {
				ac.Track = int(msg.TrueTrack)
				ac.HasTrack = true
			}

			// Speed (dump978 gives ground_speed in knots)
			if msg.GroundSpeed > 0 {
				ac.Speed = msg.GroundSpeed
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
				ac.OnGround = (msg.AirgroundState == "ground")
				ac.HasOnGround = true
			}

			// Emergency status
			if msg.Emergency != "" && msg.Emergency != "none" {
				ac.Emergency = msg.Emergency
				ac.HasEmergency = true
			}

			// Send to traffic manager
			select {
			case c.trafficOut <- ac:
			default:
				log.Printf("Warning: Traffic channel full, dropping aircraft %s", ac.ICAO)
			}
		}

		if err := scanner.Err(); err != nil {
			log.Printf("Error reading from dump978 traffic: %v", err)
		}

		conn.Close()
		log.Printf("Disconnected from dump978 traffic, reconnecting...")
		time.Sleep(2 * time.Second)
	}
}