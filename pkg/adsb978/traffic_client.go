package adsb978

import (
	"bufio"
	"encoding/json"
	"log"
	"net"
	"strings"
	"time"

	"adsb-receiver/pkg/adsb1090"
)

// TrafficClient reads decoded UAT traffic from dump978-fa JSON output
type TrafficClient struct {
	addr       string
	trafficOut chan<- adsb1090.Aircraft
}

// NewTrafficClient creates a new UAT traffic client
func NewTrafficClient(addr string, trafficOut chan<- adsb1090.Aircraft) *TrafficClient {
	return &TrafficClient{
		addr:       addr,
		trafficOut: trafficOut,
	}
}

// Read starts reading and processing UAT traffic with automatic reconnection
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
			var msg map[string]interface{}
			if err := json.Unmarshal([]byte(line), &msg); err != nil {
				continue
			}

			// Determine message type
			msgType, _ := msg["type"].(string)

			// Only process traffic messages
			switch msgType {
			case "adsb_icao", "adsb_icao_nt", "tisb_icao", "adsr_icao":
				if ac, ok := c.parseTraffic(msg); ok {
					select {
					case c.trafficOut <- ac:
					default:
						log.Printf("Warning: Traffic channel full")
					}
				}
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

// parseTraffic converts dump978 JSON to Aircraft format
func (c *TrafficClient) parseTraffic(msg map[string]interface{}) (adsb1090.Aircraft, bool) {
    ac := adsb1090.Aircraft{
        Timestamp: time.Now(),
    }

    // Get ICAO address
    if addr, ok := msg["hex"].(string); ok {
        ac.ICAO = strings.ToUpper(addr)
    } else {
        return ac, false
    }

    // Get position
    lat, hasLat := msg["lat"].(float64)
    lon, hasLon := msg["lon"].(float64)
    if hasLat && hasLon && lat != 0 && lon != 0 {
        ac.Lat = lat
        ac.Lon = lon
        ac.HasPosition = true
    } else {
        return ac, false // Must have position for traffic
    }

	if call, ok := msg["flight"].(string); ok {
        ac.Callsign = strings.ToUpper(strings.TrimSpace(call))
		ac.HasCallsign = true
    } 

    // Get altitude (barometric only - required for GDL90)
    if alt, ok := msg["alt_baro"].(float64); ok {
        ac.Altitude = int(alt)
        ac.HasAltitude = true
    }

    // Get velocity (ground track only - not heading)
    if track, ok := msg["track"].(float64); ok {
        ac.Track = int(track)
        ac.HasTrack = true
    }

    if speed, ok := msg["gs"].(float64); ok {
        ac.Speed = int(speed)
        ac.HasSpeed = true
    } else if speed, ok := msg["speed"].(float64); ok {
        ac.Speed = int(speed)
        ac.HasSpeed = true
    }

    // Get vertical velocity (NEW)
    if vv, ok := msg["vert_rate"].(float64); ok {
        ac.VertVel = int(vv)
        ac.HasVertVel = true
    }


    return ac, true
}