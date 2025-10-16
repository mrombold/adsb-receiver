// pkg/adsb978/client.go
package adsb978

import (
	"bufio"
	"encoding/hex"
	"fmt"
	"log"
	"net"
	"strings"
	"time"
)

// Client connects to dump978-fa and processes UAT messages
type Client struct {
	addr    string
	conn    net.Conn
	Frames  chan RawFrame
	Traffic chan TrafficUpdate
	Weather chan WeatherUpdate
}

// RawFrame represents a raw UAT frame from dump978-fa
type RawFrame struct {
	Direction string    // "uplink" or "downlink"
	Data      []byte    // Raw hex data
	RSSI      int       // Signal strength
	Timestamp time.Time
}

// TrafficUpdate represents decoded aircraft traffic from 978 MHz
type TrafficUpdate struct {
	ICAO      uint32
	Latitude  float64
	Longitude float64
	Altitude  int32 // feet MSL
	Track     uint16
	Speed     uint16    // knots
	VertRate  int16     // feet per minute
	Callsign  string
	Timestamp time.Time
}

// WeatherUpdate represents FIS-B weather products
type WeatherUpdate struct {
	Type      string      // "nexrad", "metar", "taf", "tfr", etc.
	ProductID int         // FIS-B product ID
	Data      interface{} // Product-specific data
	Timestamp time.Time
}

// NewClient creates a new 978 MHz client
func NewClient(addr string) *Client {
	return &Client{
		addr:    addr,
		Frames:  make(chan RawFrame, 100),
		Traffic: make(chan TrafficUpdate, 100),
		Weather: make(chan WeatherUpdate, 100),
	}
}

// Connect establishes connection to dump978-fa
func (c *Client) Connect() error {
	conn, err := net.Dial("tcp", c.addr)
	if err != nil {
		return fmt.Errorf("failed to connect to dump978: %w", err)
	}
	c.conn = conn
	log.Printf("Connected to dump978 at %s", c.addr)
	return nil
}

// Read starts reading frames from dump978-fa
func (c *Client) Read() {
	defer c.conn.Close()

	scanner := bufio.NewScanner(c.conn)
	for scanner.Scan() {
		line := scanner.Text()
		frame, err := c.parseFrame(line)
		if err != nil {
			log.Printf("Error parsing UAT frame: %v", err)
			continue
		}

		// Send raw frame
		select {
		case c.Frames <- frame:
		default:
			log.Printf("Warning: Frames channel full, dropping frame")
		}

		// Process frame
		if frame.Direction == "downlink" {
			// Aircraft traffic
			traffic, err := c.decodeTraffic(frame)
			if err != nil {
				log.Printf("Error decoding traffic: %v", err)
				continue
			}
			select {
			case c.Traffic <- traffic:
			default:
				log.Printf("Warning: Traffic channel full, dropping update")
			}
		} else if frame.Direction == "uplink" {
			// FIS-B weather data
			weather, err := c.decodeFISB(frame)
			if err != nil {
				// Many uplink frames won't decode cleanly - that's normal
				continue
			}
			select {
			case c.Weather <- weather:
			default:
				log.Printf("Warning: Weather channel full, dropping update")
			}
		}
	}

	if err := scanner.Err(); err != nil {
		log.Printf("Error reading from dump978: %v", err)
	}
}

// parseFrame parses dump978-fa output format
// Format: +/- prefix (uplink/downlink), hex data, ;rs=X; (RSSI), ;t=timestamp;
// Example: -a1b2c3d4e5f6;rs=5;t=1634567890;
func (c *Client) parseFrame(line string) (RawFrame, error) {
	if len(line) == 0 {
		return RawFrame{}, fmt.Errorf("empty line")
	}

	frame := RawFrame{
		Timestamp: time.Now(),
	}

	// Determine direction
	if line[0] == '-' {
		frame.Direction = "downlink"
	} else if line[0] == '+' {
		frame.Direction = "uplink"
	} else {
		return frame, fmt.Errorf("invalid direction prefix: %c", line[0])
	}

	// Split by semicolon to get hex data and metadata
	parts := strings.Split(line[1:], ";")
	if len(parts) < 1 {
		return frame, fmt.Errorf("invalid frame format")
	}

	// Decode hex data
	hexData := parts[0]
	data, err := hex.DecodeString(hexData)
	if err != nil {
		return frame, fmt.Errorf("invalid hex data: %w", err)
	}
	frame.Data = data

	// Parse metadata (rs=X, t=timestamp)
	for _, part := range parts[1:] {
		if strings.HasPrefix(part, "rs=") {
			fmt.Sscanf(part, "rs=%d", &frame.RSSI)
		} else if strings.HasPrefix(part, "t=") {
			var timestamp int64
			fmt.Sscanf(part, "t=%d", &timestamp)
			frame.Timestamp = time.Unix(timestamp, 0)
		}
	}

	return frame, nil
}

// decodeTraffic decodes UAT downlink traffic message
func (c *Client) decodeTraffic(frame RawFrame) (TrafficUpdate, error) {
	// UAT downlink format (simplified - you'll need full implementation)
	// This is where you'd use your FIS-B library or implement UAT decoding

	if len(frame.Data) < 18 {
		return TrafficUpdate{}, fmt.Errorf("frame too short: %d bytes", len(frame.Data))
	}

	traffic := TrafficUpdate{
		Timestamp: frame.Timestamp,
	}

	// Extract ICAO address (3 bytes)
	traffic.ICAO = uint32(frame.Data[0])<<16 | uint32(frame.Data[1])<<8 | uint32(frame.Data[2])

	// TODO: Implement full UAT message decoding
	// This requires parsing:
	// - Position (lat/lon in compact form)
	// - Altitude
	// - Velocity
	// - Track
	// - Vertical rate
	// - Callsign (if present)

	// For now, return error to indicate incomplete implementation
	return traffic, fmt.Errorf("UAT traffic decoding not yet implemented - use libfisb")
}

// decodeFISB decodes FIS-B weather products from uplink
func (c *Client) decodeFISB(frame RawFrame) (WeatherUpdate, error) {
	// FIS-B decoding is complex and should use your extracted libfisb library
	// This is a placeholder

	if len(frame.Data) < 10 {
		return WeatherUpdate{}, fmt.Errorf("uplink frame too short")
	}

	weather := WeatherUpdate{
		Timestamp: frame.Timestamp,
	}

	// TODO: Implement FIS-B decoding using your libfisb library
	// This will decode:
	// - NEXRAD (products 51-64)
	// - METARs
	// - TAFs
	// - TFRs
	// - NOTAMs
	// - Other text products

	return weather, fmt.Errorf("FIS-B decoding not yet implemented - use libfisb")
}

// Close closes the connection
func (c *Client) Close() error {
	if c.conn != nil {
		return c.conn.Close()
	}
	return nil
}

// Helper function to format ICAO as hex string
func formatICAO(icao uint32) string {
	return fmt.Sprintf("%06X", icao&0xFFFFFF)
}
