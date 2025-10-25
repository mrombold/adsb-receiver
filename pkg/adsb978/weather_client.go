package adsb978

import (
	"bufio"
	"encoding/hex"
	"log"
	"net"
	"strings"
	"time"
)

// WeatherClient reads raw UAT uplink frames for FIS-B weather
type WeatherClient struct {
	addr       string
	weatherOut chan<- []byte
}

// NewWeatherClient creates a client for raw weather frames
func NewWeatherClient(addr string, weatherOut chan<- []byte) *WeatherClient {
	return &WeatherClient{
		addr:       addr,
		weatherOut: weatherOut,
	}
}

// Read reads raw uplink frames from dump978 and forwards them
func (c *WeatherClient) Read() error {
	for {
		conn, err := net.Dial("tcp", c.addr)
		if err != nil {
			time.Sleep(5 * time.Second)
			continue
		}

		log.Printf("Connected to dump978 weather at %s", c.addr)

		scanner := bufio.NewScanner(conn)
		for scanner.Scan() {
			line := scanner.Text()

			// Format from dump978 raw port: +hexdata;rssi=X;t=timestamp;
			// + prefix = uplink (weather), - prefix = downlink (traffic)
			if len(line) == 0 || line[0] != '+' {
				continue // Skip non-uplink frames
			}

			// Extract hex data before semicolon
			parts := strings.Split(line[1:], ";")
			if len(parts) < 1 {
				continue
			}

			// Decode hex to raw bytes
			frame, err := hex.DecodeString(parts[0])
			if err != nil {
				log.Printf("Error decoding weather frame: %v", err)
				continue
			}

			// Validate frame length - UAT uplink should be exactly 432 bytes
			if len(frame) != 432 {
				log.Printf("Warning: UAT frame has unexpected length %d (expected 432)", len(frame))
				continue
			}
			// Send raw frame to weather manager
			select {
			case c.weatherOut <- frame:
			default:
				log.Printf("Warning: Weather channel full, dropping frame")
			}
		}

		if err := scanner.Err(); err != nil {
			log.Printf("Error reading from dump978 weather: %v", err)
		}

		conn.Close()
		time.Sleep(2 * time.Second)
	}
}