package gdl90

import (
	"log"
	"net"
	"strconv"
	"strings"
	"time"
	
	"adsb-receiver/pkg/traffic"
	"adsb-receiver/pkg/position"
)

// Server sends GDL90 messages via UDP
type Server struct {
	trafficMgr  *traffic.Manager
	positionMgr *position.Manager
	port        string
	targetAddr  *net.UDPAddr
}

// NewServer creates a new GDL90 server
func NewServer(trafficMgr *traffic.Manager, positionMgr *position.Manager, port string) *Server {
	// Target iPad/EFB at 192.168.10.50:4000
	// Change this to match your network or use broadcast
	targetAddr, _ := net.ResolveUDPAddr("udp4", "192.168.10.255:4000")
	
	return &Server{
		trafficMgr:  trafficMgr,
		positionMgr: positionMgr,
		port:        port,
		targetAddr:  targetAddr,
	}
}

func (s *Server) Serve() error {
	// Listen on all interfaces
	addr, err := net.ResolveUDPAddr("udp4", s.port)
	if err != nil {
		return err
	}

	conn, err := net.ListenUDP("udp4", addr)
	if err != nil {
		return err
	}
	defer conn.Close()

	// Big write buffer for bursts
	conn.SetWriteBuffer(1024 * 1024)

	log.Printf("GDL90 server listening on %s", s.port)
	log.Printf("Sending to %s", s.targetAddr.String())
	log.Println("Sending: Heartbeats + Ownship + Traffic at 1 Hz")

	// All tickers at 1 Hz - this is the key!
	heartbeatTicker := time.NewTicker(1 * time.Second)
	defer heartbeatTicker.Stop()

	ownshipTicker := time.NewTicker(1 * time.Second)
	defer ownshipTicker.Stop()

	trafficTicker := time.NewTicker(1 * time.Second)
	defer trafficTicker.Stop()

	for {
		select {
		case <-heartbeatTicker.C:
			// Check if we have GPS
			hasGPS := s.positionMgr.HasPosition()
			
			// Send ALL heartbeat messages together - this is critical!
			heartbeatBundle := [][]byte{
				MakeHeartbeat(hasGPS, false),
				MakeStratuxHeartbeat(hasGPS, false),
				MakeStratuxStatus(hasGPS, false),
			}
			
			for _, msg := range heartbeatBundle {
				if _, err := conn.WriteToUDP(msg, s.targetAddr); err != nil {
					log.Printf("Error sending heartbeat: %v", err)
				}
			}
			
			log.Printf("✓ Sent heartbeat bundle (GPS: %v)", hasGPS)

		case <-ownshipTicker.C:
			// Send ownship position if available
			if pos, ok := s.positionMgr.GetPosition(); ok {
				// DEBUG: Log raw position data
				log.Printf("RAW Position from manager: Lat=%.8f, Lon=%.8f, Alt=%.2fm, Track=%.1f°, Speed=%.1fkts",
					pos.Lat, pos.Lon, pos.Altitude, pos.Track, pos.Speed)
				
				// Convert meters to feet
				altFeet := int(pos.Altitude * 3.28084)
				
				// Send both ownship messages
				ownshipMsgs := [][]byte{
					MakeOwnshipReport(
						pos.Lat, 
						pos.Lon, 
						altFeet, 
						int(pos.Track), 
						int(pos.Speed),
					),
					MakeOwnshipGeoAltitude(altFeet),
				}
				
				for _, msg := range ownshipMsgs {
					if _, err := conn.WriteToUDP(msg, s.targetAddr); err != nil {
						log.Printf("Error sending ownship: %v", err)
					}
				}
				
				log.Printf("✓ Sent ownship: %.4f,%.4f @ %dft, %d°, %dkts",
					pos.Lat, pos.Lon, altFeet, int(pos.Track), int(pos.Speed))
			} else {
				log.Printf("✗ No valid position available")
			}

		case <-trafficTicker.C:
			// Send traffic reports at 1 Hz per target
			aircraft := s.trafficMgr.GetAircraft()
			
			if len(aircraft) > 0 {
				for _, ac := range aircraft {
					// Skip aircraft without position
					if ac.Lat == 0 && ac.Lon == 0 {
						continue
					}
					
					// Parse ICAO from hex string
					icao := parseICAO(ac.ICAO)
					
					trafficMsg := MakeTrafficReport(
						icao,
						ac.Lat,
						ac.Lon,
						ac.Altitude,
						ac.Track,
						ac.Speed,
						ac.Callsign,
					)
					
					if _, err := conn.WriteToUDP(trafficMsg, s.targetAddr); err != nil {
						log.Printf("Error sending traffic: %v", err)
					}
				}
				
				log.Printf("✓ Sent %d traffic reports", len(aircraft))
			}
		}
	}
}

// parseICAO converts hex ICAO string to uint32
func parseICAO(icaoHex string) uint32 {
	icao, _ := strconv.ParseUint(strings.TrimSpace(icaoHex), 16, 32)
	return uint32(icao)
}