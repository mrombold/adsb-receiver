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
	trafficMgr *traffic.Manager
	positionMgr *position.Manager
	port       string
}

// NewServer creates a new GDL90 server
func NewServer(trafficMgr *traffic.Manager, positionMgr *position.Manager, port string) *Server {
	return &Server{
		trafficMgr: trafficMgr,
		positionMgr: positionMgr,
		port:       port,
	}
}

// Serve starts the GDL90 server
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
	
	// Enable broadcast
	conn.SetWriteBuffer(1024 * 1024)
	
	log.Printf("GDL90 server listening on %s", s.port)
	
	// Broadcast to the local subnet
	broadcastAddr, _ := net.ResolveUDPAddr("udp4", "192.168.10.255:4000")
	
	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()
	
	for range ticker.C {
		hasGPS := s.positionMgr.HasPosition()
		
		// Send heartbeats
		heartbeat := MakeHeartbeat(hasGPS, false)
		if _, err := conn.WriteToUDP(heartbeat, broadcastAddr); err != nil {
			log.Printf("Error sending heartbeat: %v", err)
		}
		
		stratuxHeartbeat := MakeStratuxHeartbeat(hasGPS, false)
		if _, err := conn.WriteToUDP(stratuxHeartbeat, broadcastAddr); err != nil {
			log.Printf("Error sending Stratux heartbeat: %v", err)
		}
		
		// Send ownship position if we have GPS
		if pos, ok := s.positionMgr.GetPosition(); ok {
			ownship := MakeOwnshipReport(pos.Lat, pos.Lon, int(pos.Altitude*3.28084), int(pos.Track), int(pos.Speed))
			conn.WriteToUDP(ownship, broadcastAddr)
			
			ownshipAlt := MakeOwnshipGeoAltitude(int(pos.Altitude * 3.28084))
			conn.WriteToUDP(ownshipAlt, broadcastAddr)
		}
		
		// Send traffic reports
		aircraft := s.trafficMgr.GetAircraft()
		sentCount := 0
		for _, ac := range aircraft {
			// Only send if we have valid position
			if ac.Lat != 0 && ac.Lon != 0 {
				icao := parseICAO(ac.ICAO)
				traffic := MakeTrafficReport(icao, ac.Lat, ac.Lon, ac.Altitude, ac.Track, ac.Speed)
				if _, err := conn.WriteToUDP(traffic, broadcastAddr); err != nil {
					log.Printf("Error sending traffic: %v", err)
				} else {
					sentCount++
				}
			}
		}
		
		if sentCount > 0 {
			log.Printf("Sent %d traffic reports", sentCount)
		}
	}
	
	return nil
}

// parseICAO converts hex ICAO string to uint32
func parseICAO(icaoHex string) uint32 {
	icao, _ := strconv.ParseUint(strings.TrimSpace(icaoHex), 16, 32)
	return uint32(icao)
}
