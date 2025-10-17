package gdl90

import (
	"log"
	"net"
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
	// Hardcoded target: iPad at 192.168.10.50:4000
	targetAddr, _ := net.ResolveUDPAddr("udp4", "192.168.10.50:4000")
	
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

	log.Printf("GDL90 server listening on %s", s.port)
	log.Printf("Sending to iPad at %s", s.targetAddr.String())
	log.Println("Sending: Heartbeats (1Hz) + Ownship Position (5Hz)")

	// Heartbeat ticker - 1 Hz
	heartbeatTicker := time.NewTicker(1 * time.Second)
	defer heartbeatTicker.Stop()

	// Position ticker - 5 Hz (every 200ms)
	positionTicker := time.NewTicker(200 * time.Millisecond)
	defer positionTicker.Stop()

	for {
		select {
		case <-heartbeatTicker.C:
			// Check if we have GPS
			hasGPS := s.positionMgr.HasPosition()
			
			// Send heartbeats with GPS status
			heartbeat := MakeHeartbeat(hasGPS, false)
			stratuxHB := MakeStratuxHeartbeat(hasGPS, false)
			
			conn.WriteToUDP(heartbeat, s.targetAddr)
			conn.WriteToUDP(stratuxHB, s.targetAddr)
			
			log.Printf("✓ Sent heartbeats (GPS: %v)", hasGPS)

		case <-positionTicker.C:
			// Send ownship position if available
			if pos, ok := s.positionMgr.GetPosition(); ok {
				altFeet := int(pos.Altitude * 3.28084)
				
				ownship := MakeOwnshipReport(
					pos.Lat, pos.Lon, altFeet, 
					int(pos.Track), int(pos.Speed),
				)
				geo := MakeOwnshipGeoAltitude(altFeet)
				
				conn.WriteToUDP(ownship, s.targetAddr)
				conn.WriteToUDP(geo, s.targetAddr)
			}
		}
	}

	return nil
}