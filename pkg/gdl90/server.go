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

	// Broadcast to the local subnet (unchanged)
	broadcastAddr, _ := net.ResolveUDPAddr("udp4", "192.168.10.255:4000")

	// Tickers: heartbeat 1 Hz; GPS & traffic 10 Hz
	heartbeatTicker := time.NewTicker(1 * time.Second)
	defer heartbeatTicker.Stop()

	gpsTicker := time.NewTicker(100 * time.Millisecond)
	defer gpsTicker.Stop()

	trafficTicker := time.NewTicker(100 * time.Millisecond)
	defer trafficTicker.Stop()

	lastTrafficLog := time.Now()

	for {
		select {
		case <-heartbeatTicker.C:
			hasGPS := s.positionMgr.HasPosition()

			// Send heartbeats (1 Hz)
			if _, err := conn.WriteToUDP(MakeHeartbeat(hasGPS, false), broadcastAddr); err != nil {
				log.Printf("Error sending heartbeat: %v", err)
			}
			if _, err := conn.WriteToUDP(MakeStratuxHeartbeat(hasGPS, false), broadcastAddr); err != nil {
				log.Printf("Error sending Stratux heartbeat: %v", err)
			}

		case <-gpsTicker.C:
			// Ownship @ 10 Hz
			if pos, ok := s.positionMgr.GetPosition(); ok {
				altFeet := int(pos.Altitude * 3.28084)

				if _, err := conn.WriteToUDP(MakeOwnshipReport(
					pos.Lat, pos.Lon, altFeet, int(pos.Track), int(pos.Speed),
				), broadcastAddr); err != nil {
					log.Printf("Error sending ownship: %v", err)
				}
				if _, err := conn.WriteToUDP(MakeOwnshipGeoAltitude(
					altFeet,
				), broadcastAddr); err != nil {
					log.Printf("Error sending ownship geo alt: %v", err)
				}
			}

		case <-trafficTicker.C:
			// Traffic @ 10 Hz
			aircraft := s.trafficMgr.GetAircraft()
			sentCount := 0
			for _, ac := range aircraft {
				if ac.Lat == 0 && ac.Lon == 0 {
					continue
				}
				icao := parseICAO(ac.ICAO)
				if _, err := conn.WriteToUDP(MakeTrafficReport(
					icao, ac.Lat, ac.Lon, ac.Altitude, ac.Track, ac.Speed,
				), broadcastAddr); err != nil {
					log.Printf("Error sending traffic: %v", err)
				} else {
					sentCount++
				}
			}
			// Throttle this log to once per second (so 10 Hz doesn't spam)
			if sentCount > 0 && time.Since(lastTrafficLog) >= time.Second {
				log.Printf("Sent %d traffic reports", sentCount)
				lastTrafficLog = time.Now()
			}
		}
	}
}

// parseICAO converts hex ICAO string to uint32
func parseICAO(icaoHex string) uint32 {
	icao, _ := strconv.ParseUint(strings.TrimSpace(icaoHex), 16, 32)
	return uint32(icao)
}
