package gdl90

import (
	"log"
	"net"
	"time"

	"adsb-receiver/pkg/ahrs"
	"adsb-receiver/pkg/position"
	"adsb-receiver/pkg/traffic"
	"adsb-receiver/pkg/weather"
)

// Server sends GDL90 messages via UDP
type Server struct {
	trafficMgr  *traffic.Manager
	positionMgr *position.Manager
	weatherMgr  *weather.Manager
	ahrsMgr     *ahrs.Manager
	port        string
	targetAddr  *net.UDPAddr
}

// NewServer creates a new GDL90 server
func NewServer(trafficMgr *traffic.Manager, positionMgr *position.Manager, weatherMgr *weather.Manager, ahrsMgr *ahrs.Manager, port string) *Server {
	// Target iPad/EFB at broadcast address
	targetAddr, _ := net.ResolveUDPAddr("udp4", "192.168.10.255:4000")

	return &Server{
		trafficMgr:  trafficMgr,
		positionMgr: positionMgr,
		weatherMgr:  weatherMgr,
		ahrsMgr:     ahrsMgr,
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
	//log.Println("Sending: Heartbeats + Ownship + Traffic at 1 Hz")

	// All tickers at 1 Hz - this is the key!
	heartbeatTicker := time.NewTicker(1 * time.Second)
	defer heartbeatTicker.Stop()

	ownshipTicker := time.NewTicker(200 * time.Millisecond)
	defer ownshipTicker.Stop()

	// AHRS at 5 Hz (200ms) per ForeFlight recommendation
	ahrsTicker := time.NewTicker(200 * time.Millisecond)
	defer ahrsTicker.Stop()

	trafficTicker := time.NewTicker(500 * time.Millisecond)
	defer trafficTicker.Stop()

		// Weather - send immediately when available
	weatherTicker := time.NewTicker(1 * time.Second)
	defer weatherTicker.Stop()

	for {
		select {
		case <-heartbeatTicker.C:
			// Check if we have GPS
			hasGPS := s.positionMgr.HasPosition()
			
			// Send ALL heartbeat messages together - this is critical!
			heartbeatBundle := [][]byte{
				MakeHeartbeat(hasGPS, true),
				MakeStratuxHeartbeat(hasGPS, true),
				MakeStratuxStatus(hasGPS, true),
			}

			//log.Printf("Heartbeat[%d] (%d bytes): % X", 1, len(heartbeatBundle[0]), heartbeatBundle[0])
			//log.Printf("Heartbeat[%d] (%d bytes): % X", 2, len(heartbeatBundle[1]), heartbeatBundle[1])
			//log.Printf("Heartbeat[%d] (%d bytes): % X", 3, len(heartbeatBundle[2]), heartbeatBundle[2])
			
			for _, msg := range heartbeatBundle {
				if _, err := conn.WriteToUDP(msg, s.targetAddr); err != nil {
					log.Printf("Error sending heartbeat: %v", err)
				}
			}
			

		case <-ownshipTicker.C:
			// ALWAYS send ownship - required for EFB recognition
			var ownshipMsgs [][]byte
			
			if pos, ok := s.positionMgr.GetPosition(); ok {
				// Convert meters to feet
				altFeet := int(pos.Altitude * 3.28084)
				altHAEfeet := int(pos.AltitudeHAE * 3.28084)
				
				// Send valid position
				ownshipMsgs = [][]byte{
					MakeOwnshipReport(
						pos.Lat, 
						pos.Lon, 
						altFeet, 
						int(pos.Track), 
						int(pos.Speed),
						int(pos.Climb),
					),
					MakeOwnshipGeoAltitude(altHAEfeet),
				}
			} else {
				// No GPS - send invalid position (all zeros)
				// This is REQUIRED for EFB to recognize the device!
				ownshipMsgs = [][]byte{
					MakeOwnshipReport(
						0.0,   // Invalid lat
						0.0,   // Invalid lon
						0,     // Invalid altitude
						0,     // Invalid track
						0,     // Invalid speed
						0,     // Invalid vertical velocity
					),
					MakeOwnshipGeoAltitude(0),  // Invalid geo altitude
				}
			}
			
			// Send the messages
			for _, msg := range ownshipMsgs {
				if _, err := conn.WriteToUDP(msg, s.targetAddr); err != nil {
					log.Printf("Error sending ownship: %v", err)
				}
			}

		case <-ahrsTicker.C:
			// Send AHRS data if available
			var ahrsMsg []byte
			
			if ahrsData, ok := s.ahrsMgr.GetData(); ok {
				// Valid AHRS data
				ahrsMsg = MakeAHRS(
					ahrsData.Roll,
					ahrsData.Pitch,
					ahrsData.Yaw, // This is heading (0-360)
				)
			} else {
				// No AHRS - send default/invalid data
				ahrsMsg = MakeAHRS(0, 0, 0)  // Note: capital A in MakeAHRS
			}
			
			if _, err := conn.WriteToUDP(ahrsMsg, s.targetAddr); err != nil {
				log.Printf("Error sending AHRS: %v", err)
			}

		case <-trafficTicker.C:
			// Send traffic reports at 1 Hz per target
			aircraft := s.trafficMgr.GetAircraft()
			
			if len(aircraft) > 0 {
				for _, ac := range aircraft {				
					trafficMsg := MakeTrafficReport(ac)
					//log.Printf("Traffic (%d bytes): % X", len(trafficMsg), trafficMsg)
					
					if _, err := conn.WriteToUDP(trafficMsg, s.targetAddr); err != nil {
						log.Printf("Error sending traffic: %v", err)
					}
				}
				
			}

		case <-weatherTicker.C:
			// Send weather frames as they become available
			frames := s.weatherMgr.GetFrames()
			
			if len(frames) > 0 {
				for _, frame := range frames {
					// Wrap raw UAT frame in GDL90 uplink message
					weatherMsg := MakeUplinkData(frame)				
					//log.Printf("Weather (%d bytes): % X", len(weatherMsg), weatherMsg)
					if _, err := conn.WriteToUDP(weatherMsg, s.targetAddr); err != nil {
						log.Printf("Error sending weather: %v", err)
					}
				}
				
				log.Printf("✓ Sent %d weather frames", len(frames))
			}
		}
	}
}
