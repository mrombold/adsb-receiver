package main

import (
	"log"
	"time"
	
	"adsb-receiver/pkg/adsb1090"
	"adsb-receiver/pkg/traffic"
)

func main() {
	log.Println("Starting ADS-B Receiver Aggregator")
	
	// Create traffic manager
	trafficMgr := traffic.NewManager()
	go trafficMgr.Run()
	
	// Create dump1090 client
	adsb1090Client := adsb1090.NewClient("localhost:30003")
	
	// Start reading from dump1090
	go adsb1090Client.Read(trafficMgr.Updates())
	
	// Status ticker
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()
	
	for range ticker.C {
		count := trafficMgr.Count()
		aircraft := trafficMgr.GetAircraft()
		
		log.Printf("Tracking %d aircraft", count)
		for _, ac := range aircraft {
			log.Printf("  %s: %s at %d ft, %.4f,%.4f", 
				ac.ICAO, ac.Callsign, ac.Altitude, ac.Lat, ac.Lon)
		}
	}
}