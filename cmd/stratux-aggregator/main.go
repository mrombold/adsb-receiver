package main

import (
	"log"
	
	"adsb-receiver/pkg/adsb1090"
	"adsb-receiver/pkg/gdl90"
	"adsb-receiver/pkg/gps"
	"adsb-receiver/pkg/position"
	"adsb-receiver/pkg/traffic"
)

func main() {
	log.Println("Starting ADS-B Receiver Aggregator")
	
	// Create managers
	trafficMgr := traffic.NewManager()
	positionMgr := position.NewManager()
	
	go trafficMgr.Run()
	go positionMgr.Run()
	
	// Create clients
	adsb1090Client := adsb1090.NewClient("localhost:30003")
	gpsClient := gps.NewClient("localhost:2947")
	
	// Start reading from sources
	go adsb1090Client.Read(trafficMgr.Updates())
	go gpsClient.Read(positionMgr.Updates())
	
	// Create and start GDL90 server
	gdl90Server := gdl90.NewServer(trafficMgr, positionMgr, ":4000")
	log.Fatal(gdl90Server.Serve())
}
