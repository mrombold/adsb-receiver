package main

import (
	"log"
	"time"
	
	"adsb-receiver/pkg/adsb1090"
	"adsb-receiver/pkg/adsb978"
	"adsb-receiver/pkg/gdl90"
	"adsb-receiver/pkg/gps"
	"adsb-receiver/pkg/position"
	"adsb-receiver/pkg/traffic"
	"adsb-receiver/pkg/weather"
)

func main() {
	log.Println("Starting ADS-B Receiver Aggregator")
	log.Println("===================================")
	
	// Create state managers
	trafficMgr := traffic.NewManager()
	positionMgr := position.NewManager()
	weatherMgr := weather.NewManager()
	
	// Start manager goroutines
	go trafficMgr.Run()
	go positionMgr.Run()
	go weatherMgr.Run()
	
	// Create traffic clients
	adsb1090Client := adsb1090.NewClient("localhost:30003")

	// For traffic (JSON)
	adsb978TrafficClient := adsb978.NewTrafficClient("localhost:30979", trafficMgr.Updates())

	// For weather (raw frames)
	adsb978WeatherClient := adsb978.NewWeatherClient("localhost:30978", weatherMgr.Updates())

	gpsClient := gps.NewClient("localhost:2947")
	
	// Start reading from dump1090
	go adsb1090Client.Read(trafficMgr.Updates())

	// Start reading from dump978 (sends to both traffic and weather managers)
	go adsb978TrafficClient.Read()  // Traffic
	go adsb978WeatherClient.Read()  // Weather
	
	// Start reading from gpsd
	go gpsClient.Read(positionMgr.Updates())
	
	// Give services time to connect
	time.Sleep(2 * time.Second)
	
	// Create and start GDL90 server
	gdl90Server := gdl90.NewServer(trafficMgr, positionMgr, weatherMgr, ":4000")
	
	log.Println("GDL90 server starting...")
	log.Fatal(gdl90Server.Serve())
}