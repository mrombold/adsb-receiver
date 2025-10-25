// pkg/types/aircraft.go
package types

import "time"

type Aircraft struct {
	ICAO      string
	Callsign  string
	Altitude  int
	Lat       float64
	Lon       float64
	Track     int
	Speed     int
	VertVel   int
	Timestamp time.Time
	
	// New fields for enhanced GDL90 encoding
	Squawk       string // Mode A squawk code (e.g., "7700")
	Emergency    bool   // Emergency flag
	Alert        bool   // Squawk change alert
	SPI          bool   // Special Position Indicator
	OnGround     bool   // Ground squat switch active
	
	// Track which fields are valid in this update
	HasCallsign  bool
	HasPosition  bool
	HasAltitude  bool
	HasSpeed     bool
	HasTrack     bool
	HasVertVel   bool
	HasSquawk    bool
	HasEmergency bool
	HasAlert     bool
	HasSPI       bool
	HasOnGround  bool
}