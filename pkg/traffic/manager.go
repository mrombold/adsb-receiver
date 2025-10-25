package traffic

import (
	"sync"
	"time"
	"adsb-receiver/pkg/types"
)

// Manager manages aircraft traffic state
type Manager struct {
	aircraft map[string]*types.Aircraft
	mu       sync.RWMutex
	updates  chan types.Aircraft
}

// NewManager creates a new traffic manager
func NewManager() *Manager {
	return &Manager{
		aircraft: make(map[string]*types.Aircraft),
		updates:  make(chan types.Aircraft, 100),
	}
}

// Run is the main loop - ONLY this goroutine modifies the map
func (m *Manager) Run() {
	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()
	
	for {
		select {
		case update := <-m.updates:
			// Update or create aircraft
			m.mu.Lock()
			ac := m.aircraft[update.ICAO]
			if ac == nil {
				// New aircraft - copy the update
				newAc := update
				m.aircraft[update.ICAO] = &newAc
			} else {
				// Merge fields from update into existing aircraft
				// Only update fields that are marked as valid
				if update.HasCallsign {
					ac.Callsign = update.Callsign
				}
				if update.HasAltitude {
					ac.Altitude = update.Altitude
				}
				if update.HasPosition {
					ac.Lat = update.Lat
					ac.Lon = update.Lon
				}
				if update.HasSpeed {
					ac.Speed = update.Speed
				}
				if update.HasTrack {
					ac.Track = update.Track
				}
				if update.HasVertVel {
					ac.VertVel = update.VertVel
				}
				if update.HasSquawk {
					ac.Squawk = update.Squawk
				}
				if update.HasEmergency {
					ac.Emergency = update.Emergency
				}
				if update.HasAlert {
					ac.Alert = update.Alert
				}
				if update.HasSPI {
					ac.SPI = update.SPI
				}
				if update.HasOnGround {
					ac.OnGround = update.OnGround
				}

				// Always update timestamp
				ac.Timestamp = update.Timestamp
			}
			m.mu.Unlock()
			
		case <-ticker.C:
			// Cleanup stale aircraft (not seen in 15 seconds)
			m.mu.Lock()
			now := time.Now()
			for icao, ac := range m.aircraft {
				if now.Sub(ac.Timestamp) > 15*time.Second {
					delete(m.aircraft, icao)
				}
			}
			m.mu.Unlock()
		}
	}
}

// Updates returns the channel for sending aircraft updates
func (m *Manager) Updates() chan<- types.Aircraft {
	return m.updates
}

// GetAircraft returns a snapshot of all aircraft
func (m *Manager) GetAircraft() []types.Aircraft {
	m.mu.RLock()
	defer m.mu.RUnlock()
	
	list := make([]types.Aircraft, 0, len(m.aircraft))
	for _, ac := range m.aircraft {
		list = append(list, *ac)
	}
	return list
}

// Count returns the number of tracked aircraft
func (m *Manager) Count() int {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return len(m.aircraft)
}