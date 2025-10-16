package position

import (
	"sync"
	"time"
	
	"adsb-receiver/pkg/gps"
)

// Manager manages ownship position state
type Manager struct {
	position gps.Position
	mu       sync.RWMutex
	updates  chan gps.Position
	hasPos   bool
}

// NewManager creates a new position manager
func NewManager() *Manager {
	return &Manager{
		updates: make(chan gps.Position, 10),
	}
}

// Run is the main loop
func (m *Manager) Run() {
	for update := range m.updates {
		m.mu.Lock()
		m.position = update
		m.hasPos = true
		m.mu.Unlock()
	}
}

// Updates returns the channel for sending position updates
func (m *Manager) Updates() chan<- gps.Position {
	return m.updates
}

// GetPosition returns the current position
func (m *Manager) GetPosition() (gps.Position, bool) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.position, m.hasPos
}

// HasPosition returns true if we have a valid position
func (m *Manager) HasPosition() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.hasPos && time.Since(m.position.Timestamp) < 10*time.Second
}
