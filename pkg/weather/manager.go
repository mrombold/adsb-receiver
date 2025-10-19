package weather

import (
	"log"
	"sync"
	"time"
)

// Manager manages raw FIS-B weather frames
type Manager struct {
	frames  [][]byte // Raw UAT uplink frames
	mu      sync.RWMutex
	updates chan []byte
}

// NewManager creates a new weather manager
func NewManager() *Manager {
	return &Manager{
		frames:  make([][]byte, 0, 100),
		updates: make(chan []byte, 100),
	}
}

// Run is the main loop
func (m *Manager) Run() {
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case frame := <-m.updates:
			// Store raw frame
			m.mu.Lock()
			m.frames = append(m.frames, frame)
			
			// Limit stored frames to prevent memory bloat
			if len(m.frames) > 100 {
				m.frames = m.frames[1:]
			}
			m.mu.Unlock()
			
			log.Printf("Weather: Received frame (%d bytes)", len(frame))

		case <-ticker.C:
			// Periodic cleanup - clear old frames
			m.mu.Lock()
			// Keep frames fresh - weather data is continuously updated
			// Clear buffer periodically to ensure fresh data
			if len(m.frames) > 0 {
				log.Printf("Weather: Clearing %d old frames", len(m.frames))
				m.frames = make([][]byte, 0, 100)
			}
			m.mu.Unlock()
		}
	}
}

// Updates returns the channel for sending weather frame updates
func (m *Manager) Updates() chan<- []byte {
	return m.updates
}

// GetFrames returns all current weather frames and clears the buffer
// This prevents sending duplicate frames
func (m *Manager) GetFrames() [][]byte {
	m.mu.Lock()
	defer m.mu.Unlock()

	if len(m.frames) == 0 {
		return nil
	}

	// Return frames and clear buffer
	frames := make([][]byte, len(m.frames))
	copy(frames, m.frames)
	m.frames = make([][]byte, 0, 100)

	return frames
}

// HasWeather returns true if weather frames are available
func (m *Manager) HasWeather() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return len(m.frames) > 0
}

// Count returns the number of buffered weather frames
func (m *Manager) Count() int {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return len(m.frames)
}