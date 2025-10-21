package weather

import (
    "log"
    "sync"
    "time"
)

// Frame represents a UAT uplink frame with metadata
type Frame struct {
    Data      []byte
    Timestamp time.Time
}

// Manager manages raw FIS-B weather frames
type Manager struct {
    frames  map[string]*Frame  // Key: frame signature (first 8 bytes)
    mu      sync.RWMutex
    updates chan []byte
}

// NewManager creates a new weather manager
func NewManager() *Manager {
    return &Manager{
        frames:  make(map[string]*Frame),
        updates: make(chan []byte, 100),
    }
}

// Run is the main loop
func (m *Manager) Run() {
    ticker := time.NewTicker(10 * time.Second)
    defer ticker.Stop()

    for {
        select {
        case frame := <-m.updates:
            m.storeFrame(frame)

        case <-ticker.C:
            m.cleanupOldFrames()
        }
    }
}

// storeFrame stores a frame with deduplication
func (m *Manager) storeFrame(data []byte) {
    if len(data) < 8 {
        log.Printf("Weather Manager: Frame too short (%d bytes), ignoring", len(data))
        return
    }

    // Use first 8 bytes (UAT header) as signature for deduplication
    sig := string(data[:8])

    m.mu.Lock()
    defer m.mu.Unlock()

    // Check if this is a new frame or update
    if existing, ok := m.frames[sig]; ok {
        // Same frame received again - just update timestamp
        existing.Timestamp = time.Now()
    } else {
        // New frame
        m.frames[sig] = &Frame{
            Data:      data,
            Timestamp: time.Now(),
        }
        
    }

    // Limit total frames to prevent memory bloat
    if len(m.frames) > 100 {
        m.removeOldestFrame()
    }
}

// removeOldestFrame removes the oldest frame when limit is reached
func (m *Manager) removeOldestFrame() {
    var oldestKey string
    var oldestTime time.Time

    for key, frame := range m.frames {
        if oldestKey == "" || frame.Timestamp.Before(oldestTime) {
            oldestKey = key
            oldestTime = frame.Timestamp
        }
    }

    if oldestKey != "" {
        delete(m.frames, oldestKey)
        
    }
}

// cleanupOldFrames removes frames older than 60 seconds
func (m *Manager) cleanupOldFrames() {
    m.mu.Lock()
    defer m.mu.Unlock()

    now := time.Now()
    removed := 0

    for key, frame := range m.frames {
        if now.Sub(frame.Timestamp) > 60*time.Second {
            delete(m.frames, key)
            removed++
        }
    }

}

// Updates returns the channel for sending weather frame updates
func (m *Manager) Updates() chan<- []byte {
    return m.updates
}

// GetFrames returns all current weather frames WITHOUT clearing
// Frames are sent repeatedly until they expire (60s)
func (m *Manager) GetFrames() [][]byte {
    m.mu.RLock()
    defer m.mu.RUnlock()

    if len(m.frames) == 0 {
        return nil
    }

    // Return all frames without clearing
    frames := make([][]byte, 0, len(m.frames))
    for _, frame := range m.frames {
        frames = append(frames, frame.Data)
    }

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

// GetFrameAge returns the age of the oldest frame
func (m *Manager) GetFrameAge() time.Duration {
    m.mu.RLock()
    defer m.mu.RUnlock()

    if len(m.frames) == 0 {
        return 0
    }

    now := time.Now()
    var oldest time.Time

    for _, frame := range m.frames {
        if oldest.IsZero() || frame.Timestamp.Before(oldest) {
            oldest = frame.Timestamp
        }
    }

    return now.Sub(oldest)
}