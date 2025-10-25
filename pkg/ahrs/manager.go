package ahrs

import (
	"bufio"
	"encoding/json"
	"log"
	"net"
	"sync"
	"time"
)

// Data represents AHRS attitude data
type Data struct {
	Timestamp time.Time `json:"timestamp"`
	Roll      float64   `json:"roll"`      // degrees
	Pitch     float64   `json:"pitch"`     // degrees
	Yaw       float64   `json:"yaw"`       // degrees (heading)
	RollRate  float64   `json:"roll_rate"` // deg/s
	PitchRate float64   `json:"pitch_rate"` // deg/s
	YawRate   float64   `json:"yaw_rate"`  // deg/s
}

// Manager handles AHRS data from Unix socket
type Manager struct {
	socketPath string
	conn       net.Conn
	mu         sync.RWMutex
	current    Data
	hasData    bool
	lastUpdate time.Time
}

// NewManager creates a new AHRS manager
func NewManager(socketPath string) *Manager {
	return &Manager{
		socketPath: socketPath,
	}
}

// Run starts reading AHRS data from the Unix socket
func (m *Manager) Run() {
	for {
		// Try to connect
		conn, err := net.Dial("unix", m.socketPath)
		if err != nil {
			time.Sleep(2 * time.Second)
			continue
		}
		
		m.conn = conn
		log.Printf("AHRS: Connected to %s", m.socketPath)
		
		// Read data
		scanner := bufio.NewScanner(conn)
		for scanner.Scan() {
			line := scanner.Text()
			
			var data Data
			if err := json.Unmarshal([]byte(line), &data); err != nil {
				log.Printf("AHRS: JSON parse error: %v", err)
				continue
			}
			
			m.mu.Lock()
			m.current = data
			m.hasData = true
			m.lastUpdate = time.Now()
			m.mu.Unlock()
		}
		
		// Connection lost
		if err := scanner.Err(); err != nil {
			log.Printf("AHRS: Read error: %v", err)
		}
		
		conn.Close()
		m.conn = nil
		
		m.mu.Lock()
		m.hasData = false
		m.mu.Unlock()
		
		//log.Println("AHRS: Connection lost, reconnecting...")
		time.Sleep(1 * time.Second)
	}
}

// GetData returns the current AHRS data
func (m *Manager) GetData() (Data, bool) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	
	// Consider data stale after 1 second
	if m.hasData && time.Since(m.lastUpdate) < 1*time.Second {
		return m.current, true
	}
	
	return Data{}, false
}

// HasData returns true if valid AHRS data is available
func (m *Manager) HasData() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.hasData && time.Since(m.lastUpdate) < 1*time.Second
}