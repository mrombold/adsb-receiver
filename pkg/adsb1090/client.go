package adsb1090

import (
    "bufio"
    "log"
    "net"
    "strconv"
    "strings"
    "sync"
    "time"
)

type Aircraft struct {
    ICAO      string
    Callsign  string
    Altitude  int
    Lat       float64
    Lon       float64
    Track     int
    Speed     int
    Timestamp time.Time
}

type Client struct {
    addr     string
    aircraft map[string]*Aircraft  // Store partial aircraft data by ICAO
    mu       sync.Mutex
}

func NewClient(addr string) *Client {
    return &Client{
        addr:     addr,
        aircraft: make(map[string]*Aircraft),
    }
}

func (c *Client) Read(updates chan<- Aircraft) error {
    // Clean up stale aircraft every 60 seconds
    go func() {
        ticker := time.NewTicker(60 * time.Second)
        for range ticker.C {
            c.mu.Lock()
            now := time.Now()
            for icao, ac := range c.aircraft {
                if now.Sub(ac.Timestamp) > 5*time.Minute {
                    delete(c.aircraft, icao)
                }
            }
            c.mu.Unlock()
        }
    }()
    
    for {
        conn, err := net.Dial("tcp", c.addr)
        if err != nil {
            log.Printf("dump1090 connection failed: %v, retrying in 5s", err)
            time.Sleep(5 * time.Second)
            continue
        }
        
        log.Printf("Connected to dump1090 at %s", c.addr)
        
        scanner := bufio.NewScanner(conn)
        for scanner.Scan() {
            line := scanner.Text()
            if ac, ok := c.parseBaseStation(line); ok {
                // Only send if we have position data
                if ac.Lat != 0 && ac.Lon != 0 {
                    updates <- ac
                }
            }
        }
        
        if err := scanner.Err(); err != nil {
            log.Printf("dump1090 read error: %v", err)
        }
        
        conn.Close()
        log.Printf("Disconnected from dump1090, reconnecting...")
        time.Sleep(2 * time.Second)
    }
}

func (c *Client) parseBaseStation(line string) (Aircraft, bool) {
    fields := strings.Split(line, ",")
    
    if len(fields) < 11 || fields[0] != "MSG" {
        return Aircraft{}, false
    }
    
    msgType := fields[1]
    icao := strings.TrimSpace(fields[4])
    
    if icao == "" {
        return Aircraft{}, false
    }
    
    c.mu.Lock()
    defer c.mu.Unlock()
    
    // Get or create aircraft entry
    ac, exists := c.aircraft[icao]
    if !exists {
        ac = &Aircraft{
            ICAO:      icao,
            Timestamp: time.Now(),
        }
        c.aircraft[icao] = ac
    }
    
    ac.Timestamp = time.Now()
    
    // MSG type 1: Callsign
    if msgType == "1" && len(fields) >= 11 {
        if callsign := strings.TrimSpace(fields[10]); callsign != "" {
            ac.Callsign = callsign
        }
    }
    
    // MSG type 3: Position (lat/lon/alt)
    if msgType == "3" && len(fields) >= 16 {
        if fields[11] != "" {
            ac.Altitude, _ = strconv.Atoi(fields[11])
        }
        
        if fields[14] != "" && fields[15] != "" {
            ac.Lat, _ = strconv.ParseFloat(fields[14], 64)
            ac.Lon, _ = strconv.ParseFloat(fields[15], 64)
        }
        
        // MSG type 3 position is good enough to send
        return *ac, true
    }
    
    // MSG type 4: Velocity (speed/track)
    if msgType == "4" && len(fields) >= 14 {
        if fields[12] != "" {
            speed, _ := strconv.ParseFloat(fields[12], 64)
            ac.Speed = int(speed)
        }
        
        if fields[13] != "" {
            track, _ := strconv.ParseFloat(fields[13], 64)
            ac.Track = int(track)
        }
        
        // Don't send MSG type 4 alone, wait for position
        return Aircraft{}, false
    }
    
    return Aircraft{}, false
}