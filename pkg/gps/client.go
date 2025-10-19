package gps

import (
    "bufio"
    "encoding/json"
    "log"
    "net"
    "time"
)

type Position struct {
    Lat          float64
    Lon          float64
    Altitude     float64  // meters MSL
    AltitudeHAE  float64  // meters above WGS84 Ellipsoid
    Track        float64
    Speed        float64  // knots
    Climb        float64  // ft/min
    Mode         int
    Timestamp    time.Time
}

type TPVMessage struct {
    Class   string  `json:"class"`
    Mode    int     `json:"mode"`
    Time    string  `json:"time"`
    Lat     float64 `json:"lat"`
    Lon     float64 `json:"lon"`
    Alt     float64 `json:"alt"`
    AltMSL  float64 `json:"altMSL"`
    AltHAE  float64 `json:"altHAE"`
    Track   float64 `json:"track"`
    Speed   float64 `json:"speed"`
    Climb   float64 `json:"climb"`
}

type Client struct {
    addr string
}

func NewClient(addr string) *Client {
    return &Client{addr: addr}
}

func (c *Client) Read(updates chan<- Position) error {
    for {
        conn, err := net.Dial("tcp", c.addr)
        if err != nil {
            log.Printf("gpsd connection failed: %v, retrying in 5s", err)
            time.Sleep(5 * time.Second)
            continue
        }
        
        log.Printf("Connected to gpsd at %s", c.addr)
        
        conn.Write([]byte("?WATCH={\"enable\":true,\"json\":true}\n"))
        
        scanner := bufio.NewScanner(conn)
        
        for scanner.Scan() {
            line := scanner.Bytes()
            
            var msg TPVMessage
            if err := json.Unmarshal(line, &msg); err != nil {
                continue
            }
            
            if msg.Class == "TPV" && msg.Mode >= 2 {
                // Prefer altMSL, fall back to Alt
                altitude := msg.AltMSL
                if altitude == 0 {
                    altitude = msg.Alt
                }
                
                pos := Position{
                    Lat:          msg.Lat,
                    Lon:          msg.Lon,
                    Altitude:     altitude,  // meters MSL
                    AltitudeHAE:  msg.AltHAE,
                    Track:        msg.Track,
                    Speed:        msg.Speed * 1.94384, // m/s to knots
                    Climb:        msg.Climb * 196.85,  // m/s to ft/min
                    Mode:         msg.Mode,
                    Timestamp:    time.Now(),
                }
                                
                select {
                case updates <- pos:
                default:
                    // Channel full
                }
            }
        }
        
        if err := scanner.Err(); err != nil {
            log.Printf("gpsd read error: %v", err)
        }
        
        conn.Close()
        log.Printf("Disconnected from gpsd, reconnecting...")
        time.Sleep(2 * time.Second)
    }
}