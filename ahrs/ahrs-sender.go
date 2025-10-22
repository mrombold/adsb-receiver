package main

import (
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net"
	"os"
	"os/signal"
	"syscall"
	"time"
	"sync"

	"golang.org/x/exp/io/i2c"
)

const (
	// MMC5983MA registers and constants
	mmcAddress    = 0x30
	mmcWhoAmI     = 0x2F
	mmcControl0   = 0x09
	mmcControl1   = 0x0A
	mmcControl2   = 0x0B
	mmcXOut0      = 0x00
	mmcExpectedID = 0x30

	// ISM330DHCX registers and constants
	ismAddress    = 0x6B
	ismWhoAmI     = 0x0F
	ismCtrl1XL    = 0x10
	ismCtrl2G     = 0x11
	ismCtrl3C     = 0x12
	ismStatusReg  = 0x1E
	ismOutXLG     = 0x22
	ismOutXLA     = 0x28
	ismExpectedID = 0x6B

)

// Quaternion represents orientation
type Quaternion struct {
	W, X, Y, Z float64
}

// EulerAngles represents roll, pitch, yaw in radians
type EulerAngles struct {
	Roll  float64 // Rotation around X axis
	Pitch float64 // Rotation around Y axis
	Yaw   float64 // Rotation around Z axis (heading)
}

// MahonyAHRS implements the Mahony sensor fusion algorithm
type MahonyAHRS struct {
	q           Quaternion // Orientation quaternion
	kp          float64    // Proportional gain
	ki          float64    // Integral gain
	integralFBx float64    // Integral feedback terms
	integralFBy float64
	integralFBz float64
	sampleFreq  float64 // Sample frequency in Hz
}

// AHRSData represents the current attitude
type AHRSData struct {
    Timestamp time.Time `json:"timestamp"`
    Roll      float64   `json:"roll"`      // degrees
    Pitch     float64   `json:"pitch"`     // degrees
    Yaw       float64   `json:"yaw"`       // degrees (heading)
    RollRate  float64   `json:"roll_rate"` // deg/s
    PitchRate float64   `json:"pitch_rate"` // deg/s
    YawRate   float64   `json:"yaw_rate"`  // deg/s
}

// AHRSPublisher publishes AHRS data via Unix socket
type AHRSPublisher struct {
    socketPath string
    listener   net.Listener
    clients    []net.Conn
    clientsMu  sync.Mutex
    data       AHRSData
    dataMu     sync.RWMutex
}


// NewMahonyAHRS creates a new Mahony AHRS filter
func NewMahonyAHRS(sampleFreq float64) *MahonyAHRS {
	return &MahonyAHRS{
		q:          Quaternion{W: 1.0, X: 0.0, Y: 0.0, Z: 0.0}, // Identity quaternion
		kp:         2.0,  // Proportional gain (tune for response speed)
		ki:         0.01, // Integral gain (tune for drift compensation)
		sampleFreq: sampleFreq,
	}
}

// Update performs one iteration of the Mahony AHRS algorithm
// gx, gy, gz: gyroscope in rad/s
// ax, ay, az: accelerometer in m/s² (any units, will be normalized)
// mx, my, mz: magnetometer in μT (any units, will be normalized)
func (m *MahonyAHRS) Update(gx, gy, gz, ax, ay, az, mx, my, mz float64) {
	dt := 1.0 / m.sampleFreq
	q0, q1, q2, q3 := m.q.W, m.q.X, m.q.Y, m.q.Z

	// Normalize accelerometer measurement
	recipNorm := 1.0 / math.Sqrt(ax*ax+ay*ay+az*az)
	ax *= recipNorm
	ay *= recipNorm
	az *= recipNorm

	// Normalize magnetometer measurement
	recipNorm = 1.0 / math.Sqrt(mx*mx+my*my+mz*mz)
	mx *= recipNorm
	my *= recipNorm
	mz *= recipNorm

	// Reference direction of Earth's magnetic field
	hx := 2.0*mx*(0.5-q2*q2-q3*q3) + 2.0*my*(q1*q2-q0*q3) + 2.0*mz*(q1*q3+q0*q2)
	hy := 2.0*mx*(q1*q2+q0*q3) + 2.0*my*(0.5-q1*q1-q3*q3) + 2.0*mz*(q2*q3-q0*q1)
	bx := math.Sqrt(hx*hx + hy*hy)
	bz := 2.0*mx*(q1*q3-q0*q2) + 2.0*my*(q2*q3+q0*q1) + 2.0*mz*(0.5-q1*q1-q2*q2)

	// Estimated direction of gravity and magnetic field
	vx := 2.0 * (q1*q3 - q0*q2)
	vy := 2.0 * (q0*q1 + q2*q3)
	vz := q0*q0 - q1*q1 - q2*q2 + q3*q3
	wx := 2.0*bx*(0.5-q2*q2-q3*q3) + 2.0*bz*(q1*q3-q0*q2)
	wy := 2.0*bx*(q1*q2-q0*q3) + 2.0*bz*(q0*q1+q2*q3)
	wz := 2.0*bx*(q0*q2+q1*q3) + 2.0*bz*(0.5-q1*q1-q2*q2)

	// Error is cross product between estimated and measured direction of gravity and magnetic field
	ex := (ay*vz - az*vy) + (my*wz - mz*wy)
	ey := (az*vx - ax*vz) + (mz*wx - mx*wz)
	ez := (ax*vy - ay*vx) + (mx*wy - my*wx)

	// Integral feedback
	if m.ki > 0.0 {
		m.integralFBx += m.ki * ex * dt
		m.integralFBy += m.ki * ey * dt
		m.integralFBz += m.ki * ez * dt
		gx += m.integralFBx
		gy += m.integralFBy
		gz += m.integralFBz
	}

	// Proportional feedback
	gx += m.kp * ex
	gy += m.kp * ey
	gz += m.kp * ez

	// Cap gyro error corrections at 0.1 rad/sec to prevent large corrections
	const maxGyroError = 0.1
	gx = clamp(gx, -maxGyroError, maxGyroError)
	gy = clamp(gy, -maxGyroError, maxGyroError)
	gz = clamp(gz, -maxGyroError, maxGyroError)

	// Integrate rate of change of quaternion
	gx *= 0.5 * dt
	gy *= 0.5 * dt
	gz *= 0.5 * dt
	qa := q0
	qb := q1
	qc := q2
	q0 += (-qb*gx - qc*gy - q3*gz)
	q1 += (qa*gx + qc*gz - q3*gy)
	q2 += (qa*gy - qb*gz + q3*gx)
	q3 += (qa*gz + qb*gy - qc*gx)

	// Normalize quaternion
	recipNorm = 1.0 / math.Sqrt(q0*q0+q1*q1+q2*q2+q3*q3)
	m.q.W = q0 * recipNorm
	m.q.X = q1 * recipNorm
	m.q.Y = q2 * recipNorm
	m.q.Z = q3 * recipNorm
}

// UpdateIMU performs one iteration without magnetometer (6DOF mode)
func (m *MahonyAHRS) UpdateIMU(gx, gy, gz, ax, ay, az float64) {
	dt := 1.0 / m.sampleFreq
	q0, q1, q2, q3 := m.q.W, m.q.X, m.q.Y, m.q.Z

	// Normalize accelerometer measurement
	recipNorm := 1.0 / math.Sqrt(ax*ax+ay*ay+az*az)
	ax *= recipNorm
	ay *= recipNorm
	az *= recipNorm

	// Estimated direction of gravity
	vx := 2.0 * (q1*q3 - q0*q2)
	vy := 2.0 * (q0*q1 + q2*q3)
	vz := q0*q0 - q1*q1 - q2*q2 + q3*q3

	// Error is cross product between estimated and measured direction of gravity
	ex := ay*vz - az*vy
	ey := az*vx - ax*vz
	ez := ax*vy - ay*vx

	// Integral feedback
	if m.ki > 0.0 {
		m.integralFBx += m.ki * ex * dt
		m.integralFBy += m.ki * ey * dt
		m.integralFBz += m.ki * ez * dt
		gx += m.integralFBx
		gy += m.integralFBy
		gz += m.integralFBz
	}

	// Proportional feedback
	gx += m.kp * ex
	gy += m.kp * ey
	gz += m.kp * ez

	// Cap gyro error corrections
	const maxGyroError = 0.1
	gx = clamp(gx, -maxGyroError, maxGyroError)
	gy = clamp(gy, -maxGyroError, maxGyroError)
	gz = clamp(gz, -maxGyroError, maxGyroError)

	// Integrate rate of change of quaternion
	gx *= 0.5 * dt
	gy *= 0.5 * dt
	gz *= 0.5 * dt
	qa := q0
	qb := q1
	qc := q2
	q0 += (-qb*gx - qc*gy - q3*gz)
	q1 += (qa*gx + qc*gz - q3*gy)
	q2 += (qa*gy - qb*gz + q3*gx)
	q3 += (qa*gz + qb*gy - qc*gx)

	// Normalize quaternion
	recipNorm = 1.0 / math.Sqrt(q0*q0+q1*q1+q2*q2+q3*q3)
	m.q.W = q0 * recipNorm
	m.q.X = q1 * recipNorm
	m.q.Y = q2 * recipNorm
	m.q.Z = q3 * recipNorm
}

// GetEulerAngles converts quaternion to Euler angles (roll, pitch, yaw)
func (m *MahonyAHRS) GetEulerAngles() EulerAngles {
	q0, q1, q2, q3 := m.q.W, m.q.X, m.q.Y, m.q.Z

	// Roll (x-axis rotation)
	sinr_cosp := 2.0 * (q0*q1 + q2*q3)
	cosr_cosp := 1.0 - 2.0*(q1*q1+q2*q2)
	roll := math.Atan2(sinr_cosp, cosr_cosp)

	// Pitch (y-axis rotation)
	sinp := 2.0 * (q0*q2 - q3*q1)
	var pitch float64
	if math.Abs(sinp) >= 1.0 {
		pitch = math.Copysign(math.Pi/2, sinp)
	} else {
		pitch = math.Asin(sinp)
	}

	// Yaw (z-axis rotation)
	siny_cosp := 2.0 * (q0*q3 + q1*q2)
	cosy_cosp := 1.0 - 2.0*(q2*q2+q3*q3)
	yaw := math.Atan2(siny_cosp, cosy_cosp)

	return EulerAngles{
		Roll:  roll,
		Pitch: pitch,
		Yaw:   yaw,
	}
}


// Helper functions
func clamp(val, min, max float64) float64 {
	if val < min {
		return min
	}
	if val > max {
		return max
	}
	return val
}

func round(x float64) float64 {
	return math.Floor(x + 0.5)
}

// IMU initialization and reading functions
func initMMC(dev *i2c.Device) error {
	buf := make([]byte, 1)
	if err := dev.ReadReg(mmcWhoAmI, buf); err != nil {
		return fmt.Errorf("read WHO_AM_I failed: %w", err)
	}
	if buf[0] != mmcExpectedID {
		return fmt.Errorf("invalid WHO_AM_I: expected 0x%02X, got 0x%02X", mmcExpectedID, buf[0])
	}

	if err := dev.WriteReg(mmcControl1, []byte{0x80}); err != nil {
		return err
	}
	time.Sleep(10 * time.Millisecond)

	if err := dev.WriteReg(mmcControl0, []byte{0x20}); err != nil {
		return err
	}

	if err := dev.WriteReg(mmcControl2, []byte{0x81}); err != nil {
		return err
	}

	if err := dev.WriteReg(mmcControl0, []byte{0x08}); err != nil {
		return err
	}
	time.Sleep(1 * time.Millisecond)

	if err := dev.WriteReg(mmcControl0, []byte{0x10}); err != nil {
		return err
	}
	time.Sleep(1 * time.Millisecond)

	return nil
}

func initISM(dev *i2c.Device) error {
	buf := make([]byte, 1)
	if err := dev.ReadReg(ismWhoAmI, buf); err != nil {
		return fmt.Errorf("read WHO_AM_I failed: %w", err)
	}
	if buf[0] != ismExpectedID {
		return fmt.Errorf("invalid WHO_AM_I: expected 0x%02X, got 0x%02X", ismExpectedID, buf[0])
	}

	if err := dev.WriteReg(ismCtrl3C, []byte{0x01}); err != nil {
		return err
	}
	time.Sleep(50 * time.Millisecond)

	if err := dev.WriteReg(ismCtrl3C, []byte{0x44}); err != nil {
		return err
	}

	// 416 Hz, ±16g
	if err := dev.WriteReg(ismCtrl1XL, []byte{0x64}); err != nil {
		return err
	}

	// 416 Hz, ±2000 dps
	if err := dev.WriteReg(ismCtrl2G, []byte{0x6C}); err != nil {
		return err
	}

	time.Sleep(10 * time.Millisecond)
	return nil
}

func readMMC(dev *i2c.Device) ([3]float64, error) {
	if err := dev.WriteReg(mmcControl0, []byte{0x01}); err != nil {
		return [3]float64{}, err
	}

	time.Sleep(10 * time.Millisecond)

	buf := make([]byte, 7)
	if err := dev.ReadReg(mmcXOut0, buf); err != nil {
		return [3]float64{}, err
	}

	rawX := (uint32(buf[0]) << 10) | (uint32(buf[1]) << 2) | (uint32(buf[6]) >> 6)
	rawY := (uint32(buf[2]) << 10) | (uint32(buf[3]) << 2) | ((uint32(buf[6]) >> 4) & 0x03)
	rawZ := (uint32(buf[4]) << 10) | (uint32(buf[5]) << 2) | ((uint32(buf[6]) >> 2) & 0x03)

	const resolution = 262144.0
	const range_uT = 800.0

	x := (float64(rawX) - resolution/2) / (resolution / 2) * range_uT
	y := (float64(rawY) - resolution/2) / (resolution / 2) * range_uT
	z := (float64(rawZ) - resolution/2) / (resolution / 2) * range_uT

	return [3]float64{x, y, z}, nil
}

func readISM(dev *i2c.Device) ([3]float64, [3]float64, error) {
	deadline := time.Now().Add(50 * time.Millisecond)
	for time.Now().Before(deadline) {
		buf := make([]byte, 1)
		if err := dev.ReadReg(ismStatusReg, buf); err != nil {
			return [3]float64{}, [3]float64{}, err
		}
		if buf[0]&0x03 == 0x03 {
			break
		}
		time.Sleep(1 * time.Millisecond)
	}

	gyroBuf := make([]byte, 6)
	if err := dev.ReadReg(ismOutXLG, gyroBuf); err != nil {
		return [3]float64{}, [3]float64{}, err
	}

	rawGyroX := int16(gyroBuf[0]) | int16(gyroBuf[1])<<8
	rawGyroY := int16(gyroBuf[2]) | int16(gyroBuf[3])<<8
	rawGyroZ := int16(gyroBuf[4]) | int16(gyroBuf[5])<<8

	const gyroSens = 16.4
	const degToRad = 3.141592653589793 / 180.0

	gyroX := float64(rawGyroX) / gyroSens * degToRad
	gyroY := float64(rawGyroY) / gyroSens * degToRad
	gyroZ := float64(rawGyroZ) / gyroSens * degToRad

	accelBuf := make([]byte, 6)
	if err := dev.ReadReg(ismOutXLA, accelBuf); err != nil {
		return [3]float64{}, [3]float64{}, err
	}

	rawAccelX := int16(accelBuf[0]) | int16(accelBuf[1])<<8
	rawAccelY := int16(accelBuf[2]) | int16(accelBuf[3])<<8
	rawAccelZ := int16(accelBuf[4]) | int16(accelBuf[5])<<8

	const accelSens = 2048.0
	const gravity = 9.80665

	accelX := float64(rawAccelX) / accelSens * gravity
	accelY := float64(rawAccelY) / accelSens * gravity
	accelZ := float64(rawAccelZ) / accelSens * gravity

	return [3]float64{accelX, accelY, accelZ}, [3]float64{gyroX, gyroY, gyroZ}, nil
}



// NewAHRSPublisher creates a new AHRS publisher
func NewAHRSPublisher(socketPath string) (*AHRSPublisher, error) {
    // Remove existing socket file if it exists
    os.Remove(socketPath)
    
    listener, err := net.Listen("unix", socketPath)
    if err != nil {
        return nil, fmt.Errorf("failed to create socket: %w", err)
    }
    
    // Set socket permissions
    os.Chmod(socketPath, 0666)
    
    return &AHRSPublisher{
        socketPath: socketPath,
        listener:   listener,
        clients:    make([]net.Conn, 0),
    }, nil
}

// UpdateData updates the current AHRS data
func (p *AHRSPublisher) UpdateData(roll, pitch, yaw, rollRate, pitchRate, yawRate float64) {
    p.dataMu.Lock()
    defer p.dataMu.Unlock()
    
    p.data = AHRSData{
        Timestamp: time.Now(),
        Roll:      roll,
        Pitch:     pitch,
        Yaw:       yaw,
        RollRate:  rollRate,
        PitchRate: pitchRate,
        YawRate:   yawRate,
    }
}

// GetData returns the current AHRS data
func (p *AHRSPublisher) GetData() AHRSData {
    p.dataMu.RLock()
    defer p.dataMu.RUnlock()
    return p.data
}

// Run starts accepting client connections
func (p *AHRSPublisher) Run() {
    go func() {
        for {
            conn, err := p.listener.Accept()
            if err != nil {
                log.Printf("Accept error: %v", err)
                continue
            }
            
            p.clientsMu.Lock()
            p.clients = append(p.clients, conn)
            p.clientsMu.Unlock()
            
            log.Printf("Client connected (total: %d)", len(p.clients))
        }
    }()
}

// Broadcast sends current data to all connected clients
func (p *AHRSPublisher) Broadcast() {
    data := p.GetData()
    
    jsonData, err := json.Marshal(data)
    if err != nil {
        log.Printf("JSON marshal error: %v", err)
        return
    }
    
    // Add newline delimiter
    jsonData = append(jsonData, '\n')
    
    p.clientsMu.Lock()
    defer p.clientsMu.Unlock()
    
    // Send to all clients, remove dead connections
    activeClients := make([]net.Conn, 0, len(p.clients))
    for _, client := range p.clients {
        client.SetWriteDeadline(time.Now().Add(100 * time.Millisecond))
        _, err := client.Write(jsonData)
        if err == nil {
            activeClients = append(activeClients, client)
        } else {
            client.Close()
        }
    }
    p.clients = activeClients
}

// Close closes the publisher
func (p *AHRSPublisher) Close() error {
    p.clientsMu.Lock()
    defer p.clientsMu.Unlock()
    
    for _, client := range p.clients {
        client.Close()
    }
    
    if p.listener != nil {
        p.listener.Close()
    }
    
    os.Remove(p.socketPath)
    return nil
}




func main() {
	fmt.Println("=== AHRS GDL90 Sender ===\n")

	// Configuration
	const (
		sampleRate   = 100.0 // Hz
		ahrsRate     = 5.0   // Hz (ForeFlight recommends 5Hz for AHRS)
		i2cBus       = "/dev/i2c-1"
		useMag       = true  // Set to false for IMU-only (6DOF) mode
	)

	// Initialize MMC5983MA magnetometer
	fmt.Println("Initializing MMC5983MA magnetometer...")
	mmc, err := i2c.Open(&i2c.Devfs{Dev: i2cBus}, mmcAddress)
	if err != nil {
		log.Fatalf("Failed to open MMC5983MA: %v", err)
	}
	defer mmc.Close()

	if err := initMMC(mmc); err != nil {
		log.Fatalf("Failed to initialize MMC5983MA: %v", err)
	}
	fmt.Println("✓ Magnetometer initialized")

	// Initialize ISM330DHCX IMU
	fmt.Println("Initializing ISM330DHCX IMU...")
	ism, err := i2c.Open(&i2c.Devfs{Dev: i2cBus}, ismAddress)
	if err != nil {
		log.Fatalf("Failed to open ISM330DHCX: %v", err)
	}
	defer ism.Close()

	if err := initISM(ism); err != nil {
		log.Fatalf("Failed to initialize ISM330DHCX: %v", err)
	}
	fmt.Println("✓ IMU initialized")

	// Create Mahony AHRS filter
	ahrs := NewMahonyAHRS(sampleRate)
	fmt.Printf("✓ Mahony AHRS initialized (%.0f Hz)\n\n", sampleRate)

	// Create AHRS publisher
	const socketPath = "/tmp/ahrs.sock"
	publisher, err := NewAHRSPublisher(socketPath)
	if err != nil {
		log.Fatalf("Failed to create AHRS publisher: %v", err)
	}
	defer publisher.Close()

	publisher.Run()
	fmt.Printf("✓ AHRS publisher initialized (socket: %s)\n", socketPath)

	// Setup graceful shutdown
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)

	// Main loop
	sampleTicker := time.NewTicker(time.Duration(1000000/sampleRate) * time.Microsecond)
	defer sampleTicker.Stop()

	broadcastTicker := time.NewTicker(time.Duration(1000/ahrsRate) * time.Millisecond)
	defer broadcastTicker.Stop()

	fmt.Println("Starting AHRS data collection and transmission...")
	fmt.Printf("Sample rate: %.0f Hz, AHRS send rate: %.0f Hz\n", sampleRate, ahrsRate)
	if useMag {
		fmt.Println("Mode: 9DOF (gyro + accel + mag)")
	} else {
		fmt.Println("Mode: 6DOF (gyro + accel only)")
	}
	fmt.Println("Press Ctrl+C to stop\n")

	for {
		select {
		case <-sigChan:
			fmt.Println("\n\nShutdown signal received")
			return

		case <-sampleTicker.C:
			// Read IMU data
			accel, gyro, err := readISM(ism)
			if err != nil {
				log.Printf("IMU read error: %v", err)
				continue
			}

			if useMag {
				// Read magnetometer data
				mag, err := readMMC(mmc)
				if err != nil {
					// Fall back to IMU-only mode for this sample
					ahrs.UpdateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2])
				} else {
					// Update AHRS with all 9 DOF
					ahrs.Update(
						gyro[0], gyro[1], gyro[2],    // Gyroscope (rad/s)
						accel[0], accel[1], accel[2], // Accelerometer (m/s²)
						mag[0], mag[1], mag[2],       // Magnetometer (μT)
					)
				}
			} else {
				// IMU-only mode (6 DOF)
				ahrs.UpdateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2])
			}

			// Get latest orientation
			euler := ahrs.GetEulerAngles()
            rollDeg := euler.Roll * 180.0 / math.Pi
            pitchDeg := euler.Pitch * 180.0 / math.Pi
            yawDeg := euler.Yaw * 180.0 / math.Pi
            
            heading := math.Mod(yawDeg, 360.0)
            if heading < 0 {
                heading += 360.0
            }
            
            // Get gyro rates (convert to deg/s)
            rollRate := gyro[0] * 180.0 / math.Pi
            pitchRate := gyro[1] * 180.0 / math.Pi
            yawRate := gyro[2] * 180.0 / math.Pi
            
            publisher.UpdateData(rollDeg, pitchDeg, heading, rollRate, pitchRate, yawRate)
            
        case <-broadcastTicker.C:
            // Broadcast to all connected clients
            publisher.Broadcast()
		}
	}
}