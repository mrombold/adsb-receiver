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

// ========== MAHONY DCM IMPLEMENTATION ==========

// Vec3 represents a 3D vector
type Vec3 struct {
	X, Y, Z float64
}

// Mat3 represents a 3x3 matrix (stored as [row][col])
type Mat3 [3][3]float64

// MahonyFilter implements the Mahony DCM attitude filter
type MahonyFilter struct {
	dcm    Mat3    // Direction Cosine Matrix (rotation from world to body)
	bias   Vec3    // Gyro bias estimate
	KpAcc  float64 // Proportional gain for accelerometer
	KpMag  float64 // Proportional gain for magnetometer
	Ki     float64 // Integral gain for bias correction
}

// EulerAngles represents roll, pitch, yaw in radians
type EulerAngles struct {
	Roll  float64 // Roll angle (radians)
	Pitch float64 // Pitch angle (radians)
	Yaw   float64 // Yaw/Heading angle (radians)
}

// NewMahonyFilter creates a new Mahony filter with default gains
func NewMahonyFilter() *MahonyFilter {
	return &MahonyFilter{
		dcm:    identityMat3(),
		bias:   Vec3{0, 0, 0},
		KpAcc:  5.0,  // Proportional gain for accel
		KpMag:  5.0,  // Proportional gain for mag
		Ki:     0.01, // Integral gain for bias
	}
}

// Update integrates gyro, accel, and optional magnetometer measurements
func (f *MahonyFilter) Update(gyro, accel Vec3, mag *Vec3, dt float64) {
	if dt <= 0.0 || !isFinite(dt) {
		return
	}

	// 1) Compute correction vector in body frame
	errB := f.correctionBody(accel, mag)

	// 2) PI controller on gyro bias
	if f.Ki > 0.0 && errB.allFinite() {
		f.bias = f.bias.add(errB.scale(f.Ki * dt))
	}

	// 3) Corrected angular rate
	omegaB := gyro.add(errB.scale(f.KpAcc)).sub(f.bias)
	if !omegaB.allFinite() {
		return
	}

	// 4) Integrate DCM: R_dot = R * [ω]_x (forward Euler)
	wx := skewSymmetric(omegaB)
	dcmDot := f.dcm.multiply(wx)
	f.dcm = f.dcm.add(dcmDot.scale(dt))

	// 5) Keep DCM orthonormal (Gram-Schmidt on columns)
	f.orthonormalize()
}

// correctionBody computes the correction vector in body frame
func (f *MahonyFilter) correctionBody(accel Vec3, mag *Vec3) Vec3 {
	err := Vec3{0, 0, 0}

	// Gravity: world +Z up
	zWorld := Vec3{0, 0, 1}

	// Predicted gravity direction in body frame (world->body)
	gEstBody := f.dcm.transpose().multiplyVec(zWorld)

	// Measured gravity (normalize accel)
	an := accel.norm()
	if isFinite(an) && an > 0.5 {
		gMeasBody := accel.scale(1.0 / an)
		if gMeasBody.allFinite() && gEstBody.allFinite() {
			err = err.add(gMeasBody.cross(gEstBody))
		}
	}

	// Magnetometer: use horizontal component for heading
	if mag != nil {
		mn := mag.norm()
		if isFinite(mn) && mn > 1e-6 {
			mB := mag.scale(1.0 / mn)

			// Remove vertical component to isolate heading
			mVert := mB.dot(gEstBody)
			mHB := mB.sub(gEstBody.scale(mVert))
			mh := mHB.norm()

			if isFinite(mh) && mh > 1e-6 {
				mHB = mHB.scale(1.0 / mh)

				// World +X expressed in body
				exB := f.dcm.transpose().multiplyVec(Vec3{1, 0, 0})

				// Weight magnetometer relative to accel term
				w := f.KpMag / math.Max(f.KpAcc, 1e-6)
				err = err.add(mHB.cross(exB).scale(w))
			}
		}
	}

	if !err.allFinite() {
		return Vec3{0, 0, 0}
	}
	return err
}

// orthonormalize performs Gram-Schmidt orthonormalization on DCM columns
func (f *MahonyFilter) orthonormalize() {
	c0 := f.dcm.col(0)
	c1 := f.dcm.col(1)

	// 1) Normalize first column
	n0 := c0.norm()
	if isFinite(n0) && n0 > 0.0 {
		c0 = c0.scale(1.0 / n0)
	} else {
		c0 = Vec3{1, 0, 0}
	}

	// 2) Make second column orthogonal to first and normalize
	c1 = c1.sub(c0.scale(c0.dot(c1)))
	n1 := c1.norm()
	if isFinite(n1) && n1 > 0.0 {
		c1 = c1.scale(1.0 / n1)
	} else {
		c1 = Vec3{0, 1, 0}
	}

	// 3) Third column from cross product
	c2 := c0.cross(c1)
	n2 := c2.norm()
	if isFinite(n2) && n2 > 0.0 {
		c2 = c2.scale(1.0 / n2)
	} else {
		c2 = Vec3{0, 0, 1}
	}

	f.dcm.setCol(0, c0)
	f.dcm.setCol(1, c1)
	f.dcm.setCol(2, c2)
}

// GetEuler extracts Euler angles from DCM (Aerospace ZYX: yaw-pitch-roll)
func (f *MahonyFilter) GetEuler() EulerAngles {
	R := f.dcm
	// Clamp for safety against floating point drift
	s := clamp(R[2][0], -1.0, 1.0)
	pitch := -math.Asin(s)
	roll := math.Atan2(R[2][1], R[2][2])
	yaw := math.Atan2(R[1][0], R[0][0])
	return EulerAngles{Roll: roll, Pitch: pitch, Yaw: yaw}
}

// GetBias returns the current gyro bias estimate
func (f *MahonyFilter) GetBias() Vec3 {
	return f.bias
}

// GetDCM returns the current Direction Cosine Matrix
func (f *MahonyFilter) GetDCM() Mat3 {
	return f.dcm
}

// --- Vec3 methods ---

func (v Vec3) add(other Vec3) Vec3 {
	return Vec3{v.X + other.X, v.Y + other.Y, v.Z + other.Z}
}

func (v Vec3) sub(other Vec3) Vec3 {
	return Vec3{v.X - other.X, v.Y - other.Y, v.Z - other.Z}
}

func (v Vec3) scale(s float64) Vec3 {
	return Vec3{v.X * s, v.Y * s, v.Z * s}
}

func (v Vec3) dot(other Vec3) float64 {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

func (v Vec3) cross(other Vec3) Vec3 {
	return Vec3{
		v.Y*other.Z - v.Z*other.Y,
		v.Z*other.X - v.X*other.Z,
		v.X*other.Y - v.Y*other.X,
	}
}

func (v Vec3) norm() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

func (v Vec3) allFinite() bool {
	return isFinite(v.X) && isFinite(v.Y) && isFinite(v.Z)
}

// --- Mat3 methods ---

func identityMat3() Mat3 {
	return Mat3{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	}
}

func (m Mat3) add(other Mat3) Mat3 {
	var result Mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			result[i][j] = m[i][j] + other[i][j]
		}
	}
	return result
}

func (m Mat3) scale(s float64) Mat3 {
	var result Mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			result[i][j] = m[i][j] * s
		}
	}
	return result
}

func (m Mat3) multiply(other Mat3) Mat3 {
	var result Mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			sum := 0.0
			for k := 0; k < 3; k++ {
				sum += m[i][k] * other[k][j]
			}
			result[i][j] = sum
		}
	}
	return result
}

func (m Mat3) multiplyVec(v Vec3) Vec3 {
	return Vec3{
		m[0][0]*v.X + m[0][1]*v.Y + m[0][2]*v.Z,
		m[1][0]*v.X + m[1][1]*v.Y + m[1][2]*v.Z,
		m[2][0]*v.X + m[2][1]*v.Y + m[2][2]*v.Z,
	}
}

func (m Mat3) transpose() Mat3 {
	return Mat3{
		{m[0][0], m[1][0], m[2][0]},
		{m[0][1], m[1][1], m[2][1]},
		{m[0][2], m[1][2], m[2][2]},
	}
}

func (m Mat3) col(idx int) Vec3 {
	return Vec3{m[0][idx], m[1][idx], m[2][idx]}
}

func (m *Mat3) setCol(idx int, v Vec3) {
	m[0][idx] = v.X
	m[1][idx] = v.Y
	m[2][idx] = v.Z
}

func skewSymmetric(v Vec3) Mat3 {
	return Mat3{
		{0, -v.Z, v.Y},
		{v.Z, 0, -v.X},
		{-v.Y, v.X, 0},
	}
}

func isFinite(x float64) bool {
	return !math.IsNaN(x) && !math.IsInf(x, 0)
}

func clamp(x, min, max float64) float64 {
	if x < min {
		return min
	}
	if x > max {
		return max
	}
	return x
}

// ========== AHRS DATA PUBLISHER ==========

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

// NewAHRSPublisher creates a new AHRS publisher
func NewAHRSPublisher(socketPath string) (*AHRSPublisher, error) {
	os.Remove(socketPath)
	
	listener, err := net.Listen("unix", socketPath)
	if err != nil {
		return nil, fmt.Errorf("failed to create socket: %w", err)
	}
	
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
	
	jsonData = append(jsonData, '\n')
	
	p.clientsMu.Lock()
	defer p.clientsMu.Unlock()
	
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

// ========== IMU HARDWARE FUNCTIONS ==========

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

	if err := dev.WriteReg(mmcControl0, []byte{0x08}); err != nil {
		return err
	}
	time.Sleep(10 * time.Millisecond)

	if err := dev.WriteReg(mmcControl0, []byte{0x10}); err != nil {
		return err
	}
	time.Sleep(10 * time.Millisecond)

	if err := dev.WriteReg(mmcControl2, []byte{0x80}); err != nil {
		return err
	}

	return nil
}

func readMMC(dev *i2c.Device) ([3]float64, error) {
	if err := dev.WriteReg(mmcControl0, []byte{0x01}); err != nil {
		return [3]float64{}, err
	}

	for i := 0; i < 10; i++ {
		buf := make([]byte, 1)
		if err := dev.ReadReg(mmcControl0, buf); err != nil {
			return [3]float64{}, err
		}
		if buf[0]&0x01 == 0 {
			break
		}
		time.Sleep(1 * time.Millisecond)
	}

	data := make([]byte, 9)
	if err := dev.ReadReg(mmcXOut0, data); err != nil {
		return [3]float64{}, err
	}

	rawX := (uint32(data[0]) << 12) | (uint32(data[1]) << 4) | (uint32(data[6]) >> 4)
	rawY := (uint32(data[2]) << 12) | (uint32(data[3]) << 4) | (uint32(data[7]) >> 4)
	rawZ := (uint32(data[4]) << 12) | (uint32(data[5]) << 4) | (uint32(data[8]) >> 4)

	const offset = 524288.0
	const scale = 16384.0

	x := (float64(rawX) - offset) / scale
	y := (float64(rawY) - offset) / scale
	z := (float64(rawZ) - offset) / scale

	return [3]float64{x, y, z}, nil
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
	time.Sleep(10 * time.Millisecond)

	if err := dev.WriteReg(ismCtrl2G, []byte{0x8C}); err != nil {
		return err
	}

	if err := dev.WriteReg(ismCtrl1XL, []byte{0x8A}); err != nil {
		return err
	}

	return nil
}

func readISM(dev *i2c.Device) ([3]float64, [3]float64, error) {
	for i := 0; i < 10; i++ {
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
	const degToRad = math.Pi / 180.0

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

// ========== MAIN PROGRAM ==========

func main() {
	fmt.Println("=== AHRS GDL90 Sender (Mahony DCM) ===\n")

	const (
		sampleRate = 100.0  // Hz
		ahrsRate   = 10.0   // Hz (ForeFlight recommends 5Hz for AHRS)
		i2cBus     = "/dev/i2c-1"
		useMag     = true   // Set to false for IMU-only (6DOF) mode
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

	// Create Mahony DCM filter
	filter := NewMahonyFilter()
	fmt.Printf("✓ Mahony DCM Filter initialized\n")
	fmt.Printf("  Gains: KpAcc=%.2f, KpMag=%.2f, Ki=%.3f\n\n", filter.KpAcc, filter.KpMag, filter.Ki)

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

	fmt.Println("\nStarting AHRS data collection...")
	fmt.Printf("Sample rate: %.0f Hz, AHRS send rate: %.0f Hz\n", sampleRate, ahrsRate)
	if useMag {
		fmt.Println("Mode: 9DOF (gyro + accel + mag)")
	} else {
		fmt.Println("Mode: 6DOF (gyro + accel only)")
	}
	fmt.Println("Press Ctrl+C to stop\n")

	fmt.Println("Time        Roll      Pitch     Yaw       Heading")
	fmt.Println("========    =======   =======   =======   =======")

	dt := 1.0 / sampleRate
	//lastGyro := [3]float64{0, 0, 0}

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

			// Create Vec3 structures
			gyroVec := Vec3{X: gyro[0], Y: gyro[1], Z: gyro[2]}
			accelVec := Vec3{X: accel[0], Y: accel[1], Z: accel[2]}

			if useMag {
				// Read magnetometer data
				mag, err := readMMC(mmc)
				if err != nil {
					// Fall back to IMU-only mode for this sample
					filter.Update(gyroVec, accelVec, nil, dt)
				} else {
					// Update with all 9 DOF
					magVec := Vec3{X: mag[0], Y: mag[1], Z: mag[2]}
					filter.Update(gyroVec, accelVec, &magVec, dt)
				}
			} else {
				// IMU-only mode (6 DOF)
				filter.Update(gyroVec, accelVec, nil, dt)
			}

			// Get latest orientation
			euler := filter.GetEuler()
			rollDeg := euler.Roll * 180.0 / math.Pi
			pitchDeg := euler.Pitch * 180.0 / math.Pi
			yawDeg := euler.Yaw * 180.0 / math.Pi

			heading := math.Mod(yawDeg, 360.0)
			if heading < 0 {
				heading += 360.0
			}

			// Print status
			timestamp := time.Now().Format("15:04:05.00")
			fmt.Printf("%s  %7.2f°  %7.2f°  %7.2f°  %7.2f°\r",
				timestamp, rollDeg, pitchDeg, yawDeg, heading)

			// Get gyro rates (convert to deg/s)
			rollRate := gyro[0] * 180.0 / math.Pi
			pitchRate := gyro[1] * 180.0 / math.Pi
			yawRate := gyro[2] * 180.0 / math.Pi

			publisher.UpdateData(rollDeg, pitchDeg, heading, rollRate, pitchRate, yawRate)
			
			//lastGyro = gyro

		case <-broadcastTicker.C:
			// Broadcast to all connected clients
			publisher.Broadcast()
		}
	}
}