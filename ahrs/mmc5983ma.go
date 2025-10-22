package mmc5983ma

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"sync"
	"time"

	"golang.org/x/exp/io/i2c"
)

const (
	// I2C Address
	DefaultAddress = 0x30

	// Register Map
	regXOut0       = 0x00
	regXOut1       = 0x01
	regYOut0       = 0x02
	regYOut1       = 0x03
	regZOut0       = 0x04
	regZOut1       = 0x05
	regXYZOut2     = 0x06
	regTOut        = 0x07
	regStatus      = 0x08
	regControl0    = 0x09
	regControl1    = 0x0A
	regControl2    = 0x0B
	regControl3    = 0x0C
	regProductID   = 0x2F

	// Control Register Bits
	ctrl0TakeMeasT   = 0x02
	ctrl0TakeMeasM   = 0x01
	ctrl0AutoSRen    = 0x20
	ctrl0SetOperation = 0x08
	ctrl0ResetOperation = 0x10

	ctrl1BWSelection = 0x00 // 100Hz bandwidth default
	ctrl1XInhibit    = 0x01
	ctrl1YZInhibit   = 0x02
	ctrl1SwReset     = 0x80

	ctrl2CmFreqEn    = 0x80
	ctrl2EnPrdSet    = 0x08
	ctrl2PeriodicSet = 0x04
	ctrl2Hpower      = 0x01

	// Expected Product ID
	expectedProductID = 0x30

	// Timing Constants (aviation-grade conservative values)
	measurementTimeout    = 50 * time.Millisecond
	setResetTime         = 1 * time.Millisecond
	powerOnDelay         = 10 * time.Millisecond
	maxRetries           = 3
	i2cTimeout           = 100 * time.Millisecond
	healthCheckInterval  = 5 * time.Second
	
	// Data validation constants
	maxReasonableField   = 800.0  // μT, Earth's field typically 25-65 μT
	minDataVariation     = 0.001  // Minimum expected variation between readings
	
	// Resolution
	resolution18Bit = 262144.0 // 2^18
	fullScaleRange  = 8.0      // ±8 Gauss = ±800 μT
)

var (
	ErrInvalidProductID    = errors.New("invalid product ID")
	ErrCommunicationFailed = errors.New("I2C communication failed")
	ErrTimeout            = errors.New("operation timeout")
	ErrInvalidData        = errors.New("invalid sensor data")
	ErrSensorNotReady     = errors.New("sensor not ready")
	ErrDeviceClosed       = errors.New("device is closed")
	ErrHealthCheckFailed  = errors.New("health check failed")
)

// MagneticData represents a calibrated magnetometer reading
type MagneticData struct {
	X         float64   // μT
	Y         float64   // μT
	Z         float64   // μT
	Magnitude float64   // μT
	Timestamp time.Time
	Temperature float64 // °C
}

// DeviceStatus contains health and diagnostic information
type DeviceStatus struct {
	Healthy           bool
	LastError         error
	ConsecutiveErrors int
	TotalReadings     uint64
	FailedReadings    uint64
	LastReading       time.Time
	UptimeSeconds     float64
}

// Config holds driver configuration
type Config struct {
	Address          byte
	Bus              string
	Bandwidth        byte
	ContinuousMode   bool
	AutoSetReset     bool
	PeriodicSet      int // 0 = off, values 1-2000 for periodic set operations
	HealthCheckEnabled bool
}

// DefaultConfig returns aviation-grade default configuration
func DefaultConfig() *Config {
	return &Config{
		Address:          DefaultAddress,
		Bus:              "/dev/i2c-1",
		Bandwidth:        0, // 100Hz
		ContinuousMode:   true,
		AutoSetReset:     true,
		PeriodicSet:      25, // Periodic set every 25 measurements
		HealthCheckEnabled: true,
	}
}

// MMC5983MA represents the magnetometer device
type MMC5983MA struct {
	mu                sync.RWMutex
	device            *i2c.Device
	config            *Config
	closed            bool
	lastReading       MagneticData
	lastValidReading  MagneticData
	status            DeviceStatus
	startTime         time.Time
	measurementCount  uint32
	ctx               context.Context
	cancel            context.CancelFunc
	healthCheckTicker *time.Ticker
}

// New creates and initializes a new MMC5983MA driver instance
func New(config *Config) (*MMC5983MA, error) {
	if config == nil {
		config = DefaultConfig()
	}

	device, err := i2c.Open(&i2c.Devfs{Dev: config.Bus}, int(config.Address))
	if err != nil {
		return nil, fmt.Errorf("failed to open I2C device: %w", err)
	}

	ctx, cancel := context.WithCancel(context.Background())
	
	driver := &MMC5983MA{
		device:    device,
		config:    config,
		startTime: time.Now(),
		ctx:       ctx,
		cancel:    cancel,
		status: DeviceStatus{
			Healthy: false,
		},
	}

	// Initialize the sensor with retries
	var initErr error
	for attempt := 0; attempt < maxRetries; attempt++ {
		if initErr = driver.initialize(); initErr == nil {
			break
		}
		time.Sleep(time.Duration(attempt+1) * 10 * time.Millisecond)
	}

	if initErr != nil {
		device.Close()
		cancel()
		return nil, fmt.Errorf("initialization failed after %d attempts: %w", maxRetries, initErr)
	}

	driver.status.Healthy = true

	// Start health monitoring if enabled
	if config.HealthCheckEnabled {
		driver.startHealthMonitoring()
	}

	return driver, nil
}

// initialize performs sensor initialization and validation
func (m *MMC5983MA) initialize() error {
	// Power-on delay
	time.Sleep(powerOnDelay)

	// Verify product ID
	productID, err := m.readRegister(regProductID)
	if err != nil {
		return fmt.Errorf("failed to read product ID: %w", err)
	}

	if productID != expectedProductID {
		return fmt.Errorf("%w: expected 0x%02X, got 0x%02X", 
			ErrInvalidProductID, expectedProductID, productID)
	}

	// Software reset
	if err := m.writeRegister(regControl1, ctrl1SwReset); err != nil {
		return fmt.Errorf("software reset failed: %w", err)
	}
	time.Sleep(10 * time.Millisecond)

	// Configure bandwidth
	if err := m.writeRegister(regControl1, m.config.Bandwidth); err != nil {
		return fmt.Errorf("bandwidth configuration failed: %w", err)
	}

	// Configure Control0 - enable auto set/reset if configured
	ctrl0Val := byte(0)
	if m.config.AutoSetReset {
		ctrl0Val |= ctrl0AutoSRen
	}
	if err := m.writeRegister(regControl0, ctrl0Val); err != nil {
		return fmt.Errorf("control0 configuration failed: %w", err)
	}

	// Configure Control2 for continuous mode and high power
	ctrl2Val := ctrl2Hpower
	if m.config.ContinuousMode {
		ctrl2Val |= ctrl2CmFreqEn
	}
	if m.config.PeriodicSet > 0 {
		ctrl2Val |= ctrl2EnPrdSet
	}
	if err := m.writeRegister(regControl2, ctrl2Val); err != nil {
		return fmt.Errorf("control2 configuration failed: %w", err)
	}

	// Perform initial SET operation to initialize sensor
	if err := m.performSet(); err != nil {
		return fmt.Errorf("initial SET operation failed: %w", err)
	}

	// Perform initial RESET operation
	if err := m.performReset(); err != nil {
		return fmt.Errorf("initial RESET operation failed: %w", err)
	}

	return nil
}

// performSet executes a SET operation to initialize sensor offset
func (m *MMC5983MA) performSet() error {
	if err := m.writeRegister(regControl0, ctrl0SetOperation); err != nil {
		return err
	}
	time.Sleep(setResetTime)
	return nil
}

// performReset executes a RESET operation
func (m *MMC5983MA) performReset() error {
	if err := m.writeRegister(regControl0, ctrl0ResetOperation); err != nil {
		return err
	}
	time.Sleep(setResetTime)
	return nil
}

// ReadMagneticField reads and returns calibrated magnetic field data
func (m *MMC5983MA) ReadMagneticField() (*MagneticData, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.closed {
		return nil, ErrDeviceClosed
	}

	// Perform periodic SET operation for offset calibration
	m.measurementCount++
	if m.config.PeriodicSet > 0 && m.measurementCount%uint32(m.config.PeriodicSet) == 0 {
		if err := m.performSet(); err != nil {
			m.recordError(err)
			// Non-fatal, continue with measurement
		}
		time.Sleep(setResetTime)
	}

	// Trigger measurement
	if err := m.writeRegister(regControl0, ctrl0TakeMeasM); err != nil {
		m.recordError(err)
		return nil, fmt.Errorf("failed to trigger measurement: %w", err)
	}

	// Wait for measurement to complete with timeout
	data, err := m.waitAndReadData()
	if err != nil {
		m.recordError(err)
		// Return last valid reading as fallback
		if m.lastValidReading.Timestamp.IsZero() {
			return nil, err
		}
		return &m.lastValidReading, fmt.Errorf("using cached data: %w", err)
	}

	// Validate data
	if err := m.validateData(data); err != nil {
		m.recordError(err)
		if !m.lastValidReading.Timestamp.IsZero() {
			return &m.lastValidReading, fmt.Errorf("using cached data: %w", err)
		}
		return nil, err
	}

	m.lastReading = *data
	m.lastValidReading = *data
	m.status.TotalReadings++
	m.status.LastReading = time.Now()
	m.status.ConsecutiveErrors = 0

	return data, nil
}

// waitAndReadData waits for measurement completion and reads the data
func (m *MMC5983MA) waitAndReadData() (*MagneticData, error) {
	deadline := time.Now().Add(measurementTimeout)
	
	for time.Now().Before(deadline) {
		status, err := m.readRegister(regStatus)
		if err != nil {
			return nil, fmt.Errorf("failed to read status: %w", err)
		}

		// Check if measurement is done (bit 0)
		if status&0x01 != 0 {
			return m.readRawData()
		}

		time.Sleep(1 * time.Millisecond)
	}

	return nil, ErrTimeout
}

// readRawData reads raw magnetic field data from sensor
func (m *MMC5983MA) readRawData() (*MagneticData, error) {
	// Read all data registers in one burst (7 bytes)
	buf := make([]byte, 7)
	if err := m.device.Read(buf); err != nil {
		return nil, fmt.Errorf("%w: %v", ErrCommunicationFailed, err)
	}

	// Parse 18-bit values
	rawX := (uint32(buf[0]) << 10) | (uint32(buf[1]) << 2) | (uint32(buf[6]) >> 6)
	rawY := (uint32(buf[2]) << 10) | (uint32(buf[3]) << 2) | ((uint32(buf[6]) >> 4) & 0x03)
	rawZ := (uint32(buf[4]) << 10) | (uint32(buf[5]) << 2) | ((uint32(buf[6]) >> 2) & 0x03)
	rawT := uint32(buf[7])

	// Convert to Gauss then to μT
	// Formula: Field = (Raw - 2^17) / 2^17 * 8 Gauss
	x := (float64(rawX) - resolution18Bit/2) / (resolution18Bit / 2) * fullScaleRange * 100 // to μT
	y := (float64(rawY) - resolution18Bit/2) / (resolution18Bit / 2) * fullScaleRange * 100
	z := (float64(rawZ) - resolution18Bit/2) / (resolution18Bit / 2) * fullScaleRange * 100

	// Calculate magnitude
	magnitude := m.calculateMagnitude(x, y, z)

	// Convert temperature (0.8°C per LSB, -75°C offset)
	temperature := float64(rawT)*0.8 - 75.0

	return &MagneticData{
		X:           x,
		Y:           y,
		Z:           z,
		Magnitude:   magnitude,
		Timestamp:   time.Now(),
		Temperature: temperature,
	}, nil
}

// calculateMagnitude computes vector magnitude
func (m *MMC5983MA) calculateMagnitude(x, y, z float64) float64 {
	return (x*x + y*y + z*z)
}

// validateData performs aviation-grade data validation
func (m *MMC5983MA) validateData(data *MagneticData) error {
	// Check for reasonable field strength
	if data.Magnitude > maxReasonableField {
		return fmt.Errorf("%w: magnitude %.2f μT exceeds maximum %.2f μT", 
			ErrInvalidData, data.Magnitude, maxReasonableField)
	}

	// Check for stuck/frozen sensor (only if we have previous data)
	if !m.lastReading.Timestamp.IsZero() {
		deltaX := data.X - m.lastReading.X
		deltaY := data.Y - m.lastReading.Y
		deltaZ := data.Z - m.lastReading.Z
		totalDelta := deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ

		// If data hasn't changed at all in multiple readings, sensor may be stuck
		if totalDelta < minDataVariation && time.Since(m.lastReading.Timestamp) > 100*time.Millisecond {
			return fmt.Errorf("%w: sensor appears frozen (delta: %.6f)", ErrInvalidData, totalDelta)
		}
	}

	// Temperature sanity check (-40°C to +85°C is typical operating range)
	if data.Temperature < -50 || data.Temperature > 100 {
		return fmt.Errorf("%w: temperature %.1f°C out of range", ErrInvalidData, data.Temperature)
	}

	return nil
}

// ReadTemperature reads the die temperature
func (m *MMC5983MA) ReadTemperature() (float64, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.closed {
		return 0, ErrDeviceClosed
	}

	// Trigger temperature measurement
	if err := m.writeRegister(regControl0, ctrl0TakeMeasT); err != nil {
		return 0, fmt.Errorf("failed to trigger temperature measurement: %w", err)
	}

	// Wait for completion
	time.Sleep(5 * time.Millisecond)

	temp, err := m.readRegister(regTOut)
	if err != nil {
		return 0, err
	}

	// Convert: 0.8°C per LSB, -75°C offset
	temperature := float64(temp)*0.8 - 75.0

	return temperature, nil
}

// SelfTest performs comprehensive self-test
func (m *MMC5983MA) SelfTest() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.closed {
		return ErrDeviceClosed
	}

	// Verify product ID
	productID, err := m.readRegister(regProductID)
	if err != nil {
		return fmt.Errorf("self-test failed: %w", err)
	}
	if productID != expectedProductID {
		return fmt.Errorf("self-test failed: %w", ErrInvalidProductID)
	}

	// Perform SET operation and measure
	if err := m.performSet(); err != nil {
		return fmt.Errorf("self-test SET failed: %w", err)
	}
	time.Sleep(setResetTime)

	// Take a measurement to verify sensor response
	if err := m.writeRegister(regControl0, ctrl0TakeMeasM); err != nil {
		return fmt.Errorf("self-test measurement trigger failed: %w", err)
	}

	data, err := m.waitAndReadData()
	if err != nil {
		return fmt.Errorf("self-test measurement failed: %w", err)
	}

	if err := m.validateData(data); err != nil {
		return fmt.Errorf("self-test validation failed: %w", err)
	}

	return nil
}

// GetStatus returns current device status
func (m *MMC5983MA) GetStatus() DeviceStatus {
	m.mu.RLock()
	defer m.mu.RUnlock()

	status := m.status
	status.UptimeSeconds = time.Since(m.startTime).Seconds()
	return status
}

// recordError updates error tracking
func (m *MMC5983MA) recordError(err error) {
	m.status.LastError = err
	m.status.ConsecutiveErrors++
	m.status.FailedReadings++

	// Mark as unhealthy if too many consecutive errors
	if m.status.ConsecutiveErrors >= maxRetries {
		m.status.Healthy = false
	}
}

// startHealthMonitoring begins periodic health checks
func (m *MMC5983MA) startHealthMonitoring() {
	m.healthCheckTicker = time.NewTicker(healthCheckInterval)
	
	go func() {
		for {
			select {
			case <-m.ctx.Done():
				return
			case <-m.healthCheckTicker.C:
				m.performHealthCheck()
			}
		}
	}()
}

// performHealthCheck executes periodic health validation
func (m *MMC5983MA) performHealthCheck() {
	if err := m.SelfTest(); err != nil {
		m.mu.Lock()
		m.recordError(fmt.Errorf("%w: %v", ErrHealthCheckFailed, err))
		m.mu.Unlock()
	} else {
		m.mu.Lock()
		m.status.Healthy = true
		m.status.ConsecutiveErrors = 0
		m.mu.Unlock()
	}
}

// Reset performs a full device reset and reinitialization
func (m *MMC5983MA) Reset() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.closed {
		return ErrDeviceClosed
	}

	return m.initialize()
}

// Close closes the device and releases resources
func (m *MMC5983MA) Close() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.closed {
		return nil
	}

	m.closed = true
	m.cancel()

	if m.healthCheckTicker != nil {
		m.healthCheckTicker.Stop()
	}

	return m.device.Close()
}

// Low-level I2C operations with error handling

func (m *MMC5983MA) readRegister(reg byte) (byte, error) {
	buf := []byte{0}
	if err := m.device.ReadReg(reg, buf); err != nil {
		return 0, fmt.Errorf("%w: %v", ErrCommunicationFailed, err)
	}
	return buf[0], nil
}

func (m *MMC5983MA) writeRegister(reg, value byte) error {
	if err := m.device.WriteReg(reg, []byte{value}); err != nil {
		return fmt.Errorf("%w: %v", ErrCommunicationFailed, err)
	}
	return nil
}