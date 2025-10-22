package ism330dhcx

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"time"

	"golang.org/x/exp/io/i2c"
)

const (
	// I2C Addresses
	DefaultAddress    = 0x6A // SDO/SA0 pulled high
	AlternateAddress  = 0x6B // SDO/SA0 pulled low

	// Register Map
	regFuncCfgAccess   = 0x01
	regPinCtrl         = 0x02
	regFIFOCtrl1       = 0x07
	regFIFOCtrl2       = 0x08
	regFIFOCtrl3       = 0x09
	regFIFOCtrl4       = 0x0A
	regCounterBDRReg1  = 0x0B
	regCounterBDRReg2  = 0x0C
	regInt1Ctrl        = 0x0D
	regInt2Ctrl        = 0x0E
	regWhoAmI          = 0x0F
	regCtrl1XL         = 0x10
	regCtrl2G          = 0x11
	regCtrl3C          = 0x12
	regCtrl4C          = 0x13
	regCtrl5C          = 0x14
	regCtrl6C          = 0x15
	regCtrl7G          = 0x16
	regCtrl8XL         = 0x17
	regCtrl9XL         = 0x18
	regCtrl10C         = 0x19
	regAllIntSrc       = 0x1A
	regWakeUpSrc       = 0x1B
	regStatusReg       = 0x1E
	regOutTempL        = 0x20
	regOutTempH        = 0x21
	regOutXLG          = 0x22
	regOutXHG          = 0x23
	regOutYLG          = 0x24
	regOutYHG          = 0x25
	regOutZLG          = 0x26
	regOutZHG          = 0x27
	regOutXLA          = 0x28
	regOutXHA          = 0x29
	regOutYLA          = 0x2A
	regOutYHA          = 0x2B
	regOutZLA          = 0x2C
	regOutZHA          = 0x2D
	regFIFOStatus1     = 0x3A
	regFIFOStatus2     = 0x3B
	regTimestamp0      = 0x40
	regTimestamp1      = 0x41
	regTimestamp2      = 0x42
	regTimestamp3      = 0x43

	// Expected WHO_AM_I value
	expectedWhoAmI = 0x6B

	// Timing constants
	powerUpTime       = 100 * time.Millisecond
	resetTime         = 50 * time.Millisecond
	measurementTime   = 10 * time.Millisecond
	maxRetries        = 3
	healthCheckInterval = 5 * time.Second

	// Physical limits for validation
	maxAccelG         = 20.0  // Maximum reasonable acceleration in G
	maxGyroRPS        = 40.0  // Maximum reasonable rotation in rad/s (roughly 2300 deg/s)
	minDataVariation  = 0.0001 // Minimum expected variation
	
	// Temperature limits
	minTemp = -40.0
	maxTemp = 85.0
)

// Accelerometer full scale ranges
type AccelScale byte

const (
	AccelScale2G  AccelScale = 0x00 // ±2g
	AccelScale4G  AccelScale = 0x02 // ±4g
	AccelScale8G  AccelScale = 0x03 // ±8g
	AccelScale16G AccelScale = 0x01 // ±16g
)

// Gyroscope full scale ranges
type GyroScale byte

const (
	GyroScale125DPS  GyroScale = 0x02 // ±125 dps
	GyroScale250DPS  GyroScale = 0x00 // ±250 dps
	GyroScale500DPS  GyroScale = 0x04 // ±500 dps
	GyroScale1000DPS GyroScale = 0x08 // ±1000 dps
	GyroScale2000DPS GyroScale = 0x0C // ±2000 dps
)

// Output Data Rate
type ODR byte

const (
	ODR_PowerDown ODR = 0x00
	ODR_12_5Hz    ODR = 0x01
	ODR_26Hz      ODR = 0x02
	ODR_52Hz      ODR = 0x03
	ODR_104Hz     ODR = 0x04
	ODR_208Hz     ODR = 0x05
	ODR_416Hz     ODR = 0x06
	ODR_833Hz     ODR = 0x07
	ODR_1666Hz    ODR = 0x08
	ODR_3333Hz    ODR = 0x09
	ODR_6666Hz    ODR = 0x0A
)

var (
	ErrInvalidWhoAmI      = errors.New("invalid WHO_AM_I value")
	ErrCommunicationFailed = errors.New("I2C communication failed")
	ErrTimeout            = errors.New("operation timeout")
	ErrInvalidData        = errors.New("invalid sensor data")
	ErrSensorNotReady     = errors.New("sensor not ready")
	ErrDeviceClosed       = errors.New("device is closed")
	ErrHealthCheckFailed  = errors.New("health check failed")
	ErrSelfTestFailed     = errors.New("self-test failed")
)

// IMUData represents a complete IMU reading
type IMUData struct {
	AccelX      float64   // m/s²
	AccelY      float64   // m/s²
	AccelZ      float64   // m/s²
	AccelMag    float64   // m/s²
	GyroX       float64   // rad/s
	GyroY       float64   // rad/s
	GyroZ       float64   // rad/s
	GyroMag     float64   // rad/s
	Temperature float64   // °C
	Timestamp   time.Time
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
	AccelSaturation   uint64
	GyroSaturation    uint64
}

// Config holds driver configuration
type Config struct {
	Address            byte
	Bus                string
	AccelScale         AccelScale
	GyroScale          GyroScale
	AccelODR           ODR
	GyroODR            ODR
	EnableAccel        bool
	EnableGyro         bool
	EnableBlockDataUpdate bool
	HealthCheckEnabled bool
}

// DefaultConfig returns aviation-grade default configuration
func DefaultConfig() *Config {
	return &Config{
		Address:            DefaultAddress,
		Bus:                "/dev/i2c-1",
		AccelScale:         AccelScale16G,
		GyroScale:          GyroScale2000DPS,
		AccelODR:           ODR_416Hz,
		GyroODR:            ODR_416Hz,
		EnableAccel:        true,
		EnableGyro:         true,
		EnableBlockDataUpdate: true,
		HealthCheckEnabled: true,
	}
}

// ISM330DHCX represents the IMU device
type ISM330DHCX struct {
	mu                sync.RWMutex
	device            *i2c.Device
	config            *Config
	closed            bool
	accelSensitivity  float64 // LSB per g
	gyroSensitivity   float64 // LSB per dps
	lastReading       IMUData
	lastValidReading  IMUData
	status            DeviceStatus
	startTime         time.Time
	ctx               context.Context
	cancel            context.CancelFunc
	healthCheckTicker *time.Ticker
}

// New creates and initializes a new ISM330DHCX driver instance
func New(config *Config) (*ISM330DHCX, error) {
	if config == nil {
		config = DefaultConfig()
	}

	device, err := i2c.Open(&i2c.Devfs{Dev: config.Bus}, int(config.Address))
	if err != nil {
		return nil, fmt.Errorf("failed to open I2C device: %w", err)
	}

	ctx, cancel := context.WithCancel(context.Background())

	driver := &ISM330DHCX{
		device:    device,
		config:    config,
		startTime: time.Now(),
		ctx:       ctx,
		cancel:    cancel,
		status: DeviceStatus{
			Healthy: false,
		},
	}

	// Calculate sensitivities based on configuration
	driver.accelSensitivity = driver.getAccelSensitivity()
	driver.gyroSensitivity = driver.getGyroSensitivity()

	// Initialize the sensor with retries
	var initErr error
	for attempt := 0; attempt < maxRetries; attempt++ {
		if initErr = driver.initialize(); initErr == nil {
			break
		}
		time.Sleep(time.Duration(attempt+1) * 20 * time.Millisecond)
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
func (imu *ISM330DHCX) initialize() error {
	// Power-up delay
	time.Sleep(powerUpTime)

	// Verify WHO_AM_I
	whoAmI, err := imu.readRegister(regWhoAmI)
	if err != nil {
		return fmt.Errorf("failed to read WHO_AM_I: %w", err)
	}

	if whoAmI != expectedWhoAmI {
		return fmt.Errorf("%w: expected 0x%02X, got 0x%02X",
			ErrInvalidWhoAmI, expectedWhoAmI, whoAmI)
	}

	// Software reset
	if err := imu.softwareReset(); err != nil {
		return fmt.Errorf("software reset failed: %w", err)
	}

	// Configure CTRL3_C: Enable BDU, auto-increment
	ctrl3Val := byte(0x44) // BDU=1, IF_INC=1
	if err := imu.writeRegister(regCtrl3C, ctrl3Val); err != nil {
		return fmt.Errorf("CTRL3_C configuration failed: %w", err)
	}

	// Configure accelerometer
	if imu.config.EnableAccel {
		accelCtrl := (byte(imu.config.AccelODR) << 4) | (byte(imu.config.AccelScale) << 2)
		if err := imu.writeRegister(regCtrl1XL, accelCtrl); err != nil {
			return fmt.Errorf("accelerometer configuration failed: %w", err)
		}
	}

	// Configure gyroscope
	if imu.config.EnableGyro {
		gyroCtrl := (byte(imu.config.GyroODR) << 4) | byte(imu.config.GyroScale)
		if err := imu.writeRegister(regCtrl2G, gyroCtrl); err != nil {
			return fmt.Errorf("gyroscope configuration failed: %w", err)
		}
	}

	// Additional gyro configuration for high performance
	if err := imu.writeRegister(regCtrl7G, 0x00); err != nil {
		return fmt.Errorf("CTRL7_G configuration failed: %w", err)
	}

	// Configure CTRL6_C for high performance mode
	if err := imu.writeRegister(regCtrl6C, 0x00); err != nil {
		return fmt.Errorf("CTRL6_C configuration failed: %w", err)
	}

	// Wait for sensor to stabilize
	time.Sleep(measurementTime)

	return nil
}

// softwareReset performs a software reset
func (imu *ISM330DHCX) softwareReset() error {
	// Set SW_RESET bit in CTRL3_C
	if err := imu.writeRegister(regCtrl3C, 0x01); err != nil {
		return err
	}

	time.Sleep(resetTime)

	// Wait for reset to complete
	deadline := time.Now().Add(200 * time.Millisecond)
	for time.Now().Before(deadline) {
		ctrl3, err := imu.readRegister(regCtrl3C)
		if err != nil {
			return err
		}
		if ctrl3&0x01 == 0 {
			return nil
		}
		time.Sleep(5 * time.Millisecond)
	}

	return ErrTimeout
}

// ReadIMU reads both accelerometer and gyroscope data
func (imu *ISM330DHCX) ReadIMU() (*IMUData, error) {
	imu.mu.Lock()
	defer imu.mu.Unlock()

	if imu.closed {
		return nil, ErrDeviceClosed
	}

	// Wait for data ready
	if err := imu.waitForDataReady(); err != nil {
		imu.recordError(err)
		if !imu.lastValidReading.Timestamp.IsZero() {
			return &imu.lastValidReading, fmt.Errorf("using cached data: %w", err)
		}
		return nil, err
	}

	// Read all sensor data in burst mode
	data, err := imu.readSensorData()
	if err != nil {
		imu.recordError(err)
		if !imu.lastValidReading.Timestamp.IsZero() {
			return &imu.lastValidReading, fmt.Errorf("using cached data: %w", err)
		}
		return nil, err
	}

	// Validate data
	if err := imu.validateData(data); err != nil {
		imu.recordError(err)
		if !imu.lastValidReading.Timestamp.IsZero() {
			return &imu.lastValidReading, fmt.Errorf("using cached data: %w", err)
		}
		return nil, err
	}

	imu.lastReading = *data
	imu.lastValidReading = *data
	imu.status.TotalReadings++
	imu.status.LastReading = time.Now()
	imu.status.ConsecutiveErrors = 0

	return data, nil
}

// waitForDataReady waits for new data to be available
func (imu *ISM330DHCX) waitForDataReady() error {
	deadline := time.Now().Add(measurementTime * 5)

	for time.Now().Before(deadline) {
		status, err := imu.readRegister(regStatusReg)
		if err != nil {
			return fmt.Errorf("failed to read status: %w", err)
		}

		// Check for accelerometer and gyroscope data ready
		accelReady := (status & 0x01) != 0
		gyroReady := (status & 0x02) != 0

		if (!imu.config.EnableAccel || accelReady) && (!imu.config.EnableGyro || gyroReady) {
			return nil
		}

		time.Sleep(1 * time.Millisecond)
	}

	return ErrTimeout
}

// readSensorData reads raw sensor data and converts to physical units
func (imu *ISM330DHCX) readSensorData() (*IMUData, error) {
	data := &IMUData{
		Timestamp: time.Now(),
	}

	// Read temperature (2 bytes)
	tempBuf := make([]byte, 2)
	if err := imu.readRegisters(regOutTempL, tempBuf); err != nil {
		return nil, fmt.Errorf("failed to read temperature: %w", err)
	}
	rawTemp := int16(tempBuf[0]) | int16(tempBuf[1])<<8
	data.Temperature = 25.0 + float64(rawTemp)/256.0

	// Read gyroscope data (6 bytes)
	if imu.config.EnableGyro {
		gyroBuf := make([]byte, 6)
		if err := imu.readRegisters(regOutXLG, gyroBuf); err != nil {
			return nil, fmt.Errorf("failed to read gyroscope: %w", err)
		}

		rawGyroX := int16(gyroBuf[0]) | int16(gyroBuf[1])<<8
		rawGyroY := int16(gyroBuf[2]) | int16(gyroBuf[3])<<8
		rawGyroZ := int16(gyroBuf[4]) | int16(gyroBuf[5])<<8

		// Convert to rad/s (dps * π/180)
		data.GyroX = float64(rawGyroX) / imu.gyroSensitivity * math.Pi / 180.0
		data.GyroY = float64(rawGyroY) / imu.gyroSensitivity * math.Pi / 180.0
		data.GyroZ = float64(rawGyroZ) / imu.gyroSensitivity * math.Pi / 180.0
		data.GyroMag = math.Sqrt(data.GyroX*data.GyroX + data.GyroY*data.GyroY + data.GyroZ*data.GyroZ)

		// Check for saturation
		if math.Abs(float64(rawGyroX)) > 32000 || 
		   math.Abs(float64(rawGyroY)) > 32000 || 
		   math.Abs(float64(rawGyroZ)) > 32000 {
			imu.status.GyroSaturation++
		}
	}

	// Read accelerometer data (6 bytes)
	if imu.config.EnableAccel {
		accelBuf := make([]byte, 6)
		if err := imu.readRegisters(regOutXLA, accelBuf); err != nil {
			return nil, fmt.Errorf("failed to read accelerometer: %w", err)
		}

		rawAccelX := int16(accelBuf[0]) | int16(accelBuf[1])<<8
		rawAccelY := int16(accelBuf[2]) | int16(accelBuf[3])<<8
		rawAccelZ := int16(accelBuf[4]) | int16(accelBuf[5])<<8

		// Convert to m/s² (g * 9.80665)
		data.AccelX = float64(rawAccelX) / imu.accelSensitivity * 9.80665
		data.AccelY = float64(rawAccelY) / imu.accelSensitivity * 9.80665
		data.AccelZ = float64(rawAccelZ) / imu.accelSensitivity * 9.80665
		data.AccelMag = math.Sqrt(data.AccelX*data.AccelX + data.AccelY*data.AccelY + data.AccelZ*data.AccelZ)

		// Check for saturation
		if math.Abs(float64(rawAccelX)) > 32000 || 
		   math.Abs(float64(rawAccelY)) > 32000 || 
		   math.Abs(float64(rawAccelZ)) > 32000 {
			imu.status.AccelSaturation++
		}
	}

	return data, nil
}

// validateData performs aviation-grade data validation
func (imu *ISM330DHCX) validateData(data *IMUData) error {
	// Temperature validation
	if data.Temperature < minTemp || data.Temperature > maxTemp {
		return fmt.Errorf("%w: temperature %.1f°C out of range", ErrInvalidData, data.Temperature)
	}

	// Accelerometer validation
	if imu.config.EnableAccel {
		accelGs := data.AccelMag / 9.80665
		if accelGs > maxAccelG {
			return fmt.Errorf("%w: acceleration %.2fg exceeds maximum %.2fg",
				ErrInvalidData, accelGs, maxAccelG)
		}

		// Check for frozen sensor
		if !imu.lastReading.Timestamp.IsZero() {
			deltaAx := math.Abs(data.AccelX - imu.lastReading.AccelX)
			deltaAy := math.Abs(data.AccelY - imu.lastReading.AccelY)
			deltaAz := math.Abs(data.AccelZ - imu.lastReading.AccelZ)
			totalDelta := deltaAx + deltaAy + deltaAz

			if totalDelta < minDataVariation && time.Since(imu.lastReading.Timestamp) > 100*time.Millisecond {
				return fmt.Errorf("%w: accelerometer appears frozen", ErrInvalidData)
			}
		}
	}

	// Gyroscope validation
	if imu.config.EnableGyro {
		if data.GyroMag > maxGyroRPS {
			return fmt.Errorf("%w: gyro magnitude %.2f rad/s exceeds maximum %.2f rad/s",
				ErrInvalidData, data.GyroMag, maxGyroRPS)
		}

		// Check for frozen sensor
		if !imu.lastReading.Timestamp.IsZero() {
			deltaGx := math.Abs(data.GyroX - imu.lastReading.GyroX)
			deltaGy := math.Abs(data.GyroY - imu.lastReading.GyroY)
			deltaGz := math.Abs(data.GyroZ - imu.lastReading.GyroZ)
			totalDelta := deltaGx + deltaGy + deltaGz

			if totalDelta < minDataVariation && time.Since(imu.lastReading.Timestamp) > 100*time.Millisecond {
				return fmt.Errorf("%w: gyroscope appears frozen", ErrInvalidData)
			}
		}
	}

	return nil
}

// SelfTest performs comprehensive self-test
func (imu *ISM330DHCX) SelfTest() error {
	imu.mu.Lock()
	defer imu.mu.Unlock()

	if imu.closed {
		return ErrDeviceClosed
	}

	// Verify WHO_AM_I
	whoAmI, err := imu.readRegister(regWhoAmI)
	if err != nil {
		return fmt.Errorf("self-test failed: %w", err)
	}
	if whoAmI != expectedWhoAmI {
		return fmt.Errorf("self-test failed: %w", ErrInvalidWhoAmI)
	}

	// Verify data ready status works
	if err := imu.waitForDataReady(); err != nil {
		return fmt.Errorf("self-test failed: %w", err)
	}

	// Read and validate sensor data
	data, err := imu.readSensorData()
	if err != nil {
		return fmt.Errorf("self-test failed: %w", err)
	}

	if err := imu.validateData(data); err != nil {
		return fmt.Errorf("self-test failed: %w", err)
	}

	return nil
}

// GetStatus returns current device status
func (imu *ISM330DHCX) GetStatus() DeviceStatus {
	imu.mu.RLock()
	defer imu.mu.RUnlock()

	status := imu.status
	status.UptimeSeconds = time.Since(imu.startTime).Seconds()
	return status
}

// recordError updates error tracking
func (imu *ISM330DHCX) recordError(err error) {
	imu.status.LastError = err
	imu.status.ConsecutiveErrors++
	imu.status.FailedReadings++

	if imu.status.ConsecutiveErrors >= maxRetries {
		imu.status.Healthy = false
	}
}

// startHealthMonitoring begins periodic health checks
func (imu *ISM330DHCX) startHealthMonitoring() {
	imu.healthCheckTicker = time.NewTicker(healthCheckInterval)

	go func() {
		for {
			select {
			case <-imu.ctx.Done():
				return
			case <-imu.healthCheckTicker.C:
				imu.performHealthCheck()
			}
		}
	}()
}

// performHealthCheck executes periodic health validation
func (imu *ISM330DHCX) performHealthCheck() {
	if err := imu.SelfTest(); err != nil {
		imu.mu.Lock()
		imu.recordError(fmt.Errorf("%w: %v", ErrHealthCheckFailed, err))
		imu.mu.Unlock()
	} else {
		imu.mu.Lock()
		imu.status.Healthy = true
		imu.status.ConsecutiveErrors = 0
		imu.mu.Unlock()
	}
}

// Reset performs a full device reset and reinitialization
func (imu *ISM330DHCX) Reset() error {
	imu.mu.Lock()
	defer imu.mu.Unlock()

	if imu.closed {
		return ErrDeviceClosed
	}

	return imu.initialize()
}

// Close closes the device and releases resources
func (imu *ISM330DHCX) Close() error {
	imu.mu.Lock()
	defer imu.mu.Unlock()

	if imu.closed {
		return nil
	}

	imu.closed = true
	imu.cancel()

	if imu.healthCheckTicker != nil {
		imu.healthCheckTicker.Stop()
	}

	return imu.device.Close()
}

// getAccelSensitivity returns LSB/g based on configured scale
func (imu *ISM330DHCX) getAccelSensitivity() float64 {
	switch imu.config.AccelScale {
	case AccelScale2G:
		return 16384.0
	case AccelScale4G:
		return 8192.0
	case AccelScale8G:
		return 4096.0
	case AccelScale16G:
		return 2048.0
	default:
		return 2048.0
	}
}

// getGyroSensitivity returns LSB/dps based on configured scale
func (imu *ISM330DHCX) getGyroSensitivity() float64 {
	switch imu.config.GyroScale {
	case GyroScale125DPS:
		return 262.0
	case GyroScale250DPS:
		return 131.0
	case GyroScale500DPS:
		return 65.5
	case GyroScale1000DPS:
		return 32.8
	case GyroScale2000DPS:
		return 16.4
	default:
		return 16.4
	}
}

// Low-level I2C operations

func (imu *ISM330DHCX) readRegister(reg byte) (byte, error) {
	buf := []byte{0}
	if err := imu.device.ReadReg(reg, buf); err != nil {
		return 0, fmt.Errorf("%w: %v", ErrCommunicationFailed, err)
	}
	return buf[0], nil
}

func (imu *ISM330DHCX) readRegisters(reg byte, buf []byte) error {
	if err := imu.device.ReadReg(reg, buf); err != nil {
		return fmt.Errorf("%w: %v", ErrCommunicationFailed, err)
	}
	return nil
}

func (imu *ISM330DHCX) writeRegister(reg, value byte) error {
	if err := imu.device.WriteReg(reg, []byte{value}); err != nil {
		return fmt.Errorf("%w: %v", ErrCommunicationFailed, err)
	}
	return nil
}