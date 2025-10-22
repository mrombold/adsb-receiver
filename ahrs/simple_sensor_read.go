package main

import (
	"fmt"
	"log"
	"time"

	"golang.org/x/exp/io/i2c"
)

const (
	// MMC5983MA registers and constants
	mmcAddress     = 0x30
	mmcWhoAmI      = 0x2F
	mmcControl0    = 0x09
	mmcControl1    = 0x0A
	mmcControl2    = 0x0B
	mmcXOut0       = 0x00
	mmcStatus      = 0x08
	mmcExpectedID  = 0x30

	// ISM330DHCX registers and constants
	ismAddress     = 0x6B
	ismWhoAmI      = 0x0F
	ismCtrl1XL     = 0x10
	ismCtrl2G      = 0x11
	ismCtrl3C      = 0x12
	ismStatusReg   = 0x1E
	ismOutXLG      = 0x22
	ismOutXLA      = 0x28
	ismExpectedID  = 0x6B
)

func main() {
	fmt.Println("Initializing sensors...\n")

	// Open I2C bus
	bus := "/dev/i2c-1"

	// Initialize MMC5983MA magnetometer
	fmt.Println("=== MMC5983MA Magnetometer ===")
	mmc, err := i2c.Open(&i2c.Devfs{Dev: bus}, mmcAddress)
	if err != nil {
		log.Fatalf("Failed to open MMC5983MA: %v", err)
	}
	defer mmc.Close()

	if err := initMMC(mmc); err != nil {
		log.Fatalf("Failed to initialize MMC5983MA: %v", err)
	}
	fmt.Println("✓ MMC5983MA initialized\n")

	// Initialize ISM330DHCX IMU
	fmt.Println("=== ISM330DHCX IMU ===")
	ism, err := i2c.Open(&i2c.Devfs{Dev: bus}, ismAddress)
	if err != nil {
		log.Fatalf("Failed to open ISM330DHCX: %v", err)
	}
	defer ism.Close()

	if err := initISM(ism); err != nil {
		log.Fatalf("Failed to initialize ISM330DHCX: %v", err)
	}
	fmt.Println("✓ ISM330DHCX initialized\n")

	// Read sensors continuously
	fmt.Println("=== Reading Sensors ===")
	fmt.Println("Press Ctrl+C to stop\n")

	for {
		fmt.Printf("[%s]\n", time.Now().Format("15:04:05.000"))

		// Read magnetometer
		if magData, err := readMMC(mmc); err == nil {
			fmt.Printf("  Magnetometer: X=%.2f Y=%.2f Z=%.2f μT\n",
				magData[0], magData[1], magData[2])
		} else {
			fmt.Printf("  Magnetometer error: %v\n", err)
		}

		// Read IMU
		if accel, gyro, err := readISM(ism); err == nil {
			fmt.Printf("  Accelerometer: X=%.2f Y=%.2f Z=%.2f m/s²\n",
				accel[0], accel[1], accel[2])
			fmt.Printf("  Gyroscope:     X=%.3f Y=%.3f Z=%.3f rad/s\n",
				gyro[0], gyro[1], gyro[2])
		} else {
			fmt.Printf("  IMU error: %v\n", err)
		}

		fmt.Println()
		time.Sleep(500 * time.Millisecond)
	}
}

// initMMC initializes the MMC5983MA magnetometer
func initMMC(dev *i2c.Device) error {
	// Check WHO_AM_I
	buf := make([]byte, 1)
	if err := dev.ReadReg(mmcWhoAmI, buf); err != nil {
		return fmt.Errorf("read WHO_AM_I failed: %w", err)
	}
	if buf[0] != mmcExpectedID {
		return fmt.Errorf("invalid WHO_AM_I: expected 0x%02X, got 0x%02X", mmcExpectedID, buf[0])
	}

	// Software reset
	if err := dev.WriteReg(mmcControl1, []byte{0x80}); err != nil {
		return err
	}
	time.Sleep(10 * time.Millisecond)

	// Configure continuous mode with auto set/reset
	if err := dev.WriteReg(mmcControl0, []byte{0x20}); err != nil {
		return err
	}

	// Enable continuous mode, high power
	if err := dev.WriteReg(mmcControl2, []byte{0x81}); err != nil {
		return err
	}

	// Perform SET operation
	if err := dev.WriteReg(mmcControl0, []byte{0x08}); err != nil {
		return err
	}
	time.Sleep(1 * time.Millisecond)

	// Perform RESET operation
	if err := dev.WriteReg(mmcControl0, []byte{0x10}); err != nil {
		return err
	}
	time.Sleep(1 * time.Millisecond)

	return nil
}

// initISM initializes the ISM330DHCX IMU
func initISM(dev *i2c.Device) error {
	// Check WHO_AM_I
	buf := make([]byte, 1)
	if err := dev.ReadReg(ismWhoAmI, buf); err != nil {
		return fmt.Errorf("read WHO_AM_I failed: %w", err)
	}
	if buf[0] != ismExpectedID {
		return fmt.Errorf("invalid WHO_AM_I: expected 0x%02X, got 0x%02X", ismExpectedID, buf[0])
	}

	// Software reset
	if err := dev.WriteReg(ismCtrl3C, []byte{0x01}); err != nil {
		return err
	}
	time.Sleep(50 * time.Millisecond)

	// Enable block data update and auto-increment
	if err := dev.WriteReg(ismCtrl3C, []byte{0x44}); err != nil {
		return err
	}

	// Configure accelerometer: 416 Hz, ±16g
	if err := dev.WriteReg(ismCtrl1XL, []byte{0x64}); err != nil {
		return err
	}

	// Configure gyroscope: 416 Hz, ±2000 dps
	if err := dev.WriteReg(ismCtrl2G, []byte{0x6C}); err != nil {
		return err
	}

	time.Sleep(10 * time.Millisecond)
	return nil
}

// readMMC reads magnetic field data from MMC5983MA
func readMMC(dev *i2c.Device) ([3]float64, error) {
	// Trigger measurement
	if err := dev.WriteReg(mmcControl0, []byte{0x01}); err != nil {
		return [3]float64{}, err
	}

	// Wait for measurement ready
	time.Sleep(10 * time.Millisecond)

	// Read 7 bytes of data
	buf := make([]byte, 7)
	if err := dev.ReadReg(mmcXOut0, buf); err != nil {
		return [3]float64{}, err
	}

	// Parse 18-bit values
	rawX := (uint32(buf[0]) << 10) | (uint32(buf[1]) << 2) | (uint32(buf[6]) >> 6)
	rawY := (uint32(buf[2]) << 10) | (uint32(buf[3]) << 2) | ((uint32(buf[6]) >> 4) & 0x03)
	rawZ := (uint32(buf[4]) << 10) | (uint32(buf[5]) << 2) | ((uint32(buf[6]) >> 2) & 0x03)

	// Convert to μT (microtesla)
	const resolution = 262144.0 // 2^18
	const range_uT = 800.0      // ±8 Gauss = ±800 μT

	x := (float64(rawX) - resolution/2) / (resolution / 2) * range_uT
	y := (float64(rawY) - resolution/2) / (resolution / 2) * range_uT
	z := (float64(rawZ) - resolution/2) / (resolution / 2) * range_uT

	return [3]float64{x, y, z}, nil
}

// readISM reads accelerometer and gyroscope data from ISM330DHCX
func readISM(dev *i2c.Device) ([3]float64, [3]float64, error) {
	// Wait for data ready
	deadline := time.Now().Add(50 * time.Millisecond)
	for time.Now().Before(deadline) {
		buf := make([]byte, 1)
		if err := dev.ReadReg(ismStatusReg, buf); err != nil {
			return [3]float64{}, [3]float64{}, err
		}
		if buf[0]&0x03 == 0x03 { // Both accel and gyro ready
			break
		}
		time.Sleep(1 * time.Millisecond)
	}

	// Read gyroscope (6 bytes)
	gyroBuf := make([]byte, 6)
	if err := dev.ReadReg(ismOutXLG, gyroBuf); err != nil {
		return [3]float64{}, [3]float64{}, err
	}

	rawGyroX := int16(gyroBuf[0]) | int16(gyroBuf[1])<<8
	rawGyroY := int16(gyroBuf[2]) | int16(gyroBuf[3])<<8
	rawGyroZ := int16(gyroBuf[4]) | int16(gyroBuf[5])<<8

	// Convert to rad/s (±2000 dps range)
	const gyroSens = 16.4                       // LSB/dps for ±2000 dps
	const degToRad = 3.141592653589793 / 180.0

	gyroX := float64(rawGyroX) / gyroSens * degToRad
	gyroY := float64(rawGyroY) / gyroSens * degToRad
	gyroZ := float64(rawGyroZ) / gyroSens * degToRad

	// Read accelerometer (6 bytes)
	accelBuf := make([]byte, 6)
	if err := dev.ReadReg(ismOutXLA, accelBuf); err != nil {
		return [3]float64{}, [3]float64{}, err
	}

	rawAccelX := int16(accelBuf[0]) | int16(accelBuf[1])<<8
	rawAccelY := int16(accelBuf[2]) | int16(accelBuf[3])<<8
	rawAccelZ := int16(accelBuf[4]) | int16(accelBuf[5])<<8

	// Convert to m/s² (±16g range)
	const accelSens = 2048.0 // LSB/g for ±16g
	const gravity = 9.80665  // m/s²

	accelX := float64(rawAccelX) / accelSens * gravity
	accelY := float64(rawAccelY) / accelSens * gravity
	accelZ := float64(rawAccelZ) / accelSens * gravity

	return [3]float64{accelX, accelY, accelZ}, [3]float64{gyroX, gyroY, gyroZ}, nil
}