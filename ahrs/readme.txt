

standalone ahrs GO program.  Algorithm prototype is here:

pi@adsb-pi:~/adsb-receiver/ahrs $ go build -o mahony_ahrs mahony_ahrs.go
pi@adsb-pi:~/adsb-receiver/ahrs $ ./mahony_ahrs


Algo plus GDL90 transmitter is here:

# AHRS GDL90 Sender

A standalone Go program that reads IMU sensor data (gyroscope, accelerometer, and magnetometer), applies the Mahony AHRS fusion algorithm, and broadcasts orientation data using the GDL90 extended specification for ForeFlight and other EFB applications.

## Features

- **9-DOF Sensor Fusion**: Combines gyroscope, accelerometer, and magnetometer data using the Mahony AHRS algorithm
- **6-DOF Fallback**: Can operate with gyro + accel only if magnetometer is unavailable
- **GDL90 AHRS Messages**: Sends roll, pitch, and heading using ForeFlight's extended GDL90 specification
- **Real-time UDP Broadcast**: Sends attitude data at 5 Hz (recommended by ForeFlight)
- **High Sample Rate**: Runs Mahony filter at 100 Hz for smooth, accurate attitude estimation

## Hardware Requirements

- **Raspberry Pi** (or similar Linux SBC with I2C)
- **ISM330DHCX** - 6-axis IMU (gyroscope + accelerometer)
- **MMC5983MA** - 3-axis magnetometer
- Both sensors connected via I2C

## Software Dependencies

```bash
# Install Go dependencies
go get golang.org/x/exp/io/i2c
```

## Building

```bash
# Build the standalone program
go build -o ahrs-sender ahrs-sender.go

# Or run directly
go run ahrs-sender.go
```

## Configuration

Edit the constants in `main()` to match your setup:

```go
const (
    targetIP   = "192.168.10.255" // Your network broadcast or iPad IP
    targetPort = 4000              // GDL90 standard port
    sampleRate = 100.0             // Filter update rate (Hz)
    ahrsRate   = 5.0               // GDL90 send rate (Hz)
    i2cBus     = "/dev/i2c-1"      // I2C bus device
    useMag     = true              // true = 9DOF, false = 6DOF
)
```

### Network Configuration

**Option 1: UDP Broadcast** (simplest, works on same subnet)
```go
targetIP = "192.168.10.255"  // Change to match your subnet
```

**Option 2: UDP Unicast** (recommended for reliability on iOS)
```go
targetIP = "192.168.10.123"  // Your iPad/iPhone IP address
```

To find your iOS device IP, you can listen for ForeFlight's broadcast on port 63093.

## Running

### Run directly:
```bash
sudo ./ahrs-sender
```

### Run as a systemd service:

Create `/etc/systemd/system/ahrs-sender.service`:

```ini
[Unit]
Description=AHRS GDL90 Sender
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/pi/ahrs-sender
ExecStart=/home/pi/ahrs-sender/ahrs-sender
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable ahrs-sender
sudo systemctl start ahrs-sender
sudo systemctl status ahrs-sender
```

View logs:
```bash
sudo journalctl -u ahrs-sender -f
```

## Integration with Existing ADS-B Receiver

This AHRS sender is designed to work alongside your existing `stratux-aggregator` system. Both programs send to UDP port 4000:

- **stratux-aggregator**: Sends heartbeat, ownship position, traffic reports, weather
- **ahrs-sender**: Sends AHRS attitude data (roll, pitch, heading)

They complement each other perfectly - your EFB will receive both position/traffic data AND attitude information for synthetic vision.

### Running Both Together

1. **Start the ADS-B aggregator:**
```bash
cd /path/to/adsb-receiver
sudo systemctl start stratux-aggregator
```

2. **Start the AHRS sender:**
```bash
sudo systemctl start ahrs-sender
```

Both services can run simultaneously without conflict since they send different message types.

## GDL90 Protocol Details

### AHRS Message Format (0x65 0x01)

The program sends ForeFlight's extended GDL90 AHRS messages:

| Byte | Field | Description |
|------|-------|-------------|
| 0 | Message ID | 0x65 (ForeFlight extended) |
| 1 | Sub-ID | 0x01 (AHRS message) |
| 2-3 | Roll | Signed int16, units of 0.1° |
| 4-5 | Pitch | Signed int16, units of 0.1° |
| 6-7 | Heading | Unsigned int16, units of 0.1°, MSB=1 if magnetic |
| 8-9 | IAS | Unsigned int16, knots (0x7FFF = invalid) |
| 10-11 | TAS | Unsigned int16, knots (0x7FFF = invalid) |

Plus CRC-16 and GDL90 framing (flag bytes and byte stuffing).

### Message Timing

- **Filter update**: 100 Hz (every 10ms)
- **AHRS broadcast**: 5 Hz (every 200ms) - ForeFlight's recommended rate
- Sending faster than 5 Hz can cause iOS packet loss

## Mahony AHRS Algorithm

The program uses the Mahony complementary filter which:

1. **Fuses sensor data** from gyroscope, accelerometer, and magnetometer
2. **Estimates orientation** as a quaternion (continuously updated)
3. **Provides stable attitude** with low drift and good dynamic response
4. **Handles sensor errors** through proportional and integral feedback

### Tuning Parameters

In the code, you can adjust:

```go
kp: 2.0,   // Proportional gain (higher = faster correction, more noise)
ki: 0.01,  // Integral gain (higher = better drift correction)
```

- Increase `kp` if the attitude estimate is sluggish
- Increase `ki` if you see long-term drift
- Decrease both if the estimate is too noisy

## Coordinate System

The IMU coordinate frame follows the aircraft body frame convention:

- **X-axis**: Forward (nose direction)
- **Y-axis**: Right wing
- **Z-axis**: Down

Euler angles:
- **Roll**: Rotation around X-axis (positive = right wing down)
- **Pitch**: Rotation around Y-axis (positive = nose up)
- **Yaw/Heading**: Rotation around Z-axis (0° = North, 90° = East)

## Troubleshooting

### No data in ForeFlight/Garmin Pilot

1. **Check network connectivity**:
   ```bash
   # Ping your iPad
   ping 192.168.10.123
   ```

2. **Verify UDP packets are being sent**:
   ```bash
   sudo tcpdump -i any -n 'udp port 4000'
   ```

3. **Check broadcast address** matches your network subnet

4. **Try unicast instead of broadcast** - iOS handles unicast better

### Sensors not detected

```bash
# Check I2C devices
sudo i2cdetect -y 1

# Should see:
# 0x30 (MMC5983MA magnetometer)
# 0x6B (ISM330DHCX IMU)
```

### Attitude drifts over time

- **With magnetometer (9DOF)**: Heading should be stable. Check for magnetic interference (motors, wires, metal).
- **Without magnetometer (6DOF)**: Yaw will drift - this is normal and expected.

Increase integral gain (`ki`) slightly to compensate for drift:
```go
ahrs.ki = 0.02  // Increase from 0.01
```

### Attitude is noisy/jittery

Decrease proportional gain:
```go
ahrs.kp = 1.0  // Decrease from 2.0
```

### Heading is backwards/inverted

Check magnetometer axis orientation. You may need to swap or invert axes:
```go
// In readMMC(), try:
mag[0] = -x  // Invert X
mag[1] = y
mag[2] = -z  // Invert Z
```

## Performance Tips

1. **Run at high priority** (if using systemd, add `Nice=-10`)
2. **Use a real-time kernel** for most consistent timing
3. **Avoid other I2C traffic** during critical samples
4. **Keep sensor sample rate high** (100 Hz minimum)

## Testing Without Hardware

To test the GDL90 sending without real sensors, you can modify the code to use simulated data:

```go
// Replace sensor reads with:
accel := [3]float64{0.0, 0.0, 9.81}  // 1G down
gyro := [3]float64{0.0, 0.0, 0.0}    // No rotation
mag := [3]float64{20.0, 0.0, -40.0}  // Typical magnetic field

ahrs.Update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2])
```

## References

- [GDL 90 Data Interface Specification](https://www.garmin.com/en-US/legal/gdl-90-interface-specification/)
- [ForeFlight GDL90 Extended Specification](https://www.foreflight.com/connect/spec/)
- [Mahony AHRS Algorithm Paper](https://hal.archives-ouvertes.fr/hal-00488376/document)
- [ISM330DHCX Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)
- [MMC5983MA Datasheet](https://www.memsic.com/Public/Uploads/uploadfile/files/20220119/MMC5983MADatasheetRev.C.pdf)

## License

This code is provided as-is for educational and experimental purposes.

## Related Projects

- **stratux-aggregator**: Your main ADS-B receiver that this complements
- [Stratux](https://github.com/cyoung/stratux): The original open-source ADS-B receiver for aviation

## Support

This is provided as a working example. For issues:
1. Check sensor wiring and I2C addresses
2. Verify GDL90 messages with tcpdump/Wireshark
3. Test with ForeFlight's diagnostic tools
4. Compare with working Stratux/GDL90 implementations