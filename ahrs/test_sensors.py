import time
from ism330_min import ISM330
from mmc5983 import MMC5983MA

# CircuitPython I2C for the magnetometer
import board, busio

def main():
    print("Testing AHRS Sensors…\n")

    # ---------- ISM330DHCX (smbus2 minimal driver) ----------
    print("Initializing ISM330DHCX (direct smbus2)…")
    imu = ISM330(bus=1, address=0x6B)   # SDO high -> 0x6B
    imu.begin()
    ax, ay, az = imu.read_accel()
    gx, gy, gz = imu.read_gyro()
    print("✓ ISM330DHCX OK @0x6B")
    print(f"  Accel: [{ax:6.2f}, {ay:6.2f}, {az:6.2f}] m/s²")
    print(f"  Gyro:  [{gx:6.3f}, {gy:6.3f}, {gz:6.3f}] rad/s\n")

    # ---------- MMC5983MA (CircuitPython-style driver) ----------
    print("Initializing MMC5983MA (0x30)…")
    i2c = busio.I2C(board.SCL, board.SDA)
    mag = MMC5983MA(i2c, address=0x30)  # runs set/reset and oneshot-ready
    mx, my, mz = mag.magnetic
    print("✓ MMC5983MA OK @0x30")
    print(f"  Mag:   [{mx:7.2f}, {my:7.2f}, {mz:7.2f}] µT\n")

    # ---------- Stream ----------
    time.sleep(0.05)
    print("Streaming (Ctrl+C to stop)…\n")
    print("     Accel (m/s²)              Gyro (rad/s)              Mag (µT)")
    print("-"*78)
    try:
        while True:
            try:
                ax, ay, az = imu.read_accel()
                gx, gy, gz = imu.read_gyro()
            except Exception as e:
                print(f"A/G read error: {e}")
                continue

            try:
                mx, my, mz = mag.magnetic
            except Exception as e:
                print(f"MAG read error: {e}")
                continue

            print(f"A:[{ax:6.2f},{ay:6.2f},{az:6.2f}] "
                  f"G:[{gx:6.3f},{gy:6.3f},{gz:6.3f}] "
                  f"M:[{mx:6.1f},{my:6.1f},{mz:6.1f}]")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n✓ Stopped")

if __name__ == "__main__":
    main()
