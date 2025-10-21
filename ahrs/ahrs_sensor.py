"""Test AHRS sensors - ISM330DHCX + MMC5983MA"""

import time
import board
import busio

# ISM330DHCX is compatible with LSM6DSOX
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

# Use local MMC5983MA driver
from mmc56x3 import MMC5983MA


def main():
    print("Testing AHRS Sensors...")
    print()
    
    # I2C setup
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Wait for I2C to initialize
    time.sleep(0.5)
    
    # Test ISM330DHCX (using LSM6DSOX class)
    print("Initializing ISM330DHCX...")
    
    # Try both possible addresses
    imu = None
    for addr in [0x6A, 0x6B]:
        try:
            print(f"  Trying address 0x{addr:02X}...")
            imu = LSM6DSOX(i2c, address=addr)
            print(f"✓ ISM330DHCX found at 0x{addr:02X}!")
            break
        except Exception as e:
            print(f"    Not at 0x{addr:02X}: {e}")
    
    if imu is None:
        print("✗ ISM330DHCX not found at either address")
        print("\nRun: sudo i2cdetect -y 1")
        print("to see what devices are on the I2C bus")
        return
    
    try:
        # Read a test value
        time.sleep(0.1)
        ax, ay, az = imu.acceleration
        gx, gy, gz = imu.gyro
        
        print(f"  Accel: [{ax:6.2f}, {ay:6.2f}, {az:6.2f}] m/s²")
        print(f"  Gyro:  [{gx:6.3f}, {gy:6.3f}, {gz:6.3f}] rad/s")
        
    except Exception as e:
        print(f"✗ Error reading IMU: {e}")
        import traceback
        traceback.print_exc()
        return
    
    print()
    
    # Test MMC5983MA
    print("Initializing MMC5983MA...")
    try:
        mag = MMC5983MA(i2c)
        print("✓ MMC5983MA found at 0x30!")
        
        time.sleep(0.1)
        mx, my, mz = mag.magnetic
        
        print(f"  Mag: [{mx:7.2f}, {my:7.2f}, {mz:7.2f}] µT")
        
    except Exception as e:
        print(f"✗ MMC5983MA error: {e}")
        import traceback
        traceback.print_exc()
        return
    
    print()
    print("✓ Both sensors working!")
    print()
    print("Running continuous test (Ctrl+C to stop)...")
    print()
    print("     Accel (m/s²)              Gyro (rad/s)              Mag (µT)")
    print("-" * 78)
    
    try:
        while True:
            ax, ay, az = imu.acceleration
            gx, gy, gz = imu.gyro
            mx, my, mz = mag.magnetic
            
            print(f"A:[{ax:6.2f},{ay:6.2f},{az:6.2f}] "
                  f"G:[{gx:6.3f},{gy:6.3f},{gz:6.3f}] "
                  f"M:[{mx:6.1f},{my:6.1f},{mz:6.1f}]")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n✓ Stopped by user")


if __name__ == '__main__':
    main()
