"""
Simple MMC5983MA driver for CircuitPython
"""
import time
from adafruit_bus_device.i2c_device import I2CDevice

_MMC5983_I2CADDR_DEFAULT = 0x30

# Registers
_MMC5983_OUT_X_L   = 0x00
_MMC5983_STATUS    = 0x08
_MMC5983_CONTROL_0 = 0x09
_MMC5983_CONTROL_1 = 0x0A
_MMC5983_CONTROL_2 = 0x0B
_MMC5983_PRODUCT_ID= 0x2F

# CONTROL_0 bits (per datasheet)
_TM_M   = 0x01  # Trigger magnetic one-shot measurement
_SET    = 0x08  # Set magnetization
_RESET  = 0x10  # Reset magnetization
_TM_T   = 0x20  # Trigger temperature (unused here)

# STATUS bits
_M_DONE = 0x01  # Magnetic data ready

# Scale: 0.00610 µT/LSB at 18-bit (datasheet)
_LSB_TO_UT = 0.00610
_OFFSET_18 = 131072  # 2^17 for signed conversion of 18-bit packed values

class MMC5983MA:
    """Driver for MMC5983MA magnetometer (one-shot mode with SET/RESET)"""

    def __init__(self, i2c, address=_MMC5983_I2CADDR_DEFAULT):
        self.i2c_device = I2CDevice(i2c, address)
        self._buffer = bytearray(9)

        # Sanity check: product ID
        product_id = self._read_register(_MMC5983_PRODUCT_ID, 1)[0]
        if product_id != 0x30:
            raise RuntimeError(f"Failed to find MMC5983MA! Product ID: 0x{product_id:02X}")

        # Soft reset
        self._write_register(_MMC5983_CONTROL_1, 0x80)
        time.sleep(0.02)

        # Ensure continuous mode is OFF; we use one-shot
        self._write_register(_MMC5983_CONTROL_2, 0x00)

        # Do an initial SET/RESET to magnetize the sensor core
        self._set_reset()

    def _write_register(self, register, value):
        buf = bytearray([register, value])
        with self.i2c_device as i2c:
            i2c.write(buf)

    def _read_register(self, register, length):
        # Reuse internal buffer for efficiency
        buf = bytearray([register])
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buf, self._buffer, out_end=1, in_end=length)
        return self._buffer[:length]

    def _wait_data_ready(self, timeout_ms=50):
        """Poll STATUS.M_DONE up to timeout_ms; return True if ready."""
        t0 = time.monotonic_ns()
        deadline = t0 + timeout_ms * 1_000_000
        while time.monotonic_ns() < deadline:
            status = self._read_register(_MMC5983_STATUS, 1)[0]
            if status & _M_DONE:
                return True
            # Small wait (sensor max conv time is a couple ms)
            time.sleep(0.001)
        return False

    def _set_reset(self):
        """Run SET then RESET with short waits."""
        # SET pulse
        self._write_register(_MMC5983_CONTROL_0, _SET)
        time.sleep(0.002)
        # RESET pulse
        self._write_register(_MMC5983_CONTROL_0, _RESET)
        time.sleep(0.002)

    def _read_xyz_raw(self):
        """Trigger one-shot measurement and return raw 18-bit XYZ (unsigned)."""
        # Start measurement
        self._write_register(_MMC5983_CONTROL_0, _TM_M)

        # Wait for M_DONE
        if not self._wait_data_ready(timeout_ms=50):
            # If it times out, try a quick set/reset and one more attempt
            self._set_reset()
            self._write_register(_MMC5983_CONTROL_0, _TM_M)
            if not self._wait_data_ready(timeout_ms=50):
                raise RuntimeError("MMC5983MA: data-ready timeout")

        # Read 9 bytes: Xl, Xh, Yl, Yh, Zl, Zh, XYZ_2LSB, (then temp, status)
        data = self._read_register(_MMC5983_OUT_X_L, 9)

        # Pack 18-bit values (two LSBs for each axis are in data[6] as described)
        x = (data[0] << 10) | (data[1] << 2) | ((data[6] >> 6) & 0x03)
        y = (data[2] << 10) | (data[3] << 2) | ((data[6] >> 4) & 0x03)
        z = (data[4] << 10) | (data[5] << 2) | ((data[6] >> 2) & 0x03)
        return x, y, z

    @property
    def magnetic(self):
        """Return magnetic field (x, y, z) in microtesla (µT)."""
        # If you want maximum stability, uncomment to do SET/RESET every read:
        # self._set_reset()

        x, y, z = self._read_xyz_raw()

        # Convert unsigned 18-bit to signed using center offset, then scale
        x_ut = (x - _OFFSET_18) * _LSB_TO_UT
        y_ut = (y - _OFFSET_18) * _LSB_TO_UT
        z_ut = (z - _OFFSET_18) * _LSB_TO_UT
        return (x_ut, y_ut, z_ut)
