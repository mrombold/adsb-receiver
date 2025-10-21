# Minimal ISM330DHCX driver using smbus2
from smbus2 import SMBus, i2c_msg
import math, time

WHO_AM_I   = 0x0F
CTRL1_XL   = 0x10
CTRL2_G    = 0x11
CTRL3_C    = 0x12
OUTX_L_G   = 0x22
OUTX_L_A   = 0x28

class ISM330:
    def __init__(self, bus=1, address=0x6B):
        self.busnum = bus
        self.addr   = address
        self.bus    = SMBus(bus)

    def _wr(self, reg, val):
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, [reg, val]))

    def _rd(self, reg, n):
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, [reg]))
        r = i2c_msg.read(self.addr, n)
        self.bus.i2c_rdwr(r)
        return bytes(r)

    def begin(self):
        who = self._rd(WHO_AM_I, 1)[0]
        if who != 0x6B:
            raise RuntimeError(f"Unexpected WHO_AM_I 0x{who:02X}, expected 0x6B")
        # BDU=1 (bit6), IF_INC=1 (bit2) for auto-increment multi-byte reads
        self._wr(CTRL3_C, 0b01000100)
        # Accel: ODR=104 Hz (0100<<4), FS=±4g (10<<2)
        self._wr(CTRL1_XL, 0b01001000)
        # Gyro:  ODR=104 Hz (0100<<4), FS=2000 dps (11<<2)
        self._wr(CTRL2_G,  0b01001100)
        time.sleep(0.02)

    @staticmethod
    def _i16(lo, hi):
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v

    def read_gyro(self):
        b = self._rd(OUTX_L_G, 6)
        x = self._i16(b[0], b[1])
        y = self._i16(b[2], b[3])
        z = self._i16(b[4], b[5])
        # 2000 dps scale: 70 mdps/LSB → rad/s
        dps_to_rps = math.pi/180.0
        return (x*70e-3*dps_to_rps, y*70e-3*dps_to_rps, z*70e-3*dps_to_rps)

    def read_accel(self):
        b = self._rd(OUTX_L_A, 6)
        x = self._i16(b[0], b[1])
        y = self._i16(b[2], b[3])
        z = self._i16(b[4], b[5])
        # ±4g scale: 0.122 mg/LSB → m/s²
        return tuple(v*0.122e-3*9.80665 for v in (x, y, z))
