import qwiic_ism330dhcx as qwiic
import qwiic_i2c

# Force bus 1 explicitly instead of relying on env/defaults
drv = qwiic_i2c.getI2CDriver(1)  # use bus 1 on the Pi

imu = qwiic.QwiicISM330DHCX(address=0x6B, i2c_driver=drv)

print("connected?", imu.is_connected())
if not imu.is_connected():
    print("still not connected on 0x6B; trying 0x6A…")
    imu = qwiic.QwiicISM330DHCX(address=0x6A, i2c_driver=drv)

if imu.is_connected() and imu.begin():
    print("SparkFun Qwiic init OK")
else:
    print("SparkFun Qwiic still not happy — see Option B")