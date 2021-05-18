import melopero_lsm9ds1 as mp
import time
import struct

from socket import *

g = 9.81
      
def vector3_sub(x, y):
    return [x[0] - y[0], x[1] - y[1], x[2] - y[2]]
    
def vector3_sum(x, y):
    return [x[0] + y[0], x[1] + y[1], x[2] + y[2]]
    
def vector3_scal_mult(x, alpha):
    return [x[0]*alpha, x[1]*alpha, x[2]*alpha]
    
def vector3_round(x, n_digit):
    return [round(x[0], n_digit), round(x[1], n_digit), round(x[2], n_digit)]
    
def get_acc_g(sensor):
    acc = sensor.get_acc()
    return [acc[0]*g, acc[1]*g, acc[2]*g]


def read_imu(sensor, n, delay):
    gyro = [0, 0, 0]
    acc = [0, 0, 0]
    magn = [0, 0, 0]
    
    for i in range(n):
        gyro = vector3_sum(gyro, sensor.get_gyro())
        acc = vector3_sum(acc, get_acc_g(sensor))
        magn = vector3_sum(magn, sensor.get_mag())
        time.sleep(delay)
        
    gyro = vector3_scal_mult(gyro, 1.0/n)
    acc = vector3_scal_mult(acc, 1.0/n)
    magn = vector3_scal_mult(magn, 1.0/n)

    return gyro, acc, magn
        

imu_out = bytearray()

sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
sensor = mp.LSM9DS1()
sensor.use_i2c()

sensor.set_acc_range(mp.LSM9DS1.LIN_ACC_2G_RANGE)
sensor.set_gyro_range(sensor.ANGULAR_RATE_245DPS_RANGE)
sensor.set_gyro_odr(mp.LSM9DS1.ODR_952Hz)
sensor.set_acc_odr(mp.LSM9DS1.ODR_952Hz)
sensor.set_mag_odr(mp.LSM9DS1.MAG_CONTINUOUS_MODE)

gx, gy, gz = 0, 0, 0

print("Calibrating inertial unit. . .")
gyro0, acc0, magn0 = read_imu(sensor, 500, 1.0/952.0)
print("Done!")


while True:
    try:
        gyro, acc, magn = read_imu(sensor, 100, 1.0/952.0)
        
        acc_measurements =  vector3_round(vector3_sub(acc, acc0), 1)
        gyro_measurements = vector3_round(vector3_sub(gyro, gyro0), 1)
        magn_measurements = vector3_round(vector3_sub(magn, magn0), 1)

        imu_out =  struct.pack("i", 0x00)
        imu_out += struct.pack("d", time.time())
        imu_out += struct.pack("d", acc_measurements[0])
        imu_out += struct.pack("d", acc_measurements[1])
        imu_out += struct.pack("d", acc_measurements[2])
        imu_out += struct.pack("d", gyro_measurements[0])
        imu_out += struct.pack("d", gyro_measurements[1])
        imu_out += struct.pack("d", gyro_measurements[2])
        imu_out += struct.pack("d", magn_measurements[0])
        imu_out += struct.pack("d", magn_measurements[1])
        imu_out += struct.pack("d", magn_measurements[2])
        
        sock.sendto(imu_out, ("127.0.0.1", 7777))
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(e)
        

#Close the device correctly!
sensor.close()
