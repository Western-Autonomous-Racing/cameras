# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_mpu6050
import matplotlib.pyplot as plt

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
mpu = adafruit_mpu6050.MPU6050(i2c)

n = 0
start = time.time_ns()

acceleration_x = []
acceleration_y = []
acceleration_z = []
gyro_x = []
gyro_y = []
gyro_z = []
frame_time = []
time_values = []

while time.time_ns() - start < 10000000000:
    try:
        start_frame = time.time_ns()
        acceleration = mpu.acceleration
        acceleration_x.append(acceleration[0])
        acceleration_y.append(acceleration[1])
        acceleration_z.append(acceleration[2])

        gyro = mpu.gyro
        gyro_x.append(gyro[0])
        gyro_y.append(gyro[1])
        gyro_z.append(gyro[2])
        time_values.append(time.time())

        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % acceleration)
        # print("Gyro X:%.2f, Y: %.2f, Z: %.2f rad/s" % mpu.gyro)
        print("Temperature: %.2f C" % mpu.temperature)
        print("")
        n+=1
        time.sleep(0.025)
        frame_time.append((time.time_ns() - start_frame)/1000000)
        
    except KeyboardInterrupt:

        break

time_elapsed = time.time_ns() - start
print(f"Time elapsed: {time_elapsed}")
print(f"Number of iterations: {n}")
print(f"Average ms per iteration: {time_elapsed/(n*1000000)}")

plt.plot(time_values, acceleration_x, label='Accel X')
plt.plot(time_values, acceleration_y, label='Accel Y')
plt.plot(time_values, acceleration_z, label='Accel Z')
# plt.plot(time_values, gyro_x, label='Gyro X')
# plt.plot(time_values, gyro_y, label='Gyro Y')
# plt.plot(time_values, gyro_z, label='Gyro Z')
# plt.plot(time_values, frame_time, label='Frame Time')
plt.xlabel('Time')
plt.ylabel('Acceleration (m/s^2)')
plt.legend()
plt.show()
