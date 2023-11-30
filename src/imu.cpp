#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cmath>
#include <vector>

#define MPU6050_ADDR 0x68

using namespace std;

int16_t rawXacc,rawYacc,rawZacc,rawXomg,rawYomg,rawZomg;

int main(int argc, char **argv) {

	//	---------- Connecting with MPU ---------- 
	int i2cbus;
    const char *bus = "/dev/i2c-8";
    if((i2cbus = open(bus, O_RDWR)) < 0) {
        cout << "Failed to open i2c bus" << endl;
        return 1;
    }
    if(ioctl(i2cbus, I2C_SLAVE, MPU6050_ADDR) < 0) {
        cout << "Failed to connect to MPU6050" << endl;
        return 1;
    }

    char config[2] = {0};
    config[0] = 0x6B; // PWR_MGMT_1 register
    config[1] = 0x00; // Wake up device
    // ---------- Connection done, MPU ready to send data ---------- 
    
	while (1){

		write(i2cbus, config, 2);
		char reg[1] = {0x3B}; // ACCEL_XOUT_H register
		write(i2cbus, reg, 1);
		char data[14] = {0};
		if(read(i2cbus, data, 14) != 14) {
		    ROS_ERROR("Error: Input/output error\n");
		    return 1;
		}

		// Convert data to acceleration values
		rawXacc = (data[0] << 8) | data[1];
		rawYacc = (data[2] << 8) | data[3];
		rawZacc = (data[4] << 8) | data[5];

		// Convert data to omega values
		rawXomg = (data[8] << 8) | data[9];
		rawYomg = (data[10] << 8) | data[11];
		rawZomg = (data[12] << 8) | data[13];

		std::cout<<"Acceleration: X="<<rawXacc<<", Y="<<rawYacc<<", Z="<<rawZacc<<std::endl;
		std::cout<<"Gyro:         X="<<rawXomg<<", Y="<<rawYomg<<", Z="<<rawZomg<<std::endl;
		std::cout<<std::endl;

	}   

    close(i2cbus);
    return 0;
}