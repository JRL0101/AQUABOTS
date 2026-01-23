#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <stdexcept>
#include <cstdint>

using namespace std;

// Constants
const int IMU_ERROR_CODE = 23456789;
const char* I2C_DEVICE = "/dev/i2c-1";
const int MPU6050_ADDR = 0x68;

// IMU Register Map
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;
const uint8_t GYRO_XOUT_H = 0x43;

class MPU6050 {
private:
    int i2c_fd;

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        if (write(i2c_fd, data, 2) != 2) {
            throw runtime_error("I2C write failed");
        }
    }

    int16_t read16(uint8_t reg) {
        uint8_t data[2];
        if (write(i2c_fd, &reg, 1) != 1) {
            throw runtime_error("I2C register select failed");
        }
        if (read(i2c_fd, data, 2) != 2) {
            throw runtime_error("I2C read failed");
        }
        return (data[0] << 8) | data[1];
    }

public:
    MPU6050() : i2c_fd(-1) {
        // Open I2C bus
        if ((i2c_fd = open(I2C_DEVICE, O_RDWR)) < 0) {
            throw runtime_error("Failed to open I2C device");
        }

        // Set slave address
        if (ioctl(i2c_fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
            close(i2c_fd);
            throw runtime_error("Failed to acquire bus access");
        }

        // Wake up MPU6050
        writeRegister(PWR_MGMT_1, 0);
    }

    ~MPU6050() {
        if (i2c_fd >= 0) {
            close(i2c_fd);
        }
    }

    void readAll(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
        try {
            // Read accelerometer (registers 0x3B-0x40)
            int16_t accel_x = read16(ACCEL_XOUT_H);
            int16_t accel_y = read16(ACCEL_XOUT_H + 2);
            int16_t accel_z = read16(ACCEL_XOUT_H + 4);

            // Read gyroscope (registers 0x43-0x48)
            int16_t gyro_x = read16(GYRO_XOUT_H);
            int16_t gyro_y = read16(GYRO_XOUT_H + 2);
            int16_t gyro_z = read16(GYRO_XOUT_H + 4);

            // Convert to meaningful units
            const float accel_scale = 16384.0f;  // ±2g range
            const float gyro_scale = 131.0f;     // ±250°/s range

            ax = accel_x / accel_scale;
            ay = accel_y / accel_scale;
            az = accel_z / accel_scale;
            gx = gyro_x / gyro_scale;
            gy = gyro_y / gyro_scale;
            gz = gyro_z / gyro_scale;

            // Validate readings
            if (isnan(ax) || isinf(ax) || abs(ax) > 10.0f ||
                isnan(ay) || isinf(ay) || abs(ay) > 10.0f ||
                isnan(az) || isinf(az) || abs(az) > 10.0f) {
                throw runtime_error("Invalid accelerometer readings");
            }

        } catch (...) {
            ax = ay = az = gx = gy = gz = IMU_ERROR_CODE;
            throw;
        }
    }
};

int main() {
    try {
        MPU6050 imu;

        while (true) {
            float ax, ay, az, gx, gy, gz;
            
            try {
                imu.readAll(ax, ay, az, gx, gy, gz);
            } catch (...) {
                // Continue to output error values
            }

            cout << "{"
                 << "\"accel_x\":" << ax << ","
                 << "\"accel_y\":" << ay << ","
                 << "\"accel_z\":" << az << ","
                 << "\"gyro_x\":" << gx << ","
                 << "\"gyro_y\":" << gy << ","
                 << "\"gyro_z\":" << gz 
                 << "}" << endl;

            usleep(100000); // 100ms delay
        }
    } catch (const exception& e) {
        cerr << "IMU Error: " << e.what() << endl;
        cout << "{\"accel_x\":23456789,\"accel_y\":23456789,\"accel_z\":23456789,"
             << "\"gyro_x\":23456789,\"gyro_y\":23456789,\"gyro_z\":23456789}" << endl;
        return 1;
    }

    return 0;
}







	
		
		
		
		
