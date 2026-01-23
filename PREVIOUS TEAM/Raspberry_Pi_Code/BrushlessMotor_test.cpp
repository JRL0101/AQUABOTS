#include <pigpio.h>
#include <iostream>
#include <cmath>
#include <unistd.h>  // For sleep()

#define ESC_PIN 18       // Use GPIO 18 (Physical Pin 12) - Hardware PWM
#define SWITCH_PIN 17    // GPIO 17 (Physical Pin 11)
#define LED_PIN 22       // GPIO 22 (Physical Pin 15)
#define SPI_CHANNEL 0    // SPI Channel 0 for MCP3008
#define SPI_SPEED 1000000 // SPI speed (1MHz)
#define MIN_PWM 1000     // 1000us = Min ESC signal
#define MAX_PWM 2000     // 2000us = Max ESC signal

using namespace std;

// Function to map potentiometer value to PWM range
int mapPotToPWM(int potValue) {
    return (int)round(((float)potValue / 1023.0) * (MAX_PWM - MIN_PWM)) + MIN_PWM;
}

// Function to read ADC value from MCP3008 using SPI
int readADC(int channel) {
    uint8_t buffer[3];
    buffer[0] = 1;                        // Start bit
    buffer[1] = (8 + channel) << 4;       // Channel selection
    buffer[2] = 0;                        // Empty byte

    // SPI communication with MCP3008
    gpioSPIDataRW(SPI_CHANNEL, buffer, 3);
    return ((buffer[1] & 3) << 8) + buffer[2]; // Extract 10-bit result
}

int main() {
    if (gpioInitialise() < 0) {
        cout << "Failed to initialize pigpio!" << endl;
        return -1;
    }

    gpioSetMode(SWITCH_PIN, PI_INPUT);
    gpioSetPullUpDown(SWITCH_PIN, PI_PUD_UP);
    gpioSetMode(LED_PIN, PI_OUTPUT);
    gpioSetMode(ESC_PIN, PI_OUTPUT);

    // Initialize SPI
    if (gpioSPISetup(SPI_CHANNEL, SPI_SPEED) < 0) {
        cout << "Failed to initialize SPI!" << endl;
        return -1;
    }

    cout << "Initializing ESC..." << endl;
    gpioServo(ESC_PIN, MIN_PWM);  // Set ESC to minimum (idle)
    sleep(5);  // Allow time for ESC to initialize

    while (true) {
        int switchState = gpioRead(SWITCH_PIN);
        int potValue = readADC(0);  // Read from MCP3008 channel 0
        int escValue = mapPotToPWM(potValue);

        if (switchState == PI_LOW) {  // Switch ON
            gpioWrite(LED_PIN, PI_HIGH);
            gpioServo(ESC_PIN, escValue);
            cout << "Motor Speed: " << escValue << "us PWM" << endl;
        } else {  // Switch OFF
            gpioWrite(LED_PIN, PI_LOW);
            gpioServo(ESC_PIN, MIN_PWM); // Idle
            cout << "Motor Stopped!" << endl;
        }

        gpioDelay(100000); // Delay 100ms (100,000 microseconds)
    }

    gpioTerminate();
    return 0;
}

