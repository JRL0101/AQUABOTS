#include <pigpio.h>
#include <iostream>
#include <cstdlib>  // For exit()

#define SERVO_PIN 13  // GPIO 18 (Pin 12)

// Map angle (0-180) to pulse width (500-2500μs)
int mapAngleToPulse(int angle) {
    return 500 + angle * (2000 / 180);
}

int main() {
    // Initialize pigpio
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio!" << std::endl;
        return 1;
    }

    // Set servo pin as output
    gpioSetMode(SERVO_PIN, PI_OUTPUT);

    std::cout << "Servo control started. Enter angles (0-180)." << std::endl;

    while (true) {
        int angle;
        std::cout << "Enter angle (0-180): ";
        std::cin >> angle;

        // Validate input
        if (angle < 0 || angle > 180) {
            std::cerr << "Invalid angle! Use 0-180." << std::endl;
            continue;
        }

        // Calculate and send pulse width
        int pulseWidth = mapAngleToPulse(angle);
        gpioServo(SERVO_PIN, pulseWidth);
        std::cout << "Moving to " << angle << "° (Pulse: " << pulseWidth << "μs)" << std::endl;

        // Small delay for servo to respond
        gpioDelay(200000);  // 200ms delay (pigpio uses microseconds)
    }

    // Cleanup (unreachable in this loop, but good practice)
    gpioTerminate();
    return 0;
}

