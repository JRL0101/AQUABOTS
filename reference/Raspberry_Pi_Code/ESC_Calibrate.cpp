#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>

#define ESC_GPIO 18  // GPIO pin for ESC signal

using namespace std;

// Function to wait for user input
void waitForEnter(const string &message) {
    cout << message << endl;
    cout << "Press ENTER when ready...";
    cin.ignore();  // This waits for user to press Enter
}

int main() {
    // Initialize WiringPi
    if (wiringPiSetupGpio() == -1) {
        cerr << "Error: WiringPi setup failed!" << endl;
        return 1;
    }

    // Set up software PWM on ESC_GPIO with a range from 0 (0 µs) to 200 (2000 µs)
    softPwmCreate(ESC_GPIO, 0, 200);  // PWM range set to 200

    cout << "Starting ESC Calibration..." << endl;

    // Step 1: Enter Calibration Mode (Full Throttle)
    waitForEnter("Set your transmitter to 100% throttle (full throttle). Power on the ESC while holding the Set button.");
    softPwmWrite(ESC_GPIO, 190);  // Full throttle (1900 µs)
    delay(3000);  // Wait for the ESC to register full throttle

    // Step 2: Set Neutral Position
    waitForEnter("Move throttle to NEUTRAL position (middle throttle), then press Enter.");
    softPwmWrite(ESC_GPIO, 150);  // Neutral (1500 µs)
    delay(3000);  // Wait for the ESC to register neutral

    // Step 3: Set Full Throttle Position
    waitForEnter("Move throttle to FULL THROTTLE, then press Enter.");
    softPwmWrite(ESC_GPIO, 190);  // Full throttle (1900 µs)
    delay(3000);  // Wait for the ESC to register full throttle

    // Step 4: Set Full Brake Position
    waitForEnter("Move throttle to FULL BRAKE (lowest throttle), then press Enter.");
    softPwmWrite(ESC_GPIO, 110);  // Full brake (1100 µs)
    delay(3000);  // Wait for the ESC to register brake

    cout << "Calibration Complete!" << endl;

    // Return ESC to neutral
    softPwmWrite(ESC_GPIO, 150);  // Return to neutral position

    return 0;
}

