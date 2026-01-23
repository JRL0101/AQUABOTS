#include <iostream>
#include <wiringPi.h>

#define ESC_GPIO 18  // GPIO18 = physical pin 12

using namespace std;

int main() {
    if (wiringPiSetupGpio() == -1) {
        cerr << "Error: WiringPi setup failed!" << endl;
        return 1;
    }

    pinMode(ESC_GPIO, PWM_OUTPUT);
    
    // Configure hardware PWM for 50Hz
    pwmSetMode(PWM_MODE_MS);  // Mark-Space mode for servo/ESC
    pwmSetClock(192);         // 19.2MHz / 192 = 100kHz
    pwmSetRange(2000);        // 100kHz / 2000 = 50Hz (20ms period)

    //cout << "ESC control ready. Input pulse width in microseconds (1000-2000):" << endl;

    //while (true) {
        //int pulseWidth;
        //cout << "Enter pulse width (µs, 1000 to 2000): ";
        //cin >> pulseWidth;

        //if (pulseWidth < 1000 || pulseWidth > 2000) {
            //cout << "Invalid input. Enter a value between 1000 and 2000 µs." << endl;
            //continue;
        //}

        // Convert pulse width (µs) to pwmWrite value
        // Each unit = 10µs (since 100kHz base clock = 10µs steps)
        int pwm_value = 1600 / 10;

        pwmWrite(ESC_GPIO, pwm_value);
        //cout << "ESC set to " << pulseWidth << " µs (" << pwm_value << " pwmWrite)" << endl;
        delay(1000);
   // }

    return 0;
}





