#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h> // for usleep function

// Function to read the raw data from the sensor
std::string readRawData(const std::string& deviceFile) {
    std::ifstream file(deviceFile);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// Function to extract temperature from raw data
float extractTemperature(const std::string& data) {
    size_t pos = data.find("t=");
    if (pos != std::string::npos) {
        std::string tempStr = data.substr(pos + 2);
        return std::stof(tempStr) / 1000.0f; // Convert millidegrees to degrees
    }
    return -999.9f; // Error value if temperature not found
}

int main() {
    // Path to the DS18B20 sensor data file
    std::string baseDir = "/sys/bus/w1/devices/";
    std::string deviceFolder;
    std::string deviceFile;

    // Find the sensor device folder (starts with "28-")
    std::ifstream devices("/sys/bus/w1/devices/w1_bus_master1/w1_master_slaves");
    if (devices.is_open()) {
        std::getline(devices, deviceFolder);
        deviceFile = baseDir + deviceFolder + "/w1_slave";
        devices.close();
    } else {
        std::cerr << "Error: Could not find DS18B20 sensor!" << std::endl;
        return 1;
    }

    // Main loop to read temperature every second
    while (true) {
        std::string rawData = readRawData(deviceFile);
        float temperature = extractTemperature(rawData);

        if (temperature != -999.9f) {
            std::cout << "Temperature: " << temperature << "°C" << std::endl;
        } else {
            std::cerr << "Error: Could not read temperature!" << std::endl;
        }

        usleep(1000000); // Wait 1 second (in microseconds)
    }

    return 0;
}


