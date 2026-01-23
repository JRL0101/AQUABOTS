#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <wiringSerial.h>
#include <unistd.h>
#include <ctime>

using namespace std;

const int GPS_ERROR = 34567890;
const int TIMEOUT_SECONDS = 30;

struct GPSData {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    bool valid = false;
};

double safe_stod(const string& str) {
    if (str.empty()) return 0.0;
    try {
        return stod(str);
    } catch (...) {
        return 0.0;
    }
}

GPSData parseGPGGA(const string& sentence) {
    GPSData data;
    istringstream ss(sentence);
    string token;
    
    getline(ss, token, ',');
    if (token != "$GPGGA" && token != "$GNGGA") return data;

    for (int i = 0; i < 5; i++) getline(ss, token, ',');
    
    getline(ss, token, ',');
    if (token.empty() || token == "0") return data;

    getline(ss, token, ',');
    if (!token.empty() && token.size() >= 4) {
        double lat = safe_stod(token.substr(0, 2)) + safe_stod(token.substr(2)) / 60.0;
        getline(ss, token, ',');
        if (token == "S") lat = -lat;
        data.latitude = lat;
    }

    getline(ss, token, ',');
    if (!token.empty() && token.size() >= 5) {
        double lon = safe_stod(token.substr(0, 3)) + safe_stod(token.substr(3)) / 60.0;
        getline(ss, token, ',');
        if (token == "W") lon = -lon;
        data.longitude = lon;
    }

    for (int i = 0; i < 3; i++) getline(ss, token, ',');
    if (!token.empty()) {
        data.altitude = safe_stod(token);
    }

    data.valid = true;
    return data;
}

GPSData parseGNRMC(const string& sentence) {
    GPSData data;
    istringstream ss(sentence);
    string token;
    
    getline(ss, token, ',');
    if (token != "$GNRMC" && token != "$GPRMC") return data;

    getline(ss, token, ','); // Time
    getline(ss, token, ','); // Status
    if (token != "A") return data;

    getline(ss, token, ','); // Latitude
    if (!token.empty() && token.size() >= 4) {
        double lat = safe_stod(token.substr(0, 2)) + safe_stod(token.substr(2)) / 60.0;
        getline(ss, token, ','); // N/S
        if (token == "S") lat = -lat;
        data.latitude = lat;
    }

    getline(ss, token, ','); // Longitude
    if (!token.empty() && token.size() >= 5) {
        double lon = safe_stod(token.substr(0, 3)) + safe_stod(token.substr(3)) / 60.0;
        getline(ss, token, ','); // E/W
        if (token == "W") lon = -lon;
        data.longitude = lon;
    }

    // RMC doesn't have altitude, so we'll set a default
    data.altitude = 0.0;
    data.valid = true;
    return data;
}

int main() {
    int fd = serialOpen("/dev/serial0", 9600);
    if (fd < 0) {
        cout << "{\"error\":\"Failed to open serial port\"}" << endl;
        return GPS_ERROR;
    }

    time_t start_time = time(nullptr);
    string buffer;

    while (difftime(time(nullptr), start_time) < TIMEOUT_SECONDS) {
        while (serialDataAvail(fd)) {
            char c = serialGetchar(fd);
            if (c == '\n') {
                if (buffer.find("$GPGGA") != string::npos || 
                    buffer.find("$GNGGA") != string::npos) {
                    GPSData data = parseGPGGA(buffer);
                    if (data.valid) {
                        cout << "{\"lat\":" << fixed << setprecision(6) 
                             << data.latitude << ",\"lon\":" << data.longitude 
                             << ",\"altitude\":" << data.altitude << "}" << endl;
                        serialClose(fd);
                        return 0;
                    }
                }
                else if (buffer.find("$GNRMC") != string::npos || 
                         buffer.find("$GPRMC") != string::npos) {
                    GPSData data = parseGNRMC(buffer);
                    if (data.valid) {
                        cout << "{\"lat\":" << fixed << setprecision(6) 
                             << data.latitude << ",\"lon\":" << data.longitude 
                             << ",\"altitude\":0.0}" << endl;
                        serialClose(fd);
                        return 0;
                    }
                }
                buffer.clear();
            } 
            else if (c != '\r') {
                buffer += c;
            }
        }
        usleep(10000);
    }

    cout << "{\"error\":\"No valid GPS fix after " << TIMEOUT_SECONDS << " seconds\"}" << endl;
    serialClose(fd);
    return GPS_ERROR;
}






