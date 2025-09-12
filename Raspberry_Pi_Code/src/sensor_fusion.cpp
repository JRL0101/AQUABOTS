#include "sensor_fusion.h"
#include <cmath>

namespace sensor_fusion {

void LLA2ECEF(double LLA[3], double ECEF[3]) {
    int a = 6378137;
    double e = 8.1819190842622e-2;
    double lat = LLA[0];
    double lon = LLA[1];
    double alt = LLA[2];
    double N = a / std::sqrt(1 - std::pow(e, 2) * std::pow(std::sin(lat), 2));
    ECEF[0] = (N + alt) * std::cos(lat) * std::cos(lon);
    ECEF[1] = (N + alt) * std::cos(lat) * std::sin(lon);
    ECEF[2] = ((1 - std::pow(e, 2)) * N + alt) * std::sin(lat);
}

void ECEF2LLA(double ECEF[3], double LLA[3]) {
    int a = 6378137;
    double e = 8.1819190842622e-2;
    double x = ECEF[0];
    double y = ECEF[1];
    double z = ECEF[2];
    double b = std::sqrt(std::pow(a, 2) * (1 - std::pow(e, 2)));
    double ep = std::sqrt((std::pow(a, 2) - std::pow(b, 2)) / std::pow(b, 2));
    double p = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    double th = std::atan2(a * z, b * p);
    double lon = std::atan2(y, x);
    double lat = std::atan2((z + std::pow(ep, 2) * b * std::pow(std::sin(th), 3)),
                            (p - std::pow(e, 2) * a * std::pow(std::cos(th), 3)));
    double N = a / std::sqrt(1 - std::pow(e, 2) * std::pow(std::sin(lat), 2));
    double alt = p / std::cos(lat) - N;
    lon = std::fmod(lon, 2 * M_PI);
    LLA[0] = lat;
    LLA[1] = lon;
    LLA[2] = alt;
}

void QInverse(double QVector[4], double QVectorInverse[4]) {
    QVectorInverse[0] = QVector[0];
    QVectorInverse[1] = -QVector[1];
    QVectorInverse[2] = -QVector[2];
    QVectorInverse[3] = -QVector[3];
}

void QNormalize(double QVector[4], double QNormVector[4]) {
    double Qzero = QVector[0];
    double Qone = QVector[1];
    double Qtwo = QVector[2];
    double Qthree = QVector[3];
    double QNorm = std::sqrt(Qzero * Qzero + Qone * Qone + Qtwo * Qtwo + Qthree * Qthree);
    if (QNorm > 0.0) {
        QNormVector[0] = Qzero / QNorm;
        QNormVector[1] = Qone / QNorm;
        QNormVector[2] = Qtwo / QNorm;
        QNormVector[3] = Qthree / QNorm;
    } else {
        QNormVector[0] = QVector[0];
        QNormVector[1] = QVector[1];
        QNormVector[2] = QVector[2];
        QNormVector[3] = QVector[3];
    }
}

} // namespace sensor_fusion
