#pragma once

namespace sensor_fusion {
void LLA2ECEF(double LLA[3], double ECEF[3]);
void ECEF2LLA(double ECEF[3], double LLA[3]);
void QInverse(double QVector[4], double QVectorInverse[4]);
void QNormalize(double QVector[4], double QNormVector[4]);
}
