#pragma once

#ifndef PICONTROL_H
#define PICONTROL_H
#define _CRT_SECURE_NO_WARNINGS

// Integral acccumulator

void PIcontroller(double Theta,double &Control, double &integral);

#endif //PICONTROL_H
