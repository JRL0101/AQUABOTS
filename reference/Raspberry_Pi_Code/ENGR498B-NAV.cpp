// ENGR498B-NAV.cpp : This file contains the 'main' function. Program execution begins and ends there.
// Jason Mayhall
// Navigation for Aquabot executable code

// include directive libraries
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cmath>
#include <iomanip>
#include "GlobalRoutineCall.h"
#include "KalmanFilter.h"
#include "KFTYPE.H"
#include "StdAfx.h"
#include "Utility2.h"
//#include "KalmanFilter.cpp"
#define _CRT_SECURE_NO_WARNINGS


// Definitions

#define PI 3.14159265358979323846

using namespace std;

// functions will be declared outside and called inside of main
// Nav strapdown function will be performed in main

int main()
{
    //cout << "Hello World!\n";

    // Test data, Tucson AZ
    double lat = 32.2 * PI / 180;
    double lon = -110.9 * PI / 180;
    double alt = 758;

    double ECEF[3];
    double in[3] = { lat,lon,alt };

    LLA2ECEF(in, ECEF);
    std::cout << "The value x is: " << ECEF[0] << endl;
    std::cout << "The value y is: " << ECEF[1] << endl;
    std::cout << "The value z is: " << ECEF[2] << endl;

    double out[3];

    ECEF2LLA(ECEF,out);
    out[1] = out[1] * 180 / PI;
    std::cout << "The value lat is: " << out[0] << endl;
    std::cout << "The value lon is: " << out[1] << endl;
    std::cout << "The value alt is: " << out[2] << endl;
    // End test of transformation functions

    // Static Definitions

    double m_to_ft = 100 / (2.54 * 12);
    double km_to_fps = 1000 * 100 / (2.54 * 12 * 3600);

     lat = 33.6234878571429 * PI / 180;
    double initial_lat = lat;
    double sl = sin(lat);
    double cl = cos(lat);
     lon = -117.262865257576 * PI / 180;
    double initial_lon = lon;

    // ----------------------

    double omegaE = 7.292115e-5;                                                                                 // earth rate (rad/sec)
    double ae = 2.092564632545932e+007 / m_to_ft;                                                                // radius of earth in ft
    double e2 = 6.6943799014e-003;                                                                               // eccentricity of the earth
    double ga = (32.28768625787402 * (1 - 1.932e-3 * pow(sl,2)) / sqrt(1 - 6.694e-3 * pow(sl,2))) * 1 / m_to_ft; // WGS84 value of gravity
    double gr = ga / ae;                                                                                         // Schuler Effect
    double om = omegaE;

    // ----------------------

    //double R_norm = ae / sqrt(1 - e2 * pow(sl, 2));
    //double R_merid = R_norm * (1 - e2) / (1 - e2 * pow(sl, 2));
    
    // Reading IMU, GPS, and RPM files
    // Note: IMU samples 20 times/per sec and GPS samples once per sec
    // Open the file 'data.txt'from its path for reading (entering the path can be done by simply copying the path from opened txt file in the program)

    ifstream IMU_Data("C:\\Users\\Deser\\Desktop\\ENGR 498B - Spring 2025\\Data for Navigation Tests\\IMU_DATA\\IMU_DATA2.txt");
    ifstream GPS_Data("C:\\Users\\Deser\\Desktop\\ENGR 498B - Spring 2025\\Data for Navigation Tests\\GPS_Bike1.txt");
    //ifstream RPM_Data("C:\\Users\\Deser\\Desktop\\ENGR 498B - Spring 2025\\Data for Navigation Tests\\Bike_rpm1.txt"); 

    if (!IMU_Data || !GPS_Data ) {
        cerr << "Error opening file." << endl;
        return 1;  // Return an error if any of the files couldn't be opened
    }

    string line1,line2,line3;
    // Skip the header line (if necessary)
    //getline(IMU_Data, line1); // This reads and ignores the first line with headers.
    getline(GPS_Data, line2);
    //getline(RPM_Data, line3);

    int IMU_counter = 0;

    // Identity matrix -----------------------------------------------

    double iMat[3][3] = { 0 };
    for (int i = 0; i < 3; ++i) {
        iMat[i][i] = 1.0;  // Set diagonal to 1
    }

    // For IMU at different rate of sampling than the GPS -----------------------------------------------

    double dt = 0.05;
    double sample_time = 0.05;
    double Avg_time = 5 / sample_time; // Time (5 sec) to get initial roll, pitch, and heading
    double nn = Avg_time;
    int IMU_initial_counter = 1; // Counter for 1st 5 secs to get initial roll, pitch, and heading

    double roll = 0;
    double pitch = 0;
    //
    double hdg = 0.;// pi / 2;
    double cor = 2 * omegaE * sin(lat);
    double cor_z = 2 * omegaE * cos(lat);

    // Nav Strapdown Variables -----------------------------------------------

    double roll_off = 0 * PI / 180;
    double pitch_off = 0 * PI / 180;
    double hdg_off = 0 * PI / 180;

    double cr = cos(roll_off);
    double sr = sin(roll_off);
    double cp = cos(pitch_off);
    double sp = sin(pitch_off);
    //
    double ch = cos(hdg_off);
    double sh = sin(hdg_off);

    double cLL_to_body[3][3];
    cLL_to_body[0][0] = cp * ch;
    cLL_to_body[0][1] = cp * sh;
    cLL_to_body[0][2] = -sp;

    cLL_to_body[1][0] = -cr * sh+sr*sp*ch;
    cLL_to_body[1][1] = cr * ch + sr * sp * sh;
    cLL_to_body[1][2] = sr*cp;

    cLL_to_body[2][0] = sr * sh + cr * sp * ch;
    cLL_to_body[2][1] = -sr * sh + cr * sp * ch;
    cLL_to_body[2][2] = cr*cp;

    double Dc_Inst_align[3][3];

    Dc_Inst_align[0][0] = cLL_to_body[0][0];
    Dc_Inst_align[0][1] = cLL_to_body[1][0];
    Dc_Inst_align[0][2] = cLL_to_body[2][0];

    Dc_Inst_align[1][0] = cLL_to_body[0][1];
    Dc_Inst_align[1][1] = cLL_to_body[1][1];
    Dc_Inst_align[1][2] = cLL_to_body[2][1];

    Dc_Inst_align[2][0] = cLL_to_body[0][2];
    Dc_Inst_align[2][1] = cLL_to_body[1][2];
    Dc_Inst_align[2][2] = cLL_to_body[2][2];

    double Vector_DC[3][3];

    Vector_DC[0][0] = Dc_Inst_align[0][0];
    Vector_DC[0][1] = Dc_Inst_align[0][1];
    Vector_DC[0][2] = Dc_Inst_align[0][2];

    Vector_DC[1][0] = Dc_Inst_align[1][0];
    Vector_DC[1][1] = Dc_Inst_align[1][1];
    Vector_DC[1][2] = Dc_Inst_align[1][2];

    Vector_DC[2][0] = Dc_Inst_align[2][0];
    Vector_DC[2][1] = Dc_Inst_align[2][1];
    Vector_DC[2][2] = Dc_Inst_align[2][2];

    double QVectorB[4];

    QVectorB[0] = 0.5 * sqrt(Vector_DC[0][0] + Vector_DC[1][1] + Vector_DC[2][2] + 1);
    QVectorB[1] = (Vector_DC[1][2] - Vector_DC[2][1]) / (4 * QVectorB[0]);
    QVectorB[2] = (Vector_DC[2][0] - Vector_DC[0][2]) / (4 * QVectorB[0]);
    QVectorB[3] = (Vector_DC[0][1] - Vector_DC[1][0]) / (4 * QVectorB[0]);


    // -----------------------------------------------------
    // Step 1: Initialize direction cosine matrix with initial roll, pitch, hdg
    // -----------------------------------------------------

    double QVectorI[4];
    double cbody_to_LL[3][3];
    double Earth_Rate[3];
    double craft_rate[3]; // Stationary Align
    double OmegaLocal[3];
    double QTransportVctr[4];
    double QVector[4];
    double QVctrInverse[4];
    double QOmegaBody[4];
    double QOmegaBody_subtract[4];
    double QOmegaBody_subtract_new[4];
    int first_time = 1;
    //double Velocity_LL[3];
    double Last_Velocity_LL[3];
    //double Position_LL[3];
    double Last_Position_LL[3];
    double last_lat = lat;
    double last_lon = lon;
    double my_posN = 0;
    double my_posE = 0;
    double my_posD = 0;
    double Strap_posN = 0;
    double Strap_posE = 0;
    double Strap_posD = 0;
    double R_norm = ae / (sqrt(1 - e2 * pow(sl, 2)));
    double R_merid = R_norm * (1 - e2) / (1 - e2 * pow(sl, 2));

    double VelPos_Matrix[10];

    Velocity_LL[0] = 0;
    Velocity_LL[1] = 0;
    Velocity_LL[2] = 0;

    Last_Velocity_LL[0] = 0;
    Last_Velocity_LL[1] = 0;
    Last_Velocity_LL[2] = 0;

    Last_Position_LL[0] = 0;
    Last_Position_LL[1] = 0;
    Last_Position_LL[2] = 0;

    Position_LL[0] = 0;
    Position_LL[1] = 0;
    Position_LL[2] = -450;

    my_posN = 0;
    my_posE = 0;
    my_posD = 0;

    VelPos_Matrix[1] = Velocity_LL[0];
    VelPos_Matrix[2] = Velocity_LL[1];
    VelPos_Matrix[3] = Velocity_LL[2];
    VelPos_Matrix[4] = initial_lat;
    VelPos_Matrix[5] = initial_lon;
    VelPos_Matrix[6] = -Position_LL[2];
    VelPos_Matrix[7] = 0;
    VelPos_Matrix[8] = 0;
    VelPos_Matrix[9] = 0;


    int Filt_count = 0;
    int MEASUREMENT = 0;
    int RESET = 0;

    // Kalman correction vectors initialization -----------------------------------------------

    double KgyroBias[3] = { 0 };
    double KaccelBias[3] = { 0 };
    double Velocity_LL_Corrections[3] = { 0 };
    double Position_LL_Corrections[3] = { 0 };
    double KalmanOutput_delta_tiltN = 0;
    double KalmanOutput_delta_tiltE = 0;
    double KalmanOutput_delta_tiltD = 0;

    int start_alt = 1475; // starting altitude is 1475 ft.
    double lever_arm[3] = { 0 }; // lever arm distance to GPS antenna in meters

    // BEGIN THE ALIGNMENT stationary -----------------------------------------------

    int time_to_start_GPS = 69576; // GPS sigma reduced to 8 ft
    int time_bike_moved = 69483; // Zero velocity updates stopped

    //int FilterUpdate = 0;
    FilterUpdate = 0;
    double delta_ABx = 0;
    double delta_ABy = 0;
    double delta_ABz = 0;
    double delta_GBx = 0;
    double delta_GBy = 0;
    double delta_GBz = 0;
    int buffer_count = 0;
    double Udriv_sum[15][15] = { 0 };
    double Fmat_sum[15][15] = { 0 };
    double Fmat_cnt = 0;
    double Fmat_avg[15][15] = { 0 };
    double Udriv_avg[15][15] = { 0 };
    double Filter_dt;
    int roycount = 0;
    int countGPS = 2;
    //int GPS_times_used = 0;

    // Integrating IMU -----------------------------------------------

    double IMU_deltaVel_X;
    double IMU_deltaVel_Y;
    double IMU_deltaVel_Z;

    double IMU_deltaTheta_X;
    double IMU_deltaTheta_Y;
    double IMU_deltaTheta_Z;

    double w_ie_X[3][3];
    double w_en_X[3][3];

    double Qgyros[4];
    double body_rates[3];
    double Qvctr[4];
    double* QOmegaBody_subtractXdt = new double[4];
    double Omega_A;
    double Omega_B;
    double Omega_C;
    double QVector_Rate[4];
    double Q1Q2Vector[4];
    double QNormVector[4];

    double Tolerance = 1.001; // check for unity
    int Q_fail_Flag = 0;
    double Pausing;
    int AFU_MIKIE;

    double NewPhi = 0;
    double NewTheta = 0;
    double NewPsi = 0;

    int counter = 1;

    double AttitudeMatrix[4];
    double accel_body[3];
    double Abody[3];
    double del_PosN;
    double del_PosE;
    double Lat_new;
    double Lon_new;
    double altitude;
    double strap_buffer[4];
    double place = 0;

    // Place holder for coriolis compensation calculation
    double New_Velocity_LL[3] = { 0 };

    // Kalman Filter Variables -----------------------------------------------

    //KalmanInputType KInput;
    //KalmanOutputType KFoutput;
    int FILTER_RUN = 0;

    // Kalman Filter Input & Output Initialization --------------------------------

    // Input ---------------------

    KInput.GPS_lat = 0.;
    KInput.GPS_lon = 0.;
    KInput.GPS_alt = 0.;
    KInput.GPS_time = 0.;
    KInput.GPS_go = 0;

    // Output --------------------

    KFoutput.delta_Pn = 0;
    KFoutput.delta_Pe = 0;
    KFoutput.delta_Pd = 0;
    KFoutput.delta_Vn = 0;
    KFoutput.delta_Vn = 0;
    KFoutput.delta_Vn = 0;

    KFoutput.delta_tiltD = 0;
    KFoutput.delta_tiltE = 0;
    KFoutput.delta_tiltN = 0;

    KFoutput.delta_ABx = 0;
    KFoutput.delta_ABy = 0;
    KFoutput.delta_ABz = 0;

    KFoutput.delta_GBx = 0;
    KFoutput.delta_GBy = 0;
    KFoutput.delta_GBz = 0;
    int NoRun = 0;

    KalmanFilter(KInput, &KFoutput, NoRun); // Initialize the Kalman Filter Startup Routine
    

    // Kalman Filter Update Variables -----------------------------------------------

    double q0, q1, q2, q3;
    double DirCosine_Body_to_Nav_old[3][3];
    double Update_Matrix[3][3];
    double DirCosine_Body_to_Nav_new[3][3];
    int BAD_TILT;

    //double USE_GPS = 0;
    //double PosN_delta, PosE_delta, PosD_delta;
    //PosN_delta = 0; PosE_delta = 0; PosD_delta = 0;

    // F matrix update variables -----------------------------------------------

    double k[9];

    double NED_force[3];
    double f1, f2, f3;

    // Driving noise variables -----------------------------------------------
    
    //double Accel_misalign;
    //double accellSF;
    double Accel_misalign_effects[3];
    double Accel_misalign_velocity[3][3];
    double TempAccel_misalign_velocity[3][3];
    double DiagAccel_misalign_velocity[3][3] = { 0 };
    double Accel_scaling_effect[3];
    double Accel_Scale_Factor_error[3][3];
    double TempAccel_Scale_Factor_error[3][3];
    double DiagAccel_scaling_effect[3][3] = { 0 };
    double Tcbody_to_LL[3][3];
    double IdentXvel_randWalk[3][3] = { 0 };
    double IdentXangle_randWalk[3][3] = { 0 };
    double Velocity_White[3][3];
    double TempVelocity_White[3][3];
    double Gyro_misalign_effects[3];
    double Gyro_misalign_angle[3][3];
    double TempGyro_misalign_angle[3][3];
    double DiagGyro_misalign_angle[3][3] = { 0 };
    double Gyro_scaling_effect[3];
    double Gyro_Scale_Factor_error[3][3];
    double TempGyro_Scale_Factor_error[3][3];
    double DiagGyro_scaling_effect[3][3] = { 0 };
    double Gyro_White[3][3];
    double TempGyro_White[3][3];

    // GPS variables

    int GPS_times_used = 0; // GPS internal counter
    USE_GPS = 0;

    // IMU variables

    double PrevIMU_time = 0;

    // MATH UTILITY VARIABLES -----------------------------------------------

    int In = 0;
    int success_code = 0;

    //MATH_UTILITY_TEST(In, success_code);

    // System Status variables

    int MainLoopCount = 0;
    double Percent_done = 0;
    double amount = 88421; // Only for this situation knowing the amount of data obtained

    // Errors for IMU --------------------------------------

    // Errors for the 3020 IMU -----------------------------------------------------

    double vel_randWalk = 0.2 / 60; // .078 m / s / rt - hr
    double accellSF = 0.003;
    double bias_accel = 6 * 9.8e-3; // six milli - g's
    double Accel_misalign = 5e-4;

    double angle_randWalk = .22 * pi / (180. * 60); // siggn
    double bias_gyro = 20.5 * pi / (180. * 3600);
    double gyro_SF = 0.0005;
    double Gyro_misalign = 5e-4;

    // -----------------------------------------------------

    // create a file to be able to write data to plot for MATLAB comparison

    ofstream outputFile("CPPoutput.txt");  // Open file for writing

    if (!outputFile) {
        cerr << "Error opening file.\n";
        return 1;
    }

    // -------------------------------------------------------------------------------------
    // ------------------            INITIALIZATION      -----------------------------------
    // -------------------------------------------------------------------------------------
    // 
    // Case to initialize with first set of data -------------------------------------------

    while (getline(IMU_Data, line1)) {
        stringstream ss1(line1), ss3(line3); // Convert lines into a stringstream to extract values

        // Declare variables to hold data from each file
        double IMU_time, BodyRate1, BodyRate2, BodyRate3, BodyAccel1, BodyAccel2, BodyAccel3, MagneticBody1, MagneticBody2, MagneticBody3
            , DelTheta_Rad1, DelTheta_Rad2, DelTheta_Rad3, DelTheta_Msec1, DelTheta_Msec2, DelTheta_Msec3, Pressure, IMU_temperature;

        // Read data from each file
        if (ss1 >> IMU_time >> BodyRate1 >> BodyRate2 >> BodyRate3 >> BodyAccel1 >> BodyAccel2 >> BodyAccel3 >>
            MagneticBody1 >> MagneticBody2 >> MagneticBody3 >> DelTheta_Rad1 >> DelTheta_Rad2 >> DelTheta_Rad3 >>
            DelTheta_Msec1 >> DelTheta_Msec2 >> DelTheta_Msec3 >> Pressure >> IMU_temperature) {

            // Process the data here, such as printing or storing -----------------------------------

            // INU Function (loading Bike_run1_startit in MATLAB)

            IMU_temperature = IMU_temperature * 9 / 5 + 32; // converting to farenheit
            IMU_time = IMU_time + 2;
            double IMU_deltaVel[3];
            IMU_deltaVel[0] = DelTheta_Msec2; IMU_deltaVel[1] = DelTheta_Msec1; IMU_deltaVel[2] = -DelTheta_Msec3;
            double IMU_deltaTheta[3];
            IMU_deltaTheta[0] = DelTheta_Rad2; IMU_deltaTheta[1] = DelTheta_Rad1; IMU_deltaTheta[2] = -DelTheta_Rad3;

            // magnetometer
            double h11 = MagneticBody2;
            double h12 = MagneticBody1;
            double h13 = -MagneticBody3;
            // -------------------------

            double X = IMU_deltaVel[0]; double Y = IMU_deltaVel[1]; double Z = IMU_deltaVel[2];
            double acc2 = pow(X, 2) + pow(Y, 2) + pow(Z, 2);
            double acc = sqrt(acc2);

            // magnetometer calculations -------------------------------------------------- (Can comment out for no magnetometer)

            double Hx = (h11 * acc2 - h12 * X * Y - h13 * X * Z) / acc;
            double Hy = (h12 * Z - h13 * Y);


            if (IMU_initial_counter == 1) {
                VelPos_Matrix[0] = IMU_time;
            }

            if (IMU_initial_counter < nn) {
                roll = roll + atan(Y / Z);
                pitch = pitch + asin(X / acc);

                // If there was a magnetometer (heading)
                hdg = hdg + atan2(Hy, Hx) + 12 * pi / 180; // Adjusting for 12 deg East magnetic declination
                place = atan(Y / Z);
            }
            // -----------------------------------------------------
            // // Step 1: Initialize direction cosine matrix with initial roll, pitch, hdg
            // -----------------------------------------------------
            else if (IMU_initial_counter == nn) {

                roll = roll + atan(Y / Z);
                pitch = pitch + asin(X / acc);
                hdg = hdg + atan2(Hy, Hx) + 12 * PI / 180; // Adjusting for 12 deg East magnetic declination
                place = atan(Y / Z);

                roll = roll / nn;
                pitch = pitch / nn;
                //
                hdg = hdg / nn;
                //
                sl = sin(lat);
                cl = cos(lat);
                cr = cos(roll);
                sr = sin(roll);
                cp = cos(pitch);
                sp = sin(pitch);
                //
                ch = cos(hdg);
                sh = sin(hdg);
                //
                cLL2Body(cLL_to_body, sr, cr, cp, sp, ch, sh);
                TransposeMatrix(cbody_to_LL, cLL_to_body);
                findQVectorI(QVectorI, cbody_to_LL);

                Earth_Rate[0] = omegaE * cos(lat);
                Earth_Rate[1] = 0;
                Earth_Rate[2] = -omegaE * sin(lat);

                craft_rate[0] = 0;
                craft_rate[1] = 0;
                craft_rate[2] = 0;

                OmegaLocal[0] = Earth_Rate[0] + craft_rate[0];
                OmegaLocal[1] = Earth_Rate[1] + craft_rate[1];
                OmegaLocal[2] = Earth_Rate[2] + craft_rate[2];

                QTransportVctr[0] = 0;
                QTransportVctr[1] = OmegaLocal[0];
                QTransportVctr[2] = OmegaLocal[1];
                QTransportVctr[3] = OmegaLocal[2];

                QProduct(QVectorB, QVectorI, QVector); // This valueB will be constant for stationary test and another constant for moving
                QInverse(QVector, QVctrInverse);
                QProduct(QVector, QTransportVctr, QOmegaBody);
                QProduct(QOmegaBody, QVctrInverse, QOmegaBody_subtract_new); // Going to hold this value constant for stationary test

                // Storing Data here to look at it
                AttitudeMatrix[0] = IMU_time;
                AttitudeMatrix[1] = roll;
                AttitudeMatrix[2] = pitch;
                AttitudeMatrix[3] = hdg;

               
            } // end of initialization
        }
        // Increment IMU initial counter
        IMU_initial_counter++;
    }
    IMU_Data.close();

    // ---------------------------------------------------------------------------------------
    // -------------------      END OF INITIALIZATION      -----------------------------------
    // ---------------------------------------------------------------------------------------

    //

    // Reopen to use the same data but with the initialized variables with the first 5 seconds of data.

    IMU_Data.open("C:\\Users\\Deser\\Desktop\\ENGR 498B - Spring 2025\\Data for Navigation Tests\\IMU_DATA\\IMU_DATA2.txt");

    if (!IMU_Data) {
        cerr << "Error opening file." << endl;
        return 1;  // Return an error if any of the files couldn't be opened
    }
    

    // -------------------------------------------------------------------------------------

    // Loop through each line in the file
    while (getline(IMU_Data, line1)) {
        stringstream ss1(line1), ss3(line3); // Convert lines into a stringstream to extract values

        // Declare variables to hold data from each file
        double IMU_time, BodyRate1, BodyRate2,BodyRate3,BodyAccel1,BodyAccel2,BodyAccel3,MagneticBody1,MagneticBody2,MagneticBody3
            ,DelTheta_Rad1,DelTheta_Rad2,DelTheta_Rad3,DelTheta_Msec1,DelTheta_Msec2,DelTheta_Msec3,Pressure, IMU_temperature;

        // Read data from each file
        if (ss1 >> IMU_time >> BodyRate1 >> BodyRate2 >> BodyRate3 >> BodyAccel1 >> BodyAccel2 >> BodyAccel3 >>
            MagneticBody1 >> MagneticBody2 >> MagneticBody3 >> DelTheta_Rad1 >>  DelTheta_Rad2 >> DelTheta_Rad3 >>
            DelTheta_Msec1 >> DelTheta_Msec2 >> DelTheta_Msec3 >> Pressure >> IMU_temperature) {

            // Process the data here, such as printing or storing -----------------------------------

            // INU Function (loading Bike_run1_startit in MATLAB)

            IMU_temperature = IMU_temperature * 9 / 5 + 32; // converting to farenheit
            IMU_time = IMU_time + 2;
            double IMU_deltaVel[3];
            IMU_deltaVel[0] = DelTheta_Msec2; IMU_deltaVel[1] = DelTheta_Msec1; IMU_deltaVel[2] = -DelTheta_Msec3;
            double IMU_deltaTheta[3];
            IMU_deltaTheta[0] = DelTheta_Rad2; IMU_deltaTheta[1] = DelTheta_Rad1; IMU_deltaTheta[2] = -DelTheta_Rad3;

            // magnetometer
            double h11 = MagneticBody2;
            double h12 = MagneticBody1;
            double h13 = -MagneticBody3;
            // -------------------------

            double X = IMU_deltaVel[0]; double Y = IMU_deltaVel[1]; double Z = IMU_deltaVel[2];
            double acc2 = pow(X, 2) + pow(Y, 2) + pow(Z, 2);
            double acc = sqrt(acc2);

            // magnetometer calculations -------------------------------------------------- (Can comment out for no magnetometer)

            double Hx = (h11 * acc2 - h12 * X * Y - h13 * X * Z) / acc;
            double Hy = (h12 * Z - h13 * Y);
            
            // Checks to see if the bike moved and or GPS starts to be called in -------------------------------------------------

            if (IMU_time < time_bike_moved) {
                KInput.GPS_lat = initial_lat;
                KInput.GPS_lon = initial_lon;
                KInput.GPS_alt = 450;
            }

            if (IMU_time >= time_to_start_GPS) {

                USE_GPS = 1; // Tells the Kalman Filter that the GPS is being used
                //IMU_counter++; // Will start counter for GPS to be used
                //cout << "IMU counter = " << IMU_counter;
                //cout << "\n";
            }

            //KInput.GPS_go = 0;
            if (USE_GPS > 0.5) {
                // GPS is brought to front of the loop to use any of the GPS data on calculations further down in the loop if called in
                // Read a line of GPS data after every 20 lines from the IMU file -----------------------------------------------------
                if (Fmat_cnt == 19) { // || KInput.GPS_go < 0.5) {
                    // Read a new line from the GPS file
                    // BRING in Nav Aid Data...
                    KInput.GPS_go = 1;
                    //USE_GPS = 1;  // Tells the Kalman Filter that the GPS is being used

                    if (getline(GPS_Data, line2)) {
                        stringstream ss2(line2); // Convert GPS line into a stringstream to extract values

                        double  GPS_alt, GPS_lat, GPS_lon, GPS_time;

                        while(abs(KInput.GPS_time - IMU_time) >.2)
                        {
                        // Read data from GPS file
                            if (ss2 >> GPS_time >> GPS_lat >> GPS_lon >> GPS_alt) {
                                // Adjust GPS time
                                GPS_time = GPS_time * 86400; //pow(86400, -10);
                                // Process Here --------------------------------

                                GPS_lat = GPS_lat * (pi / 180);
                                GPS_lon = GPS_lon * -(pi / 180);

                                vel_randWalk = 0.5 / 60;

                                KInput.GPS_time = GPS_time;
                                KInput.GPS_lat = GPS_lat;
                                KInput.GPS_lon = GPS_lon;
                                KInput.GPS_alt = GPS_alt;

                                //KInput.GPS_go = 1;

                                GPS_times_used++;
                            }

                            else {
                                cerr << "Error parsing GPS data." << endl;
                            }
                        }
                    }
                    else {
                        cerr << "End of GPS file reached." << endl;
                        break;
                    }

                    //IMU_counter = 0;  // Reset IMU line counter after reading a GPS line
                }
                else {

                    //IMU_counter++;

                }
            }
            
            // ------------------------------------------------------------------------------------------------------------------

            if (MainLoopCount > 0) {
                dt = 1e-3 * round((IMU_time - PrevIMU_time) * 1e3);
            }
            else {
                dt = 0.05;
            }

            KInput.time = IMU_time;

            PrevIMU_time = IMU_time; // Stores the time of the previous loop to be used for calculations above in the loop

            if ((MainLoopCount % 2000) < 0.1) {
                Percent_done = round((MainLoopCount / amount * 100)); // Just showing status of routine
                std::cout << "\n";
                std::cout << "Percent done = " << Percent_done;
                std::cout << "\n";
            }

            MEASUREMENT = 0;
                // ------------------------------------------------------------------------------------------------------------------


                // This is where we check to see if the Kalman update has occurred for input to the mechanization....

                if (FilterUpdate > 0.1) {   // Will 1Hz update all our integrals (angles, velocity, and position)

                    FilterUpdate = 0;

                    // NEED initial values or initialize KFoutput and input variables somewhere in the code

                    Position_LL[0] = Position_LL[0] - KFoutput.delta_Pn;
                    Position_LL[1] = Position_LL[1] - KFoutput.delta_Pe;
                    Position_LL[2] = Position_LL[2] - KFoutput.delta_Pd;
                    Velocity_LL[0] = Velocity_LL[0] - KFoutput.delta_Vn;
                    Velocity_LL[1] = Velocity_LL[1] - KFoutput.delta_Ve;
                    Velocity_LL[2] = Velocity_LL[2] - KFoutput.delta_Vd;

                    q0 = QVectorI[0];
                    q1 = QVectorI[1];
                    q2 = QVectorI[2];
                    q3 = QVectorI[3];

                    DirCosine_Body_to_Nav_old[0][0] = (2 * pow(q0, 2) - 1 + 2 * pow(q1, 2));
                    DirCosine_Body_to_Nav_old[1][0] = (2 * q1 * q2 - 2 * q0 * q3);
                    DirCosine_Body_to_Nav_old[2][0] = (2 * q1 * q3 + 2 * q0 * q2);
                    DirCosine_Body_to_Nav_old[0][1] = (2 * q1 * q2 + 2 * q0 * q3);
                    DirCosine_Body_to_Nav_old[1][1] = (2 * pow(q0, 2) - 1 + 2 * pow(q2, 2));
                    DirCosine_Body_to_Nav_old[2][1] = (2 * q2 * q3 - 2 * q0 * q1);
                    DirCosine_Body_to_Nav_old[0][2] = (2 * q1 * q3 - 2 * q0 * q2);
                    DirCosine_Body_to_Nav_old[1][2] = (2 * q2 * q3 + 2 * q0 * q1);
                    DirCosine_Body_to_Nav_old[2][2] = (2 * pow(q0, 2) - 1 + 2 * pow(q3, 2));

                    Update_Matrix[0][0] = 1;
                    Update_Matrix[0][1] = -KFoutput.delta_tiltD;
                    Update_Matrix[0][2] = KFoutput.delta_tiltE;
                    Update_Matrix[1][0] = KFoutput.delta_tiltD;
                    Update_Matrix[1][1] = 1;
                    Update_Matrix[1][2] = -KFoutput.delta_tiltN;
                    Update_Matrix[2][0] = -KFoutput.delta_tiltE;
                    Update_Matrix[2][1] = KFoutput.delta_tiltN;
                    Update_Matrix[2][2] = 1;

                    MatMultiply(Update_Matrix, DirCosine_Body_to_Nav_old, DirCosine_Body_to_Nav_new);

                    if ((DirCosine_Body_to_Nav_new[0][0] + DirCosine_Body_to_Nav_new[1][1] + DirCosine_Body_to_Nav_new[2][2] + 1) > 0) {
                        QVectorI[0] = 0.5 * sqrt((DirCosine_Body_to_Nav_new[0][0] + DirCosine_Body_to_Nav_new[1][1] + DirCosine_Body_to_Nav_new[2][2] + 1));
                        QVectorI[1] = (DirCosine_Body_to_Nav_new[1][2] - DirCosine_Body_to_Nav_new[2][1]) / (4 * QVectorI[0]);
                        QVectorI[2] = (DirCosine_Body_to_Nav_new[2][0] - DirCosine_Body_to_Nav_new[0][2]) / (4 * QVectorI[0]);
                        QVectorI[3] = (DirCosine_Body_to_Nav_new[0][1] - DirCosine_Body_to_Nav_new[1][0]) / (4 * QVectorI[0]);
                    }
                    else {
                        BAD_TILT = 1;
                    }
                    // Normalize quaternion after this update

                    QNormalize(QVectorI, QNormVector);
                    QNormChk(QNormVector, Tolerance, Q_fail_Flag);

                    if (Q_fail_Flag > 0.5) {
                        AFU_MIKIE = Q_fail_Flag;
                        Pausing = IMU_time;
                        std::cout << "Error 1\n";
                        //std::cout << "Press Enter to continue..."; // C++ equivalent to MATLAB's "pause"
                        //std::cin.get(); // waits for Enter key
                        system("pause"); // Only works on Windows not Linux OS
                    }


                    // Now to accumulate bias error estimates

                    delta_ABx = delta_ABx + KFoutput.delta_ABx;
                    delta_ABy = delta_ABy + KFoutput.delta_ABy;
                    delta_ABz = delta_ABz + KFoutput.delta_ABz;

                    delta_GBx = delta_GBx + KFoutput.delta_GBx;
                    delta_GBy = delta_GBy + KFoutput.delta_GBy;
                    delta_GBz = delta_GBz + KFoutput.delta_GBz;

                }



                // Integrating IMU

                IMU_deltaVel_X = IMU_deltaVel[0] - delta_ABx * dt;
                IMU_deltaVel_Y = IMU_deltaVel[1] - delta_ABy * dt;
                IMU_deltaVel_Z = IMU_deltaVel[2] - delta_ABz * dt;

                IMU_deltaTheta_X = IMU_deltaTheta[0] - delta_GBx * dt;
                IMU_deltaTheta_Y = IMU_deltaTheta[1] - delta_GBy * dt;
                IMU_deltaTheta_Z = IMU_deltaTheta[2] - delta_GBz * dt;

                craft_rate[0] = Velocity_LL[1] / (ae - Position_LL[2]);
                craft_rate[1] = -Velocity_LL[0] / (ae - Position_LL[2]);
                craft_rate[2] = Velocity_LL[1] * tan(last_lat) / (ae - Position_LL[2]);

                Earth_Rate[0] = omegaE * cos(lat);
                Earth_Rate[1] = 0;
                Earth_Rate[2] = -omegaE * sin(lat);

                // MatrixA[m][n], m - rows, n - columns

                w_ie_X[0][0] = 0;
                w_ie_X[1][0] = Earth_Rate[2];
                w_ie_X[2][0] = -Earth_Rate[1];

                w_ie_X[0][1] = -Earth_Rate[2];
                w_ie_X[1][1] = 0;
                w_ie_X[2][1] = Earth_Rate[0];

                w_ie_X[0][2] = Earth_Rate[1];
                w_ie_X[1][2] = -Earth_Rate[0];
                w_ie_X[2][2] = 0;

                w_en_X[0][0] = 0;
                w_en_X[1][0] = craft_rate[2];
                w_en_X[2][0] = -craft_rate[1];

                w_en_X[0][1] = -craft_rate[2];
                w_en_X[1][1] = 0;
                w_en_X[2][1] = craft_rate[0];

                w_en_X[0][2] = craft_rate[1];
                w_en_X[1][2] = -craft_rate[0];
                w_en_X[2][2] = 0;

                OmegaLocal[0] = Earth_Rate[0] + craft_rate[0];
                OmegaLocal[1] = Earth_Rate[1] + craft_rate[1];
                OmegaLocal[2] = Earth_Rate[2] + craft_rate[2];

                // Quaternion Integration

                QTransportVctr[0] = 0;
                QTransportVctr[1] = OmegaLocal[0];
                QTransportVctr[2] = OmegaLocal[1];
                QTransportVctr[3] = OmegaLocal[2];

                QProduct(QVectorB, QVectorI, QVector);
                QInverse(QVector, QVctrInverse);
                QProduct(QVector, QTransportVctr, QOmegaBody);
                QProduct(QOmegaBody, QVctrInverse, QOmegaBody_subtract);

                // Integrate angles and compute Rotation Matrix

                Qgyros[0] = 0;
                Qgyros[1] = IMU_deltaTheta_X;
                Qgyros[2] = IMU_deltaTheta_Y;
                Qgyros[3] = IMU_deltaTheta_Z;

                body_rates[0] = IMU_deltaTheta_X * 20;
                body_rates[1] = IMU_deltaTheta_Y * 20;
                body_rates[2] = IMU_deltaTheta_Z * 20;

                QOmegaBody_subtractXdt[0] = QOmegaBody_subtract[0] * dt;
                QOmegaBody_subtractXdt[1] = QOmegaBody_subtract[1] * dt;
                QOmegaBody_subtractXdt[2] = QOmegaBody_subtract[2] * dt;
                QOmegaBody_subtractXdt[3] = QOmegaBody_subtract[3] * dt;

                TotalAngularRate(Qvctr, Qgyros, QOmegaBody_subtractXdt);

                Omega_A = Qvctr[1];
                Omega_B = Qvctr[2];
                Omega_C = Qvctr[3];

                QuaternionAngularRate(Omega_A, Omega_B, Omega_C, QVector_Rate);
                QInverse(QVectorI, QVctrInverse);
                QProduct(QVctrInverse, QVector_Rate, Q1Q2Vector);
                QNormalize(Q1Q2Vector, QNormVector);
                QNormChk(QNormVector, Tolerance, Q_fail_Flag);

                if (Q_fail_Flag > 0.5) {
                    AFU_MIKIE = Q_fail_Flag;
                    Pausing = IMU_time;
                    std::cout << "Error 2\n";
                    //std::cout << "Press Enter to continue..."; // C++ equivalent to MATLAB's "pause"
                    //std::cin.get(); // waits for Enter key
                    system("pause"); // Only works on Windows not Linux OS
                }

                QInverse(QNormVector, QVectorI);
                QProduct(QVectorB, QVectorI, QVector);
                Quaternion2Euler(QVector, NewPhi, NewTheta, NewPsi);

                counter++;
                roll = NewPhi;
                pitch = NewTheta;
                //
                hdg = NewPsi;

                // Storing Data here to look at it
                AttitudeMatrix[0] = IMU_time;
                AttitudeMatrix[1] = roll;
                AttitudeMatrix[2] = pitch;
                AttitudeMatrix[3] = hdg;

                cr = cos(roll);
                sr = sin(roll);
                cp = cos(pitch);
                sp = sin(pitch);
                //
                ch = cos(hdg);
                sh = sin(hdg);
                //
                cLL2Body(cLL_to_body, sr, cr, cp, sp, ch, sh);
                TransposeMatrix(cbody_to_LL, cLL_to_body);   // in this case

                // Integrating Accelerations

                accel_body[0] = IMU_deltaVel_X * 20;
                accel_body[1] = IMU_deltaVel_Y * 20;
                accel_body[2] = IMU_deltaVel_Z * 20;

                // gravity compensation
                Abody[0] = (accel_body[0] + cLL_to_body[0][2] * ga) * dt;
                Abody[1] = (accel_body[1] + cLL_to_body[1][2] * ga) * dt;
                Abody[2] = (accel_body[2] + cLL_to_body[2][2] * ga) * dt;

                Velocity_LL[0] = Velocity_LL[0] + (cbody_to_LL[0][0] * Abody[0] + cbody_to_LL[0][1] * Abody[1] + cbody_to_LL[0][2] * Abody[2]);
                Velocity_LL[1] = Velocity_LL[1] + (cbody_to_LL[1][0] * Abody[0] + cbody_to_LL[1][1] * Abody[1] + cbody_to_LL[1][2] * Abody[2]);
                Velocity_LL[2] = Velocity_LL[2] + (cbody_to_LL[2][0] * Abody[0] + cbody_to_LL[2][1] * Abody[1] + cbody_to_LL[2][2] * Abody[2]);

                // Coriolis compensation

                // Vel[3x1] = [Ident[3x3]-2*[3x3]-[3x3]*Vel[3x1]
                New_Velocity_LL[0] = (iMat[0][0] - 2 * w_ie_X[0][0] - w_en_X[0][0]) * Velocity_LL[0] + (iMat[0][1] - 2 * w_ie_X[0][1] - w_en_X[0][1]) * Velocity_LL[1] + (iMat[0][2] - 2 * w_ie_X[0][2] - w_en_X[0][2]) * Velocity_LL[2];
                New_Velocity_LL[1] = (iMat[1][0] - 2 * w_ie_X[1][0] - w_en_X[1][0]) * Velocity_LL[0] + (iMat[1][1] - 2 * w_ie_X[1][1] - w_en_X[1][1]) * Velocity_LL[1] + (iMat[1][2] - 2 * w_ie_X[1][2] - w_en_X[1][2]) * Velocity_LL[2];
                New_Velocity_LL[2] = (iMat[2][0] - 2 * w_ie_X[2][0] - w_en_X[2][0]) * Velocity_LL[0] + (iMat[2][1] - 2 * w_ie_X[2][1] - w_en_X[2][1]) * Velocity_LL[1] + (iMat[2][2] - 2 * w_ie_X[2][2] - w_en_X[2][2]) * Velocity_LL[2];

                Velocity_LL[0] = New_Velocity_LL[0];
                Velocity_LL[1] = New_Velocity_LL[1];
                Velocity_LL[2] = New_Velocity_LL[2];

                // Trapezoidal integration
                Position_LL[0] = Position_LL[0] + (Last_Velocity_LL[0] + 0.5 * (Velocity_LL[0] - Last_Velocity_LL[0])) * dt;
                Position_LL[1] = Position_LL[1] + (Last_Velocity_LL[1] + 0.5 * (Velocity_LL[1] - Last_Velocity_LL[1])) * dt;
                Position_LL[2] = Position_LL[2] + (Last_Velocity_LL[2] + 0.5 * (Velocity_LL[2] - Last_Velocity_LL[2])) * dt;

                Last_Velocity_LL[0] = Velocity_LL[0];
                Last_Velocity_LL[1] = Velocity_LL[1];
                Last_Velocity_LL[2] = Velocity_LL[2];

                del_PosN = Position_LL[0];
                del_PosE = Position_LL[1];

                Lat_new = lat + del_PosN * 1 / (R_merid - Position_LL[2]);
                Lon_new = lon + del_PosE * 1 / (cl * (R_norm - Position_LL[2]));
                altitude = -Position_LL[2];

                Position_LL[0] = 0;
                Position_LL[1] = 0;

                lat = Lat_new;
                lon = Lon_new;

                sl = sin(lat);
                cl = cos(lat);

                R_norm = ae / sqrt(1 - e2 * pow(sl, 2));
                R_merid = R_norm * (1 - e2) / (1 - e2 * pow(sl, 2));

                altitude = -Position_LL[2];

                ga = (32.28768625787402 * (1 - 1.932e-3 * pow(sl, 2))) / sqrt(1 - 6.694e-3 * pow(sl, 2)) * 1 / m_to_ft;

                buffer_count++;
                strap_buffer[0] = IMU_time;
                strap_buffer[1] = lat;
                strap_buffer[2] = lon;
                strap_buffer[3] = altitude;

                // F matrix update --------------------------------------------------------------------------------

                gr = ga / ae; // Schuler effect

                cor = 2 * omegaE * sl;  // Coriolis effect
                cor_z = 2 * omegaE * cl;

                k[0] = cbody_to_LL[0][0];
                k[1] = cbody_to_LL[0][1];
                k[2] = cbody_to_LL[0][2];
                k[3] = cbody_to_LL[1][0];
                k[4] = cbody_to_LL[1][1];
                k[5] = cbody_to_LL[1][2];
                k[6] = cbody_to_LL[2][0];
                k[7] = cbody_to_LL[2][1];
                k[8] = cbody_to_LL[2][2];

                // Specific Force from IMU

                NED_force[0] = cbody_to_LL[0][0] * accel_body[0] + cbody_to_LL[0][1] * accel_body[1] + cbody_to_LL[0][2] * accel_body[2];
                NED_force[1] = cbody_to_LL[1][0] * accel_body[0] + cbody_to_LL[1][1] * accel_body[1] + cbody_to_LL[1][2] * accel_body[2];
                NED_force[2] = cbody_to_LL[2][0] * accel_body[0] + cbody_to_LL[2][1] * accel_body[1] + cbody_to_LL[2][2] * accel_body[2];

                f1 = NED_force[0];
                f2 = NED_force[1];
                f3 = NED_force[2];

                F[0][3] = 1;
                F[1][4] = 1;
                F[2][5] = 1;
                F[3][0] = -gr; F[3][4] = cor_z; F[3][7] = -f3; F[3][8] = f2;
                F[4][1] = -gr; F[4][3] = -cor_z; F[4][5] = cor; F[4][6] = f3; F[4][8] = -f1;
                F[5][4] = -cor; F[5][6] = -f2; F[5][7] = f1;
                F[6][7] = -om * sl;
                F[7][6] = om * sl; F[7][8] = om * cl;
                F[8][7] = -om * cl;

                int FmatUpdate_count = 0;

                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        F[3 + i][9 + j] = k[FmatUpdate_count];
                        FmatUpdate_count++;
                    }
                }

                FmatUpdate_count = 0;

                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        F[6 + i][12 + j] = -k[FmatUpdate_count];
                        FmatUpdate_count++;
                    }
                }


                // Now to model the errors not included as states as the driving noise

                Accel_misalign_effects[0] = (0 * accel_body[0] - Accel_misalign * accel_body[1] + Accel_misalign * accel_body[2]);
                Accel_misalign_effects[1] = (Accel_misalign * accel_body[0] + 0 * accel_body[1] - Accel_misalign * accel_body[2]);
                Accel_misalign_effects[2] = (-Accel_misalign * accel_body[0] + Accel_misalign * accel_body[1] + 0 * accel_body[2]);

                Accel_misalign_effects[0] = pow(Accel_misalign_effects[0], 2);
                Accel_misalign_effects[1] = pow(Accel_misalign_effects[1], 2);
                Accel_misalign_effects[2] = pow(Accel_misalign_effects[2], 2);

                Accel_scaling_effect[0] = pow((accellSF * accel_body[0]), 2);
                Accel_scaling_effect[1] = pow((accellSF * accel_body[1]), 2);
                Accel_scaling_effect[2] = pow((accellSF * accel_body[2]), 2);

                TransposeMatrix(Tcbody_to_LL, cbody_to_LL);

                for (int i = 0; i < 3; ++i) {
                    DiagAccel_misalign_velocity[i][i] = Accel_misalign_effects[i];  // Set diagonal to Accel_scaling_effect
                }

                MatMultiply(cbody_to_LL, DiagAccel_misalign_velocity, TempAccel_misalign_velocity);
                MatMultiply(TempAccel_misalign_velocity, Tcbody_to_LL, Accel_misalign_velocity);


                for (int i = 0; i < 3; ++i) {
                    DiagAccel_scaling_effect[i][i] = Accel_scaling_effect[i];  // Set diagonal to Accel_scaling_effect
                }

                MatMultiply(cbody_to_LL, DiagAccel_scaling_effect, TempAccel_Scale_Factor_error);
                MatMultiply(TempAccel_Scale_Factor_error, Tcbody_to_LL, Accel_Scale_Factor_error);

                for (int i = 0; i < 3; ++i) {
                    IdentXvel_randWalk[i][i] = pow(vel_randWalk, 2);  // Set diagonal to vel_randWalk^2
                }
                for (int i = 0; i < 3; ++i) {
                    IdentXangle_randWalk[i][i] = pow(angle_randWalk, 2);  // Set diagonal to angle_randWalk^2
                }

                

                //MatMultiply( IdentXvel_randWalk, Tcbody_to_LL, TempVelocity_White);
                //MatMultiply(cbody_to_LL, TempVelocity_White, Velocity_White);

                MatMultiply(cbody_to_LL, IdentXvel_randWalk, TempVelocity_White);
                MatMultiply(TempVelocity_White, Tcbody_to_LL, Velocity_White);

                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        W[3 + i][3 + j] = Accel_misalign_velocity[i][j] + Accel_Scale_Factor_error[i][j] + Velocity_White[i][j];
                    }
                }

                Gyro_misalign_effects[0] = (0 * body_rates[0] - Gyro_misalign * body_rates[1] + Gyro_misalign * body_rates[2]);
                Gyro_misalign_effects[1] = (Gyro_misalign * body_rates[0] + 0 * body_rates[1] - Gyro_misalign * body_rates[2]);
                Gyro_misalign_effects[2] = (-Gyro_misalign * body_rates[0] + Gyro_misalign * body_rates[1] + 0 * body_rates[2]);

                Gyro_misalign_effects[0] = pow(Gyro_misalign_effects[0], 2);
                Gyro_misalign_effects[1] = pow(Gyro_misalign_effects[1], 2);
                Gyro_misalign_effects[2] = pow(Gyro_misalign_effects[2], 2);

                Gyro_scaling_effect[0] = pow((gyro_SF * body_rates[0]), 2);
                Gyro_scaling_effect[1] = pow((gyro_SF * body_rates[1]), 2);
                Gyro_scaling_effect[2] = pow((gyro_SF * body_rates[2]), 2);

                for (int i = 0; i < 3; ++i) {
                    DiagGyro_misalign_angle[i][i] = Gyro_misalign_effects[i];  // Set diagonal to Gyro_misalign_effects
                }

                MatMultiply(cbody_to_LL, DiagGyro_misalign_angle, TempGyro_misalign_angle);
                MatMultiply(TempGyro_misalign_angle, Tcbody_to_LL, Gyro_misalign_angle);

                for (int i = 0; i < 3; ++i) {
                    DiagGyro_scaling_effect[i][i] = Gyro_scaling_effect[i];  // Set diagonal to Gyro_scaling_effect
                }

                MatMultiply(cbody_to_LL, DiagGyro_scaling_effect, TempGyro_Scale_Factor_error);
                MatMultiply(TempGyro_Scale_Factor_error, Tcbody_to_LL, Gyro_Scale_Factor_error);

                MatMultiply(cbody_to_LL, IdentXangle_randWalk, TempGyro_White);
                MatMultiply(TempGyro_White, Tcbody_to_LL, Gyro_White);

                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        W[6 + i][6 + j] = Gyro_misalign_angle[i][j] + Gyro_Scale_Factor_error[i][j] + Gyro_White[i][j];
                    }
                }

                // Summation of matrices

                for (int i = 0; i < 15; i++) {
                    for (int j = 0; j < 15; j++) {
                        Udriv_sum[i][j] = Udriv_sum[i][j] + W[i][j];
                        Fmat_sum[i][j] = Fmat_sum[i][j] + F[i][j];
                    }
                }

                Fmat_cnt++;
                Filter_dt = Fmat_cnt * 0.05;  // 20Hz data

                FILTER_RUN = 0;

                if ( (Fmat_cnt ==20) )
                {
                    for (int i = 0; i < 15; i++) {
                        for (int j = 0; j < 15; j++) {
                            Udriv_avg[i][j] = Udriv_sum[i][j] / Fmat_cnt;
                            // W equals ...
                            W[i][j] = Udriv_avg[i][j];
                            Fmat_avg[i][j] = Fmat_sum[i][j] / Fmat_cnt;
                            // F = ...
                            F[i][j] = Fmat_avg[i][j];
                        }
                    }

                    Fmat_cnt = 0;

                    for (int i = 0; i < 15; i++) {
                        for (int j = 0; j < 15; j++) {
                            Udriv_sum[i][j] = 0 * Udriv_sum[i][j];
                            Fmat_sum[i][j] = 0 * Fmat_sum[i][j];
                        }
                    }

                    // Calculate Transition Matrix and Process Noise Covariance
                    // Go to KalmanFilter.cpp line 378
                    FilterUpdate = 0;

                    if (IMU_time < time_bike_moved) {
                        FILTER_RUN = 1;
                        KInput.mode = 1;
                        FilterUpdate = 1;
                        FilterUpdate = 1;
                    }
                    else if (USE_GPS > 0.5 && KInput.GPS_go > 0.5) {
                        if (abs(KInput.GPS_time - IMU_time) < 0.2)
                        {
                            int beeka = 1;
                        }
                        FILTER_RUN = 1;
                        KInput.mode = 1;
                        FilterUpdate = 1;
                    }
                    else
                        int beeka = 1;

                    if (FILTER_RUN > 0.5) {

                        PosN_delta = (lat - KInput.GPS_lat) * (R_merid - Position_LL[2]);
                        PosE_delta = (lon - KInput.GPS_lon) * (cl * (R_norm - Position_LL[2]));
                        PosD_delta = Position_LL[2] + KInput.GPS_alt;


                        //KalmanFilter(KInput, &KFoutput);

                        //FILTER_RUN = 0;
                        FilterUpdate = 1;
                    }
                    KalmanFilter(KInput, &KFoutput, FILTER_RUN);

                    MEASUREMENT = 0;
                    FILTER_RUN = 0;

                    KFoutput.covPn = sqrt(P[0][0]);
                    KFoutput.covPe = sqrt(P[1][1]);
                    KFoutput.covVn = sqrt(P[3][3]);
                    KFoutput.covVe = sqrt(P[4][4]);
                    KFoutput.covPd = sqrt(P[2][2]);
                    KFoutput.covPsiD = sqrt(P[8][8]);
                }
            

            // Increment IMU counter
            //IMU_counter++;

            // Increment IMU initial counter
            IMU_initial_counter++;

            // Reset Boolean for GPS being used
            //USE_GPS = 0; // Resets the sign of using the GPS for the Kalman filter

            // Counter of main loop
            MainLoopCount++;

            // You can perform any other processing with these variables
            //cout << "End of loop" << endl;

            // Writing data to a file to be able to plot for MATLAB comparison

            outputFile << fixed << setprecision(8)
                << setw(8) << IMU_time << " "
                << setw(8) << NewPhi << " "
                << setw(8) << NewTheta << " "
                << setw(8) << NewPsi << " "
                << setw(8) << Velocity_LL[0] << " "
                << setw(8) << Velocity_LL[1] << " "
                << setw(8) << Velocity_LL[2] << " "
                << setw(8) << Position_LL[0] << " "
                << setw(8) << Position_LL[1] << " "
                << setw(8) << Position_LL[2] << " "
                << setw(8) << lat <<" "
                << setw(8) << lon << " "
                << setw(8) << KInput.GPS_lat << " "
                << setw(8) << KInput.GPS_lon << " "
                << setw(8) << KInput.GPS_alt << "\n";;

            
        }
        else {
            cerr << "Error parsing line from one of the files." << endl;
            break;  // Exit the loop if any line is malformed
        }

        //std::cout << " Iteration " << IMU_initial_counter << "\n";
    } // end of Loop

    // Close the files
    IMU_Data.close();
    GPS_Data.close();
    //RPM_Data.close();
    outputFile.close();
    
    std::cout << "Data written to output.txt\n";
    std::cout << "Press Enter to continue..."; // C++ equivalent to MATLAB's "pause"
    std::cin.get(); // waits for Enter key
    

    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

// Definition of functions

