// GNC_Code.cpp : This file contains the 'main' function. Program execution begins and ends there.
// TEAM: 25030
// Jason Mayhall

// Guidance, Navigation, and Control for Aquabot executable code

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
#include <pigpio.h>
#include <iostream>
#include <cstdlib> 
// Utility ------------
#define SERVO_PIN 13 //change
#define ESC_GPIO 18  //change
#define NR_END 1
#define FREE_ARG char*
using namespace std;
// --------------------

// Kalman Filter ------
#include <memory.h>
//#include <crtdbg.h>
#pragma warning(disable : 4996)
// --------------------

// Guidance -----------
#include "Guidance.h"
// --------------------

// PI Controller ------
#include "PIcontrol.h"
// --------------------
#include "sensor_fusion.h"


#define _CRT_SECURE_NO_WARNINGS


// Definitions

#define PI 3.14159265358979323846

using namespace std;

// functions will be declared outside and called inside of main
// 
// Global Routine functions -------------------------------------------------------------------------------
// 


using namespace std;

// LLA2ECEF function - convert latitude,longitude, and altitude to earth-center, earth-fixed (ECEF) cartesian
// Input:
// LLA2ECEF(input[lat,lon,alt], output[x,y,z])
//
// x = ECEF X-coordinate (m)
// y = ECEF Y-coordinate (m)
// z = ECEF Z-coordinate (m)
// lat = geodetic latitude (radians)
// lon = longtiude (radians)
// alt = height above WGS84 ellipsoid (m)
//
// ASSUMPTION: The model is WGS84, Latitude is customary geodetic (not geocentric), input and output elements are scalars in a vector


// QNormChk function - Quaternion Normalization Check
//
// Input:  QVector[4], Tolerance
// Output: QChkFlag

void QNormChk(double QVector[4], double Tolerance, int& QChkFlag) {

    double epsilon = Tolerance;

    double Qzero = QVector[0];
    double Qone = QVector[1];
    double Qtwo = QVector[2];
    double Qthree = QVector[3];

    double QNorm = sqrt(pow(Qzero, 2) + pow(Qone, 2) + pow(Qtwo, 2) + pow(Qthree, 2));

    if (abs(QNorm) < epsilon) {
        QChkFlag = 0;
    }
    else {
        QChkFlag = 1;
    }
}

// QProduct function - Performs Quaternion Product Operation
//
// Input:  QVector1[4], QVector2[4]
// Output: Q1Q2Vector[4]

void QProduct(double QVector1[4], double QVector2[4], double Q1Q2Vector[4]) {

    double Qzero = QVector1[0] * QVector2[0] - QVector1[1] * QVector2[1] - QVector1[2] * QVector2[2] - QVector1[3] * QVector2[3];
    double Qone = QVector1[0] * QVector2[1] + QVector1[1] * QVector2[0] + QVector1[2] * QVector2[3] - QVector1[3] * QVector2[2];
    double Qtwo = QVector1[0] * QVector2[2] + QVector1[2] * QVector2[0] + QVector1[3] * QVector2[1] - QVector1[1] * QVector2[3];
    double Qthree = QVector1[0] * QVector2[3] + QVector1[3] * QVector2[0] + QVector1[1] * QVector2[2] - QVector1[2] * QVector2[1];

    Q1Q2Vector[0] = Qzero;
    Q1Q2Vector[1] = Qone;
    Q1Q2Vector[2] = Qtwo;
    Q1Q2Vector[3] = Qthree;
}

// Quaternion2Euler function - Form Euler Angles (Roll,Pitch,and Heading (phi-theta-psi) from Quaternion Vector)
//
// Input: QVector[4]
// Output: NewPhi, NewTheta, NewPsi (Roll,Pitch,Hdg)

void Quaternion2Euler(double QVector[4], double& NewPhi, double& NewTheta, double& NewPsi) {

    double Qzero = QVector[0];
    double Qone = QVector[1];
    double Qtwo = QVector[2];
    double Qthree = QVector[3];

    NewPsi = atan2(2 * (Qone * Qtwo - Qzero * Qthree), (2 * pow(Qzero, 2) + 2 * pow(Qone, 2) - 1));
    NewTheta = asin(-2 * (Qone * Qthree + Qzero * Qtwo));
    NewPhi = atan2(-2 * (Qzero * Qone - Qtwo * Qthree), (2 * pow(Qzero, 2) + 2 * pow(Qthree, 2) - 1));
}

// QuaternionAngularRate function - Form Quaternion Propagation Solution from Angular Rates
//
// Input:  OmegaA, OmegaB, OmegaC - gyro (body x-y-z) rates
// Output: QVector_Rate[4]

void QuaternionAngularRate(double OmegaA, double OmegaB, double OmegaC, double QVector_Rate[4]) {

    double OmegaMagnitude = sqrt(pow(OmegaA, 2) + pow(OmegaB, 2) + pow(OmegaC, 2));
    double Qzero_Rate = cos(OmegaMagnitude / 2.0);
    double AngleRatio;

    if (OmegaMagnitude > 0.00) {
        AngleRatio = sin(OmegaMagnitude / 2.0) / OmegaMagnitude;
    }
    else {
        AngleRatio = 0;
    }

    double Qone_Rate = AngleRatio * OmegaA;
    double Qtwo_Rate = AngleRatio * OmegaB;
    double Qthree_Rate = AngleRatio * OmegaC;

    QVector_Rate[0] = Qzero_Rate;
    QVector_Rate[1] = Qone_Rate;
    QVector_Rate[2] = Qtwo_Rate;
    QVector_Rate[3] = Qthree_Rate;

}

// TotalAngularRate function

// input: Qvctr1, Qvctr2
// output: Qvctr

void TotalAngularRate(double Qvctr[4], double Qvctr1[4], double Qvctr2[4]) {
    Qvctr[0] = Qvctr1[0] - Qvctr2[0];
    Qvctr[1] = Qvctr1[1] - Qvctr2[1];
    Qvctr[2] = Qvctr1[2] - Qvctr2[2];
    Qvctr[3] = Qvctr1[3] - Qvctr2[3];
}

// -----------------------------------------------------------------------------

// Matrix functions

void cLL2Body(double cLL_to_body[3][3], double sr, double cr, double cp, double sp, double ch, double sh) {
    cLL_to_body[0][0] = cp * ch;
    cLL_to_body[0][1] = cp * sh;
    cLL_to_body[0][2] = -sp;

    cLL_to_body[1][0] = -cr * sh + sr * sp * ch;
    cLL_to_body[1][1] = cr * ch + sr * sp * sh;
    cLL_to_body[1][2] = sr * cp;

    cLL_to_body[2][0] = sr * sh + cr * sp * ch;
    cLL_to_body[2][1] = -sr * ch + cr * sp * sh;
    cLL_to_body[2][2] = cr * cp;
}

void TransposeMatrix(double cbody_to_LL[3][3], double cLL_to_body[3][3]) {
    cbody_to_LL[0][0] = cLL_to_body[0][0];
    cbody_to_LL[0][1] = cLL_to_body[1][0];
    cbody_to_LL[0][2] = cLL_to_body[2][0];

    cbody_to_LL[1][0] = cLL_to_body[0][1];
    cbody_to_LL[1][1] = cLL_to_body[1][1];
    cbody_to_LL[1][2] = cLL_to_body[2][1];

    cbody_to_LL[2][0] = cLL_to_body[0][2];
    cbody_to_LL[2][1] = cLL_to_body[1][2];
    cbody_to_LL[2][2] = cLL_to_body[2][2];
}

void findQVectorI(double QVectorI[4], double Vector_DC[3][3]) {
    QVectorI[0] = 0.5 * sqrt(Vector_DC[0][0] + Vector_DC[1][1] + Vector_DC[2][2] + 1);
    QVectorI[1] = (Vector_DC[1][2] - Vector_DC[2][1]) / (4 * QVectorI[0]);
    QVectorI[2] = (Vector_DC[2][0] - Vector_DC[0][2]) / (4 * QVectorI[0]);
    QVectorI[3] = (Vector_DC[0][1] - Vector_DC[1][0]) / (4 * QVectorI[0]);
}

void MatMultiply(double MatA[3][3], double MatB[3][3], double MatC[3][3])
// Function:  MatMultiply(A, B, C, l, m, n)
// Matrix multiply:  C = A * B,
// where A is 3x3, B is 3x3, and C is 3x3.

{
    int i, j, k;
    int l = 3; int m = 3; int n = 3;
    double sum;

    for (i = 0; i < l; i++) {
        for (j = 0; j < n; j++) {
            sum = 0.0;
            for (k = 0; k < m; k++) sum += MatA[i][k] * MatB[k][j];
            MatC[i][j] = sum;
        }
    }
}
// End of Global Routine functions ----------------------------------------------

// Utility functions ------------------------------------------------------------

//    Utility.c
// All of the utility functions in this file make use of a 'pointer
// to an array of pointers to rows' of matrices, as advocated in
// "Numerical Recipes in C" on pages 20-23.


// Utility Functions:
//------------------------------------------------------------------
void dmab(double** A, double** B, double** C, int l, int m, int n)
// Function:  dmab(A, B, C, l, m, n)
// Matrix multiply:  C = A * B,
// where A is l-by-m, B is m-by-n, and C is l-by-n.

{
    int i, j, k;
    double sum;

    for (i = 0; i < l; i++) {
        for (j = 0; j < n; j++) {
            sum = 0.0;
            for (k = 0; k < m; k++) sum += A[i][k] * B[k][j];
            C[i][j] = sum;
        }
    }
}
//----------------- End of dmab ------------------------------------

void dmabt(double** A, double** B, double** C, int l, int m, int n)
// Function:  dmabt(A, B, C, l, m, n)
// Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
// where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.
{
    int i, j, k;
    double sum;

    for (i = 0; i < l; i++) {
        for (j = 0; j < n; j++) {
            sum = 0.0;
            for (k = 0; k < m; k++)  sum += A[i][k] * B[j][k];
            C[i][j] = sum;
        }
    }
}
//----------------- End of dmabt ------------------------------------

void dmapb(double** A, double** B, double** C, int m, int n)
// Function:  dmapb(A, b, C, m, n)
// Matrix add (plus):  C= A + B,
// where arrays A, b, and C are m-by-n.
{
    int i, j;

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++)  C[i][j] = A[i][j] + B[i][j];
    }
}
//----------------- End of dmapb ------------------------------------

void dmamb(double** A, double** B, double** C, int m, int n)
// Function:  dmamb(A, B, C, m, n)
// Matrix difference (minus):  C = A - B,
// where arrays A, B, and C are m-by-n.
{
    int i, j;

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++)  C[i][j] = A[i][j] - B[i][j];
    }
}
//----------------- End of dmamb ------------------------------------

void dmscal(double** A, double** B, int m, int n, double c)
// Function:  dmscal(A, B, m, n, c)
// Multiply matrix A by scalar double c:  B = A * c
// where arrays A and B are m-by-n.
{
    int row, col;

    for (row = 0; row < m; row++)
    {
        for (col = 0; col < n; col++)
            B[row][col] = A[row][col] * c;
    }
}
//----------------- End of dmscal -----------------------------------

void diagonal(double** A, double* B, int m)
// Function:  diagonal(A, B, m)
// Place diagonal of square matrix A into vector B
// where array A is m-by-m and array B is m-by-1
{
    int i;

    for (i = 0; i < m; i++)
        B[i] = A[i][i];
}
//----------------- End of diagonal ---------------------------------

void dmcopy(double** A, double** B, int m, int n)
// Function:  dmcopy(A, B, m, n)
// Copy m-by-n matrix A into m-by-n matrix B.
{
    int i, j;

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++)  B[i][j] = A[i][j];
    }
}
//----------------- End of dmcopy ------------------------------------

void vezero(double* A, int m)
// Function:  vezero(A, m)
// Fill m-by-1 vector A with zeroes.
{
    int i;

    for (i = 0; i < m; i++)
        A[i] = 0;
}
//----------------- End of vezero ------------------------------------

void iezero(int* indx, int m)
// Function:  iezero(A, m)
// Fill m-by-1 vector A with zeroes.
{
    int i;

    for (i = 0; i < m; i++)
        indx[i] = 0;
}
//----------------- End of iezero ------------------------------------

void dmzero(double** A, int m, int n)
// Function:  dmzero(A, m, n)
// Fill m-by-n matrix A with zeroes.
{
    int i, j;

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++)  A[i][j] = 0.0;
    }
}
//----------------- End of dmzero ------------------------------------

void dmeye(double** A, int m, int n)
// Function:  dmeye(A, m, n)
// Set m-by-n matrix A to zeros except for ones on the main diagonal.
// (When m = n, A becomes the identity matrix.)
{
    int i, j;

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            if (i == j) A[i][i] = 1.0;
            else A[i][j] = 0.0;
        }
    }
}
//----------------- End of dmeye -------------------------------------

void dmav(double** A, double* v, double* u, int m, int n)
// Function:  dmav(A, v, u, m, n)
// Matrix-vector multiply:  u = A * v,
// where A is m-by-n matrix, v is n-by-1 vector, and u is m-by-1 vector.

{
    int i, k;
    double sum;

    for (i = 0; i < m; i++) {
        sum = 0.0;
        for (k = 0; k < n; k++)  sum += A[i][k] * v[k];
        u[i] = sum;
    }
}
//----------------- End of dmav ------------------------------------

void vecsub(double* A, double* B, double* C, int m)
// Function:  vecsub(A, B, C, m)
// Vector minus (subtract):  C = A - B,
// where arrays A, B, and C are m-by-1.

{
    int i;

    for (i = 0; i < m; i++)
        C[i] = A[i] - B[i];
}
//----------------- End of vecsub ----------------------------------

void vecdiv(double* A, double* B, double* C, int m)
// Function:  vecdiv(A, B, C, m)
// Vector divide:  C = A / B,
// where arrays A, B, and C are m-by-1.

{
    int i;

    for (i = 0; i < m; i++)
        C[i] = A[i] / B[i];
}
//----------------- End of vecdiv ----------------------------------

int dmaib(double** A, double** B, double** X, double** Temp, double* c, double* dum_vv, int* indx, int m, int n)
// Function:  dmaib(A, B, X, Temp, temp_vect, temp_vect, temp_ivect, m, n)
// Given matrices A and B, evaluate X = A\B without doing an explicit
// inversion of the A matrix.  (I.e., it is more efficient than doing
// a matrix inversion followed by a matrix multiply.  However, if the
// inverse of A is required, then set B equal to the identity matrix.)
//   A is m-by-m
//   B is m-by-n
//   X is m-by-n
// The temporary (scratch) real vectors and the one integer vector are of length m.
// If the A matrix is not singular the return value is 0; otherwise it is 1
// and the "solution" for x is not valid.

{
    int i, j, invalid;  //*indx
    double d;  //*c, **Temp;  // Temp[m][m]; // Removed on 9-19-05

    //indx = ivector(0, m-1); // Removed on 9-19-05
    //c = dvector(0, m-1);	  // Removed on 9-19-05

    // Work on copy of matrix A:
    //Temp = dmatrix(0, m-1, 0, m-1); // Removed on 9-19-05
    dmcopy(A, Temp, m, m);

    invalid = ludcmp(Temp, dum_vv, m, indx, &d); // Changed on 9-19-05

    if (!invalid) {
        for (i = 0; i < n; i++) {
            for (j = 0; j < m; j++) {
                c[j] = B[j][i];
            }
            lubksb(Temp, m, indx, c);
            for (j = 0; j < m; j++) {
                X[j][i] = c[j];
            }
        }
    }

    /*    free_ivector(indx, 0, m-1);
        free_dvector(c, 0, m-1);
        free_dmatrix(Temp, 0,m-1,0,m-1); */ // Removed on 9-19-05
    return invalid;
}

//----------------- end of dmaib -----------------------------------------

// Below are functions from the 'nrutil.c' file of "Numerical Recipes in C".
// Some modifications have been made.  "float"s have been replaced by
// "double"s, and "long"s by "int"s (since very large integer values are
// not anticipated).  See other changes in 'ludcmp' and 'lubksb'.

int ludcmp(double** a, double* vv, int n, int* indx, double* d)
// Function:  ludcmp(a, vv, n, indx, &d)
// indx[n]
// a[n][n]

// Given a matrix a[n][n], this routine replaces it by the LU
// decomposition of a rowwise permutation of itself.  a and n are input.
// a is output, arranged with elements of L below the diagonal and U
// in the upper triangle.  indx[n] is an output vector that records
// the row permutation effected by the partial pivoting. Variable d
// keeps track of row interchanges, which is needed if the determinant
// of matrix a is to be determined.

// Warning: The "a" matrix is overlaid with the "LU" form.

// The C code for this function comes from "Numerical Recipes", second
// edition, but modified for indexing from 0 to n-1 instead of 1 to n.

// If the matrix is not singular, the return value is 0; otherwise it is 1.

{
    int i, imax, j, k;
    double big, dum, sum, temp;

    // Allocate n-element double-precision vector v for temp. storage:
    //double *vv;  //Removed 9-19-05
    //vv=dvector(0, n-1); //Removed 9-19-05

    *d = 1.0;
    for (i = 0; i < n; i++) {
        big = 0.0;
        for (j = 0; j < n; j++)
            if ((temp = fabs(a[i][j])) > big) big = temp;
        if (big == 0.0) {
            //printf("Singular matrix in the ludcmp function.\n");
            return 1;
        }
        vv[i] = 1.0 / big;
    }
    for (j = 0; j < n; j++) {
        for (i = 0; i < j; i++) {
            sum = a[i][j];
            for (k = 0; k < i; k++) sum -= a[i][k] * a[k][j];
            a[i][j] = sum;
        }
        big = 0.0;
        for (i = j; i < n; i++) {
            sum = a[i][j];
            for (k = 0; k < j; k++)
                sum -= a[i][k] * a[k][j];

            a[i][j] = sum;
            if ((dum = vv[i] * fabs(sum)) >= big) {
                big = dum;
                imax = i;
            }
        }
        if (j != imax) {
            for (k = 0; k < n; k++) {
                dum = a[imax][k];
                a[imax][k] = a[j][k];
                a[j][k] = dum;
            }
            *d = -(*d);
            vv[imax] = vv[j];
        }
        indx[j] = imax;
        if (a[j][j] == 0.0) a[j][j] = 1.0e-20;
        if (j != n - 1) {
            dum = 1.0 / (a[j][j]);
            for (i = j + 1; i < n; i++) a[i][j] *= dum;
        }
    }
    //free_dvector(vv, 0, n-1); //Removed 9-19-05
    return 0;
}

//----------------- end of ludcmp ----------------------------------------

void lubksb(double** a, int n, int* indx, double* b)
// Function:  lubksb(a, n, indx, b)
// indx[n]
// a[n][n], b[n]

// Solves the set of n linear equations A*x = b.  Here input 'a' is not
// the matrix A, but the LU decomposition of A determined by function
// ludcmp.  'b' is input as the right-hand-side vector and returned as
// the solution vector x.  Only 'b' is modified by this routine.

// The C code for this function comes from "Numerical Recipes", second
// edition, but modified for indexing from 0 to n-1 instead of 1 to n.

{
    int i, ii = -1, ip, j;
    double sum;

    for (i = 0; i < n; i++) {
        ip = indx[i];
        sum = b[ip];
        b[ip] = b[i];
        if (ii >= 0)
            for (j = ii; j <= i; j++) sum -= a[i][j] * b[j];
        else if (sum) ii = i;
        b[i] = sum;
    }
    for (i = n - 1; i >= 0; i--) {
        sum = b[i];
        for (j = i + 1; j < n; j++) sum -= a[i][j] * b[j];
        b[i] = sum / a[i][i];
    }
}
//----------------- end of lubksb ---------------------------------

int* ivector(int nl, int nh)

/* Allocate an int vector with subscript range v[nl..nh] */
{
    int* v;

    v = (int*)malloc((size_t)((nh - nl + 1 + NR_END) * sizeof(int)));
    if (!v) {
        printf("Allocation failure in ivector()\n");
        exit(1);
    }
    return v - nl + NR_END;
}
//----------------- End of ivector ---------------------------------

double* dvector(int nl, int nh)
/* Allocate a double vector with subscript range v[nl..nh] */
{
    double* v;

    v = (double*)malloc((size_t)((nh - nl + 1 + NR_END) * sizeof(double)));
    if (!v) {
        printf("Allocation failure in dvector()\n");
        exit(1);
    }
    return v - nl + NR_END;
}
//----------------- End of dvector ---------------------------------

double** dmatrix(int nrl, int nrh, int ncl, int nch)
/* Allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
    int i, nrow = nrh - nrl + 1, ncol = nch - ncl + 1;
    double** m;

    /* allocate pointers to rows */
    m = (double**)malloc((size_t)((nrow + NR_END) * sizeof(double*)));
    if (!m) {
        printf("Allocation failure 1 in dmatrix()");
        exit(1);
    }
    m += NR_END;
    m -= nrl;

    /* allocate rows and set pointers to them */
    m[nrl] = (double*)malloc((size_t)((nrow * ncol + NR_END) * sizeof(double)));
    if (!m[nrl]) {
        printf("Allocation failure 2 in dmatrix()\n");
        exit(1);
    }
    m[nrl] += NR_END;
    m[nrl] -= ncl;

    for (i = nrl + 1; i <= nrh; i++) m[i] = m[i - 1] + ncol;

    /* return pointer to array of pointers to rows */
    return m;
}
//------------- End of dmatrix ----------------------------------------------

double** convert_dmatrix(double* a, int nrl, int nrh, int ncl, int nch)
/* Allocate a double matrix m[nrl..nrh][ncl..nch] that points to the matrix
declared in the standard C manner as a[nrow][ncol], where nrow=nrh-nrl+1
and ncol=nch-ncl+1. The routine should be called with the address
&a[0][0] as the first argument. */

// This function does NOT allocate new memory for the data. It just sets up
// new pointers to the data.
{
    int i, j, nrow = nrh - nrl + 1, ncol = nch - ncl + 1;

    double** m;

    /* allocate pointers to rows */
    m = (double**)malloc((size_t)((nrow + NR_END) * sizeof(double*)));
    if (!m) {
        printf("Allocation failure in convert_matrix().\n");
        exit(1);
    }
    m += NR_END;
    m -= nrl;

    /* set pointers to rows */
    m[nrl] = a - ncl;
    for (i = 1, j = nrl + 1; i < nrow; i++, j++) m[j] = m[j - 1] + ncol;
    /* return pointer to array of pointers to rows */
    return m;
}
//-------------- end of convert_dmatrix ------------------------------------

void free_ivector(int* v, int nl)
/* Free an int vector allocated with ivector() */
{
    free((FREE_ARG)(v + nl - NR_END));
}
//------------- End of free_ivector -----------------------------------------

void free_dvector(double* v, int nl)
/* Free a double vector allocated with dvector() */
{
    free((FREE_ARG)(v + nl - NR_END));
}
//------------- End of free_dvector -----------------------------------------

void free_dmatrix(double** m, int nrl, int ncl)
//void free_dmatrix(double **m, int nrl, int nrh, int ncl, int nch)
/* Free a double matrix allocated by dmatrix() */
{
    free((FREE_ARG)(m[nrl] + ncl - NR_END));
    free((FREE_ARG)(m + nrl - NR_END));
}
//------------- End of free_dmatrix -----------------------------------------   

void free_convert_dmatrix(double** b, int nrl)
/* Free a matrix allocated by convert_dmatrix().  It does NOT affect the
   original matrix that was input to convert_dmatrix. */
{
    free((FREE_ARG)(b + nrl - NR_END));
}
//-------------- End of free_convert_matrix ---------------------------------

void MATH_UTILITY_TEST(int in, int success_code)
/* This will run through each of the routines in this class; the success_code will
   be returned to the calling routine as '1' for completion or will be the error code
   from the utilities below  */
{
    double** Temp1, ** Temp2, ** Temp3, ** Temp4, ** Temp5, ** _X,
        * rvect0, * rvect1, * rvect2, * rvect3, c, X[3][3];
    c = 3.141;
    int* ivect1, * ivect2, m, n, j, i, k;
    m = 3;
    n = 3;
    j = 1;

    Temp1 = dmatrix(0, 2, 0, 2); // 3 x 3 matrix to be set to [1 2 3;1 8 27;4 5 6]
    Temp1[0][0] = 1.0;
    Temp1[0][1] = 2.0;
    Temp1[0][2] = 3.0;
    Temp1[1][0] = 1.0;
    Temp1[1][1] = 8.0;
    Temp1[1][2] = 27.0;
    Temp1[2][0] = 4.0;
    Temp1[2][1] = 5.0;
    Temp1[2][2] = 6.0;

    for (i = 0; i < 3; i++)
    {
        for (k = 0; k < 3; k++)
        {
            X[i][k] = Temp1[i][k];
        }
    }


    Temp2 = dmatrix(0, 2, 0, 2); // 3 x 3 matrix to be set to expm(Temp*.2)
    Temp2[0][0] = 3.2;
    Temp2[0][1] = 4.4;
    Temp2[0][2] = 9.1;
    Temp2[1][0] = 12.0;
    Temp2[1][1] = 25.5;
    Temp2[1][2] = 52.9;
    Temp2[2][0] = 5.5;
    Temp2[2][1] = 10.5;
    Temp2[2][2] = 22.5;

    Temp3 = dmatrix(0, 2, 0, 2); // 3 x 3 matrix to be uninitialized
    Temp4 = dmatrix(0, 2, 0, 2); // 3 x 3 matrix to be uninitialized
    Temp5 = dmatrix(0, 2, 0, 2); // 3 x 3 matrix to be uninitialized

    _X = convert_dmatrix(&X[0][0], 0, 2, 0, 2);

    ivect1 = ivector(0, 2); // 3 x 1 integer vector 
    ivect1[0] = 11;
    ivect1[1] = 19;
    ivect1[2] = 47;

    ivect2 = ivector(0, 2); // 3 x 1 integer vector 

    rvect0 = dvector(0, 2);	// 3 x 1 real vector
    rvect0[0] = 1.1;
    rvect0[1] = 2.2;
    rvect0[2] = 3.3;

    rvect1 = dvector(0, 2);	// 3 x 1 real vector
    rvect1[0] = 0.1179;
    rvect1[1] = 1.9245;
    rvect1[2] = -2.3047;

    rvect2 = dvector(0, 2);	// 3 x 1 real vector
    rvect3 = dvector(0, 2);	// 3 x 1 real vector

    // Now to start the tests
    cout << "    " << endl;
    cout << "Reference Matrix-1    Row-1: " << Temp1[0][0] << " " << Temp1[0][1] << " " << Temp1[0][2] << endl;
    cout << "                      Row-2: " << Temp1[1][0] << " " << Temp1[1][1] << " " << Temp1[1][2] << endl;
    cout << "                      Row-3: " << Temp1[2][0] << " " << Temp1[2][1] << " " << Temp1[2][2] << endl;
    cout << "Reference Matrix-2    Row-1: " << Temp2[0][0] << " " << Temp2[0][1] << " " << Temp2[0][2] << endl;
    cout << "                      Row-2: " << Temp2[1][0] << " " << Temp2[1][1] << " " << Temp2[1][2] << endl;
    cout << "                      Row-3: " << Temp2[2][0] << " " << Temp2[2][1] << " " << Temp2[2][2] << endl;
    cout << "Reference Vector-1    Row-1: " << rvect0[0] << endl;
    cout << "                      Row-2: " << rvect0[1] << endl;
    cout << "                      Row-3: " << rvect0[2] << endl;
    cout << "Reference Vector-2    Row-1: " << rvect1[0] << endl;
    cout << "                      Row-2: " << rvect1[1] << endl;
    cout << "                      Row-3: " << rvect1[2] << endl;
    cout << "Reference Vector-3    Row-1: " << ivect1[0] << endl;
    cout << "(an integer vector)   Row-2: " << ivect1[1] << endl;
    cout << "                      Row-3: " << ivect1[2] << endl;
    cout << "    " << endl;

    cout << "    " << endl;
    cout << "Another copy of Ref   Row-1: " << X[0][0] << " " << X[0][1] << " " << X[0][2] << endl;
    cout << "Matrix-1 using the    Row-2: " << X[1][0] << " " << X[1][1] << " " << X[1][2] << endl;
    cout << "convert_dmatrix       Row-3: " << X[2][0] << " " << X[2][1] << " " << X[2][2] << endl;
    cout << "    " << endl;

    dmzero(Temp3, m, n);
    cout << "Math Test: dmzero -> zeros the matrix,   Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] << endl;
    cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] << endl;
    cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] << endl;
    cout << "    " << endl;

    dmeye(Temp3, m, n);
    cout << "Math Test: dmeye -> the Identity matrix, Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] << endl;
    cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] << endl;
    cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] << endl;
    cout << "    " << endl;

    dmscal(Temp3, Temp4, m, n, c);
    cout << "Math Test: dmscal -> Matrix x Scalar,    Row-1: " << Temp4[0][0] << " " << Temp4[0][1] << " " << Temp4[0][2] << endl;
    cout << "           the matrix was the identity,  Row-2: " << Temp4[1][0] << " " << Temp4[1][1] << " " << Temp4[1][2] << endl;
    cout << "           and the scalr is pi.          Row-3: " << Temp4[2][0] << " " << Temp4[2][1] << " " << Temp4[2][2] << endl;
    cout << "    " << endl;

    dmapb(Temp1, Temp2, Temp3, m, n);
    cout << "Math Test: dmapb -> Ref-1 + Ref-2,       Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] << endl;
    cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] << endl;
    cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] << endl;
    cout << "    " << endl;

    dmab(Temp1, Temp2, Temp3, m, m, n);
    cout << "Math Test: dmap -> Ref-1 * Ref-2,        Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] << endl;
    cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] << endl;
    cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] << endl;
    cout << "    " << endl;

    dmabt(Temp1, Temp2, Temp3, m, m, n);
    cout << "Math Test: dmapt -> Ref-1 * Ref-2^T,     Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] << endl;
    cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] << endl;
    cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] << endl;
    cout << "    " << endl;

    dmcopy(Temp1, Temp3, m, n);
    cout << "Math Test: dmcopy -> copy of Ref-1,      Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] << endl;
    cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] << endl;
    cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] << endl;
    cout << "    " << endl;

    dmeye(Temp3, m, n); // also tests ludcmp and lubksb
    j = dmaib(Temp1, Temp3, Temp4, Temp5, rvect2, rvect3, ivect2, m, n);
    cout << "Math Test: dmaib -> Matrix Inverse,      Row-1: " << Temp4[0][0] << " " << Temp4[0][1] << " " << Temp4[0][2] << endl;
    cout << "           the matrix was Reference-1.   Row-2: " << Temp4[1][0] << " " << Temp4[1][1] << " " << Temp4[1][2] << endl;
    cout << "                                         Row-3: " << Temp4[2][0] << " " << Temp4[2][1] << " " << Temp4[2][2] << endl;
    cout << "    " << endl;

    dmamb(Temp1, Temp2, Temp3, m, n);
    cout << "Math Test: dmamb -> Ref-1 - Ref-2,       Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] << endl;
    cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] << endl;
    cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] << endl;
    cout << "    " << endl;

    diagonal(Temp1, rvect2, m);
    cout << "Math Test: diagonal ->  of Ref-1 to a vector Row-1: " << rvect2[0] << endl;
    cout << "                                             Row-2: " << rvect2[1] << endl;
    cout << "                                             Row-3: " << rvect2[2] << endl;
    cout << "    " << endl;

    dmav(Temp2, rvect1, rvect2, m, n);
    cout << "Math Test: dmav -> Matrix-2 * vector-2,      Row-1: " << rvect2[0] << endl;
    cout << "                                             Row-2: " << rvect2[1] << endl;
    cout << "                                             Row-3: " << rvect2[2] << endl;
    cout << "    " << endl;

    vecsub(rvect0, rvect1, rvect2, m);
    cout << "Math Test: vecsub -> Vector-1 - Vector-2,    Row-1: " << rvect2[0] << endl;
    cout << "                                             Row-2: " << rvect2[1] << endl;
    cout << "                                             Row-3: " << rvect2[2] << endl;
    cout << "    " << endl;

    vecdiv(rvect0, rvect1, rvect2, m);
    cout << "Math Test: vecdiv -> Vector-1 / Vector-2,    Row-1: " << rvect2[0] << endl;
    cout << "                                             Row-2: " << rvect2[1] << endl;
    cout << "                                             Row-3: " << rvect2[2] << endl;
    cout << "    " << endl;

    vezero(rvect2, m);
    cout << "Math Test: vezero -> Above vector zeroed,    Row-1: " << rvect2[0] << endl;
    cout << "                                             Row-2: " << rvect2[1] << endl;
    cout << "                                             Row-3: " << rvect2[2] << endl;
    cout << "    " << endl;

    iezero(ivect1, m);
    cout << "Math Test: iezero -> Vector-3 vector zeroed, Row-1: " << ivect1[0] << endl;
    cout << "                                             Row-2: " << ivect1[1] << endl;
    cout << "                                             Row-3: " << ivect1[2] << endl;
    cout << "    " << endl;

    // Now free up space allocated for this test
    free_ivector(ivect1, 0);

    free_dvector(rvect0, 0);
    free_dvector(rvect1, 0);
    free_dvector(rvect2, 0);

    free_dmatrix(Temp1, 0, 0);
    free_dmatrix(Temp2, 0, 0);
    free_dmatrix(Temp3, 0, 0);
    free_dmatrix(Temp4, 0, 0);
    free_dmatrix(Temp5, 0, 0);

    free_convert_dmatrix(_X, 0);

    cin.ignore();
    success_code = in;

}

// End of Utility functions ------------------------------------------------------------------

// Kalman Filter function --------------------------------------------------------------------

//					This routine will receive an array of data and provide the best estimates
//					of NTR error data back.  The whole value data position estimates for the
//					LHS position and uncertainty along with NTR ocean current data is also output.
//




/// Defining Variables for the Kalman Filter Application.
//double static pi = 3.1415926535897932384626433832795;
//double static nmile = 6076.115486;			// conversion to feet
//double static kn_ft = nmile / 3600.0;			// conversion to ft/sec from kts
//double static ae = 2.092564632545932e+007;	// Radius of earth in feet
//double static e2 = 6.6943799014e-003;		// Ellipsoidal model of eccentricity (WGS-84)

double ORE_mount[3]; // The array that holds the LHS Track Point Mounting Angles
double last_time = 0.0;

bool Startup = true;
bool Set_Depth_Range;

short mycase = 0;



double** _X, ** _F, ** _W, ** _I, ** _P, ** _Phi, ** _Q,
** _Temp1, ** _Temp2, ** _Temp3, ** _Temp4, ** _Temp5,
** _H, ** _R, ** _Z, ** _mq, ** _mcq, ** _KalmG;

double X[15][2], F[15][15], W[15][15], I[15][15],
P[15][15], Phi[15][15], Q[15][15],
Temp1[15][15], Temp2[15][15], Temp3[15][15],
Temp4[15][15], Temp5[15][15],
H[6][15], R[6][6], Z[6][6],
mq[15][6], mcq[6][6], KalmG[15][6];


double* Y, * temp_vect, * temp_vect2, * temp_vect3, * Z_vect, * Resid, * Ratio;

int* temp_ivect;

// ///////////////////////////////////////////////////////////////////////////



// variables
double PosN_delta, PosE_delta, PosD_delta;
double USE_GPS = 0;
double Velocity_LL[3];
double Position_LL[3];
double vel_randWalk;
double angle_randWalk;
double FilterUpdate;
KalmanInputType KInput;
KalmanOutputType KFoutput;



#pragma //once

void free_arrays_proto();

void KalmanFilter(KalmanInputType KalmanInput, KalmanOutputType* _KFoutput, int FILTER_RUN)
{
    ///	Local Variables for the filter
    //KalmanInputType KInput;
    //KalmanOutputType KFoutput;
    //unsigned char inbuf2[sizeof(KFoutput)];


    FILE* fp1;
    int i, j, k, n, rows, icheck, number_rows, jj, kk;

    double R_merid, R_norm;

    double  cr, sr, cp, sp, ch, sh, step, current_time,
        dt, nddt;

    double  ratio_check;

    bool 	compute_filter = true;

    // Define variables to not get errors

    number_rows = 0; // DELETING THIS GIVES AN ERROR

    // Local Variables

    int bad;
    double bad_fix_at_time;
    //PosN_delta = 0; PosE_delta = 0; PosD_delta = 0;
    //double FilterUpdate;

    int time_to_start_GPS = 69576; // GPS sigma reduced to 8 ft

    // Errors for the IMU -----------------------------------------------------

    double accellSF = 0.05;  // 5%
    double bias_accel = 5 * 9.8e-3; // 5 milli - g's
    double Accel_misalign = 1.15; // deg

    double bias_gyro = 1.4 * pi / (180. * 3600); // deg/s
    double gyro_SF = 0.005; // .5%
    double Gyro_misalign = 1.15; // deg
    // ----------------------------------------------------------------------------

    // buffer to hold incoming data structure message for compensation from filter as needed
    unsigned char inbuf[sizeof(KInput)];


    /// Starting the filter routine! *****************
    
    memmove(inbuf, &KalmanInput, sizeof(KInput));
    memmove(&KInput, inbuf, sizeof(KInput));

    /// ALLOCATE MEMORY FOR ALL ARRAYS ... (ONCE)
    // .....................................................................
    /// The memory allocation for all arrays is done here - one time 
    /// upon startup of the parent routine. If reinitialization is 
    /// commanded, the arrays will simply be zeroed and/or placed in 
    /// the default startup state.
    /// The .INI file is also read in here upon startup of the filter.

    if (Startup)
    {

        _X = convert_dmatrix(&X[0][0], 0, (NStates - 1), 0, 1);
        _F = convert_dmatrix(&F[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _W = convert_dmatrix(&W[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _I = convert_dmatrix(&I[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _P = convert_dmatrix(&P[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _Phi = convert_dmatrix(&Phi[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _Q = convert_dmatrix(&Q[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _Temp1 = convert_dmatrix(&Temp1[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _Temp2 = convert_dmatrix(&Temp2[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _Temp3 = convert_dmatrix(&Temp3[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _Temp4 = convert_dmatrix(&Temp3[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _Temp5 = convert_dmatrix(&Temp3[0][0], 0, (NStates - 1), 0, (NStates - 1));
        _H = convert_dmatrix(&H[0][0], 0, (NMeas - 1), 0, (NStates - 1));
        _R = convert_dmatrix(&R[0][0], 0, (NMeas - 1), 0, (NMeas - 1));
        _Z = convert_dmatrix(&Z[0][0], 0, (NMeas - 1), 0, (NMeas - 1));
        _mq = convert_dmatrix(&mq[0][0], 0, (NStates - 1), 0, (NMeas - 1));
        _mcq = convert_dmatrix(&mcq[0][0], 0, (NMeas - 1), 0, (NMeas - 1));
        _KalmG = convert_dmatrix(&KalmG[0][0], 0, (NStates - 1), 0, (NMeas - 1));

        Y = dvector(0, (NMeas - 1));
        temp_vect = dvector(0, (NStates - 1));
        temp_vect2 = dvector(0, (NStates - 1));
        temp_vect3 = dvector(0, (NStates - 1));
        Z_vect = dvector(0, (NMeas - 1));
        Resid = dvector(0, (NMeas - 1));
        Ratio = dvector(0, (NMeas - 1));

        temp_ivect = ivector(0, (NStates - 1));

    }  // end if startup...

    if ((KalmanInput.mode == 2) || (Startup)) //SET ALL ELEMENTS TO ZERO AND INITIALIZE PROPERLY
    {

        dmzero(_X, NStates, 2);
        dmzero(_F, NStates, NStates);
        dmzero(_W, NStates, NStates);
        dmzero(_I, NStates, NStates);
        dmzero(_P, NStates, NStates);
        dmzero(_Phi, NStates, NStates);
        dmzero(_Q, NStates, NStates);
        dmzero(_Temp1, NStates, NStates);
        dmzero(_Temp2, NStates, NStates);
        dmzero(_Temp3, NStates, NStates);
        dmzero(_H, NMeas, NStates);
        dmzero(_R, NMeas, NMeas);
        dmzero(_Z, NMeas, NMeas);
        dmzero(_mq, NStates, NMeas);
        dmzero(_mcq, NMeas, NMeas);
        dmzero(_KalmG, NStates, NMeas);

        vezero(Y, NMeas);
        vezero(temp_vect, NStates);
        vezero(temp_vect2, NStates);
        vezero(temp_vect3, NStates);
        vezero(Z_vect, NMeas);
        vezero(Resid, NMeas);
        vezero(Ratio, NMeas);

        iezero(temp_ivect, NStates);
        // Now Initialize...................

        /// THESE LINES INITIALIZE THE DESCRIPTION MATRIX F
        for (int i = 0; i < 15; ++i) {			// Initializes every element to zero in Fmat
            for (int j = 0; j < 15; ++j) {
                F[i][j] = 0.0;
            }
        }

        /// THESE LINES INITIALIZE THE DRIVING NOISE MATRIX W
        for (int i = 0; i < 15; ++i) {			// Initializes every element to zero in Wmat
            for (int j = 0; j < 15; ++j) {
                W[i][j] = 0.0;
            }
        }

        W[3][3] = pow(vel_randWalk, 2);
        W[4][4] = W[3][3];
        W[5][5] = W[3][3];
        W[6][6] = pow(angle_randWalk, 2);
        W[7][7] = W[6][6];
        W[8][8] = W[6][6];

        /// THESE LINES INITIALIZE THE COVARIANCE MATRIX P
        P[0][0] = pow(5, 2);				// Initial Covariance Matrix  pos error is 5 meters (N & E)
        P[1][1] = P[0][0];
        P[2][2] = pow(2.0, 2);			// initial depth error is 1 meter
        P[3][3] = pow(3.0, 2);			// initial vel error is 3 m/sec (N & E)
        P[4][4] = P[3][3];
        P[5][5] = pow(1.5, 2);				// Vert Velocity ~ 1.5 m/s
        P[6][6] = pow((5 * pi / 180), 2);		// initial angle errors are 1 deg for roll & pitch
        P[7][7] = P[6][6];
        P[8][8] = pow((10 * pi / 180), 2);	// Heading error set to 5 degrees
        P[9][9] = pow(bias_accel, 2);        // IMU accel bias
        P[10][10] = P[9][9];
        P[11][11] = P[9][9];
        P[12][12] = pow(bias_gyro, 2);     // IMU gyro bias
        P[13][13] = P[12][12];
        P[14][14] = P[12][12];

    }  // end if ( (KalmanInput.mode == 2) || 

    if (Startup)
    {
        Startup = false;	// Signifying that the memory allocation is only done ONCE!

        return;				// Jumping out of the filter after initializing the memory and arrays
    }

    if (KalmanInput.mode == 3) //Free all memory allocated as program is terminating
    {
        // Free-up the arrays on the heap:
        free_arrays_proto();
        return;
    }

    /// Running Mode.... 

    if (KalmanInput.mode == 0) // Running but not launched and Kalman filter is not engaged
    {
        last_time = KalmanInput.time;
        KFoutput.time = last_time;
        KFoutput.filter_mode = KalmanInput.mode;

        KFoutput.delta_Pn = 0;
        KFoutput.delta_Pn = 0;
        KFoutput.delta_Pe = 0;
        KFoutput.delta_Pd = 0;
        KFoutput.delta_Vn = 0;
        KFoutput.delta_Ve = 0;
        KFoutput.delta_Vd = 0;
        KFoutput.delta_tiltN = 0;
        KFoutput.delta_tiltE = 0;
        KFoutput.delta_tiltD = 0;
        KFoutput.delta_ABx = 0;
        KFoutput.delta_ABy = 0;
        KFoutput.delta_ABz = 0;
        KFoutput.delta_GBx = 0;
        KFoutput.delta_GBy = 0;
        KFoutput.delta_GBz = 0;
        KFoutput.covPn = 0;
        KFoutput.covPe = 0;
        KFoutput.covVn = 0;
        KFoutput.covVe = 0;
        KFoutput.covPd = 0;
        KFoutput.covPsiD = 0;
    }

    //---------------------------------------------------------------------
    /// *** START KALMAN FILTER CODE HERE ***
    if (KalmanInput.mode == 1)		// We got the Launch signal
    {
        //---------------------------------------------------------------------

        // PART I - COMPENSATE DATA & UPDATE DESCRIPTION MATRIX (F) & DRIVING NOISE MATRIX (W)

        //----------------------------------------------------------------------------------
        current_time = KalmanInput.time;


        // PART II - CALCULATE TRANSITION MATRIX AND PROCESS NOISE COVARIANCE

        if (last_time > 0.0)
            dt = current_time - last_time;
        else
            dt = 1.0;
        last_time = current_time;

        // Create identity matrix I

        dmeye(_I, Max, Max);

        // Solve for Phi and Q

        nddt = 4;
        step = dt / 16.0;
        dmscal(_W, _Q, Max, Max, step);
        // Function:  dmscal(A, B, m, n, c)
        // Multiply matrix A by scalar double c:  B = A * c
        // where arrays A and B are m-by-n.

        dmscal(_F, _Temp1, Max, Max, step);
        // Function:  dmscal(A, B, m, n, c)
        // Multiply matrix A by scalar double c:  B = A * c
        // where arrays A and B are m-by-n.

        dmapb(_I, _Temp1, _Phi, Max, Max);  // Phi = _I + _F * step
        // Function:  dmapb(A, b, C, m, n)
        // Matrix add (plus):  C= A + B,
        // where arrays A, b, and C are m-by-n.

        for (i = 1; i <= nddt; i++)
        {
            dmab(_Phi, _Q, _Temp1, Max, Max, Max);  // _Temp1 = _Phi * _Q
        // Function:  dmab(A, B, C, l, m, n)
        // Matrix multiply:  C = A * B,
        // where A is l-by-m, B is m-by-n, and C is l-by-n.

            dmabt(_Temp1, _Phi, _Temp2, Max, Max, Max);  // _Temp2 = _Phi*_Q * _Phi^T
        // Function:  dmabt(A, B, C, l, m, n)
        // Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
        // where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.

            dmcopy(_Q, _Temp1, Max, Max);  // _Temp1 = _Q
        // Function:  dmcopy(A, B, m, n)
        // Copy m-by-n matrix A into m-by-n matrix B.

            dmapb(_Temp1, _Temp2, _Q, Max, Max);  // _Q = _Q + _Phi*_Q*_Phi^T
        // Function:  dmapb(A, b, C, m, n)
        // Matrix add (plus):  C= A + B,
        // where arrays A, b, and C are m-by-n.

            dmcopy(_Phi, _Temp1, Max, Max);  // _Temp1 = _Phi
        // Function:  dmcopy(A, B, m, n)
        // Copy m-by-n matrix A into m-by-n matrix B.

            dmab(_Phi, _Temp1, _Temp2, Max, Max, Max);  // _Temp2 = _Phi * _Phi
        // Function:  dmab(A, B, C, l, m, n)
        // Matrix multiply:  C = A * B,
        // where A is l-by-m, B is m-by-n, and C is l-by-n.

            dmcopy(_Temp2, _Phi, Max, Max);  // _Phi = _Phi * _Phi
        // Function:  dmcopy(A, B, m, n)
        // Copy m-by-n matrix A into m-by-n matrix B.

        } // for (i=1...

        /// PART III - PROPAGATE STATE VECTOR AND COVARIANCE MATRICES

        // Propagate state matrix

        dmab(_Phi, _X, _Temp1, Max, Max, 2);  // _Temp1 = _Phi * _X
        // Function:  dmab(A, B, C, l, m, n)
        // Matrix multiply:  C = A * B,
        // where A is l-by-m, B is m-by-n, and C is l-by-n.

        dmcopy(_Temp1, _X, Max, 2);   // _X = _Phi * _X
        // Function:  dmcopy(A, B, m, n)
        // Copy m-by-n matrix A into m-by-n matrix B.

        /// *** Propagate covariance matrix

        dmab(_Phi, _P, _Temp1, Max, Max, Max);  // _Temp1 = _Phi * _P
        // Function:  dmab(A, B, C, l, m, n)
        // Matrix multiply:  C = A * B,
        // where A is l-by-m, B is m-by-n, and C is l-by-n.

        dmabt(_Temp1, _Phi, _Temp2, Max, Max, Max);  // _Temp2 = _Phi * _P * _Phi^T
        // Function:  dmabt(A, B, C, l, m, n)
        // Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
        // where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.

        dmapb(_Temp2, _Q, _P, Max, Max);  // _P = _Phi*_P*_Phi^T + _Q
        // Function:  dmapb(A, b, C, m, n)
        // Matrix add (plus):  C= A + B,
        // where arrays A, b, and C are m-by-n.


        /// PART IV -- FORM THE MEASUREMENT EQUATIONS:

        //casemeasure(KalmanInput,_R,&mycase);
        number_rows = 0;
        dmzero(_H, NMeas, NStates);		// Zeros out the H matrix
        vezero(Y, NMeas);


        //-------------------------------------------------------------------------------------------------------------------------
        //If filter run is true do the belo (all of it) to generate a message
        if (FILTER_RUN > 0.5) {
            if (USE_GPS < 0.5) {
                H[5][14] = 0;
                R[5][5] = 0;
                Y[5] = 0;
                number_rows = 6;

                Y[3] = Velocity_LL[0]; // Bringing in zero velocity data
                Y[4] = Velocity_LL[1];
                Y[5] = Velocity_LL[2];

                H[3][3] = 1;
                H[4][4] = 1;
                H[5][5] = 1;

                R[0][0] = pow(.7, 2); // Small stationary measurement covariance error
                R[1][1] = pow(.7, 2);
                R[2][2] = pow(.7, 2);
                R[3][3] = pow(0.05, 2);
                R[4][4] = pow(0.05, 2);
                R[5][5] = pow(0.05, 2);

            }
            else if (USE_GPS > 0.5) {
                H[2][14] = 0;
                R[2][2] = 0;
                Y[2] = 0;
                number_rows = 3;
            }
            Y[0] = PosN_delta;   // default position data is used
            Y[1] = PosE_delta;
            Y[2] = PosD_delta;

            H[0][0] = 1;
            H[1][1] = 1;
            H[2][2] = 1;
            if (USE_GPS > 0.5) {
                R[0][0] = pow(3., 2);
                R[1][1] = pow(3., 2);
                R[2][2] = pow(5., 2);
            }

        // Kalman Gain Calculation
        /// INNOVATION CHECK $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            j = number_rows;
            jj = Max;
            kk = Max;
            dmab(_H, _P, _Temp1, j, jj, kk);  // _Temp1=_H*_P
            // where _H is j-by-jj, _P is jj-by-kk, and _Temp1 is j-by-kk.

            j = number_rows;
            jj = Max;
            kk = number_rows;
            dmabt(_Temp1, _H, _Temp2, j, jj, kk); //_Temp2 = _Temp1 * _H^T,  (^T signifies transpose) 
            // where _Temp1 is j-by-jj, _H^T is jj-by-kk (or _H is kk-by-jj), and _Temp2 is j-by-kk.
            // Function:  dmabt(A, B, C, l, m, n)
            // Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
            // where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.

            j = number_rows;
            dmapb(_Temp2, _R, _Z, j, j);
            // Function:  dmapb(A, b, C, m, n)
            // Matrix add (plus):  C= A + B,
            // where arrays A, b, and C are m-by-n.

            diagonal(_Z, Z_vect, j);
            // Function:  diagonal(A, B, m)
            // Place diagonal of square matrix A into vector B
            // where array A is m-by-m and array B is m-by-1

            for (i = 0; i < j; i++) {
                Z_vect[i] = sqrt(Z_vect[i]);
            }

            for (i = 0; i < Max; i++) {
                temp_vect[i] = X[i][0];
            }

            j = number_rows;
            dmav(_H, temp_vect, temp_vect2, j, Max);
            // Function:  dmav(A, v, u, m, n)
            // Matrix-vector multiply:  u = A * v,
            // where A is m-by-n matrix, v is n-by-1 vector, and u is m-by-1 vector.

            j = number_rows;
            vecsub(Y, temp_vect2, Resid, j);
            // Function:  vecsub(A, B, C, m)
            // Vector minus (subtract):  C = A - B,
            // where arrays A, B, and C are m-by-1.

            vecdiv(Resid, Z_vect, Ratio, j);  // Computing Ratio for innovation check
            // Function:  vecdiv(A, B, C, m)
            // Vector divide:  C = A / B,
            // where arrays A, B, and C are m-by-1.

            ratio_check = 4.5;

            bad = 0;

            for (i = 0; i < number_rows; i++) {

                if (USE_GPS > 0.5) {
                    ratio_check = 15;
                }
                else {
                    ratio_check = 4.5;
                }
                if (Ratio[i] > ratio_check) {
                    bad_fix_at_time = current_time;
                    bad = 1;
                    printf("Innovation sequence fail: Bad fix at time %f\n", current_time);
                    printf(" -Failure at measurement row = %d\n", i);
                    //std::cout << "Press Enter to continue..."; // C++ equivalent to MATLAB's "pause"
                    //std::cin.get(); // waits for Enter key
                    system("pause"); // Only works on Windows not Linux OS
                }
            }
            // if(Kinput.mode = 1)

               //

            if (bad < 1) {
                //                                            
                //------------------------------------------------------------------------------------------------------
                // -------------------------------------------------------
                // 
            // Kalman Gain Calculation ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

                // Since we could have restructured the H and R matrices above, we must now start
                // all calculations anew if number_rows > 0

                if (number_rows > 0)
                {
                    // mq=P*H^T
                    j = Max;
                    jj = Max;
                    kk = number_rows;
                    dmabt(_P, _H, _mq, j, jj, kk);
                    // Function:  dmabt(A, B, C, l, m, n)
                    // Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
                    // where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.


                    //Temp1=H*mq ...=H*P*H^T
                    j = number_rows;
                    jj = Max;
                    kk = j;
                    dmab(_H, _mq, _Temp1, j, jj, kk);  // _Temp1=_H*_mq
                    // where _H is j-by-jj, _mq is jj-by-j, and _Temp1 is j-by-j.


                    //Temp2=H*P*H^T + R
                    j = number_rows;
                    dmapb(_Temp1, _R, _Temp2, j, j);
                    // Function:  dmapb(A, b, C, m, n)
                    // Matrix add (plus):  C= A + B,
                    // where arrays A, b, and C are m-by-n.

                    dmzero(_Temp1, NStates, NStates);
                    vezero(temp_vect, NStates);
                    vezero(temp_vect2, NStates);
                    iezero(temp_ivect, NStates);

                    // mcq = inverse of (H*P*H^T + R) or Inv(Temp2)
                    j = number_rows;
                    k = dmaib(_Temp2, _I, _mcq, _Temp1, temp_vect, temp_vect2, temp_ivect, j, j);
                    if (k > 0)
                    {
                        printf("Singular Matrix!\n");
                        compute_filter = false;
                    } // Must return without any Kalman updates, if inverse has problems...
                    // Function:  dmaib(A, B, X, Temp, temp_vect, temp_vect, temp_ivect, m, n)
                    // Given matrices A and B, evaluate X = A\B without doing an explicit
                    // inversion of the A matrix.  (I.e., it is more efficient than doing
                    // a matrix inversion followed by a matrix multiply.  However, if the
                    // inverse of A is required, then set B equal to the identity matrix.)
                    //   A is m-by-m
                    //   B is m-by-n
                    //   X is m-by-n
                    // The temporary (scratch) real vectors and the one integer vector are of length m.
                    // If the A matrix is not singular the return value is 0; otherwise it is 1
                    // and the "solution" for x is not valid.

                    if (compute_filter)
                    {
                        // KalmG = P*H^T * inv(H*P*H^T + R)
                        j = Max;
                        jj = number_rows;
                        kk = number_rows;
                        dmab(_mq, _mcq, _KalmG, j, jj, kk);
                        // Function:  dmab(A, B, C, l, m, n)
                        // Matrix multiply:  C = A * B,
                        // where A is l-by-m, B is m-by-n, and C is l-by-n.
                    //-------------------------------------------------------------------------------
                    // Now to UPDATE the Matrices by the Kalman Gain

                    // Update the Covariance Matrix

                    // KalmG*H  ----> Temp1
                        j = Max;
                        jj = number_rows;
                        kk = Max;
                        dmab(_KalmG, _H, _Temp1, j, jj, kk);
                        // Function:  dmab(A, B, C, l, m, n)
                        // Matrix multiply:  C = A * B,
                        // where A is l-by-m, B is m-by-n, and C is l-by-n.

                    // (I - KalmG*H)  ------> Temp2
                        j = Max;
                        jj = Max;
                        dmamb(_I, _Temp1, _Temp2, j, jj);
                        // Function:  dmamb(A, B, C, m, n)
                        // Matrix difference (minus):  C = A - B,
                        // where arrays A, B, and C are m-by-n.

                    // P * (I - KalmG*H)^T   ----------> Temp1
                        j = Max;
                        jj = Max;
                        kk = Max;
                        dmabt(_P, _Temp2, _Temp1, j, jj, kk);
                        // Function:  dmabt(A, B, C, l, m, n)
                        // Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
                        // where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.

                    // (I - KalmG*H) * P * (I - KalmG*H)^T  ----> Temp3
                                    // Temp3 = Temp2 * Temp1
                        j = Max;
                        jj = Max;
                        kk = Max;
                        dmab(_Temp2, _Temp1, _Temp3, j, jj, kk);
                        // Function:  dmab(A, B, C, l, m, n)
                        // Matrix multiply:  C = A * B,
                        // where A is l-by-m, B is m-by-n, and C is l-by-n.

                    // R*KalmG^T     ---------> Temp1
                        j = number_rows;
                        jj = number_rows;
                        kk = Max;
                        dmabt(_R, _KalmG, _Temp1, j, jj, kk);
                        // Function:  dmabt(A, B, C, l, m, n)
                        // Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
                        // where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.

                    // KalmG * R*KalmG^T    ------> Temp2
                                    // Temp2 = KalmG * Temp1
                        j = Max;
                        jj = number_rows;
                        kk = Max;
                        dmab(_KalmG, _Temp1, _Temp2, j, jj, kk);
                        // Function:  dmab(A, B, C, l, m, n)
                        // Matrix multiply:  C = A * B,
                        // where A is l-by-m, B is m-by-n, and C is l-by-n.

                    //P = (I - KalmG*H) * P * (I - KalmG*H)^T + KalmG * R*KalmG^T
                    //P = Temp3 + Temp2
                        dmapb(_Temp3, _Temp2, _P, Max, Max);
                        // Function:  dmapb(A, b, C, m, n)
                        // Matrix add (plus):  C= A + B,
                        // where arrays A, b, and C are m-by-n.

                    // Update the State Matrix

                        for (i = 0; i < Max; i++) temp_vect[i] = X[i][0];  // temp_vect=X (1st column)

                        // temp_vect2 = H * X
                        j = number_rows;
                        dmav(_H, temp_vect, temp_vect2, j, Max);
                        // Function:  dmav(A, v, u, m, n)
                        // Matrix-vector multiply:  u = A * v,
                        // where A is m-by-n matrix, v is n-by-1 vector, and u is m-by-1 vector.

                        // Resid = Y - H*X
                        j = number_rows;
                        vecsub(Y, temp_vect2, Resid, j);
                        // Function:  vecsub(A, B, C, m)
                        // Vector minus (subtract):  C = A - B,
                        // where arrays A, B, and C are m-by-1.

                    // KalmG*Resid    --------> temp_vect
                        j = Max;
                        jj = number_rows;
                        dmav(_KalmG, Resid, temp_vect, j, jj);
                        // Function:  dmav(A, v, u, m, n)
                        // Matrix-vector multiply:  u = A * v,
                        // where A is m-by-n matrix, v is n-by-1 vector, and u is m-by-1 vector.

                    // X(1st col) = KalmG*Resid
                        for (i = 0; i < Max; i++) X[i][0] = X[i][0] + temp_vect[i];

                        // Accumulate the State Vector
                        for (i = 0; i < Max; i++) X[i][1] = X[i][0] + X[i][1];
                        for (i = 0; i < Max; i++) X[i][0] = 0.0;
                    } //if (compute_filter)

                } // if (number_rows...
            } //if bad <1
        } // if FilterRun

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

    //--------------------------------------------------------------------- 
    // *** OUTPUT DATA ***
    //---------------------------------------------------------------------
        //end of Filter run check

        KFoutput.time = current_time;

        KFoutput.delta_Pn = X[0][1]; 
        X[0][1] = 0;
        KFoutput.delta_Pe = X[1][1]; 
        X[1][1] = 0;
        KFoutput.delta_Pd = X[2][1]; 
        X[2][1] = 0;
        KFoutput.delta_Vn = X[3][1];
        X[3][1] = 0;
        KFoutput.delta_Ve = X[4][1];
        X[4][1] = 0;
        KFoutput.delta_Vd = X[5][1];
        X[5][1] = 0;

        KFoutput.delta_tiltN = X[6][1];
        X[6][1] = 0;
        KFoutput.delta_tiltE = X[7][1];
        X[7][1] = 0;
        KFoutput.delta_tiltD = X[8][1];
        X[8][1] = 0;

        KFoutput.delta_ABx = X[9][1];
        X[9][1] = 0;
        KFoutput.delta_ABy = X[10][1];
        X[10][1] = 0;
        KFoutput.delta_ABz = X[11][1];
        X[11][1] = 0;
        KFoutput.delta_GBx = X[12][1];
        X[12][1] = 0;
        KFoutput.delta_GBy = X[13][1];
        X[13][1] = 0;
        KFoutput.delta_GBz = X[14][1];
        X[14][1] = 0;


        
        KalmanInput.GPS_valid = 0;
        KalmanInput.VertVelocity_valid = 0;
        KalmanInput.LevelVelocity_valid = 0;

    } // if KalmanInput.mode == 1

    //
}
//*****************************************************************************
// This function is called at exit time to free-up the long double pointers:
//*****************************************************************************
void free_arrays_proto()
{

    free_convert_dmatrix(_X, 0);
    free_convert_dmatrix(_F, 0);
    free_convert_dmatrix(_W, 0);
    free_convert_dmatrix(_I, 0);
    free_convert_dmatrix(_P, 0);
    free_convert_dmatrix(_Phi, 0);
    free_convert_dmatrix(_Q, 0);
    free_convert_dmatrix(_Temp1, 0);
    free_convert_dmatrix(_Temp2, 0);
    free_convert_dmatrix(_Temp3, 0);
    free_convert_dmatrix(_Temp4, 0);
    free_convert_dmatrix(_Temp5, 0);
    free_convert_dmatrix(_H, 0);
    free_convert_dmatrix(_R, 0);
    free_convert_dmatrix(_Z, 0);
    free_convert_dmatrix(_mq, 0);
    free_convert_dmatrix(_mcq, 0);
    free_convert_dmatrix(_KalmG, 0);

    free_dvector(Y, 0);
    free_dvector(temp_vect, 0);
    free_dvector(temp_vect2, 0);
    free_dvector(temp_vect3, 0);
    free_dvector(Z_vect, 0);
    free_dvector(Resid, 0);
    free_dvector(Ratio, 0);

    free_ivector(temp_ivect, 0);

}
//----------------- end of free_arrays_proto ----------------------------------

// End of Kalman Filter Routine -----------------------------------------------

// PI controller --------------------------------------------------------------


void PIcontroller(double Theta, double& Control, double& integral) {

    // Input from Guidance: 
    // Theta

    // Output from PIcontroller: 
    // Control

    // Local variables inside the controller --------------

    // Gains (Kp, Ki)

    double Kp = 1;  // Proportional Gain - changes the amount of oscillations
    double Ki = .1e-4; // Integral Gain  - compensates the error remaining after the proportional gain

    // Time step - constant determined by system's processing

    double dt = 0.1;  // Change to match system's time step

    // ----------------------------------------------------
    // -------------- REAL TIME CONTROLLER ----------------
    // ----------------------------------------------------

    integral += Theta * dt;

    // Control output
    Control = Kp * Theta + Ki * integral;

    // END
}
// End of PI controller ----------------------------------------------------

// Guidance Routine --------------------------------------------------------


using namespace std;

void guidance_angle(double pos_cur[3], double vehicle_v[3], double pos_target[3], double& Theta) {

    // Pure-Pursuit Method
    // Heads to the desired position set by the USER but doesn't go along a desired path
    // Takes the angle between the unit vectors (directions) of the vehicle's velocity and heading vector

    // Inputs:
    // 
    //	pos_cur		: [X_c,Y_c,Z_c] initial position (m)
    //  vehicle_v	: [V_x,V_y,V_z] current velocity vector (m/s)
    //	pos_target	: [X_t,Y_t,Z_t] target position (Given from the GUI) (m)

    // Output:
    //
    // Theta		: Angle to deflect rudder to align the direction of the vehicle toward the desired destination (Deg)

    // NOTE:
    // USING only 1st and 2nd elements of input vectors.
    // Simplified down to a 2D problem for an UAMV only riding along the surface of the water. Thus, change in altitude in negligible;


    // Initial Local Variables

    double hdg_vector[2] = { 0 };
    double v_norm = 0;
    double hdg_norm = 0;

    double v_unit[2] = { 0 };
    double hdg_unit[2] = { 0 };

    double dot_val = 0;
    double angle = 0;
    double cross_val = 0;

    // Tolerance

    double tol = 1e-6;

    // Step 1: Heading vector from current to target
    hdg_vector[0] = pos_target[0] - pos_cur[0];
    hdg_vector[1] = pos_target[1] - pos_cur[0];

    // Step 2: Normalize both heading and velocity vectors

    v_norm = sqrt(pow(vehicle_v[0], 2) + pow(vehicle_v[1], 2));
    hdg_norm = sqrt(pow(hdg_vector[0], 2) + pow(hdg_vector[1], 2));

    if (v_norm < tol || hdg_norm < tol) { // Avoid division by zero or undefined angles
        Theta = 0;
    }
    else {

        v_unit[0] = vehicle_v[0] / v_norm;
        v_unit[1] = vehicle_v[1] / v_norm;

        hdg_unit[0] = hdg_vector[0] / hdg_norm;
        hdg_unit[1] = hdg_vector[1] / hdg_norm;

        // Step 3: Compute angle between vectors using dot product

        dot_val = v_unit[0] * hdg_unit[0] + v_unit[1] * hdg_unit[1];
        dot_val = std::max(std::min(dot_val, 1.0), -1.0); // Clamp to valid acos range
        angle = acos(dot_val);                 // Always in range [0,pi]  // CHECK 

        // Step 4: Determine sign of rotation using 2D scalar cross product

        cross_val = v_unit[0] * hdg_unit[1] - v_unit[1] * hdg_unit[0];

        // Step 5: Apply sign to angle

        if (cross_val < 0) {
            Theta = angle * (-1.0); // Turn Right
            if (Theta < -45) {
                Theta = -45; // Min boundary = -45 deg
            }
        }
        else {
            Theta = angle; // Turn Left
            if (Theta > 45) {
                Theta = 45; // Max boundary = 45 deg
            }
        }
    }

}

// End of Guidance Routine ------------------------------------------------------------

// IMU instrument reading routine -----------------------------------------------------
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <stdexcept>
#include <cstdint>
// Constants
const int IMU_ERROR_CODE = 23456789;
const char* I2C_DEVICE = "/dev/i2c-1";
const int MPU6050_ADDR = 0x68;

// IMU Register Map
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;
const uint8_t GYRO_XOUT_H = 0x43;

class MPU6050 {
private:
    int i2c_fd;

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        if (write(i2c_fd, data, 2) != 2) {
            throw runtime_error("I2C write failed");
        }
    }

    int16_t read16(uint8_t reg) {
        uint8_t data[2];
        if (write(i2c_fd, &reg, 1) != 1) {
            throw runtime_error("I2C register select failed");
        }
        if (read(i2c_fd, data, 2) != 2) {
            throw runtime_error("I2C read failed");
        }
        return (data[0] << 8) | data[1];
    }

public:
    MPU6050() : i2c_fd(-1) {
        // Open I2C bus
        if ((i2c_fd = open(I2C_DEVICE, O_RDWR)) < 0) {
            throw runtime_error("Failed to open I2C device");
        }

        // Set slave address
        if (ioctl(i2c_fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
            close(i2c_fd);
            throw runtime_error("Failed to acquire bus access");
        }

        // Wake up MPU6050
        writeRegister(PWR_MGMT_1, 0);
    }

    ~MPU6050() {
        if (i2c_fd >= 0) {
            close(i2c_fd);
        }
    }

    void readAll(double& ax, double& ay, double& az, double& gx, double& gy, double& gz) {
        try {
            // Read accelerometer (registers 0x3B-0x40)
            int16_t accel_x = read16(ACCEL_XOUT_H);
            int16_t accel_y = read16(ACCEL_XOUT_H + 2);
            int16_t accel_z = read16(ACCEL_XOUT_H + 4);

            // Read gyroscope (registers 0x43-0x48)
            int16_t gyro_x = read16(GYRO_XOUT_H);
            int16_t gyro_y = read16(GYRO_XOUT_H + 2);
            int16_t gyro_z = read16(GYRO_XOUT_H + 4);

            // Convert to meaningful units
            //const float accel_scale = 16384.0f;  // ±2g range
            //const float gyro_scale = 131.0f;     // ±250°/s range

            ax = accel_x; // accel_scale;
            ay = accel_y; // accel_scale;
            az = accel_z; // accel_scale;
            gx = gyro_x; // gyro_scale;
            gy = gyro_y; // gyro_scale;
            gz = gyro_z; // gyro_scale;

            // Validate readings
            if (isnan(ax) || isinf(ax) || abs(ax) > 10.0f ||
                isnan(ay) || isinf(ay) || abs(ay) > 10.0f ||
                isnan(az) || isinf(az) || abs(az) > 10.0f) {
                throw runtime_error("Invalid accelerometer readings");
            }

        } catch (...) {
            ax = ay = az = gx = gy = gz = IMU_ERROR_CODE;
            throw;
        }
    }
};


// End of IMU routine -----------------------------------------------------------------

// GPS instrument reading routine -----------------------------------------------------
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <exception>
struct GPSData {
    double Latitude = 0.0;
    double Longitude = 0.0;
    double Altitude = 0.0;
    bool valid = false;
};

GPSData parseGPGGA(const string& sentence) {
    GPSData data;
    istringstream ss(sentence);
    string token;

    getline(ss, token, ','); // $GPGGA or $GNGGA
    if (token != "$GPGGA" && token != "$GNGGA") return data;

    // Skip time field
    getline(ss, token, ',');

    // Latitude
    getline(ss, token, ',');
    if (!token.empty() && token.length() >= 4) {
        try {
            double deg = stod(token.substr(0, 2));
            double min = stod(token.substr(2));
            double lat = deg + min / 60.0;

            getline(ss, token, ','); // N/S
            if (token == "S") lat = -lat;
            data.Latitude = lat;
        } catch (const exception& e) {
            cerr << "Latitude parse error: " << e.what() << endl;
            return data;
        }
    } else {
        return data;
    }

    // Longitude
    getline(ss, token, ',');
    if (!token.empty() && token.length() >= 5) {
        try {
            double deg = stod(token.substr(0, 3));
            double min = stod(token.substr(3));
            double lon = deg + min / 60.0;

            getline(ss, token, ','); // E/W
            if (token == "W") lon = -lon;
            data.Longitude = lon;
        } catch (const exception& e) {
            cerr << "Longitude parse error: " << e.what() << endl;
            return data;
        }
    } else {
        return data;
    }

    // Fix quality
    getline(ss, token, ',');
    if (token.empty() || token == "0") {
        return data;
    }

    // Skip satellites and HDOP
    for (int i = 0; i < 2; i++) getline(ss, token, ',');

    // Altitude
    getline(ss, token, ',');
    if (!token.empty()) {
        try {
            data.Altitude = stod(token);
        } catch (const exception& e) {
            cerr << "Altitude parse error: " << e.what() << endl;
            return data;
        }
    }

    data.valid = true;
    return data;
}
// End of GPS routine -----------------------------------------------------------------

// Rudder routine ---------------------------------------------------------------------
int mapAngleToPulse(int angle) {
    return 500 + angle * (2000 / 180);
}
// End of Rudder routine --------------------------------------------------------------

// Motor Routine ----------------------------------------------------------------------

#include <wiringPi.h>

// ------------------------------------------------------------------------------------

// Nav strapdown function will be performed in main

// ------------------------------------------------------------------------------------
// ---------------------------------- NAV STRAPDOWN -----------------------------------
// ------------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    // Initial Variables for GUIDANCE & CONTROL
    
    
    double destination = 0;
    double destination_tolerance = 0.1;  // Set target tolerance to tell the code that the vehicle has reached the destination
    double integral = 0;
    double control = 0;

    // Static Initial Definitions  ----- 

    double m_to_ft = 100 / (2.54 * 12);
    double km_to_fps = 1000 * 100 / (2.54 * 12 * 3600);

    double lat = 0;
    double initial_lat = lat;
    double sl = sin(lat);
    double cl = cos(lat);
    double lon = 0;
    double initial_lon = lon;

    // ----------------------

    double omegaE = 7.292115e-5;                                                                                   // earth rate (rad/sec)
    double ae = 2.092564632545932e+007 / m_to_ft;                                                                  // radius of earth in ft
    double e2 = 6.6943799014e-003;                                                                                 // eccentricity of the earth
    double ga = (32.28768625787402 * (1 - 1.932e-3 * pow(sl, 2)) / sqrt(1 - 6.694e-3 * pow(sl, 2))) * 1 / m_to_ft; // WGS84 value of gravity
    double gr = ga / ae;                                                                                           // Schuler Effect
    double om = omegaE;

    // ----------------------

    // Note: IMU samples 20 times/per sec and GPS samples once per sec

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
    double BodyAccelX, BodyAccelY, BodyAccelZ, BodyRateX, BodyRateY, BodyRateZ;

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

    cLL_to_body[1][0] = -cr * sh + sr * sp * ch;
    cLL_to_body[1][1] = cr * ch + sr * sp * sh;
    cLL_to_body[1][2] = sr * cp;

    cLL_to_body[2][0] = sr * sh + cr * sp * ch;
    cLL_to_body[2][1] = -sr * sh + cr * sp * ch;
    cLL_to_body[2][2] = cr * cp;

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
    double Last_Velocity_LL[3];
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

    // BEGIN THE ALIGNMENT stationary -----------------------------------------------

    int time_to_start_GPS = 69576; // GPS sigma reduced to 8 ft

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
    double GPS_output_rate = .1; // (10 Hz)

    // IMU variables

    double PrevIMU_time = 0;
    double IMU_output_rate = .05; // (20 Hz)

    // MATH UTILITY VARIABLES -----------------------------------------------

    int In = 0;
    int success_code = 0;

    // Test Function to see if the Filter is installed correctly

    //MATH_UTILITY_TEST(In, success_code);

    // System Status variables

    int MainLoopCount = 0;

    // Errors for IMU --------------------------------------

    // Errors for the IMU -----------------------------------------------------

    double vel_randWalk = 0.3; // .3 m / s / rt - hr
    double accellSF = 0.05; // 5%
    double bias_accel = 5 * 9.8e-3; // 5 milli - g's
    double Accel_misalign = 1.15; // deg

    double angle_randWalk = 3.2 * pi / (180. * 60); // siggn deg / rt - hr
    double bias_gyro = 1.4 * pi / (180. * 3600); // deg/s
    double gyro_SF = 0.005; // 0.5%
    double Gyro_misalign = 1.15; // deg

    // -----------------------------------------------------

    // -------------------------------------------------------------------------------------
    // ------------------            INITIALIZATION      -----------------------------------
    // -------------------------------------------------------------------------------------
    // 
    // Initialize with first set of data from IMU ------------------------------------------

    //

    // Declare variables
    double IMU_time, DelTheta_Rad1, DelTheta_Rad2, DelTheta_Rad3, DelTheta_Msec1, DelTheta_Msec2, DelTheta_Msec3;

        // Read data from the IMU
        
        try {
        MPU6050 imu;

        
            
            try {
                imu.readAll(BodyAccelX, BodyAccelY, BodyAccelZ, BodyRateX, BodyRateY, BodyRateZ);
            } catch (...) {
                // Continue to output error values
            } // Check if IMU reads data 

            cout << "{"
                 << "\"accel_x\":" << BodyAccelX << ","
                 << "\"accel_y\":" << BodyAccelY << ","
                 << "\"accel_z\":" << BodyAccelZ << ","
                 << "\"gyro_x\":" << BodyRateX << ","
                 << "\"gyro_y\":" << BodyRateY << ","
                 << "\"gyro_z\":" << BodyRateZ 
                 << "}" << endl;

            usleep(10000); // 100ms delay
        
    } catch (const exception& e) {
        cerr << "IMU Error: " << e.what() << endl;
        cout << "{\"accel_x\":23456789,\"accel_y\":23456789,\"accel_z\":23456789,"
             << "\"gyro_x\":23456789,\"gyro_y\":23456789,\"gyro_z\":23456789}" << endl;
        return 1;
    }
    
    while (IMU_initial_counter <= nn){
        // Process the data here -----------------------------------

            IMU_time = IMU_time + 2;
            
            // Finding DelTheta_rad and DelTheta_Msec2
            
            // DelTheta_Rad
            DelTheta_Rad1 = BodyRateX*GPS_output_rate;
            DelTheta_Rad2 = BodyRateY*GPS_output_rate;
            DelTheta_Rad3 = BodyRateZ*GPS_output_rate;
            
            // DelTheta_Msec
            DelTheta_Msec1 = BodyAccelX*IMU_output_rate;
            DelTheta_Msec2 = BodyAccelY*IMU_output_rate;
            DelTheta_Msec3 = BodyAccelZ*IMU_output_rate;
            
            // Assign the respective variables to arrays for storing and processing
            double IMU_deltaVel[3];
            IMU_deltaVel[0] = DelTheta_Msec2; IMU_deltaVel[1] = DelTheta_Msec1; IMU_deltaVel[2] = -DelTheta_Msec3;
            double IMU_deltaTheta[3];
            IMU_deltaTheta[0] = DelTheta_Rad2; IMU_deltaTheta[1] = DelTheta_Rad1; IMU_deltaTheta[2] = -DelTheta_Rad3;

            // -------------------------

            double X = IMU_deltaVel[0]; double Y = IMU_deltaVel[1]; double Z = IMU_deltaVel[2];
            double acc2 = pow(X, 2) + pow(Y, 2) + pow(Z, 2);
            double acc = sqrt(acc2);

            // Roll and Pitch calculations -------------------------------------------------- 


            if (IMU_initial_counter == 1) {
                VelPos_Matrix[0] = IMU_time;
            }

            if (IMU_initial_counter < nn) {
                roll = roll + atan(Y / Z);
                pitch = pitch + asin(X / acc);

            }
            // -----------------------------------------------------
            // // Step 1: Initialize direction cosine matrix with initial roll, pitch, hdg
            // -----------------------------------------------------
            else if (IMU_initial_counter == nn) {

                roll = roll + atan(Y / Z);
                pitch = pitch + asin(X / acc);

                roll = roll / nn;
                pitch = pitch / nn;
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
                AttitudeMatrix[3] = 0;


            } // end of initialization
            
        // Increment IMU initial counter
        IMU_initial_counter++;
    }
    
    

    // ---------------------------------------------------------------------------------------
    // -------------------      END OF INITIALIZATION      -----------------------------------
    // ---------------------------------------------------------------------------------------
    
    // MOTOR STARTS ------------------------------------------------------------------------
    pinMode(ESC_GPIO, PWM_OUTPUT);
    
    
    pwmSetMode(PWM_MODE_MS);  // Mark-Space mode for servo/ESC
    pwmSetClock(192);         // 19.2MHz / 192 = 100kHz
    pwmSetRange(2000);        // 100kHz / 2000 = 50Hz (20ms period)

    
        int pwm_value = 1600 / 10;
        pwmWrite(ESC_GPIO, pwm_value);
        delay(1000);
    // -------------------------------------------------------------------------------------

    // Loop until destination is reached (destination == 1) CHANGEF
    
    while (destination < 0.5){
        // Read data from the IMU ---------------------------------------------------
        
        try {
        MPU6050 imu;

            
            try {
                imu.readAll(BodyAccelX, BodyAccelY, BodyAccelZ, BodyRateX, BodyRateY, BodyRateZ);
            } catch (...) {
                // Continue to output error values
            } // Check if IMU reads data 

            cout << "{"
                 << "\"accel_x\":" << BodyAccelX << ","
                 << "\"accel_y\":" << BodyAccelY << ","
                 << "\"accel_z\":" << BodyAccelZ << ","
                 << "\"gyro_x\":" << BodyRateX << ","
                 << "\"gyro_y\":" << BodyRateY << ","
                 << "\"gyro_z\":" << BodyRateZ 
                 << "}" << endl;

            usleep(10000); // 100ms delay
        
        } catch (const exception& e) {
            cerr << "IMU Error: " << e.what() << endl;
            cout << "{\"accel_x\":23456789,\"accel_y\":23456789,\"accel_z\":23456789,"
                << "\"gyro_x\":23456789,\"gyro_y\":23456789,\"gyro_z\":23456789}" << endl;
            return 1;
        }
        
        // Process the data here -----------------------------------

            IMU_time = IMU_time + 2;
            
            // Finding DelTheta_rad and DelTheta_Msec2
            
            // DelTheta_Rad
            DelTheta_Rad1 = BodyRateX*GPS_output_rate;
            DelTheta_Rad2 = BodyRateY*GPS_output_rate;
            DelTheta_Rad3 = BodyRateZ*GPS_output_rate;
            
            // DelTheta_Msec
            DelTheta_Msec1 = BodyAccelX*IMU_output_rate;
            DelTheta_Msec2 = BodyAccelY*IMU_output_rate;
            DelTheta_Msec3 = BodyAccelZ*IMU_output_rate;
            
            // Assign the respective variables to arrays for storing and processing
            double IMU_deltaVel[3];
            IMU_deltaVel[0] = DelTheta_Msec2; IMU_deltaVel[1] = DelTheta_Msec1; IMU_deltaVel[2] = -DelTheta_Msec3;
            double IMU_deltaTheta[3];
            IMU_deltaTheta[0] = DelTheta_Rad2; IMU_deltaTheta[1] = DelTheta_Rad1; IMU_deltaTheta[2] = -DelTheta_Rad3;

            // -------------------------

            double X = IMU_deltaVel[0]; double Y = IMU_deltaVel[1]; double Z = IMU_deltaVel[2];
            double acc2 = pow(X, 2) + pow(Y, 2) + pow(Z, 2);
            double acc = sqrt(acc2);
            
            // Call data from GPS -------------------------------------------------------------------
            
            USE_GPS = 1; // GPS is being used
            
            if (USE_GPS > 0.5) {
                // GPS is brought to front of the loop to use any of the GPS data on calculations further down in the loop if called in
                // Read a line of GPS data after every 20 lines from the IMU file (20 Hz) -----------------------------------------------------
                if (Fmat_cnt == 19) { //  Fmat_cnt = rate of IMU output (i.e. IMU - 20 Hz, Fmat_cnt = 19(CPP 0-19))
                    // Read a new line from the GPS file
                    // BRING in Nav Aid Data...
                    KInput.GPS_go = 1;
                    
                    // Call GPS ------------------------------------------
                            // Set correct baud rate (e.g., Adafruit GPS uses 9600)
                    system("stty -F /dev/serial0 9600");

                    ifstream serial("/dev/serial0");
                    if (!serial.is_open()) {
                        cerr << "Error: Could not open /dev/serial0" << endl;
                        KInput.GPS_lat = initial_lat;
                        KInput.GPS_lon = initial_lon;
                        KInput.GPS_alt = 0;
                        // return 34567890; error code
                    }
                    double  GPS_alt, GPS_lat, GPS_lon, GPS_time;

                    string buffer;
    
                    char c;
                    serial.get(c);
                    
                    if (c == '\n') {
                        if (buffer.find("$GPGGA") != string::npos || buffer.find("$GNGGA") != string::npos) {
                            // Optional debug:
                            // cerr << "Raw NMEA: " << buffer << endl;

                            GPSData data = parseGPGGA(buffer);
                                if (data.valid) {
                                    
                                    GPS_alt = data.Altitude;
                                    GPS_lat = data.Latitude;
                                    GPS_lon = data.Longitude;
                                    cout << "{"
                                        << "\"lat\":" << fixed << setprecision(6) << data.Latitude << ","
                                        << "\"lon\":" << data.Longitude << ","
                                        << "\"altitude\":" << data.Altitude
                                        << "}" << endl;

                                    return 0; // ← Remove or comment for continuous output
                                }
                            }
                            buffer.clear();
                        } else if (c != '\r') {
                            buffer += c;
                        }

                    double GPS_ECEF[3], GPS_LLA[3];
        
                    GPS_lat = GPS_lat * (pi / 180);
                    GPS_lon = GPS_lon * -(pi / 180);

                    vel_randWalk = 0.5 / 60;

                    KInput.GPS_time = GPS_time;
                    KInput.GPS_lat = GPS_lat;
                    KInput.GPS_lon = GPS_lon;
                    KInput.GPS_alt = GPS_alt;

                    GPS_times_used++;

                }
            }
            // END of GPS call --------------------------------------------------------------------------------------------------
            
            // ------------------------------------------------------------------------------------------------------------------
            
            if (MainLoopCount > 0) {
                dt = 1e-3 * round((IMU_time - PrevIMU_time) * 1e3);
            }
            else {
                dt = 0.05;  // IMU 20 Hz = .05
            }

            KInput.time = IMU_time;

            PrevIMU_time = IMU_time; // Stores the time of the previous loop to be used for calculations above in the loop



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
            AttitudeMatrix[3] = 0;

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

            if ((Fmat_cnt == 20)){
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

                FilterUpdate = 0;

                if (USE_GPS > 0.5 && KInput.GPS_go > 0.5) {
                    FILTER_RUN = 1;
                    KInput.mode = 1;
                    FilterUpdate = 1;
                }

                if (FILTER_RUN > 0.5) {

                    PosN_delta = (lat - KInput.GPS_lat) * (R_merid - Position_LL[2]);
                    PosE_delta = (lon - KInput.GPS_lon) * (cl * (R_norm - Position_LL[2]));
                    PosD_delta = Position_LL[2] + KInput.GPS_alt;

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


            // Counter of main loop
            MainLoopCount++;
        
        // Calculate GUIDANCE & CONTROL sections
        
        // GUI coordinates
        
        double pos_targetECEF[3], pos_targetLLA[3], pos_cur[3]; 
        
        pos_cur[0] = lat;
        pos_cur[1] = lon;
        pos_cur[2] = altitude;
        
        pos_targetLLA[0] = std::stod(argv[1]); // GUI - latitude
        pos_targetLLA[1] = std::stod(argv[2]); // GUI - longitude
        pos_targetLLA[2] = KInput.GPS_alt;     // GPS - altitude
        
        LLA2ECEF(&pos_targetLLA[3],&pos_targetECEF[3]); // convert from lat,lon,alt to x,y,
        // GUIDANCE 
        double Theta; // radians
        
        guidance_angle(&pos_cur[3],&Velocity_LL[3],&pos_targetECEF[3],Theta);
        
        // CONTROL
        
        PIcontroller(Theta,control,integral);
        
        // Send control to rudder for deflection
        // Initialize pigpio
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio!" << std::endl;
        return 1;
    }

    // Set servo pin as output
    gpioSetMode(SERVO_PIN, PI_OUTPUT);

    std::cout << "Servo control started. Enter angles (0-180)." << std::endl;

    
        double angle;
        
        angle = control * (180/PI) + 70;
        
        if(angle < 25){
            angle = 25;
        } else if (angle > 115){
            angle = 115;
        }
        
        std::cin >> angle;
        

        // Calculate and send pulse width
        int pulseWidth = mapAngleToPulse(angle);
        gpioServo(SERVO_PIN, pulseWidth);
        std::cout << "Moving to " << angle << "° (Pulse: " << pulseWidth << "μs)" << std::endl;

        // Small delay for servo to respond
        gpioDelay(200000);  // 200ms delay (pigpio uses microseconds)
    
        // Determine if destination was reached
        
        double pos_current_mag = sqrt(pow(pos_cur[0],2)+pow(pos_cur[1],2)+pow(pos_cur[2],2));
        double pos_targetECEF_mag = sqrt(pow(pos_targetECEF[0],2)+pow(pos_targetECEF[1],2)+pow(pos_targetECEF[2],2));
        
        if (abs(pos_current_mag - pos_targetECEF_mag) < destination_tolerance){
            destination = 1;
        }
        
            
    } // End of while (destination < 0.5)
 }// End of Main
