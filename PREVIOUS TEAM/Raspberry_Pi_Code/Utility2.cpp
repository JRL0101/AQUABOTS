//    Utility.c
// All of the utility functions in this file make use of a 'pointer
// to an array of pointers to rows' of matrices, as advocated in
// "Numerical Recipes in C" on pages 20-23.
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include "Utility2.h"
#define _CRT_SECURE_NO_WARNINGS
#define NR_END 1
#define FREE_ARG char*
using namespace std;


// Utility Functions:
//------------------------------------------------------------------
void dmab(double **A, double **B, double **C, int l, int m, int n)
// Function:  dmab(A, B, C, l, m, n)
// Matrix multiply:  C = A * B,
// where A is l-by-m, B is m-by-n, and C is l-by-n.

{
    int i,j,k;
    double sum;

    for (i=0; i<l; i++) {
        for (j=0; j<n; j++) {
            sum=0.0;
            for (k=0; k<m; k++) sum += A[i][k]*B[k][j];
            C[i][j]=sum;
        }
    }
}
//----------------- End of dmab ------------------------------------

void dmabt(double **A, double **B, double **C, int l, int m, int n)
// Function:  dmabt(A, B, C, l, m, n)
// Matrix multiply:  C = A * B^T,  (B^T signifies b transposed)
// where A is l-by-m, B^T is m-by-n (or B is n-by-m), and C is l-by-n.
{
    int i,j,k;
    double sum;

    for (i=0; i<l; i++) {
        for (j=0; j<n; j++) {
            sum=0.0;
            for (k=0; k<m; k++)  sum += A[i][k]*B[j][k];
            C[i][j]=sum;
        }
    }
}
//----------------- End of dmabt ------------------------------------

void dmapb(double **A, double **B, double **C, int m, int n)
// Function:  dmapb(A, b, C, m, n)
// Matrix add (plus):  C= A + B,
// where arrays A, b, and C are m-by-n.
{
    int i,j;

    for (i=0; i<m; i++) {
        for (j=0; j<n; j++)  C[i][j]=A[i][j]+B[i][j];
    }
}
//----------------- End of dmapb ------------------------------------

void dmamb(double **A, double **B, double **C, int m, int n)
// Function:  dmamb(A, B, C, m, n)
// Matrix difference (minus):  C = A - B,
// where arrays A, B, and C are m-by-n.
{
    int i,j;

    for (i=0; i<m; i++) {
        for (j=0; j<n; j++)  C[i][j]=A[i][j]-B[i][j];
    }
}
//----------------- End of dmamb ------------------------------------

void dmscal(double **A, double **B, int m, int n, double c)
// Function:  dmscal(A, B, m, n, c)
// Multiply matrix A by scalar double c:  B = A * c
// where arrays A and B are m-by-n.
{
	int row,col;

	for (row=0;row<m;row++)
	{
		for (col=0;col<n;col++)
			B[row][col] = A[row][col] * c;
	}
}
//----------------- End of dmscal -----------------------------------

void diagonal(double **A, double *B, int m)
// Function:  diagonal(A, B, m)
// Place diagonal of square matrix A into vector B
// where array A is m-by-m and array B is m-by-1
{
	int i;

	for (i=0;i<m;i++)
		B[i] = A[i][i];
}
//----------------- End of diagonal ---------------------------------

void dmcopy(double **A, double **B, int m, int n)
// Function:  dmcopy(A, B, m, n)
// Copy m-by-n matrix A into m-by-n matrix B.
{
    int i,j;

    for (i=0; i<m; i++) {
        for (j=0; j<n; j++)  B[i][j]=A[i][j];
    }
}
//----------------- End of dmcopy ------------------------------------

void vezero(double *A, int m)
// Function:  vezero(A, m)
// Fill m-by-1 vector A with zeroes.
{
	int i;

	for (i=0;i<m;i++)
		A[i] = 0;
}
//----------------- End of vezero ------------------------------------

void iezero(int *indx, int m)
// Function:  iezero(A, m)
// Fill m-by-1 vector A with zeroes.
{
	int i;

	for (i=0;i<m;i++)
		indx[i] = 0;
}
//----------------- End of iezero ------------------------------------

void dmzero(double **A, int m, int n)
// Function:  dmzero(A, m, n)
// Fill m-by-n matrix A with zeroes.
{
    int i,j;

    for (i=0; i<m; i++) {
        for (j=0; j<n; j++)  A[i][j]=0.0;
    }
}
//----------------- End of dmzero ------------------------------------

void dmeye(double **A, int m, int n)
// Function:  dmeye(A, m, n)
// Set m-by-n matrix A to zeros except for ones on the main diagonal.
// (When m = n, A becomes the identity matrix.)
{
    int i,j;

    for (i=0; i<m; i++) {
        for (j=0; j<n; j++) {
            if (i == j) A[i][i]=1.0;
            else A[i][j]=0.0;
        }
    }
}
//----------------- End of dmeye -------------------------------------

void dmav(double **A, double *v, double *u, int m, int n)
// Function:  dmav(A, v, u, m, n)
// Matrix-vector multiply:  u = A * v,
// where A is m-by-n matrix, v is n-by-1 vector, and u is m-by-1 vector.

{
    int i,k;
    double sum;

    for (i=0; i<m; i++) {
        sum=0.0;
        for (k=0; k<n; k++)  sum += A[i][k]*v[k];
        u[i]=sum;
    }
}
//----------------- End of dmav ------------------------------------

void vecsub(double *A, double *B, double *C, int m)
// Function:  vecsub(A, B, C, m)
// Vector minus (subtract):  C = A - B,
// where arrays A, B, and C are m-by-1.

{
	int i;

	for (i=0;i<m;i++)
		C[i] = A[i] - B[i];
}
//----------------- End of vecsub ----------------------------------

void vecdiv(double *A, double *B, double *C, int m)
// Function:  vecdiv(A, B, C, m)
// Vector divide:  C = A / B,
// where arrays A, B, and C are m-by-1.

{
	int i;

	for (i=0;i<m;i++)
		C[i] = A[i] / B[i];
}
//----------------- End of vecdiv ----------------------------------

int dmaib(double **A, double **B, double **X, double **Temp, double *c, double *dum_vv, int *indx, int m, int n)
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

    if (! invalid) {
        for (i=0; i<n; i++) {
            for (j=0; j<m; j++) {
                c[j] = B[j][i];
            }
            lubksb(Temp, m, indx, c);
            for (j=0; j<m; j++) {
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

int ludcmp(double **a, double *vv, int n, int *indx, double *d)
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
    int i,imax,j,k;
    double big,dum,sum,temp;

    // Allocate n-element double-precision vector v for temp. storage:
    //double *vv;  //Removed 9-19-05
    //vv=dvector(0, n-1); //Removed 9-19-05

    *d = 1.0;
    for (i=0;i<n;i++) {
        big=0.0;
        for (j=0;j<n;j++)
            if ((temp=fabs(a[i][j])) > big) big=temp;
        if (big == 0.0) {
            //printf("Singular matrix in the ludcmp function.\n");
            return 1;
        }
        vv[i]=1.0/big;
    }
    for (j=0;j<n;j++) {
        for (i=0;i<j;i++) {
            sum=a[i][j];
            for (k=0;k<i;k++) sum -= a[i][k]*a[k][j];
            a[i][j]=sum;
        }
        big=0.0;
        for (i=j;i<n;i++) {
            sum=a[i][j];
            for (k=0;k<j;k++)
                sum -= a[i][k]*a[k][j];

            a[i][j]=sum;
            if ( (dum=vv[i]*fabs(sum)) >= big) {
                big=dum;
                imax=i;
            }
        }
        if (j != imax) {
            for (k=0;k<n;k++) {
                dum=a[imax][k];
                a[imax][k]=a[j][k];
                a[j][k]=dum;
            }
            *d = -(*d);
            vv[imax]=vv[j];
        }
        indx[j]=imax;
        if (a[j][j] == 0.0) a[j][j]=1.0e-20;
        if (j != n-1) {
            dum=1.0/(a[j][j]);
            for (i=j+1;i<n;i++) a[i][j] *= dum;
        }
    }
    //free_dvector(vv, 0, n-1); //Removed 9-19-05
    return 0;
}

//----------------- end of ludcmp ----------------------------------------

void lubksb(double **a, int n, int *indx, double *b)
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
    int i,ii=-1,ip,j;
    double sum;

    for (i=0;i<n;i++) {
        ip=indx[i];
        sum=b[ip];
        b[ip]=b[i];
        if (ii>=0)
            for (j=ii;j<=i;j++) sum -= a[i][j]*b[j];
        else if (sum) ii=i;
        b[i]=sum;
    }
    for (i=n-1;i>=0;i--) {
        sum=b[i];
        for (j=i+1;j<n;j++) sum -= a[i][j]*b[j];
        b[i]=sum/a[i][i];
    }
}
//----------------- end of lubksb ---------------------------------

int *ivector(int nl, int nh)

/* Allocate an int vector with subscript range v[nl..nh] */
{
    int *v;
   
    v=(int *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(int)));
    if (!v) {
        printf("Allocation failure in ivector()\n");
        exit(1);
    }
    return v-nl+NR_END;
}
//----------------- End of ivector ---------------------------------
   
double *dvector(int nl, int nh)
/* Allocate a double vector with subscript range v[nl..nh] */
{
    double *v;
   
    v=(double *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(double)));
    if (!v) {
        printf("Allocation failure in dvector()\n");
        exit(1);
    }
    return v-nl+NR_END;
}
//----------------- End of dvector ---------------------------------
   
double **dmatrix(int nrl, int nrh, int ncl, int nch)
/* Allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
    int i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
    double **m;
   
    /* allocate pointers to rows */
    m=(double **) malloc((size_t)((nrow+NR_END)*sizeof(double*)));
    if (!m) {
        printf("Allocation failure 1 in dmatrix()");
        exit(1);
    }
    m += NR_END;
    m -= nrl;
   
    /* allocate rows and set pointers to them */
    m[nrl]=(double *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(double)));
    if (!m[nrl]) {
        printf("Allocation failure 2 in dmatrix()\n");
        exit(1);
    }
    m[nrl] += NR_END;
    m[nrl] -= ncl;
   
    for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;
   
    /* return pointer to array of pointers to rows */
    return m;
}
//------------- End of dmatrix ----------------------------------------------

double **convert_dmatrix(double *a, int nrl, int nrh, int ncl, int nch)
/* Allocate a double matrix m[nrl..nrh][ncl..nch] that points to the matrix
declared in the standard C manner as a[nrow][ncol], where nrow=nrh-nrl+1
and ncol=nch-ncl+1. The routine should be called with the address
&a[0][0] as the first argument. */

// This function does NOT allocate new memory for the data. It just sets up
// new pointers to the data.
{
    int i,j,nrow=nrh-nrl+1,ncol=nch-ncl+1;

    double **m;
   
    /* allocate pointers to rows */
    m=(double **) malloc((size_t) ((nrow+NR_END)*sizeof(double*)));
    if (!m) {
        printf("Allocation failure in convert_matrix().\n");
        exit(1);
    }
    m += NR_END;
    m -= nrl;
   
    /* set pointers to rows */
    m[nrl]=a-ncl;
    for(i=1,j=nrl+1;i<nrow;i++,j++) m[j]=m[j-1]+ncol;
    /* return pointer to array of pointers to rows */
    return m;
}
//-------------- end of convert_dmatrix ------------------------------------
   
void free_ivector(int *v, int nl)
/* Free an int vector allocated with ivector() */
{
    free((FREE_ARG) (v+nl-NR_END));
}
//------------- End of free_ivector -----------------------------------------
   
void free_dvector(double *v, int nl)
/* Free a double vector allocated with dvector() */
{
    free((FREE_ARG) (v+nl-NR_END));
}
//------------- End of free_dvector -----------------------------------------

void free_dmatrix(double **m, int nrl, int ncl)
//void free_dmatrix(double **m, int nrl, int nrh, int ncl, int nch)
/* Free a double matrix allocated by dmatrix() */
{
    free((FREE_ARG) (m[nrl]+ncl-NR_END));
    free((FREE_ARG) (m+nrl-NR_END));
}
//------------- End of free_dmatrix -----------------------------------------   

void free_convert_dmatrix(double **b, int nrl)
/* Free a matrix allocated by convert_dmatrix().  It does NOT affect the
   original matrix that was input to convert_dmatrix. */
{
    free((FREE_ARG) (b+nrl-NR_END));
}
//-------------- End of free_convert_matrix ---------------------------------

void MATH_UTILITY_TEST(int in, int success_code)
/* This will run through each of the routines in this class; the success_code will
   be returned to the calling routine as '1' for completion or will be the error code
   from the utilities below  */
{
	double **Temp1, **Temp2, **Temp3, **Temp4,  **Temp5, **_X,
		   *rvect0, *rvect1, *rvect2, *rvect3, c, X[3][3];
	c=3.141;
	int *ivect1, *ivect2, m, n, j, i, k;
	m=3;
	n=3;
	j=1;
	
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

	for (i=0;i<3;i++)
	{
			for (k=0;k<3;k++)
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

	_X  = convert_dmatrix(&X[0][0],  0,2, 0,2);

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
	cout << "Reference Matrix-1    Row-1: " << Temp1[0][0] << " " << Temp1[0][1] << " " << Temp1[0][2] <<   endl;
	cout << "                      Row-2: " << Temp1[1][0] << " " << Temp1[1][1] << " " << Temp1[1][2] <<   endl;
	cout << "                      Row-3: " << Temp1[2][0] << " " << Temp1[2][1] << " " << Temp1[2][2] <<   endl;
	cout << "Reference Matrix-2    Row-1: " << Temp2[0][0] << " " << Temp2[0][1] << " " << Temp2[0][2] <<   endl;
	cout << "                      Row-2: " << Temp2[1][0] << " " << Temp2[1][1] << " " << Temp2[1][2] <<   endl;
	cout << "                      Row-3: " << Temp2[2][0] << " " << Temp2[2][1] << " " << Temp2[2][2] <<   endl;
	cout << "Reference Vector-1    Row-1: " << rvect0[0] <<   endl;
	cout << "                      Row-2: " << rvect0[1] <<   endl;
	cout << "                      Row-3: " << rvect0[2] <<   endl;
	cout << "Reference Vector-2    Row-1: " << rvect1[0] <<   endl;
	cout << "                      Row-2: " << rvect1[1] <<   endl;
	cout << "                      Row-3: " << rvect1[2] <<   endl;
	cout << "Reference Vector-3    Row-1: " << ivect1[0] <<   endl;
	cout << "(an integer vector)   Row-2: " << ivect1[1] <<   endl;
	cout << "                      Row-3: " << ivect1[2] <<   endl;
	cout << "    " << endl;

	cout << "    " << endl;
	cout << "Another copy of Ref   Row-1: " << X[0][0] << " " << X[0][1] << " " << X[0][2] <<   endl;
	cout << "Matrix-1 using the    Row-2: " << X[1][0] << " " << X[1][1] << " " << X[1][2] <<   endl;
	cout << "convert_dmatrix       Row-3: " << X[2][0] << " " << X[2][1] << " " << X[2][2] <<   endl;
	cout << "    " << endl;

	dmzero(Temp3, m, n);
	cout << "Math Test: dmzero -> zeros the matrix,   Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] <<   endl;
	cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] <<   endl;
	cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] <<   endl;
	cout << "    " << endl;

	dmeye(Temp3, m, n);
	cout << "Math Test: dmeye -> the Identity matrix, Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] <<   endl;
	cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] <<   endl;
	cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] <<   endl;
	cout << "    " << endl;

	dmscal(Temp3, Temp4, m, n, c);
	cout << "Math Test: dmscal -> Matrix x Scalar,    Row-1: " << Temp4[0][0] << " " << Temp4[0][1] << " " << Temp4[0][2] <<   endl;
	cout << "           the matrix was the identity,  Row-2: " << Temp4[1][0] << " " << Temp4[1][1] << " " << Temp4[1][2] <<   endl;
	cout << "           and the scalr is pi.          Row-3: " << Temp4[2][0] << " " << Temp4[2][1] << " " << Temp4[2][2] <<   endl;
	cout << "    " << endl;

	dmapb(Temp1, Temp2, Temp3, m, n);
	cout << "Math Test: dmapb -> Ref-1 + Ref-2,       Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] <<   endl;
	cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] <<   endl;
	cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] <<   endl;
	cout << "    " << endl;

    dmab(Temp1, Temp2, Temp3, m, m, n);
	cout << "Math Test: dmap -> Ref-1 * Ref-2,        Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] <<   endl;
	cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] <<   endl;
	cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] <<   endl;
	cout << "    " << endl;

    dmabt(Temp1, Temp2, Temp3, m, m, n);
	cout << "Math Test: dmapt -> Ref-1 * Ref-2^T,     Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] <<   endl;
	cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] <<   endl;
	cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] <<   endl;
	cout << "    " << endl;

    dmcopy(Temp1, Temp3, m, n);
	cout << "Math Test: dmcopy -> copy of Ref-1,      Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] <<   endl;
	cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] <<   endl;
	cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] <<   endl;
	cout << "    " << endl;

	dmeye(Temp3, m, n); // also tests ludcmp and lubksb
    j= dmaib(Temp1, Temp3, Temp4, Temp5, rvect2, rvect3, ivect2, m, n);
	cout << "Math Test: dmaib -> Matrix Inverse,      Row-1: " << Temp4[0][0] << " " << Temp4[0][1] << " " << Temp4[0][2] <<   endl;
	cout << "           the matrix was Reference-1.   Row-2: " << Temp4[1][0] << " " << Temp4[1][1] << " " << Temp4[1][2] <<   endl;
	cout << "                                         Row-3: " << Temp4[2][0] << " " << Temp4[2][1] << " " << Temp4[2][2] <<   endl;
	cout << "    " << endl;

    dmamb(Temp1, Temp2, Temp3, m, n);
	cout << "Math Test: dmamb -> Ref-1 - Ref-2,       Row-1: " << Temp3[0][0] << " " << Temp3[0][1] << " " << Temp3[0][2] <<   endl;
	cout << "                                         Row-2: " << Temp3[1][0] << " " << Temp3[1][1] << " " << Temp3[1][2] <<   endl;
	cout << "                                         Row-3: " << Temp3[2][0] << " " << Temp3[2][1] << " " << Temp3[2][2] <<   endl;
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

