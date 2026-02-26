// Utility.h
#define NR_END 1
#define FREE_ARG char*
// Declarations:
void dmab(double **, double **, double **, int, int, int);
void dmabt(double **, double **, double **, int, int, int);
void dmapb(double **, double **, double **, int, int);
void dmamb(double **, double **, double **, int, int);
void dmcopy(double **, double **, int, int);
void dmzero(double **, int, int);
void dmeye(double **, int, int);
void dmav(double **, double *, double *, int, int);
int dmaib(double **, double **, double **, double **, double *, double *, int *, int, int);
int ludcmp(double **, double *, int, int *, double *);
void lubksb(double **, int, int *, double *);
int *ivector(int, int);
double *dvector(int, int);
double **dmatrix(int, int, int, int);
double **convert_dmatrix(double *, int, int, int, int);
void free_ivector(int *, int);
void free_dvector(double *, int);
void free_dmatrix(double **, int, int);
//void free_dmatrix(double **, int, int, int, int);
void free_convert_dmatrix(double **, int);
void dmscal(double **, double **, int, int, double);  // Roy Jr
void diagonal(double **, double *, int);
void vezero(double *, int);
void iezero(int *, int);
void vecsub(double *, double *, double *, int);
void vecdiv(double *, double *, double *, int);
void MATH_UTILITY_TEST(int, int);





