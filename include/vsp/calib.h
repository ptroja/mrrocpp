#define	SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
#ifndef MAX
  #define MAX(a,b) ((a > b) ? a : b)
#endif
#define SQR(a) ((a)*(a))
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define IMIN(a,b) ((a) < (b) ? (a) : (b))

void nrerror(char error_text[]);
double norm2(double x, double y);
double pythag(double a, double b);

double *vvector(long n);
void free_vector(double *v);
int *ivector(long n);
void free_ivector(int *v);
double **matrix(long m, long n);
void free_matrix(double **m);

//void comp_distortion_oulu(double **xd, double **x);
void comp_distortion_oulu(double **xd, double **x, double *kc);
void compute_homography(double **m, double **M, double **H);
void rodrigues_m(double **R, double *omckk);
void rodrigues_v(double *omckk, double **R, double **dRdom);
//void compute_extrinsic_init(double **x_kk, double **X_kk, double *omckk, double *Tckk, double **Rckk);
void compute_extrinsic_init(double **x_kk, double **X_kk, double *omckk, double *Tckk, double **Rckk, double *fc, double *cc, double *kc);
//void compute_extrinsic_refine(double **x_kk, double **X_kk, double *omckk, double *Tckk, double **Rckk);
void compute_extrinsic_refine(double **x_kk, double **X_kk, double *omckk, double *Tckk, double **Rckk, double *fc, double *cc, double *kc);
//void project_points(double **X_kk, double *omckk, double *Tckk, double **x, double **dxdom, double **dxdT);
void project_points(double **X_kk, double *omckk, double *Tckk, double **x, double **dxdom, double **dxdT, double *fc, double *cc, double *kc);
void rigid_motion(double **X_kk, double *omckk, double *Tckk, double **Y, double **dYdom, double **dYdT);
