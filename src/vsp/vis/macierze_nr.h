double *vvector(long n);
void free_vector(double *v);
int *ivector(long n);
void free_ivector(int *v);
double **matrix(long m, long n);
void free_matrix(double **ma);






void m_svd(double **a, int m, int n, double *w, double **v);
void m_gaussj(double **a, double **b);
void m_inverse(double **a);
void m_transpose (double **a,double **b);
void m_zeros(double **a);
void v_zeros(double *a);
void m_eye(double **a);

void m_insert_v_c(double **a, double *b, int c);

void m_qrdecomp(double **a, double **r, double **q);
void m_multiply_m(double **a,double **b, double **c);
void m_add_m(double **a,double **b, double **c);
void m_substract_m(double **a,double **b, double **c);
void m_multiply_v(double **a, double *b, double *c);
void m_multiply_s(double **a, double s);
void v_multiply_s(double *a, double s);
void v_multiply_v_m(double *a, double *b, double **c);
void m_copy(double **a, double **b);
void v_copy(double *a, double *b);
void m_copy_ext(double **a, double **b);
void m_substract_v(double **a,double *b);
void v_substract_s(double *a, double s);
void v_substract_v(double *a, double *b);

void m_subvector_r(double **a, int n, double *b);
void m_subvector_c(double **a, int n, double *b);

void m_mean_r(double **a, double *b);
double m_trace(double **a);
double v_mean(double *a);
double v_mean_abs(double *a);
double v_norm(double *a);
double v_dot(double *a, double *b);
void v_cross(double *a, double *b, double *c);

void m_print(double **a);
void v_print(double *a);
