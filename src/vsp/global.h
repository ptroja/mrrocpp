
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



