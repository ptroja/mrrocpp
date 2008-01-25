#include<stdio.h>
#include<stddef.h>
#include<stdlib.h>
#include<math.h>

#ifndef SQR
#define SQR(a) ((a)*(a))
#endif

extern int alloc_m, alloc_v;

void nrerror(char error_text[])
{
	fprintf(stderr,"Numerical Recipes run-time error...\n");
	fprintf(stderr,"%s\n",error_text);
	fprintf(stderr,"...now exiting to system...\n");
	getchar();
	exit(1);
}

double norm2(double x, double y)
{
	return(sqrt(x*x+y*y));
}

double pythag(double a, double b)
{
	double absa,absb;

	absa=fabs(a); absb=fabs(b);
	if (absa > absb) return absa*sqrt(1.0+SQR(absb/absa));
	else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}
