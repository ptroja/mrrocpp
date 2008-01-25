#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "vsp/calib.h"
#include "vsp/macierze_nr.h"

extern int alloc_m, alloc_v;


double *vvector(long n)
/*************************************************************************
*   This routine allocates space for a double vector using malloc and sets
*   the offset so that the array can be referenced from ilow to ihigh.
*
*   ALGORITHM:  taken from Numerical Recipes in C,Press et.al.,
*               Cambridge Press 1990,pg705.
*   INPUTS: ilow - lowest value in index range
*           ihigh - highest value in index range
*   OUTPUTS: none
*   RETURNS: pointer to allocated space with the proper offset
*
**************************************************************************/
{
	double *v;
	v=(double *)malloc((n+1)*sizeof(double));
	if (!v) nrerror("allocation failure in vector()");
	*v=(double)n;
	alloc_v++;
	return v;
}

void free_vector(double *v)
/* free a double vector allocated with vector() */
{
	free(v);
	alloc_v--;
}

int *ivector(long n)
{
	int *v;
	v=(int *)malloc((n+1)*sizeof(int));
	if (!v) nrerror("allocation failure in ivector()");
	*v=(int)n;
	return v;
}

void free_ivector(int *v)
{
	free(v);
}

double **matrix(long m, long n)
{
	long i;
	double **ma;

	/* allocate pointers to rows */
	ma=(double **) malloc((m+1)*sizeof(double*));
	if (!ma) nrerror("allocation failure 1 in dmatrix()");
	
	/*
	printf("wektor: ");
	for(i=0; i<m+1; i++)
		printf("%d ",ma[i]);
	printf("\n");	
	*/
	for(i=0; i<m+1; i++)
	{
		//printf("m:%d n:%d aktualny wiersz:%d\n",m,n,i);
		ma[i]=(double *)malloc((n+1)*sizeof(double));
		//if (!ma[i]) nrerror("allocation failure 2 in dmatrix()");
		}

	/* return pointer to array of pointers to rows */
	ma[1][0]=m; ma[0][1]=n;
	alloc_m++;
	//printf("to byla macierz nr %d\n",alloc_m);
	return ma;
}

void free_matrix(double **ma)
/* free a double matrix allocated by matrix() */
{
	free( ma[0]);
	free(ma);
	alloc_m--;
}

void m_svd(double **a, int m, int n, double *w, double **v)
{
	int flag,i,its,j,jj,k,l,nm;
	double anorm,c,f,g,h,s,scale,x,y,z, *rv1; 

	rv1=vvector(n);


	g=scale=anorm=0.0;
	for (i=1;i<=n;i++) {
		l=i+1; rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i <= m) {
			for (k=i;k<=m;k++) scale += fabs(a[k][i]);
			if (scale) {
				for (k=i;k<=m;k++) {
					a[k][i] /= scale; s += a[k][i]*a[k][i];
				}
				f=a[i][i]; g = -SIGN(sqrt(s),f);
				h=f*g-s; a[i][i]=f-g;
				for (j=l;j<=n;j++) {
					for (s=0.0,k=i;k<=m;k++) s += a[k][i]*a[k][j];
					f=s/h;
					for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
				}
				for (k=i;k<=m;k++) a[k][i] *= scale;
			}
		}
		w[i]=scale *g; g=s=scale=0.0;
		if (i <= m && i != n) {
			for (k=l;k<=n;k++) scale += fabs(a[i][k]);
			if (scale) {
				for (k=l;k<=n;k++) {
					a[i][k] /= scale; s += a[i][k]*a[i][k];
				}
				f=a[i][l]; g = -SIGN(sqrt(s),f);
				h=f*g-s; a[i][l]=f-g;
				for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
				for (j=l;j<=m;j++) {
					for (s=0.0,k=l;k<=n;k++) s += a[j][k]*a[i][k];
					for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
				}
				for (k=l;k<=n;k++) a[i][k] *= scale;
			}
		}
		anorm=MAX(anorm,(fabs(w[i])+fabs(rv1[i])));
	}
	for (i=n;i>=1;i--) {
		if (i < n) {
			if (g) {
				for (j=l;j<=n;j++)
					v[j][i]=(a[i][j]/a[i][l])/g;
				for (j=l;j<=n;j++) {
					for (s=0.0,k=l;k<=n;k++) s += a[i][k]*v[k][j];
					for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
				}
			}
			for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
		}
		v[i][i]=1.0; g=rv1[i];
		l=i;
	}
	for (i=IMIN(m,n);i>=1;i--) {
		l=i+1; g=w[i];
		for (j=l;j<=n;j++) a[i][j]=0.0;
		if (g) {
			g=1.0/g;
			for (j=l;j<=n;j++) {
				for (s=0.0,k=l;k<=m;k++) s += a[k][i]*a[k][j];
				f=(s/a[i][i])*g;
				for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
			}
			for (j=i;j<=m;j++) a[j][i] *= g;
		} else for (j=i;j<=m;j++) a[j][i]=0.0;
		++a[i][i];
	}
	for (k=n;k>=1;k--) {
		for (its=1;its<=30;its++) {
			flag=1;
			for (l=k;l>=1;l--) {
				nm=l-1;
				if ((double)(fabs(rv1[l])+anorm) == anorm) {
					flag=0;
					break;
				}
				if ((double)(fabs(w[nm])+anorm) == anorm) break;
			}
			if (flag) {
				c=0.0; s=1.0;
				for (i=l;i<=k;i++) {
					f=s*rv1[i];	rv1[i]=c*rv1[i];
					if ((double)(fabs(f)+anorm) == anorm) break;
					g=w[i];	h=pythag(f,g);
					w[i]=h;	h=1.0/h;
					c=g*h; s = -f*h;
					for (j=1;j<=m;j++) {
						y=a[j][nm]; z=a[j][i];
						a[j][nm]=y*c+z*s;
						a[j][i]=z*c-y*s;
					}
				}
			}
			z=w[k];
			if (l == k) {
				if (z < 0.0) {
					w[k] = -z;
					for (j=1;j<=n;j++) v[j][k] = -v[j][k];
				}
				break;
			}
			if (its == 30) printf("no convergence in 30 svdcmp iterations");
			x=w[l]; nm=k-1;
			y=w[nm]; g=rv1[nm];
			h=rv1[k]; 
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
			g=pythag(f,1.0);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			c=s=1.0;
			for (j=l;j<=nm;j++) {
				i=j+1; g=rv1[i];
				y=w[i]; h=s*g;
				g=c*g; z=pythag(f,h);
				rv1[j]=z; c=f/z;
				s=h/z; f=x*c+g*s;
				g = g*c-x*s; h=y*s;
				y *= c;
				for (jj=1;jj<=n;jj++) {
					x=v[jj][j];	z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=pythag(f,h); w[j]=z;
				if (z) {
					z=1.0/z; c=f*z;
					s=h*z;
				}
				f=c*g+s*y; x=c*y-s*g;
				for (jj=1;jj<=m;jj++) {
					y=a[jj][j];	z=a[jj][i];
					a[jj][j]=y*c+z*s;
					a[jj][i]=z*c-y*s;
				}
			}
			rv1[l]=0.0;	rv1[k]=f;
			w[k]=x;
		}
	}
	free_vector(rv1);
}


void m_gaussj(double **a, double **b)
/*------------------------------------------------------
* a = coefficient matrix 
* b = solution	matrix	  
* Notes:  1) on exit, a contains the inverted matrix 
*		   2) on entry,	b contains the lhs vector, and
*			  on exit, it contains the solution	vector
--------------------------------------------------------*/
{
	int	*indxc,*indxr,*ipiv;
	int	i,icol,irow,j,k,l,ll;
	double big,dum,pivinv,temp;
	int n,m;

	n=(int)a[0][1];
	m=(int)b[0][1];

	indxc=ivector(n);
	indxr=ivector(n);
	ipiv=ivector(n);
	for	(j=1;j<=n;j++) ipiv[j]=0;
	for	(i=1;i<=n;i++) {
		big=0.0;
		for	(j=1;j<=n;j++)
			if (ipiv[j]	!= 1)
				for	(k=1;k<=n;k++) {
					if (ipiv[k]	== 0) {
						if (fabs(a[j][k]) >= big) {
							big=fabs(a[j][k]);
							irow=j;
							icol=k;
						}
					} else if (ipiv[k] > 1)	nrerror("GAUSSJ: Singular Matrix-1");
				}
				++(ipiv[icol]);
				if (irow !=	icol) {
					for	(l=1;l<=n;l++) SWAP(a[irow][l],a[icol][l])
						for	(l=1;l<=m;l++) SWAP(b[irow][l],b[icol][l])
				}
				indxr[i]=irow;
				indxc[i]=icol;
				if (a[icol][icol] == 0.0) nrerror("GAUSSJ: Singular	Matrix-2");
				pivinv=1.0/a[icol][icol];
				a[icol][icol]=1.0;
				for	(l=1;l<=n;l++) a[icol][l] *= pivinv;
				for	(l=1;l<=m;l++) b[icol][l] *= pivinv;
				for	(ll=1;ll<=n;ll++)
					if (ll != icol)	{
						dum=a[ll][icol];
						a[ll][icol]=0.0;
						for	(l=1;l<=n;l++) a[ll][l]	-= a[icol][l]*dum;
						for	(l=1;l<=m;l++) b[ll][l]	-= b[icol][l]*dum;
					}
	}
	for	(l=n;l>=1;l--) {
		if (indxr[l] !=	indxc[l])
			for	(k=1;k<=n;k++)
				SWAP(a[k][indxr[l]],a[k][indxc[l]]);
	}
	free_ivector(ipiv);
	free_ivector(indxr);
	free_ivector(indxc);
}

void m_inverse(double **a)
{
	int	*indxc,*indxr,*ipiv;
	int	i,icol,irow,j,k,l,ll;
	double big,dum,pivinv,temp;
	int n;

	n=(int)a[0][1];

	indxc=ivector(n);
	indxr=ivector(n);
	ipiv=ivector(n);
	for	(j=1;j<=n;j++) ipiv[j]=0;
	for	(i=1;i<=n;i++) {
		big=0.0;
		for	(j=1;j<=n;j++)
			if (ipiv[j]	!= 1)
				for	(k=1;k<=n;k++) {
					if (ipiv[k]	== 0) {
						if (fabs(a[j][k]) >= big) {
							big=fabs(a[j][k]);
							irow=j;
							icol=k;
						}
					} else if (ipiv[k] > 1)	nrerror("GAUSSJ: Singular Matrix-1");
				}
				++(ipiv[icol]);
				if (irow !=	icol) {
					for	(l=1;l<=n;l++) SWAP(a[irow][l],a[icol][l])
				}
				indxr[i]=irow;
				indxc[i]=icol;
				if (a[icol][icol] == 0.0) nrerror("GAUSSJ: Singular	Matrix-2");
				pivinv=1.0/a[icol][icol];
				a[icol][icol]=1.0;
				for	(l=1;l<=n;l++) a[icol][l] *= pivinv;
				for	(ll=1;ll<=n;ll++)
					if (ll != icol)	{
						dum=a[ll][icol];
						a[ll][icol]=0.0;
						for	(l=1;l<=n;l++) a[ll][l]	-= a[icol][l]*dum;
					}
	}
	for	(l=n;l>=1;l--) {
		if (indxr[l] !=	indxc[l])
			for	(k=1;k<=n;k++)
				SWAP(a[k][indxr[l]],a[k][indxc[l]]);
	}
	free_ivector(ipiv);
	free_ivector(indxr);
	free_ivector(indxc);
}

void m_transpose(double **a, double **b)
{	//transpozycja
	if (a[0][1]!=b[1][0] || a[1][0]!=b[0][1]) nrerror("m_transpose: matrices do not mach");
	for (int i=1; i<=a[1][0]; i++ ) 
		for (int j=1; j<=a[0][1]; j++ ) 
			b[j][i] = a[i][j];

}

void m_zeros(double **a)
{	//zeruje macierz
	for (int i=1; i<=a[1][0]; i++ ) 
		for (int j=1; j<=a[0][1]; j++ ) 
			a[i][j]=0;

}

void v_zeros(double *a)
{	//zeruje wektor
	for (int i=1; i<=a[0]; i++ ) 
		a[i]=0;
}

void m_eye(double **a)
{	//wstawia jedynki na glownej przekatnej, reszta to zera
	for (int i=1; i<=a[1][0]; i++ ) 
		for (int j=1; j<=a[0][1]; j++ ) 
			if(i==j)  a[i][j]=1;
			else
				a[i][j]=0;
}

void m_insert_v_c(double **a, double *b, int c)
{
	/*
	Funkcja wstawia (w miejscu) do macierzy a wektor b pionowo na miejscu c
	*/
	if (a[0][1]!=b[0]) nrerror("m_insert_v_c: matrix and vector do not mach");
	for (int i=1; i<=b[0]; i++ )
		a[i][c]=b[i];
}

void m_qrdecomp(double **a, double **r, double **q)
{	//dekompozycja QR
	int i,j,k,m,n;
	double *Rdiag, nrm, s;

	m=(int)a[1][0];
	n=(int)a[0][1];

	Rdiag=vvector(n);
	// Main loop.
	for (k = 1; k <= n; k++)
	{
		// Compute 2-norm of k-th column without under/overflow.
		nrm = 0.0;
		for (i = k; i <= m; i++)
			nrm = pythag(nrm, a[i][k]);
		

		if (nrm != 0.0)
		{
			// Form k-th Householder vector.
			if (a[k][k] < 0)
				nrm = - nrm;
			for (i = k; i <= m; i++)
				a[i][k] = a[i][k]/nrm;
			
			a[k][k] = a[k][k]+1.0;

			// Apply transformation to remaining columns.
			for (j = k + 1; j <= n; j++)
			{
				s = 0.0;
				for (i = k; i <= m; i++)
					s += a[i][k] * a[i][j];
				s = (- s) / a[k][k];
				for (i = k; i <= m; i++)
					a[i][j] += s * a[i][k];
			}
		}
		Rdiag[k] = - nrm;
	}
	for (int i = 1; i <= m; i++)
		for (int j = 1; j <= n; j++)
			{
			if (i < j)
				r[i][j] = a[i][j];
			else
			if (i == j)
				r[i][j] = Rdiag[i];
			else
				r[i][j] = 0.0;
			}
	

	for (int k = m ; k >= 1; k--)
	{
		for (int i = 1; i <= m; i++)
			q[i][k] = 0.0;
		q[k][k] = 1.0;
		for (int j = k; j <= m; j++)
			if (a[k][k] != 0 && k<m)
			{
				double s = 0.0;
				for (int i = k; i <= m; i++)
					s += a[i][k] * q[i][j];
				s = (- s) / a[k][k];
				for (int i = k; i <= m; i++)
					q[i][j] += s * a[i][k];
				
			}
	}
	free_vector(Rdiag);
}

void m_multiply_m(double **a, double **b, double **c)
{	//mnozy macierze

	
if (a[0][1]!=b[1][0]) nrerror("m_multiply_m: input matrices do not match");
//printf("A(%.1fx%.1f)*B(%.1fx%.1f)=C(%.1fx%.1f)\n",a[1][0],a[0][1],b[1][0],b[0][1],c[1][0],c[0][1]);
if (a[1][0]!=c[1][0] || b[0][1]!=c[0][1]) nrerror("m_multiply_m: output matrix does not fit");
	
	int m=(int)a[1][0];
	int n=(int)b[0][1];
	//c[1][0]=m;
	//c[0][1]=n;

	for(int i = 1; i <= m; i++ )  /* Initialization */
		for(int j = 1; j <= n; j++ )
			c[ i ][ j ] = 0.0;

	for(int i = 1; i <=m ; i++ )
		for(int j = 1; j <= n; j++ )
			for(int k = 1; k <= b[1][0]; k++ )
				c[ i ][ j ] += a[ i ][ k ] * b[ k ][ j ];
}

void m_add_m(double **a,double **b, double **c)
{	//dodaje macierze
	
	//printf("A(%.1fx%.1f)+B(%.1fx%.1f)=C(%.1fx%.1f)\n",a[1][0],a[0][1],b[1][0],b[0][1],c[1][0],c[0][1]);
	if (a[1][0]!=b[1][0]&&a[1][0]!=c[1][0] || a[0][1]!=b[0][1]&&a[0][1]!=c[0][1]) nrerror("m_add_m: matrices do not match");

	int m=(int)a[1][0];
	int n=(int)b[0][1];
	//c[1][0]=m;
	//c[0][1]=n;

	for(int i = 1; i <= m; i++ )
		for(int j = 1; j <= n; j++ )
			c[ i ][ j ] = a[i][j]+b[i][j];

}

void m_substract_m(double **a,double **b, double **c)
{
	if (a[1][0]!=b[1][0] || a[0][1]!=b[0][1]) nrerror("m_substract_m: input matrices do not match");
	if (a[1][0]!=c[1][0] || a[0][1]!=c[0][1]) nrerror("m_substract_m: output matrix does not fit");

	int m=(int)a[1][0];
	int n=(int)b[0][1];
	//c[1][0]=m;
	//c[0][1]=n;

	for(int i = 1; i <= m; i++ )
		for(int j = 1; j <= n; j++ )
			c[ i ][ j ] = a[i][j]-b[i][j];

}
void m_multiply_v(double **a, double *b, double *c)
{
	if (a[1][0]!=b[0]) nrerror("m_multiply_v: input matrix and vector do not match");
	if (a[1][0]!=c[0]) nrerror("m_multiply_v: output vector does not fit");
	int m=(int)a[1][0];
	int n=(int)b[0];
	//c[0]=m;

	for(int i = 1; i <= m; i++ )  /* Initialization */
		c[ i ] = 0.0;

	for(int i = 1; i <=m ; i++ )
 		for(int j = 1; j <= n; j++ )
			c[ i ] += a[ i ][ j ] * b[ j ];

}

void m_copy(double **a, double **b)
{
	if (a[0][1]!=b[0][1] &&  a[1][0]!=b[1][0]) nrerror("nieprawid³owe rozmiary macierzy do kopiowania");

	for(int i=1;i<=a[1][0];i++)
		for(int j=1;j<=a[0][1];j++)
			b[i][j]=a[i][j];
}

void m_copy_ext(double **a, double **b)
{
	/*
	Kopiuje mniejsza macierz a do wiekszej b, puste miejsca uzupelnia jedynkami
	*/
	for(int i=1;i<=b[1][0];i++)
		for(int j=1;j<=b[0][1];j++)
			if(i>a[1][0] || j>a[0][1])
				b[i][j]=1;
			else
				b[i][j]=a[i][j];
}

void v_copy(double *a, double *b)
{
	/*
	Kopiuje wektory
	*/
	for(int i=1;i<=a[0];i++)
		b[i]=a[i];
}

void m_multiply_s(double **a, double s)
{
	/*
	Funkcja mno¿y w miejscu macierz przez skalar
	*/
	for(int i=1;i<=a[1][0];i++)
		for(int j=1;j<=a[0][1];j++)
			a[i][j]=a[i][j]*s;
}

void v_multiply_s(double *a, double s)
{
	/*
	Funkcja mno¿y w miejscu wektor przez skalar
	*/
	for(int i=1;i<=a[0];i++)
		a[i]=a[i]*s;
}


void v_multiply_v_m(double *a, double *b, double **c)
{
	/*
	Funkcja mno¿y dwa wektory traktuj¹c drugi jako transponowany, czyli a*b'
	w wyniku dostajemy macierz

	*/
	for(int i=1;i<=a[0];i++)
		for(int j=1;j<=b[0];j++)
		{
			c[i][j]=a[i]*b[j];
		}
}
void m_substract_v(double **a,double *b)
{
	/*
	Funkcja odejmuje w miejscu od macierzy wektor w ten sposob, ze od wszystkich elementow 
	wiersza macierzy odejmowany jest jeden element odpowiadajacego mu wiersza wektora
	*/
	for(int i=1;i<=a[1][0];i++)
		for(int j=1;j<=a[0][1];j++)
			a[i][j]=a[i][j]-b[i];

}

void v_substract_s(double *a, double s)
{
	/*
	Funkcja odejmuje skalar od wektora w miejscu
	*/
	for(int i=1;i<=a[0];i++)
		a[i]=a[i]-s;
}

void v_substract_v(double *a, double *b)
{
	/*
	Odejmuje wektory
	*/
	for(int i=1;i<=a[0];i++)
		a[i]=a[i]-b[i];
}


void m_subvector_r(double **a, int n, double *b)
{
	/*
	Funkcja wycina wektor z macierzy z wiersza o numerze n
	*/

	b[0]=a[0][1];
	for(int i=1;i<=a[0][1];i++)
		b[i]=a[n][i];
}

void m_subvector_c(double **a, int n, double *b)
{
	/*
	Funkcja wycina wektor z macierzy z kolumny o numerze n
	*/
	b[0]=a[1][0];
	for(int i=1;i<=a[1][0];i++)
		b[i]=a[i][n];
}

void m_mean_r(double **a, double *b){
	/*
	Funkcja oblicza wektor zawierajacy srednie z wierszy danej macierzy
	*/
	for(int i=1;i<=a[1][0];i++)
		b[i]=0;

	for(int i=1;i<=a[1][0];i++)
	{
		for(int j=1;j<=a[0][1];j++)
			b[i]=b[i]+a[i][j];

		b[i]=b[i]/a[0][1];
	}
}

double m_trace(double **a)
{
	/*
	Œlad macierzy - tylko dla kwadratowej bo ogólna wersja nie jest potrzebna
	*/
	double tr=0;
	for(int i=1;i<=a[1][0];i++)
		tr=tr+a[i][i];
	return tr;
}

double v_mean(double *a)
{
	/*
	Funkcja oblicza œredni¹ elementow wektora
	*/
	double s=0;
	for(int i=1;i<=a[0];i++)
		s+=a[i];
	return s/=a[0];
}

double v_mean_abs(double *a)
{
	/*
	Funkcja oblicza œredni¹ wartosci bezwzglednych elementow wektora
	*/
	double s=0;
	for(int i=1;i<=a[0];i++)
		s+=fabs(a[i]);
	return s/=a[0];
}

double v_norm(double *a)
{
	/*
	Funkcja oblicza kwadratowa norme wektora 
	*/
	double n=0;
	for(int i=1; i<=a[0];i++)
		n=n+a[i]*a[i];
	return sqrt(n);
}

double v_dot(double *a, double *b)
{
	/*
	Funkcja liczy iloczyn skalarny wektorow
	*/
	double d=0;
	for(int i=1; i<=a[0];i++)
		d=d+a[i]*b[i];
	return d;
}

void v_cross(double *a, double *b, double *c)
{
	/*
	Funkcja liczy iloczyn wektorowy dwóch trzyelementowych wektorów
	*/
	c[1]=a[2]*b[3]-b[2]*a[3];
	c[2]=a[3]*b[1]-b[3]*a[1];
	c[3]=a[1]*b[2]-b[1]*a[2];

}

void m_print(double **a)
{
	for(int i=1; i<=a[1][0]; i++)
	{
		for(int j=1; j<=a[0][1]; j++)
			printf("%5.8f ", a[i][j]);
		printf("\n");
	}
}

void v_print(double *a)
{
	for(int i=1; i<=a[0]; i++)
		printf("%5.6f ", a[i]);
}



