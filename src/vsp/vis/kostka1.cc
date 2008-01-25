#include <stdlib.h>
#include <math.h>
#include <stdio.h>
//#include <conio.h>
#include <time.h>
#include "global.h"
#include "calib.h"
#include "macierze_nr.h"



double *cc, *fc, *kc; //globalnie widoczne parametry kamery
int alloc_m=0, alloc_v=0; //globalnie widoczne liczby zaalokowanych macierzy i wektorow
struct timespec s_time, e_time;

void main(int argc, char* argv[])
{
	double **x_kk, **X_kk;

	cc=vector(2);
	fc=vector(2);
	kc=vector(5);
	cc[1]=302.626828187928;
	cc[2]=244.557213158262;
	fc[1]=657.849558506737;
	fc[2]=658.391549732184;
	kc[1]=-0.260884763223763;
	kc[2]=0.155839754454464;
	kc[3]=4.49472818666492e-005;
	kc[4]=-7.78398247727111e-005;
	kc[5]=0;


	x_kk=matrix(2,4);
	X_kk=matrix(3,4);
/*
	x_kk[1][1]=190.5555; x_kk[1][2]=188.6857; x_kk[1][3]=167.6885; x_kk[1][4]=165.5155; 
	x_kk[2][1]=179.9946; x_kk[2][2]=193.3430; x_kk[2][3]=178.9330; x_kk[2][4]=191.9369; 
	X_kk[1][1]=0; X_kk[1][2]=30; X_kk[1][3]=0; X_kk[1][4]=30; 
	X_kk[2][1]=30; X_kk[2][2]=30; X_kk[2][3]=0; X_kk[2][4]=0; 
	X_kk[3][1]=0; X_kk[3][2]=0; X_kk[3][3]=0; X_kk[3][4]=0; 
*/
	x_kk[1][1]=289.1; x_kk[1][2]=290.4; x_kk[1][3]=245.6; x_kk[1][4]=247.1; 
	x_kk[2][1]=119.4; x_kk[2][2]=162.7; x_kk[2][3]=120.3; x_kk[2][4]=163.6; 
	X_kk[1][1]=0; X_kk[1][2]=30; X_kk[1][3]=0; X_kk[1][4]=30; 
	X_kk[2][1]=30; X_kk[2][2]=30; X_kk[2][3]=0; X_kk[2][4]=0; 
	X_kk[3][1]=0; X_kk[3][2]=0; X_kk[3][3]=0; X_kk[3][4]=0; 


	double *omckk=vector(3);
	double *Tckk=vector(3);
	double **Rckk=matrix(3,3);
	
	clock_gettime( CLOCK_REALTIME , &s_time);
	compute_extrinsic_init(x_kk,X_kk,omckk,Tckk,Rckk);
	compute_extrinsic_refine(x_kk,X_kk,omckk,Tckk,Rckk);
	clock_gettime( CLOCK_REALTIME , &e_time);
	
	printf( "VSP= %f\n",(double)(e_time.tv_nsec-s_time.tv_nsec));
	printf( "VSP= %d\n",(e_time.tv_nsec-s_time.tv_nsec));

	printf("Wektor translacji:\n");
	v_print(Tckk);
	printf("\nWektor rotacji:\n");
	v_print(omckk);
	printf("\nMacierz rotacji:\n");
	m_print(Rckk);
	free_matrix(Rckk);
	free_matrix(x_kk);
	free_matrix(X_kk);
	free_vector(cc);
	free_vector(fc);
	free_vector(kc);
	free_vector(omckk);
	free_vector(Tckk);

	printf("\n\nZaalokowano macierzy: %d",alloc_m);
	printf("\n\nZaalokowano wektorow: %d",alloc_v);
	getchar();

}
