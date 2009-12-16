#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "vsp/vis/macierze_nr.h"
#include "vsp/vis/calib.h"

void comp_distortion_oulu(double **xd, double **x, double *kc)
{
	/*
	%Compensates for radial and tangential distortion. Model From Oulu university.
	%For more informatino about the distortion model, check the forward projection mapping function:
	%project_points.m
	%
	%INPUT: xd: distorted (normalized) point coordinates in the image plane (2xN matrix)
	%       k: Distortion coefficients (radial and tangential) (4x1 vector)
	%
	%OUTPUT: x: undistorted (normalized) point coordinates in the image plane (2xN matrix)
	%
	%Method: Iterative method for compensation.
	*/
	m_copy(xd,x);
	double *r_2=vvector(4), *k_radial=vvector(4);
	double **delta_x=matrix(2,4);

	for(int kk=1;kk<=10;kk++)
	{
		for(int i=1;i<=4;i++)
			r_2[i]=x[1][i]*x[1][i]+x[2][i]*x[2][i];
		for(int i=1;i<=4;i++)
			k_radial[i] =  1 + kc[1] * r_2[i] + kc[2] * r_2[i]*r_2[i] + kc[3] * r_2[i]*r_2[i]*r_2[i];
		for(int i=1;i<=2;i++)
			for(int j=1;j<=4;j++)
			{
				delta_x[i][j]=2*kc[3]*x[i][j]*x[2][j]+ kc[4]*(r_2[j] + 2*x[i][j])*(r_2[j] + 2*x[i][j]);
				x[i][j]=(xd[i][j]-delta_x[i][j])/k_radial[j];
			}
	}
	free_vector(r_2);
	free_vector(k_radial);
	free_matrix(delta_x);
}

void compute_homography(double **m, double **M, double **H)
{
	/*
	%Computes the planar homography between the point coordinates on the plane (M) and the image
	%point coordinates (m).
	%
	%INPUT: m: homogeneous coordinates in the image plane (3xN matrix)
	%       M: homogeneous coordinates in the plane in 3D (3xN matrix)
	%
	%OUTPUT: H: Homography matrix (3x3 homogeneous matrix)
	%        Hnorm: Normalization matrix used on the points before homography computation
	%               (useful for numerical stability is points in pixel coordinates)
	%        inv_Hnorm: The inverse of Hnorm
	%
	%Definition: m ~ H*M where "~" means equal up to a non zero scalar factor.
	%
	%Method: First computes an initial guess for the homography through quasi-linear method.
	%        Then, if the total number of points is larger than 4, optimize the solution by minimizing
	%        the reprojection error (in the least squares sense).
	%
	%
	%Important functions called within that program:
	%
	%comp_distortion_oulu: Undistorts pixel coordinates.
	%
	%compute_homography.m: Computes the planar homography between points on the grid in 3D, and the image plane.
	%
	%project_points.m: Computes the 2D image projections of a set of 3D points, and also returns te Jacobian
	%                  matrix (derivative with respect to the intrinsic and extrinsic parameters).
	%                  This function is called within the minimization loop.
	*/

	double *ax=vvector(4), *ay=vvector(4);
	m_subvector_r(m,1,ax);
	m_subvector_r(m,2,ay);
	double mxx=v_mean(ax), myy=v_mean(ay);
	v_substract_s(ax,mxx);
	v_substract_s(ay,myy);
	double scxx=v_mean_abs(ax), scyy=v_mean_abs(ay);

	free_vector(ax); free_vector(ay);

	double **Hnorm=matrix(3,3);
	Hnorm[1][1]=1/scxx; Hnorm[1][2]=0; Hnorm[1][3]=-mxx/scxx;
	Hnorm[2][1]=0; Hnorm[2][2]=1/scyy; Hnorm[2][3]=-myy/scyy;
	Hnorm[3][1]=0; Hnorm[3][2]=0; Hnorm[3][3]=1;
	double **inv_Hnorm=matrix(3,3);
	inv_Hnorm[1][1]=scxx; inv_Hnorm[1][2]=0; inv_Hnorm[1][3]=mxx;
	inv_Hnorm[2][1]=0; inv_Hnorm[2][2]=scyy; inv_Hnorm[2][3]=myy;
	inv_Hnorm[3][1]=0; inv_Hnorm[3][2]=0; inv_Hnorm[3][3]=1;

	for(int i=1;i<=4;i++)
	{
		m[3][i]=1;
		M[3][i]=1;
	}

	double **mn=matrix(3,4);
	m_multiply_m(Hnorm,m,mn);

	double **L=matrix(8,9);
	m_zeros(L);
	for(int i=1;i<=4;i++)
		for(int j=1;j<=3;j++)
		{
			L[2*i-1][j]=M[j][i];
			L[2*i-1][j+6]=-M[j][i]*mn[1][i];
			L[2*i][j+3]=M[j][i];
			L[2*i][j+6]=-M[j][i]*mn[2][i];
		}
		free_matrix(mn);

		double **t_L=matrix((int)L[0][1],(int)L[1][0]);
		m_transpose(L,t_L);
		double **R=matrix((int)L[0][1],(int)L[1][0]);
		double **Q=matrix((int)L[0][1],(int)L[0][1]);
		free_matrix(L);

		m_qrdecomp(t_L, R, Q);

		free_matrix(R);
		free_matrix(t_L);
		double **Hrem=matrix(3,3);
		for(int i=1;i<=3;i++)
			for(int j=1;j<=3;j++)
				Hrem[i][j]=Q[(i-1)*3+j][9]/Q[9][9];
		free_matrix(Q);
		m_multiply_m(inv_Hnorm,Hrem,H);
		free_matrix(inv_Hnorm);
		free_matrix(Hrem);
		free_matrix(Hnorm);

}

void rodrigues_m(double **R, double *omckk)
{
	double tr=(m_trace(R)-1)/2;
	double theta = acos(tr);

	if (sin(theta) >= 1e-5)
	{
		double vth = 1/(2*sin(theta));
		omckk[1] = theta*vth*(R[3][2]-R[2][3]);
		omckk[2] = theta*vth*(R[1][3]-R[3][1]);
		omckk[3] = theta*vth*(R[2][1]-R[1][2]);
	}

}
void rodrigues_v(double *omckk, double **R, double **dRdom)
{
	//dla przypadku gdy argumentem jest wektor a nie macierz
	double theta =v_norm(omckk);
	if(theta<=2e-16)
		//raczej to sie nie zdarza
	{
		m_eye(R);
		m_zeros(dRdom);
		dRdom[2][3]=1;
		dRdom[3][2]=-1;
		dRdom[4][3]=-1;
		dRdom[6][1]=1;
		dRdom[7][2]=1;
		dRdom[8][1]=-1;
	}
	else
	{
		double **dm3din=matrix(4,3);
		m_eye(dm3din);
		dm3din[4][1]=omckk[1]/theta;
		dm3din[4][2]=omckk[2]/theta;
		dm3din[4][3]=omckk[3]/theta;

		double *omega=vvector(3);
		v_copy(omckk,omega);
		v_multiply_s(omega,1/theta);

		double **dm2dm3=matrix(4,4);
		m_zeros(dm2dm3);
		dm2dm3[1][1]=1/theta;
		dm2dm3[2][2]=1/theta;
		dm2dm3[3][3]=1/theta;
		dm2dm3[1][4]=-omckk[1]/(theta*theta);
		dm2dm3[2][4]=-omckk[2]/(theta*theta);
		dm2dm3[3][4]=-omckk[3]/(theta*theta);
		dm2dm3[4][4]=1;

		double alpha=cos(theta);
		double beta=sin(theta);
		double gamma=1-alpha;

		double **omegav=matrix(3,3);
		omegav[1][1]=0; omegav[1][2]=-omega[3]; omegav[1][3]=omega[2];
		omegav[2][1]=omega[3]; omegav[2][2]=0; omegav[2][3]=-omega[1];
		omegav[3][1]=-omega[2]; omegav[3][2]=omega[1]; omegav[3][3]=0;
		double **A=matrix(3,3);
		v_multiply_v_m(omega,omega,A);

		double **dm1dm2=matrix(21,4);
		m_zeros(dm1dm2);
		dm1dm2[1][4]=-beta; dm1dm2[2][4]=alpha; dm1dm2[3][4]=beta;
		dm1dm2[5][3]=1; dm1dm2[6][2]=-1; dm1dm2[7][3]=-1;
		dm1dm2[9][1]=1; dm1dm2[10][2]=1; dm1dm2[11][1]=-1;
		dm1dm2[13][1]=2*omega[1]; dm1dm2[14][1]=omega[2]; dm1dm2[15][1]=omega[3]; dm1dm2[16][1]=omega[2]; dm1dm2[19][1]=omega[3];
		dm1dm2[14][2]=omega[1]; dm1dm2[16][2]=omega[1]; dm1dm2[17][2]=2*omega[2]; dm1dm2[18][2]=omega[3]; dm1dm2[20][2]=omega[3];
		dm1dm2[15][3]=omega[1]; dm1dm2[18][3]=omega[2]; dm1dm2[19][3]=omega[1]; dm1dm2[20][3]=omega[2]; dm1dm2[21][3]=2*omega[3];

		m_eye(R);
		m_multiply_s(R,alpha);
		double **R2=matrix(3,3);
		m_copy(omegav,R2);
		m_multiply_s(R2,beta);

		double **R3=matrix(3,3);
		m_copy(A,R3);
		m_multiply_s(R3,gamma);

		m_add_m(R,R2,R);
		m_add_m(R,R3,R);

		double **dRdm1=matrix(9,21);
		m_zeros(dRdm1);
		dRdm1[1][1]=1; dRdm1[5][1]=1; dRdm1[9][1]=1;
		dRdm1[1][2]=omegav[1][1]; dRdm1[2][2]=omegav[2][1]; dRdm1[3][2]=omegav[3][1];
		dRdm1[4][2]=omegav[1][2]; dRdm1[5][2]=omegav[2][2]; dRdm1[6][2]=omegav[3][2];
		dRdm1[7][2]=omegav[1][3]; dRdm1[8][2]=omegav[2][3]; dRdm1[9][2]=omegav[3][3];
		for(int i=1;i<10;i++)
		{
			dRdm1[i][i+3]=beta;
			dRdm1[i][i+12]=gamma;
		}

		dRdm1[1][3]=A[1][1]; dRdm1[2][3]=A[2][1]; dRdm1[3][3]=A[3][1];
		dRdm1[4][3]=A[1][2]; dRdm1[5][3]=A[2][2]; dRdm1[6][3]=A[3][2];
		dRdm1[7][3]=A[1][3]; dRdm1[8][3]=A[2][3]; dRdm1[9][3]=A[3][3];

		// teraz obliczamy dRdom = dRdm1 *dm1dm2 * dm2dm3 * dm3din;
		// tworzymy wiec tymczasowe macierze o odpowiednich rozmiarach
		double **Rt1=matrix(9,4);
		m_multiply_m(dRdm1,dm1dm2,Rt1);
		double **Rt2=matrix(9,4);
		m_multiply_m(Rt1,dm2dm3,Rt2);
		m_multiply_m(Rt2,dm3din,dRdom);
		free_matrix(dm3din);
		free_matrix(dm2dm3);
		free_matrix(dm1dm2);
		free_matrix(omegav);
		free_matrix(A);
		free_matrix(R2);
		free_matrix(R3);
		free_matrix(dRdm1);
		free_matrix(Rt1);
		free_matrix(Rt2);
		free_vector(omega);

	}
	//koniec rodrigeza dla argumentu jako wektor
}

void compute_extrinsic_init(double **x_kk, double **X_kk, double *omckk, double *Tckk, double **Rckk, double *fc, double *cc, double *kc)
{

	double **x_distort=matrix(2,4);
	double **xn=matrix(3,4); //w oryginale bylo 2,4 ale potem nastepowalo rozszerzenie



	for(int j=1;j<=2;j++)
		for(int i=1;i<=4;i++)
			x_distort[j][i]=(x_kk[j][i]-cc[j])/fc[j];

	comp_distortion_oulu(x_distort, xn, kc);

	double *X_mean=vvector(3);
	m_mean_r(X_kk,X_mean);

	double **X_new=matrix(3,4);
	m_copy(X_kk,X_new);
	m_substract_v(X_new,X_mean);

	double **H=matrix(3,3);

	compute_homography(xn,X_new,H);

	double *U1=vvector(3), *U2=vvector(3), *U3=vvector(3);
	m_subvector_c(H,1,U1);
	m_subvector_c(H,2,U2);
	double sc=(v_norm(U1)+v_norm(U2))/2;
	m_multiply_s(H,1/sc);

	v_multiply_s(U1,1/v_norm(U1));
	v_copy(U1,U3);
	v_multiply_s(U3,v_dot(U1,U2));
	v_substract_v(U2,U3);
	v_multiply_s(U2,1/v_norm(U2));
	v_cross(U1,U2,U3);
	m_insert_v_c(Rckk,U1,1);
	m_insert_v_c(Rckk,U2,2);
	m_insert_v_c(Rckk,U3,3);

	rodrigues_m(Rckk,omckk);


	double *tmp=vvector(3);
	m_subvector_c(H,3,Tckk);
	//v_multiply_s(X_mean,-1);
	m_multiply_v(Rckk,X_mean,tmp);
	v_substract_v(Tckk,tmp);

	free_matrix(x_distort);
	free_matrix(xn);
	free_matrix(X_new);
	free_matrix(H);
	free_vector(X_mean);
	free_vector(U1);
	free_vector(U2);
	free_vector(U3);
	free_vector(tmp);


}


void compute_extrinsic_refine(double **x_kk, double **X_kk, double *omckk, double *Tckk, double **Rckk, double *fc, double *cc, double *kc)
{
	double **param=matrix(6,1); //wyjatkowo jako macierz bo potem latwiej dorobic
	double **x=matrix(2,4);
	double **ex=matrix(2,4);
	double **dxdom=matrix(8,3); double **dxdT=matrix(8,3);
	double **JJ=matrix(8,6), **JJt=matrix(6,8), **JJ2=matrix(6,6), **JJ3=matrix(6,8);
	double **param_innov=matrix(6,1), **param_up=matrix(6,1), **ex2=matrix(8,1);
	double *param_innov_v=vvector(6),*param_up_v=vvector(6); //wektorki do obliczenia normy
	double	change = 1;
	int iter = 0;

	for(int i=1;i<=3;i++)
	{
		param[i][1]=omckk[i];
		param[i+3][1]=Tckk[i];
	}

	while ((change > 1e-10)&&(iter < 30))
	{

		project_points(X_kk,omckk,Tckk,x,dxdom,dxdT,fc,cc,kc);
		m_substract_m(x_kk,x,ex);
		for(int i=1;i<=8;i++)
			for(int j=1;j<=3;j++)
			{
				JJ[i][j]=dxdom[i][j];
				JJ[i][j+3]=dxdT[i][j];
				JJt[j][i]=dxdom[i][j];
				JJt[j+3][i]=dxdT[i][j];
			}
			m_multiply_m(JJt,JJ,JJ2);

			m_inverse(JJ2);
			m_multiply_m(JJ2,JJt,JJ3);


			for(int i=1;i<=4;i++)
			{
				ex2[2*i-1][1]=ex[1][i]; ex2[2*i][1]=ex[2][i];
			}
			m_multiply_m(JJ3,ex2,param_innov);
			m_add_m(param,param_innov,param_up);

			m_subvector_c(param_innov,1,param_innov_v);
			m_subvector_c(param_up,1,param_up_v);
			change=v_norm(param_innov_v)/v_norm(param_up_v);

			m_copy(param_up,param);
			iter++;
			for(int i=1;i<=3;i++)
			{
				omckk[i]=param[i][1];
				Tckk[i]=param[i+3][1];
			}

			//printf("\n%f\n",change);
	}
	double **tmp=matrix(9,3);
	rodrigues_v(omckk,Rckk,tmp);
	//Rckk = w_rodrigues(omckk);
	free_matrix(param);
	free_matrix(x);
	free_matrix(ex);
	free_matrix(dxdom);
	free_matrix(dxdT);
	free_matrix(JJ);
	free_matrix(JJt);
	free_matrix(JJ2);
	free_matrix(JJ3);
	free_matrix(param_innov);
	free_matrix(param_up);
	free_matrix(ex2);
	free_matrix(tmp);
	free_vector(param_innov_v);
	free_vector(param_up_v);
}

void project_points(double **X_kk, double *omckk, double *Tckk, double **x, double **dxdom, double **dxdT, double *fc, double *cc, double *kc)
{
	double **Y=matrix(3,4), **dYdom=matrix(12,3), **dYdT=matrix(12,3);
	rigid_motion(X_kk,omckk,Tckk,Y,dYdom,dYdT);
	double *Inv_Z=vvector(4);
	//m_print(dYdT);
	//printf("\n");
	Inv_Z[1]=1.0/Y[3][1]; Inv_Z[2]=1.0/Y[3][2]; Inv_Z[3]=1.0/Y[3][3]; Inv_Z[4]=1.0/Y[3][4];

	//v_print(Inv_Z);

	for(int i=1;i<3;i++)
		for(int j=1;j<5;j++)
			x[i][j]=Y[i][j]*Inv_Z[j];
	double **bb=matrix(4,3), **ccc=matrix(4,3); //bo cc juz jest

	for(int i=1;i<=3;i++)
		for(int j=1;j<=4;j++)
		{
			bb[j][i]=-x[1][j]*Inv_Z[j];
			ccc[j][i]=-x[2][j]*Inv_Z[j];
		}
		double **dxdomt=matrix(8,3); //odpowiada to dxdom
		m_zeros(dxdomt);
		for(int i=1;i<=4;i++)
			for(int j=1;j<=3;j++)
			{
				dxdomt[i*2-1][j]=Inv_Z[i]*dYdom[i*3-2][j]+bb[i][j]*dYdom[3*i][j];
				dxdomt[i*2][j]=Inv_Z[i]*dYdom[i*3-1][j]+ccc[i][j]*dYdom[3*i][j];
			}
			double **dxdTt=matrix(8,3); //odpowiada to dxdT
			m_zeros(dxdTt);
			for(int i=1;i<=4;i++)
				for(int j=1;j<=3;j++)
				{
					dxdTt[i*2-1][j]=Inv_Z[i]*dYdT[i*3-2][j]+bb[i][j]*dYdT[3*i][j];
					dxdTt[i*2][j]=Inv_Z[i]*dYdT[i*3-1][j]+ccc[i][j]*dYdT[3*i][j];
				}
				//% Add distortion:
				double *r2=vvector(4);
				for(int i=1;i<=4;i++)
					r2[i]=x[1][i]*x[1][i]+x[2][i]*x[2][i];
				double **dr2dom=matrix(4,3), **dr2dT=matrix(4,3);
				//dr2dom = 2*((x(1,:)')*ones(1,3)) .* dxdom(1:2:end,:) + 2*((x(2,:)')*ones(1,3)) .* dxdom(2:2:end,:);
				for(int i=1;i<=4;i++)
					for(int j=1;j<=3;j++)
					{
						dr2dom[i][j]=2*x[1][i]*dxdomt[2*i-1][j]+2*x[2][i]*dxdomt[2*i][j];
						dr2dT[i][j]=2*x[1][i]*dxdTt[2*i-1][j]+2*x[2][i]*dxdTt[2*i][j];
					}
					double *r4=vvector(4), *r6=vvector(4);
					for(int i=1;i<=4;i++)
					{
						r4[i]=r2[i]*r2[i];
						r6[i]=r2[i]*r2[i]*r2[i];
					}
					double **dr4dom=matrix(4,3), **dr4dT=matrix(4,3), **dr6dom=matrix(4,3), **dr6dT=matrix(4,3);
					for(int i=1;i<=4;i++)
						for(int j=1;j<=3;j++)
						{
							dr4dom[i][j]=2*r2[i]*dr2dom[i][j];
							dr4dT[i][j]=2*r2[i]*dr2dT[i][j];
							dr6dom[i][j]=3*r4[i]*dr2dom[i][j];
							dr6dT[i][j]=3*r4[i]*dr2dT[i][j];
						}
						//	% Radial distortion:

						double *cdist=vvector(4);
						for(int i=1;i<=4;i++)
							cdist[i]=1;
						for(int i=1;i<=4;i++)
							cdist[i]+=kc[1]*r2[i]+kc[2]*r4[i]+kc[5]*r6[i];

						double **dcdistdom=matrix(4,3), **tmp=matrix(4,3);
						m_copy(dr2dom,tmp);
						m_multiply_s(tmp,kc[1]);
						m_copy(tmp,dcdistdom);
						m_copy(dr4dom,tmp);
						m_multiply_s(tmp,kc[2]);
						m_add_m(tmp,dcdistdom,dcdistdom);
						m_copy(dr6dom,tmp);
						m_multiply_s(tmp,kc[5]);
						m_add_m(tmp,dcdistdom,dcdistdom);

						double **dcdistdT=matrix(4,3);
						m_copy(dr2dT,tmp);
						m_multiply_s(tmp,kc[1]);
						m_copy(tmp,dcdistdT);
						m_copy(dr4dT,tmp);
						m_multiply_s(tmp,kc[2]);
						m_add_m(tmp,dcdistdT,dcdistdT);
						m_copy(dr6dT,tmp);
						m_multiply_s(tmp,kc[5]);
						m_add_m(tmp,dcdistdT,dcdistdT);

						double **dcdistdk=matrix(4,5);
						m_zeros(dcdistdk);
						for(int i=1;i<=4;i++)
						{
							dcdistdk[i][1]=r2[i];
							dcdistdk[i][2]=r4[i];
							dcdistdk[i][5]=r6[i];
						}

						double **xd1=matrix(2,4);
						for(int i=1;i<=4;i++)
						{
							xd1[1][i]=x[1][i]*cdist[i];
							xd1[2][i]=x[2][i]*cdist[i];
						}

						double **dxd1dom=matrix(8,3), **coeff=matrix(8,3);
						double **dxd1dT=matrix(8,3);
						m_zeros(dxd1dom);
						for(int i=1;i<=4;i++)
							for(int j=1;j<=3;j++)
							{
								dxd1dom[i*2-1][j]=x[1][i]*dcdistdom[i][j];
								dxd1dom[i*2][j]=x[2][i]*dcdistdom[i][j];
							}
							for(int i=1;i<=4;i++)
							{
								coeff[1][i]=cdist[1];coeff[2][i]=cdist[1];
								coeff[3][i]=cdist[2];coeff[4][i]=cdist[2];
								coeff[5][i]=cdist[3];coeff[6][i]=cdist[3];
								coeff[7][i]=cdist[4];coeff[8][i]=cdist[4];
							}
							for(int i=1;i<=8;i++)
								for(int j=1;j<=3;j++)
								{
									dxd1dom[i][j]+=coeff[i][j]*dxdomt[i][j];
								}


								m_zeros(dxd1dT);
								for(int i=1;i<=4;i++)
									for(int j=1;j<=3;j++)
									{
										dxd1dT[i*2-1][j]=x[1][i]*dcdistdT[i][j];
										dxd1dT[i*2][j]=x[2][i]*dcdistdT[i][j];
									}
									for(int i=1;i<=8;i++)
										for(int j=1;j<=3;j++)
										{
											dxd1dT[i][j]+=coeff[i][j]*dxdTt[i][j];
										}

										double **dxd1dk=matrix(8,5);
										m_zeros(dxd1dk);
										for(int i=1;i<=4;i++)
											for(int j=1;j<=5;j++)
											{
												dxd1dk[i*2-1][j]=x[1][i]*dcdistdk[i][j];
												dxd1dk[i*2][j]=x[2][i]*dcdistdk[i][j];
											}
											//% tangential distortion:

											double *a1=vvector(4), *a2=vvector(4), *a3=vvector(4);
											for(int i=1;i<=4;i++)
											{
												a1[i]=2*x[1][i]*x[2][i];
												a2[i]=r2[i]+2*x[1][i]*x[1][i];
												a3[i]=r2[i]+2*x[2][i]*x[2][i];
											}
											double **delta_x=matrix(2,4);
											for(int i=1;i<=4;i++)
											{
												delta_x[1][i]=kc[3]*a1[i]+kc[4]*a2[i];
												delta_x[2][i]=kc[3]*a3[i]+kc[4]*a1[i];
											}


											double **aa=matrix(4,3); //bb i cc byly juz wczesniej zaalokowane

											for(int i=1;i<=4;i++)
												for(int j=1;j<=3;j++)
												{
													aa[i][j]=2*kc[3]*x[2][i]+6*kc[4]*x[1][i];
													bb[i][j]=2*kc[3]*x[1][i]+2*kc[4]*x[2][i];
													ccc[i][j]=6*kc[3]*x[2][i]+2*kc[4]*x[1][i];
												}

												double **ddelta_xdom=matrix(8,3);
												m_zeros(ddelta_xdom);
												for(int i=1;i<=4;i++)
													for(int j=1;j<=3;j++)
													{
														ddelta_xdom[i*2-1][j]=aa[i][j]*dxdomt[i*2-1][j]+bb[i][j]*dxdomt[i*2][j];
														ddelta_xdom[i*2][j]=bb[i][j]*dxdomt[i*2-1][j]+ccc[i][j]*dxdomt[i*2][j];
													}

													double **ddelta_xdT=matrix(8,3);
													m_zeros(ddelta_xdT);
													for(int i=1;i<=4;i++)
														for(int j=1;j<=3;j++)
														{
															ddelta_xdT[i*2-1][j]=aa[i][j]*dxdTt[i*2-1][j]+bb[i][j]*dxdTt[i*2][j];
															ddelta_xdT[i*2][j]=bb[i][j]*dxdTt[i*2-1][j]+ccc[i][j]*dxdTt[i*2][j];
														}
														double **ddelta_xdk=matrix(8,5);
														m_zeros(ddelta_xdk);
														for(int i=1;i<=4;i++)

														{
															ddelta_xdk[2*i-1][3]=a1[i];
															ddelta_xdk[2*i-1][4]=a2[i];
															ddelta_xdk[2*i][3]=a3[i];
															ddelta_xdk[2*i][4]=a1[i];
														}

														double **xd2=matrix(2,4);
														m_add_m(xd1,delta_x,xd2);

														double **dxd2dom=matrix(8,3), **dxd2dT=matrix(8,3), **dxd2dk=matrix(8,5);
														m_add_m(dxd1dom,ddelta_xdom,dxd2dom);
														m_add_m(dxd1dT,ddelta_xdT,dxd2dT);

														m_add_m(dxd1dk,ddelta_xdk,dxd2dk);

														//% Add Skew:
														// olewka bo alpha=0 czyli xd3=xd2

														//% Compute: dxd3dom, dxd3dT, dxd3dk, dxd3dalpha														//dxd3dom=dxd2dom
														//dxd3dT=dxd2dT
														//dxd3dk=dxd2dk
														//dxd3dalpha nie robimy bo to tylko przerobiona xd2
														//% Pixel coordinates:
														//double **xp=matrix(2,4);
														for(int i=1;i<=4;i++)
														{
															x[1][i]=xd2[1][i]*fc[1]+cc[1];
															x[2][i]=xd2[2][i]*fc[2]+cc[2];
														}
														double *coeff2=vvector(8), *dxpdalpha=vvector(8);
														//double **dxpdom=matrix(8,3), **dxpdT=matrix(8,3), **dxpdk=matrix(8,5);
														//double **dxpdf=matrix(8,2);

														for(int i=1;i<=4;i++)
														{
															coeff2[2*i-1]=fc[1]; coeff2[2*i]=fc[2];
														}

														for(int i=1;i<=8;i++)
															for(int j=1;j<=3;j++)
															{
																dxdom[i][j]=coeff2[i]*dxd2dom[i][j];
																dxdT[i][j]=coeff2[i]*dxd2dT[i][j];
															}
															/*
															for(int i=1;i<=8;i++)
															for(int j=1;j<=5;j++)
															dxpdk[i][j]=coeff2[i]*dxd2dk[i][j];

															for(int i=1;i<=4;i++)
															{
															dxpdalpha[2*i-1]=coeff2[2*i-1]*xd2[2][i];
															dxpdalpha[2*i]=0;
															dxpdf[2*i][1]=0; dxpdf[2*i-1][1]=xd2[1][i];
															dxpdf[2*i-1][2]=0; dxpdf[2*i][2]=xd2[2][i];
															}
															*/


															//m_print(dxpdf);
															//free_matrix(dxd1dT); //witek powinna byc zwalniana ale zwalnianie powoduje segmentation fault
															free_matrix(Y);
															free_matrix(dYdom);
															free_matrix(dYdT);
															free_matrix(aa);
															free_matrix(bb);
															free_matrix(ccc);
															free_matrix(dxdomt);
															free_matrix(dxdTt);
															free_matrix(dr2dom);
															free_matrix(dr2dT);
															free_matrix(dr4dom);
															free_matrix(dr4dT);
															free_matrix(dr6dom);
															free_matrix(dr6dT);
															free_matrix(dcdistdom);
															free_matrix(tmp);
															free_matrix(dcdistdT);
															free_matrix(dcdistdk);
															free_matrix(xd1);
															free_matrix(dxd1dom);
															free_matrix(coeff);

															free_matrix(dxd1dk);
															free_matrix(delta_x);
															free_matrix(ddelta_xdom);
															free_matrix(ddelta_xdT);
															free_matrix(ddelta_xdk);
															free_matrix(xd2);
															free_matrix(dxd2dom);
															free_matrix(dxd2dT);
															free_matrix(dxd2dk);
															free_vector(Inv_Z);
															free_vector(r2);
															free_vector(r4);
															free_vector(r6);
															free_vector(cdist);
															free_vector(a1);
															free_vector(a2);
															free_vector(a3);
															free_vector(coeff2);
															free_vector(dxpdalpha);
}
//%[Y,dYdom,dYdT] = rigid_motion(X,om,T)




void rigid_motion(double **X_kk, double *omckk, double *Tckk, double **Y, double **dYdom, double **dYdT)
{
	double **R=matrix(3,3), **dRdom=matrix(9,3);
	rodrigues_v(omckk,R,dRdom);
	m_multiply_m(R,X_kk,Y);
	for(int i=1;i<4;i++)
		for(int j=1;j<5;j++)
			Y[i][j]+=Tckk[i];
	double **dYdR=matrix(12,9);
	m_zeros(dYdR);
	m_zeros(dYdT);
	for(int i=1;i<=X_kk[1][0];i++)
		for(int j=1;j<=X_kk[0][1];j++)
		{
			dYdR[j*3-2][i*3-2]=X_kk[i][j];
			dYdR[j*3-1][i*3-1]=X_kk[i][j];
			dYdR[j*3][i*3]=X_kk[i][j];
		}

		for(int i=1;i<=4;i++)
		{
			dYdT[i*3-2][1]=1;
			dYdT[i*3-1][2]=1;
			dYdT[i*3][3]=1;
		}
		m_multiply_m(dYdR,dRdom,dYdom);
		//printf("dYdT rodrigues\n");
		//m_print(dYdT);
		//printf("\n");
		free_matrix(R);
		free_matrix(dRdom);
		free_matrix(dYdR);
}
