#include <string.h>
#include <unistd.h>

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_g_smooth2.h"
#include "ecp/common/ecp_t_calibration.h"
#include "ecp/common/ecp_t_calib_axzb.h"
#include "gsl/gsl_vector.h"
#include "gsl/gsl_matrix.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
calib_axzb::calib_axzb(lib::configurator &_config) : calibration(_config)
{
	fdf.n = 12;  // number of function components
	fdf.f = &objective_function;
	fdf.df = &objective_function_df;
	fdf.fdf = &objective_function_fdf;
}

void calib_axzb::main_task_algorithm(void)
{
	calibration::main_task_algorithm();
}

/*!
 * Objective function for eih calibration
 * v = [x, y, z, alfa, beta, gamma, u, v, w, fi, teta, psi]
 * params contains matrices K, M and vectors k, m
 */
double calib_axzb::objective_function (const gsl_vector *v, void *params)
{
	int i;

	// value of objective function
	double wynik = 0.0;

	// temporary vector for calculations
	gsl_vector *temp_vector;
	temp_vector = gsl_vector_calloc (3);

	// temporary matrix for calculations
	gsl_matrix *temp_matrix;
	temp_matrix = gsl_matrix_calloc(3, 3);

	// translation from robot base to chessboard frame
	gsl_vector *d;
	d = gsl_vector_calloc (3);

	// translation from camera frame to gripper frame
	gsl_vector *s;
	s = gsl_vector_calloc (3);

	// translation from robot base to gripper frame
	gsl_vector *k;
	k = gsl_vector_calloc (3);

	// translation from chessboard frame to camera frame
	gsl_vector *m;
	m = gsl_vector_calloc (3);

	// vector to store angles and transform them to rotation matrix
	gsl_vector *angles;
	angles = gsl_vector_calloc (3);

	// rotation from camera frame to gripper frame
	gsl_matrix *S;
	S = gsl_matrix_calloc(3, 3);

	// rotation from robot base to chessboard frame
	gsl_matrix *D;
	D = gsl_matrix_calloc(3, 3);

	// rotation from robot base to gripper frame
	gsl_matrix *K;
	K = gsl_matrix_calloc(3, 3);

	// rotation from chessboard frame to camera frame
	gsl_matrix *M;
	M = gsl_matrix_calloc(3, 3);

	objective_function_parameters *p = (objective_function_parameters *)params;

	// initialize d, s, D, S from vector v
	for(i = 0; i < 12; ++i)
	{
		if(i < 3)
		{
			// x, y, z
			gsl_vector_set(d, i, gsl_vector_get(v,i));
		}
		else if (i < 6)
		{
			// alfa, beta, gammma
			gsl_vector_set(angles, i-3, gsl_vector_get(v,i));
		}
		else if (i < 9)
		{
			// u, v, w
			gsl_vector_set(s, i-6, gsl_vector_get(v,i));
		}
		else if (i == 9)
		{
			// calculate matrix D from angles alfa, beta, gamma
			angles_to_rotation_matrix(angles, D);

			// fi
			gsl_vector_set(angles, i-9, gsl_vector_get(v,i));
		}else
		{
			// teta, psi
			gsl_vector_set(angles, i-9, gsl_vector_get(v,i));
		}
	}
	// calculate matrix S from angles fi, teta, psi
	angles_to_rotation_matrix(angles, S);

	// calculate
	// sum(i) = { [ (d + D * (m(i) + M(i) * s) )- k(i) ]^T * [ (d + D * (m(i) + M(i) * s) )- k(i) ] +
	// + c * trace[(D * M(i) * S - K(i))^T * (D * M(i) * S - K(i))] }	-	for i = 0 to i < number_of_measures
	for(i = 0; i < p->number_of_measures; ++i)
	{
		// extract data previously stored from structure: struct objective_function_parameters
		extract_vector(p->m, m, i);
		extract_vector(p->k, k, i);
		extract_matrix(p->M, M, i);
		extract_matrix(p->K, K, i);

		// change transformation from camera to object -> object to camera
		gsl_matrix_transpose(M);
		matrix_vector_multiply(M, m);
//		gsl_vector_scale(m, -1.0);

		// temp_vector = s
		gsl_vector_memcpy (temp_vector, s);
		// temp_matrix = D
		gsl_matrix_memcpy (temp_matrix, D);

		// temp_vector = M(i) * s
		matrix_vector_multiply(M, temp_vector);

		// temp_vector = m(i) + M(i) * s
		gsl_vector_add(temp_vector, m);

		// temp_vector = D * (m(i) + M(i) * s)
		matrix_vector_multiply(D, temp_vector);

		// temp_vector = (d + D * (m(i) + M(i) * s)
		gsl_vector_add(temp_vector, d);

		// temp_vector = (d + D * (m(i) + M(i) * s) )- k(i)
		gsl_vector_sub(temp_vector, k);

		// wynik += [(d + D * (m(i) + M(i) * s) )- k(i) ]^T * [ (d + D * (m(i) + M(i) * s) )- k(i)]
		wynik += transposed_vector_multiply(temp_vector,temp_vector);

		// temp_matrix = D * M(i)
		matrix_matrix_multiply(temp_matrix, M);

		// temp_matrix = D * M(i) * S
		matrix_matrix_multiply(temp_matrix, S);

		// temp_matrix = D * M(i) * S - K(i)
		gsl_matrix_sub (temp_matrix, K);

		// copy and transpose matrix temp_matrix -> M = (D * M(i) * S - K(i))^T
		gsl_matrix_transpose_memcpy (M, temp_matrix);

		// M = (D * M(i) * S - K(i))^T * (D * M(i) * S - K(i))
		matrix_matrix_multiply(M, temp_matrix);

		// wynik += c * trace[(D * M(i) * S - K(i))^T * (D * M(i) * S - K(i))]
		wynik += (p->magical_c) * (calculate_matrix_trace(M));
	}

	// free allocated memory
	gsl_matrix_free(temp_matrix);
	gsl_vector_free(temp_vector);
	gsl_vector_free(angles);
	gsl_matrix_free(K);
	gsl_vector_free(k);
	gsl_matrix_free(M);
	gsl_vector_free(m);
	gsl_matrix_free(D);
	gsl_vector_free(d);
	gsl_matrix_free(S);
	gsl_vector_free(s);

	return wynik;
}

void calib_axzb::objective_function_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	int i;

	for(i = 0; i < 12; ++i)
	{
		// set initial values of derivatives to zero
		gsl_vector_set(df, i, 0.0);
	}

	// temporary variable
	double wynik = 0.0;

	// temporary matrix for calculations
	gsl_matrix *temp_matrix;
	temp_matrix = gsl_matrix_calloc(3, 3);
	// second temporary matrix for calculations
	gsl_matrix *temp_matrix2;
	temp_matrix2 = gsl_matrix_calloc(3, 3);
	// third temporary matrix for calculations
	gsl_matrix *temp_matrix3;
	temp_matrix3 = gsl_matrix_calloc(3, 3);

	// vector to store angles and transform them to rotation matrix
	gsl_vector *angles;
	angles = gsl_vector_calloc (3);

	// temporary vector for calculations
	gsl_vector *temp_vector;
	temp_vector = gsl_vector_calloc (3);
	// second temporary vector for calculations
	gsl_vector *temp_vector2;
	temp_vector2 = gsl_vector_calloc (3);
	// third temporary vector for calculations
	gsl_vector *temp_vector3;
	temp_vector3 = gsl_vector_calloc (3);

	// translation from robot base to gripper frame
	gsl_vector *k;
	k = gsl_vector_calloc (3);

	// translation from camera frame to chessboard frame
	gsl_vector *m;
	m = gsl_vector_calloc (3);

	// translation from robot frame to chessboard frame
	gsl_vector *d;
	d = gsl_vector_calloc (3);

	// rotation from robot frame to chessboard frame
	gsl_matrix *D;
	D = gsl_matrix_calloc(3, 3);

	// translation from camera frame to gripper frame
	gsl_vector *s;
	s = gsl_vector_calloc (3);

	// rotation from from camera frame to gripper frame
	gsl_matrix *S;
	S = gsl_matrix_calloc(3, 3);

	// derivatives of matrices (dafa, dbeta, dgamma)
	gsl_matrix *dS_dfi;
	dS_dfi = gsl_matrix_calloc(3, 3);

	gsl_matrix *dS_dteta;
	dS_dteta = gsl_matrix_calloc(3, 3);

	gsl_matrix *dS_dpsi;
	dS_dpsi = gsl_matrix_calloc(3, 3);

	gsl_matrix *dD_dalfa;
	dD_dalfa = gsl_matrix_calloc(3, 3);

	gsl_matrix *dD_dbeta;
	dD_dbeta = gsl_matrix_calloc(3, 3);

	gsl_matrix *dD_dgamma;
	dD_dgamma = gsl_matrix_calloc(3, 3);

	// rotation from robot base to gripper frame
	gsl_matrix *K;
	K = gsl_matrix_calloc(3, 3);

	// rotation from from camera frame to chessboard frame
	gsl_matrix *M;
	M = gsl_matrix_calloc(3, 3);

	objective_function_parameters *p = (objective_function_parameters *)params;

	// initialize d, s, D, S from vector v
	for(i = 0; i < 12; ++i)
	{
		if(i < 3)
		{
			// x, y, z
			gsl_vector_set(d, i, gsl_vector_get(v,i));
		}
		else if (i < 6)
		{
			// alfa, beta, gammma
			gsl_vector_set(angles, i-3, gsl_vector_get(v,i));
		}
		else if (i < 9)
		{
			// u, v, w
			gsl_vector_set(s, i-6, gsl_vector_get(v,i));
		}
		else if (i == 9)
		{
			// calculate matrix D from angles alfa, beta, gamma
			angles_to_rotation_matrix(angles, D);

			// calculate derivative of matrix D - dD/dalfa
			angles_to_rotation_matrix_dalfa(angles, dD_dalfa);

			// calculate derivative of matrix D - dD/dbeta
			angles_to_rotation_matrix_dbeta(angles, dD_dbeta);

			// calculate derivative of matrix D - dD/dgammma
			angles_to_rotation_matrix_dgamma(angles, dD_dgamma);

			// fi
			gsl_vector_set(angles, i-9, gsl_vector_get(v,i));
		}else
		{
			// teta, psi
			gsl_vector_set(angles, i-9, gsl_vector_get(v,i));
		}
	}

	// calculate matrix S from angles fi, teta, psi
	angles_to_rotation_matrix(angles, S);

	// calculate derivative of matrix S - dS/dfi
	angles_to_rotation_matrix_dalfa(angles, dS_dfi);

	// calculate derivative of matrix S - dS/dteta
	angles_to_rotation_matrix_dbeta(angles, dS_dteta);

	// calculate derivative of matrix S - dS/dpsi
	angles_to_rotation_matrix_dgamma(angles, dS_dpsi);

	for(i = 0; i < p->number_of_measures; ++i)
	{
		// extract previously stored data
		extract_vector(p->m, m, i);
		extract_vector(p->k, k, i);
		extract_matrix(p->M, M, i);
		extract_matrix(p->K, K, i);

		// change transformation from camera to object -> object to camera
		gsl_matrix_transpose(M);
		matrix_vector_multiply(M, m);
//		gsl_vector_scale(m, -1.0);

		// temp_vector = s
		gsl_vector_memcpy(temp_vector, s);

		// temp_vector = M * s
		matrix_vector_multiply(M, temp_vector);

		// temp_vector = m + M * s
		gsl_vector_add(temp_vector, m);

		// temp_vector = D * (m + M * s)
		matrix_vector_multiply(D, temp_vector);

		// temp_vector = d + D * (m + M * s)
		gsl_vector_add(temp_vector, d);

		// temp_vector = d + D * (m + M * s) - k
		gsl_vector_sub(temp_vector, k);

		// temp_vector = 2 * [d + D * (m + M * s) - k]
		gsl_vector_scale(temp_vector, 2.0);

		// temp_vector3 = 2 * [d + D * (m + M * s) - k]
		gsl_vector_memcpy(temp_vector3, temp_vector);

		// get previous value of dW/dx
		wynik = gsl_vector_get(df, 0);
		// wynik += 2 * [d + D * (m + M * s) - k]^T * [1 0 0]
		wynik += gsl_vector_get(temp_vector3, 0);
		// update value of dW/dx
		gsl_vector_set(df, 0, wynik);


		// get previous value of dW/dy
		wynik = gsl_vector_get(df, 1);
		// wynik += 2 * [d + D * (m + M * s) - k]^T * [0 1 0]
		wynik += gsl_vector_get(temp_vector3, 1);
		// update value of dW/dy
		gsl_vector_set(df, 1, wynik);


		// get previous value of dW/dz
		wynik = gsl_vector_get(df, 2);
		// wynik += 2 * [d + D * (m + M * s) - k]^T * [0 0 1]
		wynik += gsl_vector_get(temp_vector3, 2);
		// update value of dW/dz
		gsl_vector_set(df, 2, wynik);


		// temp_matrix = D
		gsl_matrix_memcpy(temp_matrix, D);

		// temp_matrix = D * M
		matrix_matrix_multiply(temp_matrix, M);

		// temp_vector = 2 * [d + D * (m + M * s) - k]^T * D * M
		transposed_vector_matrix_multiply(temp_vector, temp_matrix);


		// get previous value of dW/du
		wynik = gsl_vector_get(df, 6);
		// wynik += 2 * [d + D * (m + M * s) - k]^T * D * M * [1 0 0]
		wynik += gsl_vector_get(temp_vector, 0);
		// update value of dW/du
		gsl_vector_set(df, 6, wynik);


		// get previous value of dW/dv
		wynik = gsl_vector_get(df, 7);
		// wynik += 2 * [d + D * (m + M * s) - k]^T * D * M * [0 1 0]
		wynik += gsl_vector_get(temp_vector, 1);
		// update value of dW/dv
		gsl_vector_set(df, 7, wynik);


		// get previous value of dW/dw
		wynik = gsl_vector_get(df, 8);
		// wynik += 2 * [d + D * (m + M * s) - k]^T * D * M * [0 0 1]
		wynik += gsl_vector_get(temp_vector, 2);
		// update value of dW/dw
		gsl_vector_set(df, 8, wynik);


		// temp_vector2 = s
		gsl_vector_memcpy(temp_vector2, s);

		// temp_vector2 = M * s
		matrix_vector_multiply(M, temp_vector2);

		// temp_vector = m + M * s
		gsl_vector_add(temp_vector2, m);

		// temp_matrix2 = M
		gsl_matrix_memcpy(temp_matrix2, M);

		// temp_matrix2 = M * S
		matrix_matrix_multiply(temp_matrix2, S);

		// temp_matrix3 = D
		gsl_matrix_memcpy(temp_matrix3, D);

		// temp_matrix3 = D * M
		matrix_matrix_multiply(temp_matrix3, M);

		// temp_matrix3 = D * M * S
		matrix_matrix_multiply(temp_matrix3, S);

		// temp_matrix3 = D * M * S - K
		gsl_matrix_sub(temp_matrix3, K);

		// temp_matrix3 = [D * M * S - K]^T
		gsl_matrix_transpose(temp_matrix3);

		// dW/dalfa

		// temp_vector = m + M * s
		gsl_vector_memcpy(temp_vector, temp_vector2);

		// temp_vector = dD_dalfa * (m + M * s)
		matrix_vector_multiply(dD_dalfa, temp_vector);

		// get previous value of dW/dalfa
		wynik = gsl_vector_get(df, 3);

		// wynik += 2 * [d + D * (m + M * s) - k]^T * dD_dalfa * (m + M * s)
		wynik += transposed_vector_multiply(temp_vector3, temp_vector);

		// temp_matrix = [D * M * S - K]^T
		gsl_matrix_memcpy(temp_matrix, temp_matrix3);

		// temp_matrix = [D * M * S - K]^T * dD_dalfa
		matrix_matrix_multiply(temp_matrix, dD_dalfa);

		// temp_matrix = [D * M * S - K]^T * dD_dalfa * M * S
		matrix_matrix_multiply(temp_matrix, temp_matrix2);

		// wynik += 2 * c * tr([D * M * S - K]^T * dD_dalfa * M * S)
		wynik += 2.0 * (p->magical_c) * calculate_matrix_trace(temp_matrix);

		// update value of dW/dalfa
		gsl_vector_set(df, 3, wynik);

		// dW/dbeta

		// temp_vector = m + M * s
		gsl_vector_memcpy(temp_vector, temp_vector2);

		// temp_vector = dD_dbeta * (m + M * s)
		matrix_vector_multiply(dD_dbeta, temp_vector);

		// get previous value of dW/dbeta
		wynik = gsl_vector_get(df, 4);

		// wynik += 2 * [d + D * (m + M * s) - k]^T * dD_dbeta * (m + M * s)
		wynik += transposed_vector_multiply(temp_vector3, temp_vector);

		// temp_matrix = [D * M * S - K]^T
		gsl_matrix_memcpy(temp_matrix, temp_matrix3);

		// temp_matrix = [D * M * S - K]^T * dD_dbeta
		matrix_matrix_multiply(temp_matrix, dD_dbeta);

		// temp_matrix = [D * M * S - K]^T * dD_dbeta * M * S
		matrix_matrix_multiply(temp_matrix, temp_matrix2);

		// wynik += 2 * c * tr([D * M * S - K]^T * dD_dbeta * M * S)
		wynik += 2.0 * (p->magical_c) * calculate_matrix_trace(temp_matrix);

		// update value of dW/dbeta
		gsl_vector_set(df, 4, wynik);

		// dW/dgamma

		// temp_vector = m + M * s
		gsl_vector_memcpy(temp_vector, temp_vector2);

		// temp_vector = dD_dgamma * (m + M * s)
		matrix_vector_multiply(dD_dgamma, temp_vector);

		// get previous value of dW/dgamma
		wynik = gsl_vector_get(df, 5);

		// wynik += 2 * [d + D * (m + M * s) - k]^T * dD_dgamma * (m + M * s)
		wynik += transposed_vector_multiply(temp_vector3, temp_vector);

		// temp_matrix = [D * M * S - K]^T
		gsl_matrix_memcpy(temp_matrix, temp_matrix3);

		// temp_matrix = [D * M * S - K]^T * dD_dgamma
		matrix_matrix_multiply(temp_matrix, dD_dgamma);

		// temp_matrix = [D * M * S - K]^T * dD_dgamma * M * S
		matrix_matrix_multiply(temp_matrix, temp_matrix2);

		// wynik += 2 * c * tr([D * M * S - K]^T * dD_dgamma * M * S)
		wynik += 2.0 * (p->magical_c) * calculate_matrix_trace(temp_matrix);

		// update value of dW/dalfa
		gsl_vector_set(df, 5, wynik);


		// temp_matrix2 = [D * M * S - K]^T
		gsl_matrix_memcpy(temp_matrix2, temp_matrix3);

		// temp_matrix2 = [D * M * S - K]^T * D
		matrix_matrix_multiply(temp_matrix2, D);

		// temp_matrix2 = [D * M * S - K]^T * D * M
		matrix_matrix_multiply(temp_matrix2, M);


		// dW/dfi

		// temp_matrix = [D * M * S - K]^T * D * M
		gsl_matrix_memcpy(temp_matrix, temp_matrix2);

		// temp_matrix = [D * M * S - K]^T * D * M * dS/dfi
		matrix_matrix_multiply(temp_matrix, dS_dfi);

		// get previous value of dW/dfi
		wynik = gsl_vector_get(df, 9);

		// wynik += 2 * c * tr([D * M * S - K]^T * D * M dS/dfi)
		wynik += 2.0 * (p->magical_c) * calculate_matrix_trace(temp_matrix);

		// update value of dW/dfi
		gsl_vector_set(df, 9, wynik);

		// dW/dteta

		// temp_matrix = [D * M * S - K]^T * D * M
		gsl_matrix_memcpy(temp_matrix, temp_matrix2);

		// temp_matrix = [D * M * S - K]^T * D * M * dS/dteta
		matrix_matrix_multiply(temp_matrix, dS_dteta);

		// get previous value of dW/dteta
		wynik = gsl_vector_get(df, 10);

		// wynik += 2 * c * tr([D * M * S - K]^T * D * M dS/dteta)
		wynik += 2.0 * (p->magical_c) * calculate_matrix_trace(temp_matrix);

		// update value of dW/dteta
		gsl_vector_set(df, 10, wynik);

		// dW/dpsi

		// temp_matrix = [D * M * S - K]^T * D * M
		gsl_matrix_memcpy(temp_matrix, temp_matrix2);

		// temp_matrix = [D * M * S - K]^T * D * M * dS/dpsi
		matrix_matrix_multiply(temp_matrix, dS_dpsi);

		// get previous value of dW/dpsi
		wynik = gsl_vector_get(df, 11);

		// wynik += 2 * c * tr([D * M * S - K]^T * D * M dS/dpsi)
		wynik += 2.0 * (p->magical_c) * calculate_matrix_trace(temp_matrix);

		// update value of dW/dpsi
		gsl_vector_set(df, 11, wynik);
	}

	// free allocated memory
	gsl_matrix_free(temp_matrix);
	gsl_matrix_free(temp_matrix2);
	gsl_matrix_free(temp_matrix3);
	gsl_vector_free(angles);
	gsl_vector_free(m);
	gsl_vector_free(k);
	gsl_vector_free(s);
	gsl_vector_free(d);
	gsl_vector_free(temp_vector);
	gsl_vector_free(temp_vector2);
	gsl_vector_free(temp_vector3);
	gsl_matrix_free(K);
	gsl_matrix_free(M);
	gsl_matrix_free(S);
	gsl_matrix_free(D);
	gsl_matrix_free(dS_dfi);
	gsl_matrix_free(dS_dteta);
	gsl_matrix_free(dS_dpsi);
	gsl_matrix_free(dD_dalfa);
	gsl_matrix_free(dD_dbeta);
	gsl_matrix_free(dD_dgamma);
}

/*!
 * Calculate objective function and her derivative together.
 */
void calib_axzb::objective_function_fdf(const gsl_vector *v, void *params, double *f, gsl_vector *df)
{
	*f = (*objective_function)(v, params);
	(*objective_function_df)(v, params, df);
}

task* return_created_ecp_task (lib::configurator &_config)
{
	return new calib_axzb(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
