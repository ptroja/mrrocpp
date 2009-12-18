#include "ecp_t_calib_axxb.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
calib_axxb::calib_axxb(lib::configurator &_config) : calibration(_config, 6)
{
	fdf.n = 6;  // number of function components
	fdf.f = &objective_function;
	fdf.df = &objective_function_df;
	fdf.fdf = &objective_function_fdf;
}

void calib_axxb::main_task_algorithm(void)
{
	calibration::main_task_algorithm();
	ecp_termination_notice();
}

/*!
 * Objective function for eih calibration
 * v = [x, y, z, alfa, beta, gamma]
 * params contains matrices K, M and vectors k, m
 * A * X = X * B
 */
double calib_axxb::objective_function(const gsl_vector *v, void *params)
{
	int i;

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

	// translation from robot base to gripper frame
	gsl_vector *k2;
	k2 = gsl_vector_calloc (3);

	// translation from chessboard frame to camera frame
	gsl_vector *m;
	m = gsl_vector_calloc (3);

	// translation from chessboard frame to camera frame
	gsl_vector *m2;
	m2 = gsl_vector_calloc (3);

	// translation from gripper frame to camera frame
	gsl_vector *x;
	x = gsl_vector_calloc (3);

	// rotation from gripper frame to camera frame
	gsl_matrix *X;
	X = gsl_matrix_calloc(3, 3);

	// rotation from robot base to gripper frame
	gsl_matrix *K2;
	K2 = gsl_matrix_calloc(3, 3);

	// rotation from robot base to gripper frame
	gsl_matrix *K;
	K = gsl_matrix_calloc(3, 3);

	// rotation from chessboard frame to camera frame
	gsl_matrix *M2;
	M2 = gsl_matrix_calloc(3, 3);

	// rotation from chessboard frame to camera frame
	gsl_matrix *M;
	M = gsl_matrix_calloc(3, 3);

	objective_function_parameters *p = (objective_function_parameters *)params;

	// initialize x, X from vector v
	for(i = 0; i < 6; ++i)
	{
		if(i < 3)
		{
			// x, y, z
			gsl_vector_set(x, i, gsl_vector_get(v,i));
		}
		else if (i < 6)
		{
			// fi, teta, psi
			gsl_vector_set(angles, i-3, gsl_vector_get(v,i));
		}
	}

	// calculate matrix X from angles fi, teta, psi
	angles_to_rotation_matrix(angles, X);

	for(i = 0; i < p->number_of_measures - 1; ++i)
	{
		// K = K1
		extract_matrix(p->K, K, i);

		// k = k1
		extract_vector(p->k, k, i);

		// K2 = K2
		extract_matrix(p->K, K2, i + 1);

		// k2 = k2
		extract_vector(p->k, k2, i + 1);

		// M = M1
		extract_matrix(p->M, M, i);

		// m = m1
		extract_vector(p->m, m, i);

		// M2 = M2
		extract_matrix(p->M, M2, i + 1);

		// m2 = m2
		extract_vector(p->m, m2, i + 1);

		// change transformation from object to camera -> camera to object
//		gsl_matrix_transpose(M);
//		matrix_vector_multiply(M, m);
		gsl_vector_scale(m, -1.0);
//		gsl_matrix_transpose(M2);
//		matrix_vector_multiply(M2, m2);
		gsl_vector_scale(m2, -1.0);

		// temp_matrix = K1
		gsl_matrix_memcpy (temp_matrix, K);

		// temp_matrix = K1 * X
		matrix_matrix_multiply(temp_matrix, X);

		// temp_matrix = K1 * X * M1
		matrix_matrix_multiply(temp_matrix, M);

		// temp_matrix2 = K2
		gsl_matrix_memcpy (temp_matrix2, K2);

		// temp_matrix2 = K2 * X
		matrix_matrix_multiply(temp_matrix2, X);

		// temp_matrix2 = K2 * X * M2
		matrix_matrix_multiply(temp_matrix2, M2);

		// temp_matrix = K1 * X * M1 - K2 * X * M2
		gsl_matrix_sub(temp_matrix, temp_matrix2);

		//temp_matrix3 = K1 * X * M1 - K2 * X * M2
		gsl_matrix_memcpy (temp_matrix3, temp_matrix);

		//temp_matrix = [K1 * X * M1 - K2 * X * M2]^T
		gsl_matrix_transpose(temp_matrix);

		// temp_matrix = [K1 * X * M1 - K2 * X * M2]^T * [K1 * X * M1 - K2 * X * M2]
		matrix_matrix_multiply(temp_matrix, temp_matrix3);

		// wynik += c * tr([K1 * X * M1 - K2 * X * M2]^T * [K1 * X * M1 - K2 * X * M2])
		wynik += p->magical_c * calculate_matrix_trace(temp_matrix);

		// temp_vector = x
		gsl_vector_memcpy(temp_vector, x);

		// temp_vector2 = m1
		gsl_vector_memcpy(temp_vector2, m);

		// temp_vector2 = X * m1
		matrix_vector_multiply(X, temp_vector2);

		// temp_vector2 = K1 * X * m1
		matrix_vector_multiply(K, temp_vector2);

		// temp_vector = K1 * x
		matrix_vector_multiply(K, temp_vector);

		// temp_vector2 = K1 * X * m1 + K1 * x
		gsl_vector_add(temp_vector2, temp_vector);

		// temp_vector2 = K1 * X * m1 + K1 * x + k1
		gsl_vector_add(temp_vector2, k);

		// temp_vector3 = K1 * X * m1 + K1 * x + k1
		gsl_vector_memcpy(temp_vector3, temp_vector2);

		// temp_vector = x
		gsl_vector_memcpy(temp_vector, x);

		// temp_vector2 = m2
		gsl_vector_memcpy(temp_vector2, m2);

		// temp_vector2 = X * m2
		matrix_vector_multiply(X, temp_vector2);

		// temp_vector2 = K2 * X * m2
		matrix_vector_multiply(K2, temp_vector2);

		// temp_vector = K2 * x
		matrix_vector_multiply(K2, temp_vector);

		// temp_vector2 = K2 * X * m2 + K2 * x
		gsl_vector_add(temp_vector2, temp_vector);

		// temp_vector2 = K2 * X * m2 + K2 * x + k2
		gsl_vector_add(temp_vector2, k2);

		// temp_vector3 = K1 * X * m1 + K1 * x + k1 - (K2 * X * m2 + K2 * x + k2)
		gsl_vector_sub(temp_vector3, temp_vector2);

		wynik += transposed_vector_multiply(temp_vector3, temp_vector3);
	}

	// free allocated memory
	gsl_matrix_free(temp_matrix);
	gsl_matrix_free(temp_matrix2);
	gsl_matrix_free(temp_matrix3);
	gsl_vector_free(angles);
	gsl_vector_free(m);
	gsl_vector_free(m2);
	gsl_vector_free(k);
	gsl_vector_free(k2);
	gsl_vector_free(x);
	gsl_vector_free(temp_vector);
	gsl_vector_free(temp_vector2);
	gsl_vector_free(temp_vector3);
	gsl_matrix_free(K);
	gsl_matrix_free(M);
	gsl_matrix_free(K2);
	gsl_matrix_free(M2);
	gsl_matrix_free(X);
	return wynik;
}

/*!
 * Calculate derivative of the objective function
 */
void calib_axxb::objective_function_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	int i;

	// value of derivative dx, dy, dz
	gsl_vector_set(df, 0, 0.0);
	gsl_vector_set(df, 1, 0.0);
	gsl_vector_set(df, 2, 0.0);
	// value of derivative dfi, dteta and dpsi
	gsl_vector_set(df, 3, 0.0);
	gsl_vector_set(df, 4, 0.0);
	gsl_vector_set(df, 5, 0.0);

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

	// translation from robot base to gripper frame
	gsl_vector *k2;
	k2 = gsl_vector_calloc (3);

	// translation from chessboard frame to camera frame
	gsl_vector *m;
	m = gsl_vector_calloc (3);

	// translation from chessboard frame to camera frame
	gsl_vector *m2;
	m2 = gsl_vector_calloc (3);

	// translation from gripper frame to camera frame
	gsl_vector *x;
	x = gsl_vector_calloc (3);

	// rotation from gripper frame to camera frame
	gsl_matrix *X;
	X = gsl_matrix_calloc(3, 3);

	gsl_matrix *dX_dfi;
	dX_dfi = gsl_matrix_calloc(3, 3);

	gsl_matrix *dX_dteta;
	dX_dteta = gsl_matrix_calloc(3, 3);

	gsl_matrix *dX_dpsi;
	dX_dpsi = gsl_matrix_calloc(3, 3);

	// rotation from robot base to gripper frame
	gsl_matrix *K2;
	K2 = gsl_matrix_calloc(3, 3);

	// rotation from robot base to gripper frame
	gsl_matrix *K;
	K = gsl_matrix_calloc(3, 3);

	// rotation from chessboard frame to camera frame
	gsl_matrix *M2;
	M2 = gsl_matrix_calloc(3, 3);

	// rotation from chessboard frame to camera frame
	gsl_matrix *M;
	M = gsl_matrix_calloc(3, 3);

	objective_function_parameters *p = (objective_function_parameters *)params;

	// initialize x, X from vector v
	for(i = 0; i < 6; ++i)
	{
		if(i < 3)
		{
			// x, y, z
			gsl_vector_set(x, i, gsl_vector_get(v,i));
		}
		else if (i < 6)
		{
			// fi, teta, psi
			gsl_vector_set(angles, i-3, gsl_vector_get(v,i));
		}
	}

	// calculate matrix X from angles fi, teta, psi
	angles_to_rotation_matrix(angles, X);

	// calculate derivative of matrix X - dX/dfi
	angles_to_rotation_matrix_dalfa(angles, dX_dfi);

	// calculate derivative of matrix X - dX/dteta
	angles_to_rotation_matrix_dbeta(angles, dX_dteta);

	// calculate derivative of matrix X - dX/dpsi
	angles_to_rotation_matrix_dgamma(angles, dX_dpsi);

	for(i = 0; i < p->number_of_measures - 1; ++i)
	{
		// K = K1
		extract_matrix(p->K, K, i);

		// k = k1
		extract_vector(p->k, k, i);

		// K2 = K2
		extract_matrix(p->K, K2, i + 1);

		// k2 = k2
		extract_vector(p->k, k2, i + 1);

		// M = M1
		extract_matrix(p->M, M, i);

		// m = m1
		extract_vector(p->m, m, i);

		// M2 = M2
		extract_matrix(p->M, M2, i + 1);

		// m2 = m2
		extract_vector(p->m, m2, i + 1);

		// change transformation from object to camera -> camera to object
//		gsl_matrix_transpose(M);
//		matrix_vector_multiply(M, m);
		gsl_vector_scale(m, -1.0);
//		gsl_matrix_transpose(M2);
//		matrix_vector_multiply(M2, m2);
		gsl_vector_scale(m2, -1.0);

		// temp_matrix = K1
		gsl_matrix_memcpy (temp_matrix, K);

		// temp_matrix = K1 * X
		matrix_matrix_multiply(temp_matrix, X);

		// temp_matrix = K1 * X * M1
		matrix_matrix_multiply(temp_matrix, M);

		// temp_matrix2 = K2
		gsl_matrix_memcpy (temp_matrix2, K2);

		// temp_matrix2 = K2 * X
		matrix_matrix_multiply(temp_matrix2, X);

		// temp_matrix2 = K2 * X * M2
		matrix_matrix_multiply(temp_matrix2, M2);

		// temp_matrix = K1 * X * M1 - K2 * X * M2
		gsl_matrix_sub(temp_matrix, temp_matrix2);

		//temp_matrix3 = K1 * X * M1 - K2 * X * M2
		gsl_matrix_memcpy (temp_matrix3, temp_matrix);

		// temp_vector = x
		gsl_vector_memcpy(temp_vector, x);

		// temp_vector2 = m1
		gsl_vector_memcpy(temp_vector2, m);

		// temp_vector2 = X * m1
		matrix_vector_multiply(X, temp_vector2);

		// temp_vector2 = K1 * X * m1
		matrix_vector_multiply(K, temp_vector2);

		// temp_vector = K1 * x
		matrix_vector_multiply(K, temp_vector);

		// temp_vector2 = K1 * X * m1 + K1 * x
		gsl_vector_add(temp_vector2, temp_vector);

		// temp_vector2 = K1 * X * m1 + K1 * x + k1
		gsl_vector_add(temp_vector2, k);

		// temp_vector3 = K1 * X * m1 + K1 * x + k1
		gsl_vector_memcpy(temp_vector3, temp_vector2);

		// temp_vector = x
		gsl_vector_memcpy(temp_vector, x);

		// temp_vector2 = m2
		gsl_vector_memcpy(temp_vector2, m2);

		// temp_vector2 = X * m2
		matrix_vector_multiply(X, temp_vector2);

		// temp_vector2 = K2 * X * m2
		matrix_vector_multiply(K2, temp_vector2);

		// temp_vector = K2 * x
		matrix_vector_multiply(K2, temp_vector);

		// temp_vector2 = K2 * X * m2 + K2 * x
		gsl_vector_add(temp_vector2, temp_vector);

		// temp_vector2 = K2 * X * m2 + K2 * x + k2
		gsl_vector_add(temp_vector2, k2);

		// temp_vector3 = K1 * X * m1 + K1 * x + k1 - (K2 * X * m2 + K2 * x + k2)
		gsl_vector_sub(temp_vector3, temp_vector2);

		// temp_matrix = K1
		gsl_matrix_memcpy (temp_matrix, K);

		// temp_matrix = K1 - K2
		gsl_matrix_sub(temp_matrix, K2);

		// temp_vector = [0 0 0]^T
		gsl_vector_set_zero(temp_vector);

		// temp_vector = [1 0 0]^T
		gsl_vector_set(temp_vector, 0, 1.0);

		// temp_vector = (K1 - K2) * [1 0 0]^T
		matrix_vector_multiply(temp_matrix, temp_vector);

		// get previous value of dx
		wynik = gsl_vector_get(df, 0);
		// wynik += 2 * [K1 * X * m1 + K1 * x + k1 - (K2 * X * m2 + K2 * x + k2)]^T *
		// *[(K1 - K2) * [1 0 0]^T]
		wynik += 2.0 * transposed_vector_multiply(temp_vector, temp_vector3);
		// update value of dy
		gsl_vector_set(df, 0, wynik);

		// temp_vector = [0 0 0]^T
		gsl_vector_set_zero(temp_vector);

		// temp_vector = [0 1 0]^T
		gsl_vector_set(temp_vector, 1, 1.0);

		// temp_vector = (K1 - K2) * [0 1 0]^T
		matrix_vector_multiply(temp_matrix, temp_vector);

		// get previous value of dy
		wynik = gsl_vector_get(df, 1);
		// wynik += 2 * [K1 * X * m1 + K1 * x + k1 - (K2 * X * m2 + K2 * x + k2)]^T *
		// *[(K1 - K2) * [0 1 0]^T]
		wynik += 2.0 * transposed_vector_multiply(temp_vector, temp_vector3);
		// update value of dy
		gsl_vector_set(df, 1, wynik);

		// temp_vector = [0 0 0]^T
		gsl_vector_set_zero(temp_vector);

		// temp_vector = [0 0 1]^T
		gsl_vector_set(temp_vector, 2, 1.0);

		// temp_vector = (K1 - K2) * [0 0 1]^T
		matrix_vector_multiply(temp_matrix, temp_vector);

		// get previous value of dz
		wynik = gsl_vector_get(df, 2);
		// wynik += 2 * [K1 * X * m1 + K1 * x + k1 - (K2 * X * m2 + K2 * x + k2)]^T *
		// *[(K1 - K2) * [0 0 1]^T]
		wynik += 2.0 * transposed_vector_multiply(temp_vector, temp_vector3);
		// update value of dy
		gsl_vector_set(df, 2, wynik);

		// dfi

		// temp_matrix = K1
		gsl_matrix_memcpy (temp_matrix, K);

		// temp_matrix = K1 * dX/dfi
		matrix_matrix_multiply(temp_matrix, dX_dfi);

		// temp_matrix = K1 * dX/dfi * M1
		matrix_matrix_multiply(temp_matrix, M);

		// temp_matrix2 = K2
		gsl_matrix_memcpy (temp_matrix2, K2);

		// temp_matrix2 = K2 * dX/dfi
		matrix_matrix_multiply(temp_matrix2, dX_dfi);

		// temp_matrix2 = K2 * dX/dfi* M2
		matrix_matrix_multiply(temp_matrix2, M2);

		// temp_matrix = K1 * dX/dfi * M1 - K2 * dX/dfi * M2
		gsl_matrix_sub(temp_matrix, temp_matrix2);

		// temp_matrix2 = K1 * dX/dfi * M1 - K2 * dX/dfi * M2
		gsl_matrix_memcpy (temp_matrix2, temp_matrix);

		// temp_matrix2 = [K1 * dX/dfi * M1 - K2 * dX/dfi * M2]^T
		gsl_matrix_transpose(temp_matrix2);

		// temp_matrix2 = [K1 * dX/dfi * M1 - K2 * dX/dfi * M2]^T * (K1 * X * M1 - K2 * X * M2)
		matrix_matrix_multiply(temp_matrix2, temp_matrix3);

		// get previous value of dfi
		wynik = gsl_vector_get(df, 3);
		// wynik += 2.0 * c * tr[(K1 * dX/dfi * M1 - K2 * dX/dfi * M2)^T * (K1 * X * M1 - K2 * X * M2)]
		wynik += 2.0 * (p->magical_c) * calculate_matrix_trace(temp_matrix2);

		// temp_vector = m1
		gsl_vector_memcpy(temp_vector, m);

		// temp_vector = dX/dfi * m1
		matrix_vector_multiply(dX_dfi, temp_vector);

		// temp_vector = K1 * dX/dfi * m1
		matrix_vector_multiply(K, temp_vector);

		// temp_vector2 = m2
		gsl_vector_memcpy(temp_vector2, m2);

		// temp_vector2 = dX/dfi * m2
		matrix_vector_multiply(dX_dfi, temp_vector2);

		// temp_vector2 = K2 * dX/dfi * m2
		matrix_vector_multiply(K2, temp_vector2);

		// temp_vector = K1 * dX/dfi * m1 - K2 * dX/dfi * m2
		gsl_vector_sub(temp_vector, temp_vector2);

		// wynik += 2.0 * [K1 * dX/dfi * m1 - K2 * dX/dfi * m2]^T * [K1 * X * m1 + K1 * x + k1 - (K2 * X * m2 + K2 * x + k2)]
		wynik += 2.0 * transposed_vector_multiply(temp_vector, temp_vector3);

		// update value of dfi
		gsl_vector_set(df, 3, wynik);

		// dteta

		// temp_matrix = K1
		gsl_matrix_memcpy (temp_matrix, K);

		// temp_matrix = K1 * dX/dteta
		matrix_matrix_multiply(temp_matrix, dX_dteta);

		// temp_matrix = K1 * dX/dteta * M1
		matrix_matrix_multiply(temp_matrix, M);

		// temp_matrix2 = K2
		gsl_matrix_memcpy (temp_matrix2, K2);

		// temp_matrix2 = K2 * dX/dteta
		matrix_matrix_multiply(temp_matrix2, dX_dteta);

		// temp_matrix2 = K2 * dX/dteta* M2
		matrix_matrix_multiply(temp_matrix2, M2);

		// temp_matrix = K1 * dX/dteta * M1 - K2 * dX/dteta * M2
		gsl_matrix_sub(temp_matrix, temp_matrix2);

		// temp_matrix2 = K1 * dX/dteta * M1 - K2 * dX/dteta * M2
		gsl_matrix_memcpy (temp_matrix2, temp_matrix);

		// temp_matrix2 = [K1 * dX/dteta * M1 - K2 * dX/dteta * M2]^T
		gsl_matrix_transpose(temp_matrix2);

		// temp_matrix2 = [K1 * dX/dteta * M1 - K2 * dX/dteta * M2]^T * (K1 * X * M1 - K2 * X * M2)
		matrix_matrix_multiply(temp_matrix2, temp_matrix3);

		// get previous value of dteta
		wynik = gsl_vector_get(df, 4);
		// wynik += 2.0 * c * tr[(K1 * dX/dteta * M1 - K2 * dX/dteta * M2)^T * (K1 * X * M1 - K2 * X * M2)]
		wynik += 2.0 * (p->magical_c) * calculate_matrix_trace(temp_matrix2);

		// temp_vector = m1
		gsl_vector_memcpy(temp_vector, m);

		// temp_vector = dX/dteta * m1
		matrix_vector_multiply(dX_dteta, temp_vector);

		// temp_vector = K1 * dX/dteta * m1
		matrix_vector_multiply(K, temp_vector);

		// temp_vector2 = m2
		gsl_vector_memcpy(temp_vector2, m2);

		// temp_vector2 = dX/dteta * m2
		matrix_vector_multiply(dX_dteta, temp_vector2);

		// temp_vector2 = K2 * dX/dteta * m2
		matrix_vector_multiply(K2, temp_vector2);

		// temp_vector = K1 * dX/dteta * m1 - K2 * dX/dteta * m2
		gsl_vector_sub(temp_vector, temp_vector2);

		// wynik += 2.0 * [K1 * dX/dteta * m1 - K2 * dX/dteta * m2]^T * [K1 * X * m1 + K1 * x + k1 - (K2 * X * m2 + K2 * x + k2)]
		wynik += 2.0 * transposed_vector_multiply(temp_vector, temp_vector3);

		// update value of dteta
		gsl_vector_set(df, 4, wynik);

		// dpsi

		// temp_matrix = K1
		gsl_matrix_memcpy (temp_matrix, K);

		// temp_matrix = K1 * dX/dpsi
		matrix_matrix_multiply(temp_matrix, dX_dpsi);

		// temp_matrix = K1 * dX/dpsi * M1
		matrix_matrix_multiply(temp_matrix, M);

		// temp_matrix2 = K2
		gsl_matrix_memcpy (temp_matrix2, K2);

		// temp_matrix2 = K2 * dX/dpsi
		matrix_matrix_multiply(temp_matrix2, dX_dpsi);

		// temp_matrix2 = K2 * dX/dpsi* M2
		matrix_matrix_multiply(temp_matrix2, M2);

		// temp_matrix = K1 * dX/dpsi * M1 - K2 * dX/dpsi * M2
		gsl_matrix_sub(temp_matrix, temp_matrix2);

		// temp_matrix2 = K1 * dX/dpsi * M1 - K2 * dX/dpsi * M2
		gsl_matrix_memcpy (temp_matrix2, temp_matrix);

		// temp_matrix2 = [K1 * dX/dpsi * M1 - K2 * dX/dpsi * M2]^T
		gsl_matrix_transpose(temp_matrix2);

		// temp_matrix2 = [K1 * dX/dpsi * M1 - K2 * dX/dpsi * M2]^T * (K1 * X * M1 - K2 * X * M2)
		matrix_matrix_multiply(temp_matrix2, temp_matrix3);

		// get previous value of dpsi
		wynik = gsl_vector_get(df, 5);
		// wynik += 2.0 * c * tr[(K1 * dX/dpsi * M1 - K2 * dX/dpsi * M2)^T * (K1 * X * M1 - K2 * X * M2)]
		wynik += 2.0 * (p->magical_c) * calculate_matrix_trace(temp_matrix2);

		// temp_vector = m1
		gsl_vector_memcpy(temp_vector, m);

		// temp_vector = dX/dpsi * m1
		matrix_vector_multiply(dX_dpsi, temp_vector);

		// temp_vector = K1 * dX/dpsi * m1
		matrix_vector_multiply(K, temp_vector);

		// temp_vector2 = m2
		gsl_vector_memcpy(temp_vector2, m2);

		// temp_vector2 = dX/dpsi * m2
		matrix_vector_multiply(dX_dpsi, temp_vector2);

		// temp_vector2 = K2 * dX/dpsi * m2
		matrix_vector_multiply(K2, temp_vector2);

		// temp_vector = K1 * dX/dpsi * m1 - K2 * dX/dpsi * m2
		gsl_vector_sub(temp_vector, temp_vector2);

		// wynik += 2.0 * [K1 * dX/dpsi * m1 - K2 * dX/dpsi * m2]^T * [K1 * X * m1 + K1 * x + k1 - (K2 * X * m2 + K2 * x + k2)]
		wynik += 2.0 * transposed_vector_multiply(temp_vector, temp_vector3);

		// update value of dpsi
		gsl_vector_set(df, 5, wynik);
	}

	// free allocated memory
	gsl_matrix_free(temp_matrix);
	gsl_matrix_free(temp_matrix2);
	gsl_matrix_free(temp_matrix3);
	gsl_vector_free(angles);
	gsl_vector_free(m);
	gsl_vector_free(m2);
	gsl_vector_free(k);
	gsl_vector_free(k2);
	gsl_vector_free(x);
	gsl_vector_free(temp_vector);
	gsl_vector_free(temp_vector2);
	gsl_vector_free(temp_vector3);
	gsl_matrix_free(K);
	gsl_matrix_free(M);
	gsl_matrix_free(K2);
	gsl_matrix_free(M2);
	gsl_matrix_free(X);
	gsl_matrix_free(dX_dfi);
	gsl_matrix_free(dX_dteta);
	gsl_matrix_free(dX_dpsi);
}

/*!
 * Calculate objective function and her derivative together.
 */
void calib_axxb::objective_function_fdf(const gsl_vector *v, void *params, double *f, gsl_vector *df)
{
	*f = objective_function(v, params);
	objective_function_df(v, params, df);
}

task* return_created_ecp_task (lib::configurator &_config)
{
	return new calib_axxb(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
