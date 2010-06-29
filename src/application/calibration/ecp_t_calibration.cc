#include "ecp_t_calibration.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
calibration::calibration(lib::configurator &_config) : task(_config)
{
}

void calibration::main_task_algorithm(void)
{
	int i;
	char buffer[20]; //for sprintf

	fdf.params = (void*)&ofp;

	//calibration
	gsl_vector *x, *temp;
	gsl_matrix *X;

	// initialize starting point
	x = gsl_vector_calloc(dimension);

	temp = gsl_vector_calloc(3);
	X = gsl_matrix_calloc(3, 3);

	int status;
	size_t count = 0;

	const gsl_multimin_fdfminimizer_type *T2;
	gsl_multimin_fdfminimizer *s2;
	T2 = gsl_multimin_fdfminimizer_vector_bfgs2;
	s2 = gsl_multimin_fdfminimizer_alloc (T2, dimension);

	sr_ecp_msg->message("");

	count = 0;

	gsl_multimin_fdfminimizer_set (s2, &fdf, x, 0.1, 0.1);

	do
	{
		++count;
		status = gsl_multimin_fdfminimizer_iterate (s2);

		if (status) { //check if solver is stuck
			sr_ecp_msg->message("Solver is stuck");
			break;
		}

		status = gsl_multimin_test_gradient (s2->gradient, 1e-5);

		if (status == GSL_SUCCESS)
			sr_ecp_msg->message("Converged to minimum at");

//		sprintf(buffer, "%i", count);
//		sr_ecp_msg->message(buffer);
//		for (i = 0; i < dimension; ++i){
//			cout<<gsl_vector_get(s2->x,i)<<" ";
//		}
	}
	while (status == GSL_CONTINUE && count < 1000);

//	sprintf(buffer, "%f6.3", ofp.magical_c);
//	sr_ecp_msg->message(buffer);

	for (i = 0; i < dimension; ++i){
		sprintf(buffer, "%f6.3", gsl_vector_get(s2->x,i));
		sr_ecp_msg->message(buffer);
		if(i < 3){
			gsl_vector_set(temp, i, gsl_vector_get(s2->x, i));
		}
//		if (i % 3 == 2)
//			sr_ecp_msg->message("\t");
	}
	angles_to_rotation_matrix(temp, X);

	printf("X = [");
	fflush(stdout);
	for (i = 0; i < 3; ++i){
		for (int j = 0; j < 3; ++j){
			printf("%0.15lg ", gsl_matrix_get(X, i, j));
			fflush(stdout);
		}
		if(i == 2){
			printf("%0.15lg]", gsl_vector_get(temp, i));
			fflush(stdout);
		}else{
			printf("%0.15lg; ", gsl_vector_get(temp, i));
			fflush(stdout);
		}
	}

//	sr_ecp_msg->message("Function value=");
//	sprintf(buffer, "%f6.3", s2->f);
//	sr_ecp_msg->message(buffer);

	gsl_multimin_fdfminimizer_free (s2);
}

/*!
 * Function to multiply matrix with matrix
 */
bool calibration::matrix_matrix_multiply(gsl_matrix * matrix1, const gsl_matrix * matrix2)
{
	if(matrix1->size2 != matrix2->size2 || matrix1->size1 != matrix2->size1 || matrix1->size1 != matrix1->size2)
	{
		sr_ecp_msg->message("rozmiary macierzy sie nie zgadzaja - musza byc kwadratowe o takich samych wymiarach");
		return false;
	}

	// temporary matrix
	gsl_matrix * temp;
	temp = gsl_matrix_calloc (matrix1->size2,matrix1->size2);
	double wynik;

	for (int a = 0; a < matrix1->size1; ++a)
	{
		for (int b = 0; b < matrix2->size1; ++b)
		{
			wynik = 0.0;
			for (int i = 0; i < matrix2->size1; ++i)
				wynik += gsl_matrix_get(matrix1, a, i) * gsl_matrix_get(matrix2, i, b);
			gsl_matrix_set (temp, a, b, wynik);
		}
	}
	// substitute matrix1 with temp
	gsl_matrix_memcpy (matrix1, temp);

	//release temporary matrix
	gsl_matrix_free(temp);

	return true;
}

/*!
 * Function to multiply vector with vector (first is transposed)
 */
double calibration::transposed_vector_multiply(const gsl_vector * transposed, const gsl_vector * vector)
{
	// vector sizes must be equal
	if(transposed->size != vector->size)
	{
		sr_ecp_msg->message("wektory o roznych rozmiarach");
		return 0.0;
	}
	double wynik = 0.0;
	// temporary vector to copy first vector
	gsl_vector * temp;
	temp = gsl_vector_calloc (transposed->size);
	// copy vector (temp = transposed)
	gsl_vector_memcpy (temp, transposed);
	// multiply two vectors (a = a * b ; a1 = a1 * b1, a2 = a2 * b2 ...)
	gsl_vector_mul (temp, vector);
	// calculate a1 + a2 + ...
	for(int i = 0; i < temp->size; ++i)
		wynik += gsl_vector_get (temp, i);
	gsl_vector_free(temp);
	return wynik;
}

/*!
 * Function to multiply matrix with vector
 */
bool calibration::matrix_vector_multiply(const gsl_matrix * matrix, gsl_vector * vector)
{
	int i,j;
	double temp[vector->size];
	// we multiply only square matrix with equal size vector
	if(matrix->size2 != vector->size || matrix->size1 != vector->size )
	{
		sr_ecp_msg->message("rozmiar macierzy i wektora sie nie zgadzaja");
		return false;
	}

	// initiate temporary variable with zeros
	for (j = 0; j < vector->size; ++j)
	{
		temp[j] = 0.0;
	}

	// calculate a = A * b
	for (i = 0; i < vector->size; ++i)
	{
		for (j = 0; j < vector->size; ++j)
		{
			temp[i] += gsl_matrix_get(matrix, i, j) * gsl_vector_get(vector, j);
		}
	}

	// substitute temp into vector
	for (j = 0; j < vector->size; ++j)
	{
		gsl_vector_set(vector, j, temp[j]);
	}

	return true;
}

/*!
 * Function to calculate angles from rotation matrix (angles in roll pitch and yaw)
 */
bool calibration::rotation_matrix_to_angles(const gsl_matrix * rotation, gsl_vector * angles)// angles = [alfa, beta, gamma]
{
	if(rotation->size2 != angles->size || rotation->size1 != angles->size || angles->size != 3)
	{
		sr_ecp_msg->message("rozmiar macierzy i wektora sie nie zgadzaja");
		return false;
	}

	double x,y;
	x = gsl_matrix_get(rotation, 0, 0);
	y = gsl_matrix_get(rotation, 1, 0);
	// alpha = atan2(r21, r11)
	gsl_vector_set(angles, 0, atan2(y,x));

	x = sqrt( pow( gsl_matrix_get( rotation, 0, 0 ), 2) + pow(gsl_matrix_get(rotation, 1, 0),2));
	y = -gsl_matrix_get(rotation, 2, 0);
	// beta = atan2(-r31, sqrt(r11^2 + r21^2))
	gsl_vector_set(angles, 1, atan2(y,x));

	x = gsl_matrix_get(rotation, 2, 2);
	y = gsl_matrix_get(rotation, 2, 1);
	// gamma = atan2(r32, r33)
	gsl_vector_set(angles, 2, atan2(y,x));

	return true;
}

/*!
 * Function to calculate rotation matrix from angles
 */
bool calibration::angles_to_rotation_matrix(const gsl_vector * angles, gsl_matrix * rotation) // angles = [alfa, beta, gamma]
{
	if(rotation->size2 != angles->size || rotation->size1 != angles->size || angles->size != 3)
	{
		sr_ecp_msg->message("rozmiar macierzy i wektora sie nie zgadzaja");
		return false;
	}

	// set rotation to identity matrix
	gsl_matrix_set_identity(rotation);

	gsl_matrix * temp;
	temp = gsl_matrix_calloc (3,3);
	// set temp to identity matrix
	gsl_matrix_set_identity(temp);

	// rotation angles
	double alfa, beta, gamma;
	alfa = gsl_vector_get(angles, 0);
	beta = gsl_vector_get(angles, 1);
	gamma = gsl_vector_get(angles, 2);

/*						1		0		0
		rotation = [	0		cos(alfa)	-sin	]
						0		sin		cos		*/
	gsl_matrix_set (rotation, 1, 1, cos(alfa));
	gsl_matrix_set (rotation, 1, 2, -1.0*sin(alfa));
	gsl_matrix_set (rotation, 2, 1, sin(alfa));
	gsl_matrix_set (rotation, 2, 2, cos(alfa));

	/*			cos(beta)	0		sin
	  	 temp	 = [	0		1		0	]
				-sin		0		cos		*/
	gsl_matrix_set (temp, 0, 0, cos(beta));
	gsl_matrix_set (temp, 2, 0, -1.0*sin(beta));
	gsl_matrix_set (temp, 0, 2, sin(beta));
	gsl_matrix_set (temp, 2, 2, cos(beta));

	// rotation(alfa,beta) = rotation(alfa) * temp(beta)
	matrix_matrix_multiply(rotation, temp);
	// set temp to identity matrix
	gsl_matrix_set_identity(temp);

/*				cos(gamma)	-sin		0
		temp = [	sin		cos		0	]
				0		0		1		*/
	gsl_matrix_set (temp, 0, 0, cos(gamma));
	gsl_matrix_set (temp, 0, 1, -1.0*sin(gamma));
	gsl_matrix_set (temp, 1, 0, sin(gamma));
	gsl_matrix_set (temp, 1, 1, cos(gamma));


	// rotation(alfa,beta,gamma) = rotation(alfa,beta) * temp(gamma)
	matrix_matrix_multiply(rotation, temp);

	gsl_matrix_free(temp);
	return true;
}

/*!
 * Function to calculate derivative of rotation matrix from angles - dR/dalfa
 */
bool calibration::angles_to_rotation_matrix_dalfa(const gsl_vector * angles, gsl_matrix * rotation) // angles = [alfa, beta, gamma]
{
	if(rotation->size2 != angles->size || rotation->size1 != angles->size || angles->size != 3)
	{
		sr_ecp_msg->message("rozmiar macierzy i wektora sie nie zgadzaja");
		return false;
	}

	// set rotation to identity matrix
	gsl_matrix_set_identity(rotation);

	gsl_matrix * temp;
	temp = gsl_matrix_calloc (3,3);
	// set temp to identity matrix
	gsl_matrix_set_identity(temp);

	// rotation angles
	double alfa, beta, gamma;
	alfa = gsl_vector_get(angles, 0);
	beta = gsl_vector_get(angles, 1);
	gamma = gsl_vector_get(angles, 2);

/*				1		0		0
	rotation	=[	0		cos(alfa)	-sin	]
				0		sin		cos

				0		0		0
	rotation/dalfa	=[	0		-sin(alfa)	-cos	]
				0		cos		-sin
*/
	gsl_matrix_set (rotation, 1, 1, -1.0 * sin(alfa));
	gsl_matrix_set (rotation, 1, 2, -1.0 * cos(alfa));
	gsl_matrix_set (rotation, 2, 1, cos(alfa));
	gsl_matrix_set (rotation, 2, 2, -1.0 * sin(alfa));
	gsl_matrix_set (rotation, 0, 0, 0.0);

	/*			cos(beta)	0		sin
	  	 temp	 = [	0		1		0	]
				-sin		0		cos		*/
	gsl_matrix_set (temp, 0, 0, cos(beta));
	gsl_matrix_set (temp, 2, 0, -1.0*sin(beta));
	gsl_matrix_set (temp, 0, 2, sin(beta));
	gsl_matrix_set (temp, 2, 2, cos(beta));

	// rotation(alfa,beta) = rotation(alfa) * temp(beta)
	matrix_matrix_multiply(rotation, temp);
	// set temp to identity matrix
	gsl_matrix_set_identity(temp);

/*				cos(gamma)	-sin		0
		temp = [	sin		cos		0	]
				0		0		1		*/
	gsl_matrix_set (temp, 0, 0, cos(gamma));
	gsl_matrix_set (temp, 0, 1, -1.0*sin(gamma));
	gsl_matrix_set (temp, 1, 0, sin(gamma));
	gsl_matrix_set (temp, 1, 1, cos(gamma));

	// rotation(alfa,beta,gamma) = rotation(alfa,beta) * temp(gamma)
	matrix_matrix_multiply(rotation, temp);

	gsl_matrix_free(temp);
	return true;
}

/*!
 * Function to calculate derivative of rotation matrix from angles - dR/dbeta
 */
bool calibration::angles_to_rotation_matrix_dbeta(const gsl_vector * angles, gsl_matrix * rotation) // angles = [alfa, beta, gamma]
{
	if(rotation->size2 != angles->size || rotation->size1 != angles->size || angles->size != 3)
	{
		sr_ecp_msg->message("rozmiar macierzy i wektora sie nie zgadzaja");
		return false;
	}

	// set rotation to identity matrix
	gsl_matrix_set_identity(rotation);

	gsl_matrix * temp;
	temp = gsl_matrix_calloc (3,3);
	// set temp to identity matrix
	gsl_matrix_set_identity(temp);

	// rotation angles
	double alfa, beta, gamma;
	alfa = gsl_vector_get(angles, 0);
	beta = gsl_vector_get(angles, 1);
	gamma = gsl_vector_get(angles, 2);

/*				1		0		0
		rotation = [	0		cos(alfa)	-sin	]
				0		sin		cos		*/
	gsl_matrix_set (rotation, 1, 1, cos(alfa));
	gsl_matrix_set (rotation, 1, 2, -1.0*sin(alfa));
	gsl_matrix_set (rotation, 2, 1, sin(alfa));
	gsl_matrix_set (rotation, 2, 2, cos(alfa));

	/*			cos(beta)	0		sin
	  	 temp	 = [	0		1		0	]
				-sin		0		cos

				-sin(beta)	0		cos
	 temp/dbeta	= [	0		0		0	]
				-cos		0		-sin		*/
	gsl_matrix_set (temp, 0, 0, -1.0 * sin(beta));
	gsl_matrix_set (temp, 2, 0, -1.0 * cos(beta));
	gsl_matrix_set (temp, 0, 2, cos(beta));
	gsl_matrix_set (temp, 2, 2, -1.0 * sin(beta));
	gsl_matrix_set (temp, 1, 1, 0.0);

	// rotation(alfa,beta) = rotation(alfa) * temp(beta)
	matrix_matrix_multiply(rotation, temp);
	// set temp to identity matrix
	gsl_matrix_set_identity(temp);

/*				cos(gamma)	-sin		0
		temp = [	sin		cos		0	]
				0		0		1		*/
	gsl_matrix_set (temp, 0, 0, cos(gamma));
	gsl_matrix_set (temp, 0, 1, -1.0*sin(gamma));
	gsl_matrix_set (temp, 1, 0, sin(gamma));
	gsl_matrix_set (temp, 1, 1, cos(gamma));

	// rotation(alfa,beta,gamma) = rotation(alfa,beta) * temp(gamma)
	matrix_matrix_multiply(rotation, temp);

	gsl_matrix_free(temp);
	return true;
}

/*!
 * Function to calculate derivative of rotation matrix from angles - dR/dgamma
 */
bool calibration::angles_to_rotation_matrix_dgamma(const gsl_vector * angles, gsl_matrix * rotation) // angles = [alfa, beta, gamma]
{
	if(rotation->size2 != angles->size || rotation->size1 != angles->size || angles->size != 3)
	{
		sr_ecp_msg->message("rozmiar macierzy i wektora sie nie zgadzaja");
		return false;
	}

	// set rotation to identity matrix
	gsl_matrix_set_identity(rotation);

	gsl_matrix * temp;
	temp = gsl_matrix_calloc (3,3);
	// set temp to identity matrix
	gsl_matrix_set_identity(temp);

	// rotation angles
	double alfa, beta, gamma;
	alfa = gsl_vector_get(angles, 0);
	beta = gsl_vector_get(angles, 1);
	gamma = gsl_vector_get(angles, 2);

/*				1		0		0
		rotation = [	0		cos(alfa)	-sin	]
				0		sin		cos		*/
	gsl_matrix_set (rotation, 1, 1, cos(alfa));
	gsl_matrix_set (rotation, 1, 2, -1.0*sin(alfa));
	gsl_matrix_set (rotation, 2, 1, sin(alfa));
	gsl_matrix_set (rotation, 2, 2, cos(alfa));

	/*			cos(beta)	0		sin
	  	 temp	 = [	0		1		0	]
				-sin		0		cos		*/
	gsl_matrix_set (temp, 0, 0, cos(beta));
	gsl_matrix_set (temp, 2, 0, -1.0*sin(beta));
	gsl_matrix_set (temp, 0, 2, sin(beta));
	gsl_matrix_set (temp, 2, 2, cos(beta));

	// rotation(alfa,beta) = rotation(alfa) * temp(beta)
	matrix_matrix_multiply(rotation, temp);
	// set temp to identity matrix
	gsl_matrix_set_identity(temp);

/*				cos(gamma)	-sin		0
		temp = [	sin		cos		0	]
				0		0		1

				-sin(gamma)	-cos		0
	temp/dgamma	=[	cos		-sin		0	]
				0		0		0		*/
	gsl_matrix_set (temp, 0, 0, -1.0 * sin(gamma));
	gsl_matrix_set (temp, 0, 1, -1.0 * cos(gamma));
	gsl_matrix_set (temp, 1, 0, cos(gamma));
	gsl_matrix_set (temp, 1, 1, -1.0 * sin(gamma));
	gsl_matrix_set (temp, 2, 2, 0.0);

	// rotation(alfa,beta,gamma) = rotation(alfa,beta) * temp(gamma)
	matrix_matrix_multiply(rotation, temp);

	gsl_matrix_free(temp);
	return true;
}

/*!
 * Function to calculate matrix trace
 */
double calibration::calculate_matrix_trace(const gsl_matrix * matrix)
{
	if(matrix->size2 != matrix->size1 )
	{
		sr_ecp_msg->message("rozmiar macierzy i wektora sie nie zgadzaja");
		return 0;
	}
	double wynik = 0.0;
	// calculate trace(matrix) = a11 + a22 + ...
	for(int i = 0; i < matrix->size1; ++i)
		wynik += gsl_matrix_get (matrix, i, i);
	return wynik;
}

/*!
 * Function to extract translation vector from vector which stores all translation vectors
 */
bool calibration::extract_vector(const gsl_vector * source, gsl_vector * destination, int position)
{
	int i, j;
	if(position < 0)
	{
		sr_ecp_msg->message("Pozycja w wektorze jest < 0");
		return false;
	}
	for(i = 3 * position, j = 0; i < 3 * position + 3; ++i, ++j)
	{
		gsl_vector_set(destination, j, gsl_vector_get(source,i));
	}
	return true;
}

/*!
 * Function to extract rotation matrix from matrix which stores all rotation matrices
 */
bool calibration::extract_matrix(const gsl_matrix * source, gsl_matrix * destination, int position)
{
	int i, j, k;
	if(position < 0)
	{
		sr_ecp_msg->message("Pozycja w wektorze jest < 0");
		return false;
	}

	for(i = 3 * position, k = 0; i < 3 * position + 3; ++i, ++k)
	{
		for(j = 0; j < 3; ++j)
		{
			gsl_matrix_set(destination, k, j, gsl_matrix_get(source,i,j));
		}
	}
	return true;
}

/*!
 * Function to calculate vector (transposed) * matrix
 */
bool calibration::transposed_vector_matrix_multiply( gsl_vector * vector, const gsl_matrix * matrix)
{
	int i,j;
	double temp[vector->size];
	// we multiply only square matrix with equal size vector
	if(matrix->size2 != vector->size || matrix->size1 != vector->size )
	{
		sr_ecp_msg->message("rozmiar macierzy i wektora sie nie zgadzaja");
		return false;
	}

	// initiate temporary variable with zeros
	for (j = 0; j < vector->size; ++j)
	{
		temp[j] = 0.0;
	}

	// calculate a = A * b
	for (i = 0; i < vector->size; ++i)
	{
		for (j = 0; j < vector->size; ++j)
		{
			temp[i] += gsl_matrix_get(matrix, j, i) * gsl_vector_get(vector, j);
		}
	}

	// substitute temp into vector
	for (j = 0; j < vector->size; ++j)
	{
		gsl_vector_set(vector, j, temp[j]);
	}

	return true;
}

//task* calibration::return_created_ecp_task (lib::configurator &_config)
//{
//	return new calibration(_config);
//}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
