#include <string.h>
#include <unistd.h>
#include <cmath>
#include <stdio.h>

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_g_smooth2.h"
#include "ecp/common/ecp_t_calibration.h"
#include "gsl/gsl_vector.h"
#include "gsl/gsl_matrix.h"
#include "gsl/gsl_multimin.h"

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
	int i,j,k;
	char buffer[20]; //for sprintf

	objective_function_parameters ofp;

	fdf.params = (void*)&ofp;

//TODO: START -> acq_task -> write_data(ofp)
	ofp.number_of_measures = 400;
	// translation vector (from robot base to tool frame) - received from MRROC
	ofp.k = gsl_vector_calloc (3*ofp.number_of_measures);
	// rotation matrix (from robot base to tool frame) - received from MRROC
	ofp.K = gsl_matrix_calloc (3*ofp.number_of_measures, 3);
	// translation vector (from chessboard base to camera frame)
	ofp.m = gsl_vector_calloc (3*ofp.number_of_measures);
	// rotation matrix (from chessboard base to camera frame)
	ofp.M = gsl_matrix_calloc (3*ofp.number_of_measures, 3);

	FILE *FP;
	FP = fopen("../trj/acquire/Mp27.txt", "r");
	gsl_matrix_fscanf(FP,ofp.M);
	fclose(FP);
	FP = fopen("../trj/acquire/mp27.txt","r");
	gsl_vector_fscanf(FP,ofp.m);
	fclose(FP);
	FP = fopen("../trj/acquire/kp27.txt","r");
	gsl_vector_fscanf(FP,ofp.k);
	fclose(FP);
	FP = fopen("../trj/acquire/Kp27.txt","r");
	gsl_matrix_fscanf(FP,ofp.K);
	fclose(FP);
//TODO: STOP -> acq_task -> write_data(ofp)

	////////////////// eih calibration
	gsl_vector *x, *step_size;

	// initialize starting point
	x = gsl_vector_calloc(12);
	step_size = gsl_vector_calloc(12);

	int status;
	size_t count = 0;
	double size;

	const gsl_multimin_fdfminimizer_type *T2;
	gsl_multimin_fdfminimizer *s2;
	T2 = gsl_multimin_fdfminimizer_vector_bfgs2;
	s2 = gsl_multimin_fdfminimizer_alloc (T2, 12);

	float temp, c, min = 1000.0;

//	cout.precision(3);
//	cout.width(6);
//	cout.setf(ios::fixed,ios::floatfield);
	for(ofp.magical_c = 0.1; ofp.magical_c < 200.64; ofp.magical_c *= 2.0)
	{
		gsl_vector_set(x, 0, 0.0);
		gsl_vector_set(x, 1, 0.0);
		gsl_vector_set(x, 2, 0.0);
		gsl_vector_set(x, 3, 0.0);
		gsl_vector_set(x, 4, 0.0);
		gsl_vector_set(x, 5, 0.0);
		gsl_vector_set(x, 6, 0.0);
		gsl_vector_set(x, 7, 0.0);
		gsl_vector_set(x, 8, 0.0);
		gsl_vector_set(x, 9, 0.0);
		gsl_vector_set(x, 10, 0.0);
		gsl_vector_set(x, 11, 0.0);

		sr_ecp_msg->message("");
		for(k = 0; k < 3; ++k)
		{
			count = 0;

			gsl_multimin_fdfminimizer_set (s2, &fdf, x, 0.1, 0.1);

			do
			{
				++count;
				status = gsl_multimin_fdfminimizer_iterate (s2);

				if (status)  //check if solver is stuck
					break;

				status = gsl_multimin_test_gradient (s2->gradient, 1e-5);

				if (status == GSL_SUCCESS)
					sr_ecp_msg->message("Converged to minimum at");

//				cout<<count<<endl;
//				sprintf(buffer, "%i", count);
//				sr_ecp_msg->message(buffer);

//				for (i = 0; i < 12; ++i){
//					cout<<gsl_vector_get(s2->x,i)<<" ";
//				}
//				cout<<"Function value="<< s2->f<< endl;
			}
			while (status == GSL_CONTINUE && count < 1000);

			if (k == 2)
			{
				sprintf(buffer, "%f6.3", ofp.magical_c);
				sr_ecp_msg->message(buffer);
//				cout<<endl<<ofp.magical_c<<endl;


				for (i = 0; i < 12; ++i){
					sprintf(buffer, "%f6.3", gsl_vector_get(s2->x,i));
					sr_ecp_msg->message(buffer);
//					cout<<gsl_vector_get(s2->x,i)<<" ";
					if (i % 3 == 2)
						sr_ecp_msg->message("\t");
				}

				sr_ecp_msg->message("Function value=");
				sprintf(buffer, "%f6.3", s2->f);
				sr_ecp_msg->message(buffer);
//				cout<<"Function value="<< s2->f<< endl;

				temp = sqrt(pow(gsl_vector_get(s2->x, 6), 2) + pow(gsl_vector_get(s2->x, 7), 2) + pow(gsl_vector_get(s2->x, 8), 2));

				sr_ecp_msg->message("srednio = ");
				sprintf(buffer, "%f6.3", temp);
				sr_ecp_msg->message(buffer);
//				cout <<"srednio = " << temp<<endl;

				gsl_vector *angles;
				gsl_matrix *matrix;
				angles = gsl_vector_calloc(3);
				gsl_vector_set(angles,0,gsl_vector_get(s2->x, 9));
				gsl_vector_set(angles,1,gsl_vector_get(s2->x, 10));
				gsl_vector_set(angles,2,gsl_vector_get(s2->x, 11));
				matrix = gsl_matrix_calloc(3, 3);
				angles_to_rotation_matrix(angles, matrix);
				gsl_vector_set(angles,0,gsl_vector_get(s2->x, 6));
				gsl_vector_set(angles,1,gsl_vector_get(s2->x, 7));
				gsl_vector_set(angles,2,gsl_vector_get(s2->x, 8));
				gsl_matrix_transpose(matrix);
				gsl_matrix_scale (matrix, -1.0);

				matrix_vector_multiply(matrix, angles);
				for(i = 0; i < 3; ++i){
					sprintf(buffer, "%f6.3", gsl_vector_get(angles, i));
					sr_ecp_msg->message(buffer);
					sr_ecp_msg->message("\t");
//					cout<<gsl_vector_get(angles, i)<<"\t";
				}

			}

			for (j = 0; j < 12; ++j){
				gsl_vector_set(x, j, gsl_vector_get(s2->x, j));
			}



		}
	}

	gsl_multimin_fdfminimizer_free (s2);

/*	gsl_multimin_function f;
	f.n = 12;
	f.f = &objective_function;
	f.params = (void*)&ofp;

	// minimizer and solver
	const gsl_multimin_fminimizer_type* T;
	T = gsl_multimin_fminimizer_nmsimplex2;
	gsl_multimin_fminimizer* solver;
	solver = gsl_multimin_fminimizer_alloc(T, 12);
	float temp, c, min = 1000.0;

	cout.precision(3);
	cout.width(6);
	cout.setf(ios::fixed,ios::floatfield);

	for(ofp.magical_c = 0.01; ofp.magical_c < 200.64; ofp.magical_c *= 2.0)
	{
		gsl_vector_set(x, 0, 0.7);
		gsl_vector_set(x, 1, 2.0);
		gsl_vector_set(x, 2, 0.0);
		gsl_vector_set(x, 3, 0.0);
		gsl_vector_set(x, 4, 0.0);
		gsl_vector_set(x, 5, 0.0);
		gsl_vector_set(x, 6, 0.0);
		gsl_vector_set(x, 7, 0.0);
		gsl_vector_set(x, 8, 0.0);
		gsl_vector_set(x, 9, 0.0);
		gsl_vector_set(x, 10, 0.0);
		gsl_vector_set(x, 11, 0.0);

		cout<<endl;
		for(k = 0; k < 5; ++k)
		{

	//		ofp.magical_c = 0.1;

			count = 0;

			//set initial step sizes
			for (j = 0; j < 12; ++j){
				gsl_vector_set(step_size, j, 6.5);
			}

			gsl_multimin_fminimizer_set (solver, &f, x, step_size);

			do
			{
				++count;
				status = gsl_multimin_fminimizer_iterate (solver);

				if (status)  //check if solver is stuck
					break;

				size = gsl_multimin_fminimizer_size(solver);

				status = gsl_multimin_test_size (size, 1e-1);

//				if (status == GSL_SUCCESS)
//					cout<<"Converged to minimum at"<<endl;

//				cout<<count<<endl;

//				for (i = 0; i < 12; ++i){
//					cout<<gsl_vector_get(solver->x,i)<<" ";
//				}
//				cout<<"Function value="<< solver->fval<< "size ="<<size << endl;
			}
			while (status == GSL_CONTINUE && count < 100);

			if (k == 4)
			{
				cout<<ofp.magical_c<<endl;


				for (i = 0; i < 12; ++i){
					cout<<gsl_vector_get(solver->x,i)<<" ";
					if (i % 3 == 2)
						cout<<"\t";
				}

//				cout<<"Function value="<< solver->fval<< "size ="<<size << endl;

				temp = sqrt(pow(gsl_vector_get(solver->x, 6), 2) + pow(gsl_vector_get(solver->x, 7), 2) + pow(gsl_vector_get(solver->x, 8), 2));

				cout <<endl<< "srednio = " << temp<< endl;
			}

			if(temp < min)
			{
				min = temp;
				c = ofp.magical_c;
			}

			for (j = 0; j < 12; ++j){
				gsl_vector_set(x, j, gsl_vector_get(solver->x, j));
			}
		}
	}
//	cout << " minimum" << min <<"  c = "<<c<<endl;

	gsl_multimin_fminimizer_free (solver);
*/
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
