#if !defined(_ECP_T_CALIBRATION_H)
#define _ECP_T_CALIBRATION_H

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/configurator.h"
#include "base/ecp/ecp_task.h"
#include "gsl/gsl_vector.h"
#include "gsl/gsl_matrix.h"
#include "gsl/gsl_multimin.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class calibration: public common::task::task  {
	protected:
		gsl_multimin_function_fdf fdf;

		struct objective_function_parameters
		{
			// rotation matrix (from robot base to tool frame) - received from MRROC
			gsl_matrix *K;
			// rotation matrix (from chessboard base to camera frame)
			gsl_matrix *M;
			// translation vector (from robot base to tool frame) - received from MRROC
			gsl_vector *k;
			// translation vector (from chessboard base to camera frame)
			gsl_vector *m;
			// how many measurements were taken
			int number_of_measures;
			// coefficient to equalize rotation and translation parts in overall sum (proposed range: 300 - 2000)
			double magical_c;
		} ofp;

		int dimension;

		// KONSTRUKTORY
		calibration(lib::configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void main_task_algorithm(void);

	public:
		static bool matrix_matrix_multiply(gsl_matrix * matrix1, const gsl_matrix * matrix2);
		static double transposed_vector_multiply(const gsl_vector * transposed, const gsl_vector * vector);
		static bool matrix_vector_multiply(const gsl_matrix * matrix, gsl_vector * vector);
		static bool rotation_matrix_to_angles(const gsl_matrix * rotation, gsl_vector * angles); // angles = [alfa, beta, gamma]
		static bool angles_to_rotation_matrix(const gsl_vector * angles, gsl_matrix * rotation); // angles = [alfa, beta, gamma]
		static bool angles_to_rotation_matrix_dalfa(const gsl_vector * angles, gsl_matrix * rotation); // angles = [alfa, beta, gamma]
		static bool angles_to_rotation_matrix_dbeta(const gsl_vector * angles, gsl_matrix * rotation); // angles = [alfa, beta, gamma]
		static bool angles_to_rotation_matrix_dgamma(const gsl_vector * angles, gsl_matrix * rotation); // angles = [alfa, beta, gamma]
		static double calculate_matrix_trace(const gsl_matrix * matrix);
		static bool extract_vector(const gsl_vector * source, gsl_vector * destination, int position);
		static bool extract_matrix(const gsl_matrix * source, gsl_matrix * destination, int position);
		static bool transposed_vector_matrix_multiply( gsl_vector * vector, const gsl_matrix * matrix);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp



#endif
