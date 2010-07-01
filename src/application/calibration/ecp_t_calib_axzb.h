#if !defined(_ECP_T_CALIB_AXZB_H)
#define _ECP_T_CALIB_AXZB_H

#include <string.h>
#include <unistd.h>
#include "ecp_t_calibration.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class calib_axzb: public common::task::calibration  {
	public:
		// KONSTRUKTORY
		calib_axzb(lib::configurator &_config);

		void main_task_algorithm(void);

		static double objective_function (const gsl_vector *v, void *params);
		static void objective_function_df(const gsl_vector *v, void *params, gsl_vector *df);
		static void objective_function_fdf(const gsl_vector *v, void *params, double *f, gsl_vector *df);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
