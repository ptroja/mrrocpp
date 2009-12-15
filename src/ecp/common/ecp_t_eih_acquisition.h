/// \file ecp_t_eih_acquisition.h
/// \brief definicja zadania akwizycji danych potrzebnych do kalibracji ukladu oko - reka
/// \author 2009 Jakub Kosiorek
///////////////////////////////////////////////////////////////////////////////

#if !defined(_ECP_T_EIH_ACQUISITION_H)
#define _ECP_T_EIH_ACQUISITION_H

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth2.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp_mp/sensor/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/common/ecp_t_acquisition.h"
#include "ecp/common/ecp_g_eihcalibration.h"
#include "gsl/gsl_vector.h"
#include "gsl/gsl_matrix.h"

#define POSTUMENT 0
#define TRACK 1

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

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
};


class eihacquisition: public common::task::acquisition {
	private:
		std::string smooth_path;
		int delay_ms, robot, M;
		double A, C, D, E;
		bool calibrated;

	protected:
		int write_data(void);
		// generator do wodzenia za nos
		generator::eih_nose_run* nose;
		// generator smooth2
		generator::smooth2* smooth2gen;
		// generator do wysylania danych do fradii
		generator::eihgenerator* generator;

	public:
		eihacquisition(lib::configurator &_config);

		void main_task_algorithm(void);

		bool store_data(void);

		objective_function_parameters ofp;
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
