/// \file task/ecp_t_eih_acquisition.h
/// \brief definicja zadania akwizycji danych potrzebnych do kalibracji ukladu oko - reka
/// \author 2009 Jakub Kosiorek
///////////////////////////////////////////////////////////////////////////////

#if !defined(_ECP_ST_ACQ_EIH_H)
#define _ECP_ST_ACQ_EIH_H

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp/common/generator/ecp_g_transparent.h"
#include "ecp/common/generator/ecp_g_force.h"
#include "ecp_g_eih_nose_run.h"
#include "ecp_st_acquisition.h"
#include "ecp_g_eihcalibration.h"
#include "gsl/gsl_vector.h"
#include "gsl/gsl_matrix.h"

#define POSTUMENT 0
#define TRACK 1

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class acq_eih: public acquisition {
private:
	std::string smooth_path;
	int delay_ms, robot, M;
	double A, C, D, E, vel, acc;
	bool calibrated;
	struct objective_function_parameters {
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
	} ofp;

		ecp_mp::sensor::fradia_sensor<lib::empty_t, chessboard_t, eihcalibration_t> *fradia;
	protected:
		std::string K_fp;
		std::string kk_fp;
		std::string M_fp;
		std::string mm_fp;

	// generator do wodzenia za nos
	generator::eih_nose_run* nose;
	// generator smooth
	generator::smooth* smoothgen;
	// generator do wysylania danych do fradii
	generator::eihgenerator* generator;
	bool store_data(void);
	void main_task_algorithm(void);

public:
	acq_eih(task &_ecp_t);
	void write_data(std::string _K_fp, std::string _k_fp, std::string _M_fp,
			std::string _m_fp, int _number_of_measures);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
