/// \file ecp_t_eihcalibration.h
/// \brief definicja zadania kalibracji ukladu oko - reka
/// \author 2009 Jakub Kosiorek
///////////////////////////////////////////////////////////////////////////////

#if !defined(_ECP_T_EIHCALIBRATION_H)
#define _ECP_T_EIHCALIBRATION_H

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp/common/ecp_g_eihcalibration.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class eihcalibration: public common::task::task {
	std::string smooth_path;

	protected:
		// generator do wodzenia za nos
		generator::eih_nose_run* nose;
		// generator smooth
		generator::smooth* smoothgen;
		// generator o zadany przyrost polozenia/orientacji
		generator::linear* linear_gen;
		// generator do wysylania danych do fradii
		generator::eihgenerator* generator;
		//Trajectory description.
		lib::trajectory_description td;

	public:
		eihcalibration(lib::configurator &_config);
		~eihcalibration();
		void task_initialization(void);
		void main_task_algorithm(void);

		//Metods modifing td.
		void set_td_coordinates(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6);
		void init_td(lib::POSE_SPECIFICATION ps, int internode_no);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
