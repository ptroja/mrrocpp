#include <stdexcept>

#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/irp6p_m/ecp_r_irp6p_m.h"

#include "ecp_t_axzb_eih.h"
#include "ecp_st_acq_eih.h"

using namespace std;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
axzb_eih::axzb_eih(lib::configurator &_config) :
	calib_axzb(_config)
{
}

void axzb_eih::main_task_algorithm(void)
{
	printf("void axzb_eih::main_task_algorithm(void) 1\n");
	fflush(stdout);
	ofp.number_of_measures = config.value <int> ("measures_count");
	ofp.magical_c = config.value <double> ("magical_c");
	std::string K_file_path = config.value <std::string> ("K_file_path");
	std::string kk_file_path = config.value <std::string> ("kk_file_path");
	std::string M_file_path = config.value <std::string> ("M_file_path");
	std::string mm_file_path = config.value <std::string> ("mm_file_path");

	//run a subtask to get the data if needed
	if (config.value <int> ("acquire")) {
		// TODO: acq_eih jest do poprawy, patrz konstruktor
		acq_eih* acq_task = new acq_eih(*this);
		printf("void axzb_eih::main_task_algorithm(void) 1a\n");
		fflush(stdout);
		acq_task->write_data(K_file_path, kk_file_path, M_file_path, mm_file_path, ofp.number_of_measures);
		printf("void axzb_eih::main_task_algorithm(void) 1b\n");
		fflush(stdout);
		delete acq_task;
	}

	printf("void axzb_eih::main_task_algorithm(void) 2\n");
	fflush(stdout);
	//load the data
	// translation vector (from robot base to tool frame) - received from MRROC
	ofp.k = gsl_vector_calloc(3 * ofp.number_of_measures);
	// rotation matrix (from robot base to tool frame) - received from MRROC
	ofp.K = gsl_matrix_calloc(3 * ofp.number_of_measures, 3);
	// translation vector (from chessboard base to camera frame)
	ofp.m = gsl_vector_calloc(3 * ofp.number_of_measures);
	// rotation matrix (from chessboard base to camera frame)
	ofp.M = gsl_matrix_calloc(3 * ofp.number_of_measures, 3);
	FILE *FP;
	if ((FP = fopen(K_file_path.c_str(), "r")) == NULL) {
		throw runtime_error("fopen(K_file_path = " + K_file_path + "): " + string(strerror(errno)));
	}
	gsl_matrix_fscanf(FP, ofp.K);
	fclose(FP);
	if ((FP = fopen(kk_file_path.c_str(), "r")) == NULL) {
		throw runtime_error("fopen(kk_file_path = " + kk_file_path + "): " + string(strerror(errno)));
	}
	gsl_vector_fscanf(FP, ofp.k);
	fclose(FP);
	if ((FP = fopen(M_file_path.c_str(), "r")) == NULL) {
		throw runtime_error("fopen(M_file_path = " + M_file_path + "): " + string(strerror(errno)));
	}
	gsl_matrix_fscanf(FP, ofp.M);
	fclose(FP);
	if ((FP = fopen(mm_file_path.c_str(), "r")) == NULL) {
		throw runtime_error("fopen(mm_file_path = " + mm_file_path +"): " + string(strerror(errno)));
	}
	gsl_vector_fscanf(FP, ofp.m);
	fclose(FP);

	printf("void axzb_eih::main_task_algorithm(void) 3\n");
	fflush(stdout);
	calib_axzb::main_task_algorithm();
	printf("void axzb_eih::main_task_algorithm(void) 4\n");
	fflush(stdout);

	ecp_termination_notice();
}

task* return_created_ecp_task(lib::configurator &_config)
{
	return new axzb_eih(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
