
#if !defined(_ECP_T_KCZ_FORCE_H)
#define _ECP_T_KCZ_FORCE_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_force.h"
#include "application/kalibracja/ecp_t_acquisition.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class acq_force: public acquisition {

  protected:
	  common::generator::pcbird_nose_run* nose_run;

	  std::string K_fp;
	  std::string kk_fp;
	  std::string M_fp;
	  std::string mm_fp;
	  int number_of_measures;

  public:
	  acq_force(lib::configurator &_config);

	  void main_task_algorithm(void);

	  void write_data(std::string K_fp, std::string k_fp, std::string M_fp, std::string m_fp, int number_of_measures);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif

