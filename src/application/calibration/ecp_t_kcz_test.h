#if !defined(_ECP_T_KCZ_TEST_H)
#define _ECP_T_KCZ_TEST_H

#include "base/ecp/ecp_task.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"

namespace mrrocpp {

namespace lib {
	class configurator;
}

namespace ecp {
namespace common {
namespace task {

class kcz_test: public common::task::task {

  protected:
	  common::generator::newsmooth* smoothgen2;

  public:
	  kcz_test(lib::configurator &_config);

	  void main_task_algorithm(void);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif

