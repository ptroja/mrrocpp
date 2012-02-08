#if !defined(_ECP_T_REFLEXXES_H)
#define _ECP_T_REFLEXXES_H

#include <memory>
#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_task.h"
#include "ecp_g_reflexxes.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class reflexxes : public common::task::task  {
protected:
	std::unique_ptr<common::generator::reflexxes> gen;

public:
	// KONSTRUKTORY
	reflexxes(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_T_REFLEXXES_H */
