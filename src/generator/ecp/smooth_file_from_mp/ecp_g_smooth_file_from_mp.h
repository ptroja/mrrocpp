#if !defined(_ECP_SUBTASK_SMOOTH_FILE_FROM_MP_H)
#define _ECP_SUBTASK_SMOOTH_FILE_FROM_MP_H

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_generator.h"
#include "ecp_mp_g_smooth_file_from_mp.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class newsmooth;

class smooth_file_from_mp : public common::generator::generator
{
private:
	boost::shared_ptr <newsmooth> sgen;
	std::string path;
	const bool detect_jerks;

public:
	smooth_file_from_mp(task::task & _ecp_t, lib::ECP_POSE_SPECIFICATION pose_spec, std::string _LABEL, bool _detect_jerks =
			true);

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
