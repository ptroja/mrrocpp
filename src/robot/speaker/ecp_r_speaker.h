// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// robot - speaker
//
// -------------------------------------------------------------------------

#if !defined(_ECP_R_SPEAKER_H)
#define _ECP_R_SPEAKER_H

#include "base/ecp/ecp_robot.h"
#include "robot/speaker/const_speaker.h"
#include "robot/speaker/robot_buffers_speaker.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {

typedef ecp::common::robot::_ecp_robot<lib::speaker::c_buffer, lib::speaker::r_buffer> _base_robot_t;

//! Klasa dla robota speaker
class robot : public _base_robot_t
{
public:
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);

	robot(common::task::task_base& _ecp_object);
};

} // namespace speaker
} // namespace ecp
} // namespace mrrocpp

#endif
