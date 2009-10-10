// -------------------------------------------------------------------------
// Plik:			ecp_mp_t_pouring.h
// Autor:		Przemek Pilacinski
// Data:		2008
// -------------------------------------------------------------------------

#ifndef __ECP_MP_T_POURING_H
#define __ECP_MP_T_POURING_H

namespace mrrocpp {
namespace ecp_mp {
namespace task {

enum POURING_ECP_STATES {
	ECP_GEN_TRANSPARENT,
	ECP_GEN_POURING,
	ECP_END_POURING,
	ECP_GEN_SMOOTH,
	GRIP,
	LET_GO,
	WEIGHT
};

enum POURING_GRIPPER_OP {
	RCSC_GO_VAR_1,
	RCSC_GO_VAR_2
};

enum POURING_PHASES {
	POURING_PHASE_1,
	POURING_PHASE_2
};

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif
