// -------------------------------------------------------------------------
// Proces:
// Plik:			ecp_mp_t_rcsc.h
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Ogolna struktura obrazow czujnika
// Autor:		yoyek
// Data:		2007
// -------------------------------------------------------------------------

#ifndef __ECP_MP_T_MULTIPLAYER_H
#define __ECP_MP_T_MULTIPLAYER_H

namespace mrrocpp {
namespace ecp_mp {

enum RCSC_ECP_STATES {
	ECP_WEIGHT_MEASURE_GENERATOR,
	ECP_GEN_TRANSPARENT,
	ECP_GEN_BIAS_EDP_FORCE,
	MULTIPLAYER_GRIPPER_OPENING,
	ECP_GEN_SMOOTH,
	ECP_GEN_TAKE_FROM_ROVER,
	ECP_GEN_GRAB_FROM_ROVER
};

enum MULTIPLAYER_GRIPPER_OP {MULTIPLAYER_GO_VAR_1, MULTIPLAYER_GO_VAR_2};

} // namespace ecp_mp
} // namespace mrrocpp

#endif
