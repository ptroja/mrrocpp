// -------------------------------------------------------------------------
// Plik:			ecp_mp_t_pouring.h
// Autor:		Przemek Pilacinski
// Data:		2008
// -------------------------------------------------------------------------

#ifndef __ECP_MP_T_POURING_H
#define __ECP_MP_T_POURING_H

enum POURING_ECP_STATES {ECP_GEN_TRANSPARENT = 0, ECP_GEN_POURING,
	ECP_END_POURING, ECP_GEN_SMOOTH, GRIP, LET_GO
};

enum POURING_GRIPPER_OP {RCSC_GO_VAR_1 = 0, RCSC_GO_VAR_2};

enum POURING_PHASES { POURING_PHASE_1 = 0, POURING_PHASE_2,
 };

#endif
