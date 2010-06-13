// -------------------------------------------------------------------------
// Plik:			ecp_mp_t_fsautomat.h
// Autor:		Marek Kisiel
// Data:			2008
// -------------------------------------------------------------------------

#ifndef __ECP_MP_T_FSAUTOMAT_H
#define __ECP_MP_T_FSAUTOMAT_H
/*enum POURING_ECP_STATES {ECP_GEN_TRANSPARENT = 0, ECP_GEN_POURING,
 ECP_END_POURING, ECP_GEN_SMOOTH, GRIP, LET_GO, WEIGHT
 };*/

namespace mrrocpp {
namespace ecp_mp {
namespace task {

enum POURING_GRIPPER_OP
{
	RCSC_GO_VAR_1, RCSC_GO_VAR_2
};

enum POURING_PHASES
{
	POURING_PHASE_1, POURING_PHASE_2
};

const std::string ECP_GEN_TRANSPARENT = "ECP_GEN_TRANSPARENT";
const std::string ECP_GEN_TFF_NOSE_RUN = "ECP_GEN_TFF_NOSE_RUN";
const std::string ECP_GEN_TEACH_IN = "ECP_GEN_TEACH_IN";
const std::string ECP_GEN_SMOOTH = "ECP_GEN_SMOOTH";
const std::string ECP_GEN_TFF_RUBIK_GRAB = "ECP_GEN_TFF_RUBIK_GRAB";
const std::string ECP_GEN_TFF_RUBIK_FACE_ROTATE = "ECP_GEN_TFF_RUBIK_FACE_ROTATE";
const std::string ECP_GEN_TFF_GRIPPER_APPROACH = "ECP_GEN_TFF_GRIPPER_APPROACH";
const std::string RCSC_GRIPPER_OPENING = "RCSC_GRIPPER_OPENING";
const std::string ECP_GEN_SPEAK = "ECP_GEN_SPEAK";
const std::string ECP_WEIGHT_MEASURE_GENERATOR = "ECP_WEIGHT_MEASURE_GENERATOR";
const std::string ECP_TOOL_CHANGE_GENERATOR = "ECP_TOOL_CHANGE_GENERATOR";

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif// -------------------------------------------------------------------------
