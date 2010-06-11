// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_H
#define __EDP_H

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#define IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION  4000.0  // Liczba impulsow enkodera na obrot walu - musi byc float
#define IRP6_POSTUMENT_AXIS_6_INC_PER_REVOLUTION  2000.0  // Liczba impulsow enkodera na obrot walu - musi byc float
#define IRP6_POSTUMENT_AXIS_7_INC_PER_REVOLUTION  128.0  // Liczba impulsow enkodera na obrot walu - musi byc float
#define IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION   682.0  // Liczba impulsow rezolwera na obrot walu - musi byc float
#define IRP6_ON_TRACK_AXIS_6_INC_PER_REVOLUTION  2000.0  // Liczba impulsow enkodera na obrot walu - musi byc float
#define IRP6_ON_TRACK_AXIS_7_INC_PER_REVOLUTION  128.0  // Liczba impulsow enkodera na obrot walu - musi byc float
namespace mrrocpp {
namespace edp {
namespace common {

enum FORCE_ORDER
{
	FORCE_SET_TOOL, FORCE_CONFIGURE
};

enum MT_ORDER
{
	MT_GET_CONTROLLER_STATE, MT_SET_ROBOT_MODEL, MT_GET_ARM_POSITION, MT_GET_ALGORITHMS, MT_MOVE_ARM, MT_SYNCHRONISE
};

enum ERROR_TYPE
{
	NO_ERROR, Fatal_erroR, NonFatal_erroR_1, NonFatal_erroR_2, NonFatal_erroR_3, NonFatal_erroR_4, System_erroR
};

enum STATE
{
	GET_STATE, GET_SYNCHRO, SYNCHRO_TERMINATED, GET_INSTRUCTION, EXECUTE_INSTRUCTION, WAIT, WAIT_Q
};

enum TRANSLATION_ENUM
{
	WITH_TRANSLATION, WITHOUT_TRANSLATION
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
