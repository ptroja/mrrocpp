// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_TYPEDEFS_H
#define __EDP_TYPEDEFS_H

namespace mrrocpp {
namespace edp {
namespace common {

enum MT_ORDER
{
	MT_GET_CONTROLLER_STATE, MT_SET_ROBOT_MODEL, MT_GET_ARM_POSITION, MT_GET_ALGORITHMS, MT_MOVE_ARM, MT_SYNCHRONISE
};

enum ERROR_TYPE
{
	NO_ERROR, Fatal_erroR, NonFatal_erroR_1, NonFatal_erroR_2, NonFatal_erroR_3, NonFatal_erroR_4, System_erroR
};

enum TRANSLATION_ENUM
{
	WITH_TRANSLATION, WITHOUT_TRANSLATION
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
