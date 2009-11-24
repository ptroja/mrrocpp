// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP spm
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------



#ifndef __SG_SPM_H
#define __SG_SPM_H

#include "edp/common/edp.h"

#include "edp/common/servo_gr.h"

namespace mrrocpp {
namespace edp {
namespace spm {


class servo_buffer : public common::servo_buffer
{
		// Bufor polecen przysylanych z EDP_MASTER dla SERVO
		// Obiekt z algorytmem regulacji


	public:
		effector &master;


		servo_buffer(effector &_master); // konstruktor
		~servo_buffer(void); // destruktor
};

} // namespace spm
} // namespace edp
} // namespace mrrocpp



#endif
