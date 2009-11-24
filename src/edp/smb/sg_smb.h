// -------------------------------------------------------------------------
//                            sg_local.h
// Definicje struktur danych i metod dla procesu EDP smb
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------



#ifndef __SG_SMB_H
#define __SG_SMB_H

#include "edp/common/edp.h"

#include "edp/common/servo_gr.h"

namespace mrrocpp {
namespace edp {
namespace smb {


class servo_buffer : public common::servo_buffer
{
		// Bufor polecen przysylanych z EDP_MASTER dla SERVO
		// Obiekt z algorytmem regulacji


	public:
		effector &master;


		servo_buffer(effector &_master); // konstruktor
		~servo_buffer(void); // destruktor
};

} // namespace smb
} // namespace edp
} // namespace mrrocpp



#endif
