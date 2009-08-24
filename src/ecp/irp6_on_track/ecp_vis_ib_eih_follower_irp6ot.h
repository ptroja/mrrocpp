/*
 * ecp_vis_ib_eih_follower_irp6ot.h
 *
 *  Created on: Dec 9, 2008
 *      Author: rtulwin
 */

#ifndef ECP_VIS_IB_EIH_FOLLOWER_IRP6OT_H_
#define ECP_VIS_IB_EIH_FOLLOWER_IRP6OT_H_

#include "ecp/common/ecp_visual_servo.h"

#include <string.h>
#include <math.h>

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

#include <iostream>

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

#define MOTION_STEPS 25


class ecp_vis_ib_eih_follower_irp6ot: public common::ecp_visual_servo {

public:
	//Wirtualny sensor
	lib::sensor *vsp_fradia;
	//Pozycja w nastepnym kroku.
    double next_position[8];
    //Orientacja koncokwi wzgledem ukladu bazowego.
    double alpha;

    //zmienne ruchu
    double a_max[3]; //maks przyspieszenie;
    double s[3]; //droga jednego makrokroku ruchu
    double v[3]; //aktualna predkosc
    double v_max[3]; //maks predkosc;

    double s_z; //droga do przebycia w z
    double z_start; //pozycja początkowa z
    double s_acc; //droga przyspieszania, hamowania w z
    bool z_stop;

    //int z_counter;
	//flaga pierwszego makrokroku
	bool first_move;

	ecp_vis_ib_eih_follower_irp6ot(common::task::task& _ecp_task);
	virtual ~ecp_vis_ib_eih_follower_irp6ot();
	virtual bool first_step(void);
	virtual bool next_step_without_constraints();
	virtual void entertain_constraints();
};

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_VIS_IB_EIH_FOLLOWER_IRP6OT_H_ */


