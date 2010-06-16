/**
 * \file ecp_g_vis_ib_eih_follower_irp6ot.h
 *
 * \date 15-08-2009
 * \author rtulwin
 */

#ifndef ECP_VIS_IB_EIH_FOLLOWER_IRP6OT_H_
#define ECP_VIS_IB_EIH_FOLLOWER_IRP6OT_H_

#include <iostream>
#include <string.h>
#include <math.h>

#include "ecp/common/generator/ecp_g_visual_servo.h"
#include "ecp_mp/sensor/ecp_mp_s_fradia_sensor.h"

// uchyb w follower
typedef struct _tracker {
	bool tracking;
	int x;
	int y;
} tracker_t;

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

#define MOTION_STEPS 25

typedef enum _HD_MODE
{
	WITHOUT_ROTATION, PERFORM_ROTATION
} hd_mode_t;

class ecp_vis_ib_eih_follower_irp6ot : public common::generator::ecp_visual_servo {

public:
	ecp_mp::sensor::fradia_sensor<lib::empty_t, tracker_t, hd_mode_t> *vsp_fradia; //wirtualny sensor

    double next_position[8]; 	//pozycja w nastepnym kroku.
    //double alpha; //orientacja koncokwi wzgledem ukladu bazowego.
    double u[2]; //uchyb

    //zmienne ruchu
    double a_max[3]; //maks przyspieszenie;
    double s[3]; //droga jednego makrokroku ruchu
    double v[3]; //aktualna predkosc
    double v_max[3]; //maks predkosc
    double v_stop[2]; //predkosc przy ktorej mozna sie zatrzymac
    double v_min[2]; //minimalna mozliwa do ustawienia predkosc maksymalna

    double s_z; //droga do przebycia w z
    double z_start; //pozycja poczatkowa z
    double s_acc; //droga przyspieszania, hamowania w z
    bool z_stop; //flaga zatrzymania z
    double z_s; //droga przebyta w z

    int dir[2]; //kierunki dla x i y
    bool change[2]; //flaga zmiany kierunki dla x i y
    bool reached[2]; // flaga dotarcia do celu dla x i y

	bool first_move; //flaga pierwszego makrokroku

	lib::Homog_matrix homog_matrix;

	ecp_vis_ib_eih_follower_irp6ot(common::task::task& _ecp_task);
	virtual bool first_step(void);
	virtual bool next_step_without_constraints();
	virtual void limit_step();
	void reduce_velocity(double a, double t, double s, int i);
};

} // namespace generator
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_VIS_IB_EIH_FOLLOWER_IRP6OT_H_ */
