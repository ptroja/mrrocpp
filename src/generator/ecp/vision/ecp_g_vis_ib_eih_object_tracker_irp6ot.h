/**
 * \file ecp_g_vis_ib_eih_object_tracker_irp6ot.h
 * \date 29-10-2009
 * \author rtulwin
 *
 * \brief
 */

#ifndef ECP_VIS_IB_EIH_OBJECT_TRACKER_IRP6OT_H_
#define ECP_VIS_IB_EIH_OBJECT_TRACKER_IRP6OT_H_

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "generator/ecp/ecp_g_visual_servo.h"
#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"
#include "sensor/fradia/object_tracker.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

#define MOTION_STEPS 25
#define MAX_AXES_NUM 3

typedef ecp_mp::sensor::fradia_sensor<lib::empty_t, ecp_mp::sensor::object_tracker_t> fradia_sensor_tracker;

/**
 *
 */
class ecp_vis_ib_eih_object_tracker_irp6ot: public common::generator::ecp_visual_servo {
public:
	/**
	 *
	 */
	fradia_sensor_tracker *vsp_fradia; //wirtualny sensor
	/**
	 *
	 */
	double next_position[lib::MAX_SERVOS_NR]; 	//pozycja w nastepnym kroku.
	/**
	 *
	 */
    double u[MAX_AXES_NUM]; //tablica uchybw
	/**
	 *
	 */
    bool tracking; //jesli true, obiekt jest sledzony, jesli false, zagubiony (przychodzi z fradii)
	/**
	 *
	 */
    double t;//czas makrokroku

    //zmienne ruchu
	/**
	 *
	 */
	double a_max[MAX_AXES_NUM]; //maks przyspieszenie;
	/**
	 *
	 */
    double s[MAX_AXES_NUM]; //droga jednego makrokroku ruchu
	/**
	 *
	 */
    double v[MAX_AXES_NUM]; //aktualna predkosc
	/**
	 *
	 */
    double v_max[MAX_AXES_NUM]; //maks predkosc
	/**
	 *
	 */
    double v_max_act[MAX_AXES_NUM]; //aktualna predkosc maksymalna
	/**
	 *
	 */
    double v_stop[MAX_AXES_NUM]; //predkosc przy ktorej mozna sie zatrzymac
	/**
	 *
	 */
    double v_min[MAX_AXES_NUM]; //minimalna mozliwa do ustawienia predkosc maksymalna
	/**
	 *
	 */
    double u_stop[MAX_AXES_NUM]; //uchyb przy ktorym sie zatrzymujemy (uchyb minimalny)
	/**
	 *
	 */
    double u_max[MAX_AXES_NUM]; //maximal possible error
	/**
	 *
	 */
    int dir[MAX_AXES_NUM]; //kierunki
	/**
	 *
	 */
    bool change[MAX_AXES_NUM]; //flaga zmiany kierunku
	/**
	 *
	 */
    bool reached[MAX_AXES_NUM]; // flaga dotarcia do celu
	/**
	 *
	 */
	lib::Homog_matrix homog_matrix;
	/**
	 *
	 */
	ecp_vis_ib_eih_object_tracker_irp6ot(common::task::task& _ecp_task);
	/**
	 *
	 */
	virtual bool first_step(void);
	/**
	 *
	 */
	virtual bool next_step_without_constraints();
	/**
	 *
	 */
	virtual void limit_step();

private:
	/**
	 *
	 */
	bool read_parametres();
};

} // namespace generator
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_VIS_IB_EIH_OBJECT_TRACKER_IRP6OT_H_ */
