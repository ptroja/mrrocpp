/*
 * ecp_vis_ib_eih_planar_irp6ot.h
 *
 *  Created on: Dec 9, 2008
 *      Author: pwilkows
 */

#ifndef ECP_VIS_IB_EIH_PLANAR_IRP6OT_H_
#define ECP_VIS_IB_EIH_PLANAR_IRP6OT_H_

#include "ecp/common/ecp_visual_servo.h"

#include <string.h>
#include <math.h>

#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

#include <iostream>

#define MOTION_STEPS 25


class ecp_vis_ib_eih_planar_irp6ot: public ecp_visual_servo {
public:
	//Wirtualny sensor
	::sensor *vsp_fradia;
	//Pozycja w nastepnym kroku.
    double next_position[8];
    //Obliczone sterowanie dla osi x;
    double x;
    //Obliczone sterowanie dla osi y;
    double y;
    //Przyrost polozenia w makrokroku
    double s;

    //Orientacja koncokwi wzgledem ukladu bazowego.
    double alpha;

	//Maksymalna wartosc  predkosci.
	double v_max;
	//Wartosc przyspieszenia z jakim osiagana jest maksymalna predkosc.
	double  a;
	//Minimalna  wartosc predkosci do jakiej schodzimy przy hamowaniu.
	double v_min;
	//Predkosc chwilowa.
	double v;
	//Dystans wyrazony w pikselach, przy ktorym nastepuje hamowanie.
	double breaking_dist;
	//Czas makrokroku.
	double t_m;
	//Droga do przejechania w nastepnym makrokroku.

	bool breaking;

	ecp_vis_ib_eih_planar_irp6ot(ecp_task& _ecp_task);
	virtual ~ecp_vis_ib_eih_planar_irp6ot();
	virtual bool first_step(void);
	virtual bool next_step_without_constraints();
	virtual void entertain_constraints();
	virtual void retrieve_parameters();
};

#endif /* ECP_VIS_IB_EIH_PLANAR_IRP6OT_H_ */


