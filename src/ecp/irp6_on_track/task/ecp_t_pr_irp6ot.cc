#include <stdio.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/common/generator/ecp_g_force.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"
#include "ecp/irp6_on_track/task/ecp_t_pr_irp6ot.h"

#include "ecp/common/generator/ecp_g_jarosz.h"

#include "ecp_mp/sensor/ecp_mp_sensor.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

void pr::short_move_up ()
{
	/*
    lib::trajectory_description tdes;

    tdes.arm_type = lib::XYZ_EULER_ZYZ;
    tdes.interpolation_node_no = 1;
    tdes.internode_step_no = 200;
    tdes.value_in_step_no = tdes.internode_step_no - 2;
    // Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
    tdes.coordinate_delta[0] = 0.0; // przyrost wspolrzednej X
    tdes.coordinate_delta[1] = 0.0;// przyrost wspolrzednej Y
    tdes.coordinate_delta[2] = 0.005;   // przyrost wspolrzednej Z
    tdes.coordinate_delta[3] = 0.0;   // przyrost wspolrzednej FI
    tdes.coordinate_delta[4] = 0.0;   // przyrost wspolrzednej TETA
    tdes.coordinate_delta[5] = 0.0;   // przyrost wspolrzednej PSI
    tdes.coordinate_delta[6] = 0.0;   // przyrost dla rozwarcia chwytaka
    // Generator trajektorii prostoliniowej
    common::generator::linear lg(*this, tdes, 0);
    lg.Move();
    */
}

// KONSTRUKTORY
pr::pr(lib::configurator &_config) : task(_config)
{
    ecp_m_robot = new robot (*this);

    // Powolanie czujnikow
    sensor_m[lib::SENSOR_FORCE_ON_TRACK] =
        new ecp_mp::sensor::schunk (lib::SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);

    // Konfiguracja wszystkich czujnikow
    for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin();
            sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
    {
        sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
        sensor_m_iterator->second->configure_sensor();
    }

    usleep(1000*100);

    ecp_tryb = config.value<int>("tryb");

    ynrlg = new common::generator::y_nose_run_force (*this, 8);
    ynrlg->sensor_m = sensor_m;

    if (ecp_tryb==1)
    {
        tig = new common::generator::y_drawing_teach_in_force (*this, 8);
        tig->sensor_m = sensor_m;
    }
    else if (ecp_tryb==2)
    {
        tig = new common::generator::y_advanced_drawing_teach_in_force(*this, 8);
        tig->sensor_m = sensor_m;

    }
    else
    {
        printf("Bledny argument wywolania ecp - Bledny tryb pracy\n");
        sr_ecp_msg->message("Bledny tryb pracy - popraw plik konfiguracyjny");
    }

    sr_ecp_msg->message("ECP loaded");
}

pr::~pr()
{
	delete tig;
}

void pr::main_task_algorithm(void)
{
	if (ecp_tryb==1)
	{
		sr_ecp_msg->message("ECP powielanie rysunku");
	}
	else if (ecp_tryb==2)
	{
		sr_ecp_msg->message("ECP zaawansowane powielanie rysunku");
	}

	for(;;)
	{

		sr_ecp_msg->message("NOWA SERIA");
		sr_ecp_msg->message("Wodzenie za nos do pozycji rozpoczecia nauki");
		sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
		ynrlg->Move();

		if (choose_option ("1 - Load drawing, 2 - Learn drawing", 2) == lib::OPTION_ONE)
		{
			sr_ecp_msg->message("Wczytywanie trajektorii");
			tig->load_file_from_ui ();
		}
		else
		{

			sr_ecp_msg->message("Wodzenie za nos");
			sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
			ynrlg->Move();

			sr_ecp_msg->message("Uczenie trajektorii");
			sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
			tig->flush_pose_list(); // Usuniecie listy pozycji, o ile istnieje
			tig->teach_or_move=common::generator::y_drawing_teach_in_force::YG_TEACH;
			tig->Move();

			sr_ecp_msg->message("Krotki ruch w gore");
			short_move_up();

			sr_ecp_msg->message("Wodzenie za nos");
			sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
			ynrlg->Move();
		}

		while (operator_reaction ("Reproduce drawing?"))
		{
			sr_ecp_msg->message("Wodzenie za nos do poczatku odtwarzania");
			sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
			ynrlg->Move();

			sr_ecp_msg->message("Odtwarzanie nauczonej trajektorii");
			tig->teach_or_move=common::generator::y_drawing_teach_in_force::YG_MOVE;
			tig->Move();

			sr_ecp_msg->message("Krotki ruch w gore");
			short_move_up();

			sr_ecp_msg->message("Wodzenie za nos");
			sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
			ynrlg->Move();

		}

		if ( operator_reaction ("Save drawing ") )
		{
			sr_ecp_msg->message("Zapisywanie trajektorii");
			tig->save_file (lib::PF_VELOCITY);
		}
	}
}

}
} // namespace irp6ot

namespace common {
namespace task {

common::task::task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::pr(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


