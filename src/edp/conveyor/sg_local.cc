/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

// Klasa edp_conveyor_effector.
#include "edp/conveyor/edp_conveyor_effector.h"
// Klasa hardware_interface.
#include "edp/conveyor/hi_local.h"
// Klasa servo_buffer.
#include "edp/conveyor/sg_local.h"



namespace mrrocpp {
namespace edp {
namespace conveyor {

extern effector* master;
//extern uint64_t kk;	// numer pomiaru od momentu startu pomiarow

/*-----------------------------------------------------------------------*/
lib::BYTE servo_buffer::Move_a_step (void)
{
	// wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH ora SYNCHRO_ZERO

	Move_1_step ();
	if (master.is_synchronised())
	{// by Y aktualizacja transformera am jedynie sens po synchronizacji (kiedy robot zna swoja pozycje)
		// by Y - do dokonczenia
		for (int i=0; i < CONVEYOR_NUM_OF_SERVOS; i++)
		{
			if (!(master.test_mode))
			{
				//  master.update_servo_current_motor_pos(regulator_ptr[i]->get_position_inc(0)*2*M_PI/IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION,  i);
				master.update_servo_current_motor_pos_abs(hi->get_position(i)*(2*M_PI)/IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION, i);
			}
		}
		master.servo_joints_and_frame_actualization_and_upload(); // by Y - aktualizacja trasformatora
	}
	return convert_error();
}
/*-----------------------------------------------------------------------*/



/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer (effector &_master) : common::servo_buffer(_master), master(_master)
{
	hi = new hardware_interface(_master);

	// utworzenie tablicy regulatorow

	// Serwomechanizm 1
	regulator_ptr[0] = new common::NL_regulator_1_irp6p (0, 0, 0.333, 6.2, 5.933, 0.35, master); // tasmociag dla irp6 postument

	send_after_last_step = false;
	clear_reply_status();
	clear_reply_status_tmp();

	for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
	{
		command.parameters.move.abs_position[j]=0.0;
	}
}

/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/

void servo_buffer::synchronise (void)
{
	common::regulator* crp = NULL; // wskaznik aktualnie synchronizowanego napedu

	int j;

	double synchro_step = 0.0;   // zadany przyrost polozenia

	if(master.test_mode)
	{
		// W.S. Tylko przy testowaniu
		clear_reply_status();
		clear_reply_status_tmp();
		reply_to_EDP_MASTER();
		return;
	}


	// zerowanie regulatorow
	for (j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
	{
		crp = regulator_ptr[j];
		crp->clear_regulator();
		hi->reset_position(j);
	}


	// zatrzymanie na chwile robota
	for (j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
	{
		synchro_step=0.0;
		crp = regulator_ptr[j];
		crp->insert_new_step(synchro_step);
	}

	for (j = 0; j < 25; j++)
		Move_1_step();

	//	kk = 0;
	clear_reply_status();
	clear_reply_status_tmp();

	reply_to_EDP_MASTER();
	return;
}

/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
servo_buffer::~servo_buffer(void)
{
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
void servo_buffer::get_all_positions (void)
{
	// Przepisanie aktualnych polozen servo do pakietu wysylkowego
	for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++)
	{
		// przyrost polozenia w impulsach
		servo_data.abs_position[i]  = hi->get_position(i)*(2*M_PI)/IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION;
		servo_data.position[i]  = regulator_ptr[i]->get_position_inc(1);
		servo_data.current[i]   = regulator_ptr[i]->get_meassured_current();
		servo_data.PWM_value[i] = regulator_ptr[i]->get_PWM_value();
		servo_data.algorithm_no[i] = regulator_ptr[i]->get_algorithm_no();
		servo_data.algorithm_parameters_no[i] = regulator_ptr[i]->get_algorithm_parameters_no();
	}
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint64_t servo_buffer::compute_all_set_values (void)
{
	// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
	uint64_t status = OK; // kumuluje numer bledu

	for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++)
	{
		if (master.test_mode)
		{
			regulator_ptr[j]->insert_new_pos_increment(regulator_ptr[j]->return_new_step()
					*IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI));
		}
		else
		{
			regulator_ptr[j]->insert_meassured_current(hi->get_current(j));
			regulator_ptr[j]->insert_new_pos_increment(hi->get_increment(j));
		}
		// obliczenie nowej wartosci zadanej dla napedu
		status |= ((uint64_t) regulator_ptr[j]->compute_set_value()) << 2*j;
		// przepisanie obliczonej wartosci zadanej do hardware interface
		hi->insert_set_value(j, regulator_ptr[j]->get_set_value());
	}
	return status;
}
/*-----------------------------------------------------------------------*/

} // namespace conveyor

namespace common {


servo_buffer* return_created_servo_buffer (manip_and_conv_effector &_master)
{
	return new conveyor::servo_buffer ((conveyor::effector &)(_master));
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

