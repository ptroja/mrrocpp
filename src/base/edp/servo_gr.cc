/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <unistd.h>
#include <cerrno>
#include <ctime>

#include "base/lib/typedefs.h"
#include "base/lib/mis_fun.h"
#include "base/edp/edp_typedefs.h"
#include "base/lib/mis_fun.h"
#include "base/edp/reader.h"
#include "base/edp/HardwareInterface.h"
#include "base/edp/servo_gr.h"
#include "base/edp/regulator.h"

#include "base/edp/edp_e_motor_driven.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace common {

void servo_buffer::load_hardware_interface(void)
{
	send_after_last_step = false;
	clear_reply_status();
	clear_reply_status_tmp();

	for (int j = 0; j < master.number_of_servos; j++) {
		command.parameters.move.abs_position[j] = 0.0;
	}
}

/*-----------------------------------------------------------------------*/
uint8_t servo_buffer::Move_a_step(void)
{
	// wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH oraz SYNCHRO_ZERO

	Move_1_step();
	if (master.is_synchronised()) {// by Y aktualizacja transformera am jedynie sens po synchronizacji (kiedy robot zna swoja pozycje)
		// by Y - do dokonczenia
		for (int i = 0; i < master.number_of_servos; i++) {
			if (!(master.robot_test_mode)) {
				master.update_servo_current_motor_pos_abs(hi->get_position(i) * (2 * M_PI) / axe_inc_per_revolution[i], i);
			}
		}

		master.compute_servo_joints_and_frame();// by Y - aktualizacja trasformatora
	}
	return convert_error();
}
/*-----------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void servo_buffer::set_robot_model_servo_algorithm(const lib::c_buffer &instruction)
{
	// ustawienie algorytmw serworegulacji oraz ich parametrow
	// zmiana algorytmu regulacji
	/* Uformowanie rozkazu zmiany algorytmw serworegulacji oraz ich parametrow dla procesu SERVO_GROUP */
	servo_command.instruction_code = SERVO_ALGORITHM_AND_PARAMETERS;
	for (int i = 0; i < master.number_of_servos; i++) {
		servo_command.parameters.servo_alg_par.servo_algorithm_no[i]
				= instruction.robot_model.servo_algorithm.servo_algorithm_no[i];
		servo_command.parameters.servo_alg_par.servo_parameters_no[i]
				= instruction.robot_model.servo_algorithm.servo_parameters_no[i];
	}
	/* Wyslanie rozkazu zmiany algorytmw serworegulacji oraz ich parametrow procesowi SERVO_GROUP */
	send_to_SERVO_GROUP(); //
}

/*--------------------------------------------------------------------------*/
void servo_buffer::send_to_SERVO_GROUP()
{
	// int command_size; // dulgosci rozkazu
	// sigset_t set, old_set; // zmienne opisujace sygnaly przysylane do procesu
	// Wyslanie polecenia do SERVO_GROUP

	/*
	 // by Y - wywalone
	 // Obliczenie dugoci rozkazu
	 switch(servo_command.instruction_code) {
	 case lib::READ:
	 case lib::SYNCHRONISE:
	 command_size = (int) (((uint8_t*) (&servo_command.address_byte)) - ((uint8_t*) (&servo_command.instruction_code)));
	 break;
	 case lib::MOVE:
	 command_size = (int) (((uint8_t*) (&servo_command.parameters.move.address_byte)) - ((uint8_t*) (&servo_command.instruction_code)));
	 break;
	 case lib::SERVO_ALGORITHM_AND_PARAMETERS:
	 command_size = (int) (((uint8_t*) (&servo_command.parameters.servo_alg_par.address_byte)) - ((uint8_t*) (&servo_command.instruction_code)));
	 break;
	 }; // end: switch
	 // if (Send(&servo_command, &sg_reply, command_size, sizeof(lib::servo_group_reply)) < 0) {
	 */

	/* sigemptyset ( &set);
	 sigaddset ( &set, SIGUSR1);

	 SignalProcmask( 0,thread_id, SIG_BLOCK, &set, NULL ); // by Y uniemozliwienie jednoczesnego wystawiania spotkania do serwo przez edp_m i readera
	 */

	{
		boost::lock_guard <boost::mutex> lock(servo_command_mtx);
		servo_command_rdy = true;
	}

	{
		boost::unique_lock <boost::mutex> lock(sg_reply_mtx);
		while (!sg_reply_rdy) {
			sg_reply_cond.wait(sg_reply_mtx);
		}
		sg_reply_rdy = false;
	}

	//   SignalProcmask( 0,thread_id, SIG_UNBLOCK, &set, NULL );

	if ((sg_reply.error.error0 != OK) || (sg_reply.error.error1 != OK)) {
		printf("a: %llx, :%llx\n", sg_reply.error.error0, sg_reply.error.error1);
		throw Fatal_error(sg_reply.error.error0, sg_reply.error.error1);
	} // end: if((sg_reply.error.error0 != OK) || (sg_reply.error.error1 != OK))

	// skopiowanie odczytow do transformera

	for (int i = 0; i < master.number_of_servos; i++) {

		master.current_motor_pos[i] = sg_reply.abs_position[i];

		//	 printf("current motor pos: %d\n", current_motor_pos[0]);

		if (master.robot_test_mode) {
			// W.S. Tylko przy testowaniu
			master.current_motor_pos[i] = master.desired_motor_pos_new[i];
		}

		master.reply.PWM_value[i] = sg_reply.PWM_value[i];
		master.reply.current[i] = sg_reply.current[i];
		master.reply.robot_model.servo_algorithm.servo_algorithm_no[i] = sg_reply.algorithm_no[i];
		master.reply.robot_model.servo_algorithm.servo_parameters_no[i] = sg_reply.algorithm_parameters_no[i];

	}

	// przepisanie stanu regulatora chwytaka
	master.reply.arm.pf_def.gripper_reg_state = sg_reply.gripper_reg_state;

	// printf("edp_irp6s_and_conv_effector::send_to_SERVO_GROUP: %f, %f\n", current_motor_pos[4], sg_reply.abs_position[4]);

	// 	printf("current motor pos: %f\n", current_motor_pos[0]*IRP6_ON_TRACK_INC_PER_REVOLUTION/2*M_PI );

}
/*--------------------------------------------------------------------------*/

void servo_buffer::operator()()
{
	// servo buffer has to be created before servo thread starts
	//	std::auto_ptr<servo_buffer> sb(return_created_servo_buffer()); // bufor do komunikacji z EDP_MASTER

	load_hardware_interface();

	lib::set_thread_priority(pthread_self(), 79);

	// signal master thread to continue executing
	thread_started.command();

	master.sb_loaded.wait();
	/* BEGIN SERVO_GROUP */

	for (;;) {
		// komunikacja z transformation
		if (!get_command()) {
			// scoped-locked reader data update
			{
				boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

				master.rb_obj->step_data.servo_mode = false; // tryb bierny
			}

			/* Nie otrzymano nowego polecenia */
			/* Krok bierny - zerowy przyrost polozenia */
			// Wykonanie pojedynczego kroku ruchu
			Move_passive();
		} else {
			// nowe polecenie
			{
				boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

				master.rb_obj->step_data.servo_mode = true; // tryb czynny
			}

			switch (command_type())
			{
				case SYNCHRONISE:
					synchronise(); // synchronizacja
					break;
				case MOVE:
					Move(); // realizacja makrokroku ruchu
					break;
				case READ:
					Read(); // Odczyt polozen
					break;
				case SERVO_ALGORITHM_AND_PARAMETERS:
					Change_algorithm(); // Zmiana algorytmu serworegulacji lub jego parametrow
					break;
				default:
					// niezidentyfikowane polecenie (blad) nie moze wystapic, bo juz
					// wczesniej zostalo wychwycone przez get_command()
					break;
			}
		} // end: else
	}
} // end: main() SERVO_GROUP


/*-----------------------------------------------------------------------*/
void servo_buffer::get_all_positions(void)
{
	// Przepisanie aktualnych polozen servo do pakietu wysylkowego
	for (int i = 0; i < master.number_of_servos; i++) {

		servo_data.abs_position[i] = hi->get_position(i) * (2 * M_PI) / axe_inc_per_revolution[i];

		// przyrost polozenia w impulsach
		servo_data.position[i] = regulator_ptr[i]->get_position_inc(1);
		servo_data.current[i] = regulator_ptr[i]->get_measured_current();
		servo_data.PWM_value[i] = regulator_ptr[i]->get_PWM_value();
		servo_data.algorithm_no[i] = regulator_ptr[i]->get_algorithm_no();
		servo_data.algorithm_parameters_no[i] = regulator_ptr[i]->get_algorithm_parameters_no();
	}

}
/*-----------------------------------------------------------------------*/

void servo_buffer::clear_reply_status(void)
{
	// zeruje skladowe reply_status
	reply_status.error0 = OK;
	reply_status.error1 = OK;
}

void servo_buffer::clear_reply_status_tmp(void)
{
	// zeruje skladowe reply_status_tmp
	reply_status_tmp.error0 = OK;
	reply_status_tmp.error1 = OK;
}

// input_buffer
SERVO_COMMAND servo_buffer::command_type() const
{
	return command.instruction_code;
}

servo_buffer::servo_buffer(motor_driven_effector &_master) :

	servo_command_rdy(false), sg_reply_rdy(false),

	thread_started(), master(_master)
{

}

/*-----------------------------------------------------------------------*/
bool servo_buffer::get_command(void)
{
	// Odczytanie polecenia z EDP_MASTER o ile zostalo przyslane
	bool new_command_available = false;

	{
		boost::lock_guard <boost::mutex> lock(servo_command_mtx);
		if (servo_command_rdy) {
			command = servo_command;
			servo_command_rdy = false;
			new_command_available = true;
		}
	}

	if (new_command_available) { // jezeli jest nowa wiadomosc

		/* Otrzymano nowe polecenie */
		// Ewentualna reakcja na uprzednie wystapienie bledu
		if ((reply_status.error0 != OK) || (reply_status.error1 != OK)) {
			// Powiadomienie EDP_MASTER o uprzednio wykrytym bledzie
			reply_status.error0 |= SERVO_ERROR_IN_PASSIVE_LOOP;
			clear_reply_status_tmp();
			reply_to_EDP_MASTER();
			return false; // Potraktowac jakby nie bylo polecenia
		} // end: if

		// Uprzednio nie bylo bledu => wstepna analiza polecenia
		switch (command_type())
		{
			case SYNCHRONISE:
				return true; // wyjscie bez kontaktu z EDP_MASTER
			case MOVE:
				return true; // wyjscie bez kontaktu z EDP_MASTER
			case READ:
				return true; // wyjscie bez kontaktu z EDP_MASTER
			case SERVO_ALGORITHM_AND_PARAMETERS:
				return true; // wyjscie bez kontaktu z EDP_MASTER
			default: // otrzymano niezidentyfikowane polecenie => blad
				reply_status.error0 = UNIDENTIFIED_SERVO_COMMAND;
				clear_reply_status_tmp();

				reply_to_EDP_MASTER();
				return false; // Potraktowac jakby nie bylo polecenia
		} // end: switch
	} else {

		return false;
	}
} // end: servo_buffer::get_command
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
uint8_t servo_buffer::Move_1_step(void)
{
	// wykonac ruch o krok
	// Odebranie informacji o uprzednio zrealizowanym polozeniu oraz ewentualnej awarii
	// Obliczenie nowej wartosci zadanej
	// Wyslanie wartosci zadanej do hardware'u

	reply_status_tmp.error1 = compute_all_set_values(); // obliczenie nowej wartosci zadanej dla regulatorow
	reply_status_tmp.error0 = hi->read_write_hardware(); // realizacja kroku przez wszystkie napedy oraz
	// odczyt poprzedniego polozenia
	master.step_counter++;

	// scoped-locked reader data update
	{
		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

		if (clock_gettime(CLOCK_REALTIME, &master.rb_obj->step_data.measure_time) == -1) {
			perror("clock_gettime()");

			/*
			 BOOST_THROW_EXCEPTION(
			 System_error() <<
			 boost::errinfo_errno(errno) <<
			 boost::errinfo_api_function("clock_gettime")
			 );
			 */
		}

		master.rb_obj->step_data.step = master.step_counter;

		master.rb_obj->new_data = true;
		master.rb_obj->cond.notify_one();
	}

	if (reply_status_tmp.error0 || reply_status_tmp.error1) {
		//         std::cout<<"w move 1 step error detected\n";
		return ERROR_DETECTED; // info o awarii
	} else
		return NO_ERROR_DETECTED;
}

/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
uint8_t servo_buffer::convert_error(void)
{
	// zwraca NO_ERROR_DETECTED, gdy OK lub wykryto SYNCHRO_SWITCH oraz SYNCHRO_ZERO,
	// w ktorejs osi, a w pozostalych przypadkach dokonuje konwersji numeru bledu

	// wycinamy po dwa najmlodsze bity z kazdej piatki zwiazanej z napedem
	uint64_t err1 = OK;
	for (int i = 0; i < master.number_of_servos; i++) {
		reply_status_tmp.error0 >>= 2;
		err1 |= (reply_status_tmp.error0 & 0x07) << (3 * i);
		reply_status_tmp.error0 >>= 3;
	}

	reply_status_tmp.error0 = err1;
	if (reply_status_tmp.error0 || reply_status_tmp.error1) {
		return ERROR_DETECTED; // info o awarii
	} else
		return NO_ERROR_DETECTED;
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
void servo_buffer::Move_passive(void)
{ //
	// stanie w miejscu - krok bierny

	for (int j = 0; j < master.number_of_servos; j++) {
		regulator_ptr[j]->insert_new_step(0.0); // zerowy przyrost polozenia dla wszystkich napedow
	}

	// Wykonanie pojedynczego kroku ruchu
	if (Move_a_step()) {
		// informacja dla EDP -> w trakcie realizacji petli biernej nastapila awaria
		reply_status.error0 = reply_status_tmp.error0 | SERVO_ERROR_IN_PASSIVE_LOOP;
		reply_status.error1 = reply_status_tmp.error1;
		clear_reply_status_tmp();
		// awaria w petli biernej nie bedzie naprawiana
	} // end: if

	/* Czy przeslac informacje o stanie SERVO do EDP_MASTER? */
	if (send_after_last_step) { // Tak, przeslac
		reply_to_EDP_MASTER();
	}
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
void servo_buffer::Move(void)
{

	double new_increment[master.number_of_servos];

	// wykonanie makrokroku ruchu

	// okreslenie momentu wyslania informacji o zakonczeniu pierwszej fazy ruchu  do READING_BUFFER
	if (command.parameters.move.return_value_in_step_no > command.parameters.move.number_of_steps)
		send_after_last_step = true;
	else
		send_after_last_step = false;

	/*
	 regulator_ptr[0]->insert_new_step((command.parameters.move.abs_position[0] - hi->get_position(0)*(2*M_PI)/AXIS_0_TO_5_INC_PER_REVOLUTION) /
	 command.parameters.move.number_of_steps));
	 */

	for (int k = 0; k < master.number_of_servos; k++) {
		new_increment[k] = command.parameters.move.macro_step[k] / command.parameters.move.number_of_steps;
	}

	// realizacja makrokroku przez wszystkie napedy;  i - licznik krokow ruchu
	for (uint16_t j = 0; j < command.parameters.move.number_of_steps; j++) {
		// by Y
		// XXX by ptroja
		if ((command.parameters.move.return_value_in_step_no == 0) && (j
				== command.parameters.move.return_value_in_step_no)) {
			// czy juz wyslac info do EDP_MASTER?
			// 	     std::cout<<"fsD\n";
			if (reply_status.error0 || reply_status.error1) {
				std::cout << "w 1 reply error\n";
			}

			reply_to_EDP_MASTER();
		}
		// end by Y
		// Wykonanie pojedynczego kroku ruchu z jednoczesnym
		// sprawdzeniem, czy w jakims serwomechanizmie nastapila awaria

		for (int k = 0; k < master.number_of_servos; k++) {
			regulator_ptr[k]->insert_new_step(new_increment[k]);
			if (master.robot_test_mode) {
				master.update_servo_current_motor_pos_abs(regulator_ptr[k]->previous_abs_position + new_increment[k]
						* j, k);
			}
		}

		if (Move_a_step() == NO_ERROR_DETECTED) { // NO_ERROR_DETECTED
			//  std::cout<<"NO_ERROR_DETECTED\n";
			if ((command.parameters.move.return_value_in_step_no > 0) && (j
					== command.parameters.move.return_value_in_step_no - 1)) {
				// czy juz wyslac info do EDP_MASTER?
				if (reply_status.error0 || reply_status.error1) {
					std::cout << "w drugim reply error\n";
				}
				reply_to_EDP_MASTER();
			}
		} else { // ERROR_DETECTED
			//  std::cout<<"ERROR_DETECTED\n";
			if (j > command.parameters.move.return_value_in_step_no - 1) {
				reply_status.error0 = reply_status_tmp.error0 | SERVO_ERROR_IN_PHASE_2;
				reply_status.error1 = reply_status_tmp.error1;
				clear_reply_status_tmp();
			} else {
				reply_status.error0 = reply_status_tmp.error0 | SERVO_ERROR_IN_PHASE_1;
				reply_status.error1 = reply_status_tmp.error1;
				clear_reply_status_tmp();
				std::cout << "ERROR_DETECTED 2\n";
				reply_to_EDP_MASTER();
			}
			break; // przerwac ruch, bo byl blad
		}
	}

	for (int i = 0; i < master.number_of_servos; i++) {
		regulator_ptr[i]->previous_abs_position = command.parameters.move.abs_position[i];
	}
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
void servo_buffer::reply_to_EDP_MASTER(void)
{
	// przeslanie stanu SERVO_GROUP do EDP_MASTER z inicjatywy SERVO_GROUP

	servo_data.error.error0 = reply_status.error0;
	servo_data.error.error1 = reply_status.error1;
	// W.S. cprintf("err0 = %lx    err1 = %lx\n",servo_data.error.error0,servo_data.error.error1);
	// Przepisanie aktualnych polozen servo do pakietu wysylkowego
	get_all_positions();

	// Wyslac informacje do EDP_MASTER

	{
		boost::lock_guard <boost::mutex> lock(sg_reply_mtx);

		sg_reply = servo_data;
		sg_reply_rdy = true;
	}

	// TODO: should not call notify with mutex locked?
	sg_reply_cond.notify_one();

	// Wyzerowac zmienne sygnalizujace stan procesu
	clear_reply_status();
	send_after_last_step = false;
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
void servo_buffer::ppp(void) const
{
	// wydruk kontrolny polecenia przysylanego z EDP_MASTER
	std::cout << " macro_step= " << command.parameters.move.macro_step << "\n";
	std::cout << " number_of_steps= " << command.parameters.move.number_of_steps << "\n";
	std::cout << " return_value_in_step_no= " << command.parameters.move.return_value_in_step_no << "\n";
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
servo_buffer::~servo_buffer(void)
{

	// Destruktor grupy regulatorow
	// Zniszcyc regulatory
	for (int j = 0; j < master.number_of_servos; j++)
		delete regulator_ptr[j];

	delete hi;

	delete thread_id;
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
void servo_buffer::Read(void)
{
	// odczytac aktualne polozenie
	// wyslac do EDP_MASTER
	reply_to_EDP_MASTER();
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
void servo_buffer::Change_algorithm(void)
{
	// Zmiana numeru algorytmu regulacji oraz numeru zestawu jego parametrow
	for (int j = 0; j < master.number_of_servos; j++) {
		regulator_ptr[j]->insert_algorithm_no(command.parameters.servo_alg_par.servo_algorithm_no[j]);
		regulator_ptr[j]->insert_algorithm_parameters_no(command.parameters.servo_alg_par.servo_parameters_no[j]);
	}
	reply_to_EDP_MASTER();
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
uint64_t servo_buffer::compute_all_set_values(void)
{
	// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
	uint64_t status = OK; // kumuluje numer bledu


	for (int j = 0; j < master.number_of_servos; j++) {
		if (master.robot_test_mode) {
			regulator_ptr[j]->insert_new_pos_increment(regulator_ptr[j]->return_new_step() * axe_inc_per_revolution[j]
					/ (2 * M_PI));
		} else {
			regulator_ptr[j]->insert_measured_current(hi->get_current(j));
			regulator_ptr[j]->insert_new_pos_increment(hi->get_increment(j));
		}
		// obliczenie nowej wartosci zadanej dla napedu
		status |= ((uint64_t) regulator_ptr[j]->compute_set_value()) << 2 * j;
		// przepisanie obliczonej wartosci zadanej do hardware interface
		hi->insert_set_value(j, regulator_ptr[j]->get_set_value());
	}
	return status;
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/

void servo_buffer::synchronise(void)
{

	//	master.msg->message("synchro start");
	//printf("synchro \n");
	if (master.robot_test_mode) {
		// W.S. Tylko przy testowaniu
		clear_reply_status();
		clear_reply_status_tmp();
		reply_to_EDP_MASTER();
		return;
	}

	for (int j = 0; j < (master.number_of_servos); j++) {

		command.parameters.move.abs_position[j] = 0.0;
	}; // end: for

	// szeregowa synchronizacja serwomechanizmow
	for (int k = 0; k < (master.number_of_servos); k++) {
		int j = synchro_axis_order[k];
		// wskaznik aktualnie synchronizowanego napedu

		common::regulator* crp = regulator_ptr[j];

		synchro_choose_axis_to_move(crp, j);
		if (!move_to_synchro_area(crp, j))
			return;
		if (!synchro_stop_for_a_while(crp, j))
			return;

		move_from_synchro_area(crp, j);
		switch (synchro_move_to_encoder_zero(crp, j))
		{
			case 0:
				return;
			case 1:
				continue;
			default:
				break;
		}

	} // end: for

	// zatrzymanie na chwile robota
	for (int k = 0; k < (master.number_of_servos); k++) {
		regulator_ptr[k]->insert_new_step(0.0);
	};
	for (int i = 0; i < SYNCHRO_FINAL_STOP_STEP_NUMBER; i++) {
		Move_1_step();
	}

	// kk = 0;

	// printf("koniec synchro\n");
	reply_to_EDP_MASTER();
	return;

}

void servo_buffer::synchro_choose_axis_to_move(common::regulator* &crp, int j)
{

	double synchro_step = 0.0; // zadany przyrost polozenia
	// printf("os synchronizowana: %d \n",j);
	for (int l = 0; l < (master.number_of_servos); l++) {
		// zerowy przyrost polozenia dla wszystkich napedow procz j-tego
		if (l == j) {
			synchro_step = synchro_step_coarse[l] / SYNCHRO_NS;
		} else {
			synchro_step = 0.0;
		}
		regulator_ptr[l]->insert_new_step(synchro_step);
	} // end: for

	clear_reply_status();
	clear_reply_status_tmp();

}

int servo_buffer::move_to_synchro_area(common::regulator* &crp, int j)
{

	double synchro_step = 0.0; // zadany przyrost polozenia


	// ruch do wykrycia wylacznika synchronizacji
	for (;;) {
		do {

			if (synchro_step > synchro_step_coarse[j]) {
				synchro_step += synchro_step_coarse[j] / SYNCHRO_NS;
				crp->insert_new_step(synchro_step);
			}

			//		printf("aaa: %d, %x\n", j, reply_status_tmp.error0);
		} while (Move_1_step() == NO_ERROR_DETECTED); // end: while
		//		printf("aaa: %d, %x\n", j, reply_status_tmp.error0);
		// analiza przyslanego bledu (czy wjechano na wylacznik synchronizacji?)
		// jezeli nie, to blad
		switch ((reply_status_tmp.error0 >> (5 * j)) & 0x000000000000001FULL)
		{
			case SYNCHRO_SWITCH_ON:
				//  printf("aaa: SYNCHRO_SWITCH_ON\n");
			case SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO:
				// cprintf("B=%lx\n", reply_status_tmp.error0);
				//		printf("aaa: SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO\n");
				break;
			case ALL_RIGHT:
			case SYNCHRO_ZERO:
				//	master.msg->message("SYNCHRO_ZERO");
				//	printf("aaa: SYNCHRO_ZERO\n");
				continue;
			default:
				//	master.msg->message("default");
				//	printf("aaa: default\n");
				// awaria w trakcie synchronizacji
				convert_error();
				reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_SWITCH_EXPECTED;
				reply_status.error1 = reply_status_tmp.error1;
				clear_reply_status_tmp();
				reply_to_EDP_MASTER();
				return 0;
		} // end: switch
		break;
	} // end: for (;;)

	//	printf("przed clear_reply_status \n");

	clear_reply_status();
	clear_reply_status_tmp();
	return 1;
}

int servo_buffer::synchro_stop_for_a_while(common::regulator* &crp, int j)
{

	double synchro_step = 0.0; // zadany przyrost polozenia
	// zatrzymanie na chwile robota

	crp->insert_new_step(synchro_step);
	for (int i = 0; i < SYNCHRO_STOP_STEP_NUMBER; i++) {
		Move_1_step();
		//  printf("aabb: %d, %x\n", j, reply_status_tmp.error0);
		switch ((reply_status_tmp.error0 >> (5 * j)) & 0x000000000000001FULL)
		{
			case SYNCHRO_SWITCH_ON:
			case SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO:
			case ALL_RIGHT:
			case SYNCHRO_ZERO:
				continue;
			default:
				// awaria w trakcie stania
				convert_error();
				reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_DELAY_ERROR;
				reply_status.error1 = reply_status_tmp.error1;
				clear_reply_status_tmp();
				reply_to_EDP_MASTER();
				return 0;
		} // end: switch
	} // end: for (i...)
	// cprintf("C=%lx\n", reply_status_tmp.error0);

	clear_reply_status();
	clear_reply_status_tmp();

	return 1;

}

void servo_buffer::move_from_synchro_area(common::regulator* &crp, int j)
{

	// zjazd z wylacznika synchronizacji
	// W.S.  crp->insert_new_step(lib::SYNCHRO_STEP_FINE);
	double synchro_step = -synchro_step_fine[j] / SYNCHRO_NS;

	crp->insert_new_step(synchro_step);

	// wlaczenie sledzenia zera rezolwera (synchronizacja osi)
	hi->start_synchro(j);
	delay(1);
	Move_1_step();
	while (1) {
		//  		printf("babb: %d\n", j);
		Move_1_step();
		// W.S. -----------------------------------------------------
		//	printf("ccc: %d\n", j);

		if (synchro_step < -synchro_step_fine[j]) {
			synchro_step -= synchro_step_fine[j] / SYNCHRO_NS;
			crp->insert_new_step(synchro_step);
		}

		// W.S. -----------------------------------------------------
		//    	   printf("bbbb if: %llx\n", ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL));
		switch ((reply_status_tmp.error0 >> (5 * j)) & 0x000000000000001FULL)
		{
			case SYNCHRO_SWITCH_ON:
				//    	printf("bcbb:���SYNCHRO_SWITCH_ON\n");
			case SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO:
				//     	printf("bfbb: SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO\n");
				continue;
			default:
				//   	printf("baabb: default\n");
				break;
		} // end: switch
		break;
	} // end: while
	//	 printf("D\n ");
}

int servo_buffer::synchro_move_to_encoder_zero(common::regulator* &crp, int j)
{

	// analiza powstalej sytuacji (czy zjechano z wylacznika synchronizacji)
	// jezeli nie, to blad
	switch (((reply_status_tmp.error0 >> (5 * j)) & 0x000000000000001FULL))
	{
		case SYNCHRO_ZERO: // zjechano z wylacznika synchronizacji i SYNCHRO_ZERO jest od razu
			//     printf("SYNCHRO_ZERO\n");
			hi->finish_synchro(j);

			//	 printf("SYNCHRO_ZERO\n");
			hi->reset_position(j);
			crp->clear_regulator();
			delay(1);
			return 1;
		case OK:
			// ruch do wykrycia zera rezolwera
			//    printf("OK\n");
			for (;;) {
				Move_1_step();
				//        printf("OK Move_1_step\n");
				//      if ( ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739FULL) != OK)
				// by Y - wyciecie SYNCHRO_SWITCH_ON - ze wzgledu na wystepujace drgania
				if (((reply_status_tmp.error0 >> (5 * j)) & 0xCE739CE739CE739DULL) != OK) {
					//   printf("OK os: %d, if: %llx, %llx\n", j, reply_status_tmp.error0, ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739FULL));
					// Usuniecie bitow SYNCHRO_ZERO i SYNCHRO_SWITCH_ON z wszystkich osi
					// oprocz synchronizowanej
					// osie zsynchronizowane nie sa analizowane
					break;
				}
			}
			// end: for (;;)
			//     if ( ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL) != lib::SYNCHRO_ZERO) {
			// by Y - wyciecie SYNCHRO_SWITCH_ON
			if (((reply_status_tmp.error0 >> (5 * j)) & 0x000000000000001DULL) != SYNCHRO_ZERO) {
				// 	  printf("OK convert_error: %llx\n", ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL));
				convert_error();
				reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_ERROR;
				reply_status.error1 = reply_status_tmp.error1;
				clear_reply_status_tmp();
				// Wypelnic servo_data
				reply_to_EDP_MASTER();
				return 0;
			} else {
				hi->finish_synchro(j);
				hi->reset_position(j);
				crp->clear_regulator();
				delay(1);
				return 1;
			}
			// end: else
		default:
			//    	printf(" default error\n");
			// awaria w trakcie synchronizacji
			convert_error();
			reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_ERROR;
			reply_status.error1 = reply_status_tmp.error1;
			clear_reply_status_tmp();
			// Wypelnic servo_data
			reply_to_EDP_MASTER();
			return 0;
	} // end: switch

	return 2;

	// zakonczenie synchronizacji danej osi i przejscie do trybu normalnego


}

/*-----------------------------------------------------------------------*/

} // namespace common
} // namespace edp
} // namespace mrrocpp

