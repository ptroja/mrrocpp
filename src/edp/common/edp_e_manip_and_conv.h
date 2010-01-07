// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_E_MANIP_AND_CONV_H
#define __EDP_E_MANIP_AND_CONV_H

#include <stdint.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"

#if defined(USE_MESSIP_SRR)
#include <messip.h>
#endif

#include "edp/common/edp_effector.h"

// Konfigurator
#include "lib/configurator.h"

#ifdef DOCENT_SENSOR
#include <boost/function.hpp>
#endif

namespace mrrocpp {
namespace edp {
namespace sensor {
class force;
}
namespace common {

// TODO: remove forward declarations
class manip_trans_t;
class in_out_buffer;
class vis_server;
class servo_buffer;
class edp_vsp;
class reader_buffer;

// base class for EDP robots with manipulators and conveyor

// forward declaration


/************************ edp_irp6s_and_conv_effector ****************************/
class manip_and_conv_effector: public effector, public kinematics::common::kinematics_manager
{
protected:

#ifdef DOCENT_SENSOR
	void onReaderStarted();
	void onReaderStopped();
#endif

	uint16_t motion_steps; // liczba krokow ruchu zadanego (makrokroku)

	//Liczba krokow pierwszej fazy ruchu, czyli krok, w ktorym ma zostac
	//przekazana informacja o realizacji pierwszej fazy ruchu:
	//0 < value_in_step_no <= motion_steps + 1
	//Dla value_in_step_no = motion_steps wiadomosc dotrze po zrealizowaniu
	//makrokroku, ale informacja o polozeniu bedzie dotyczyc realizacji
	//przedostatniego kroku makrokroku.
	//Dla value_in_step_no = motion_steps + 1 wiadomosc dotrze po zrealizowaniu
	//jednego kroku obiegu petli ruchu jalowego po zakonczeniu makrokroku,
	//ale informacja o polozeniu bedzie dotyczyc realizacji calego makrokroku.
	//Dla value_in_step_no < motion_steps wiadomosc dotrze przed zrealizowaniem
	//makrokroku i informacja o polozeniu bedzie dotyczyc realizacji srodkowej
	//fazy makrokroku.
	uint16_t value_in_step_no;


#ifdef DOCENT_SENSOR
	boost::function<void()> startedCallback_;
	bool startedCallbackRegistered_;
	boost::function<void()> stoppedCallback_;
	bool stoppedCallbackRegistered_;
#endif

	friend class servo_buffer;

	void set_outputs(const lib::c_buffer &instruction); // ustawienie wyjsc binarnych

	void get_inputs(lib::r_buffer *local_reply); // odczytanie wejsc binarnych

	// kasuje zmienne - uwaga najpierw nalezy ustawic number_of_servos
	void reset_variables();

	void compute_motors(const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (silnikami)

	void compute_joints(const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (stawami)

	void move_servos();

	// Wyslanie polecenia ruchu do SERVO_GROUP oraz odebranie wyniku
	// realizacji pierwszej fazy ruchu


	lib::MotorArray servo_current_motor_pos; // Polozenia walow silnikow -// dla watku edp_servo    XXXX

	lib::MotorArray global_current_motor_pos; // Polozenia walow silnikow -// globalne dla procesu EDP  XXXX

	lib::JointArray global_current_joints; // Wspolrzedne wewnetrzne -// globalne dla procesu EDP   XXXXX

	lib::JointArray servo_current_joints; // Wspolrzedne wewnetrzne -// dla watku EDP_SERVO   XXXXXX

	boost::mutex edp_irp6s_effector_mutex; // mutex    XXXXXX

	lib::JointArray desired_joints; // Wspolrzedne wewnetrzne -
	// ostatnio obliczone (zadane) (w radianach)

	//double current_joints[MAX_SERVOS_NR];       // Wspolrzedne wewnetrzne -
	lib::JointArray current_joints;
	// ostatnio odczytane (w radianach) // by Y dla watku EDP_MASTER

	lib::MotorArray desired_motor_pos_old;
	// Polozenia walow silnikow -
	// poprzednio obliczone (zadane) (w radianach)
	lib::MotorArray desired_motor_pos_new;
	// Polozenia walow silnikow -
	// aktualnie obliczone (zadane) (w radianach)

	lib::MotorArray current_motor_pos; // Polozenia walow silnikow -
	// ostatnio odczytane (w radianach)


	//    int16_t PWM_value[MAX_SERVOS_NR];             // wartosci zadane wypelnienia PWM
	//    int16_t current[MAX_SERVOS_NR];                // prad sterujacy


public:

	void master_joints_read(double*);
#ifdef DOCENT_SENSOR
	void registerReaderStartedCallback(boost::function<void()> startedCallback);
	void registerReaderStoppedCallback(boost::function<void()> stoppedCallback);
#endif

	in_out_buffer *in_out_obj; // bufor wejsc wyjsc
	reader_buffer *rb_obj;
	manip_trans_t *mt_tt_obj;
	servo_buffer* sb;
	vis_server* vis_obj;
	sensor::force *vs;
	edp_vsp* edp_vsp_obj;

	manip_and_conv_effector(lib::configurator &_config, lib::robot_name_t l_robot_name); // konstruktor
	virtual ~manip_and_conv_effector();

	virtual void set_rmodel(lib::c_buffer &instruction) = 0; // zmiana narzedzia
	virtual void get_rmodel(lib::c_buffer &instruction) = 0; // odczytanie narzedzia


	lib::controller_state_t controller_state_edp_buf; // do okreslenia stanu robota
	unsigned long step_counter;

	short number_of_servos; // by Y ilosc serwomechanizmow  XXX
	// w zaleznosci od tego czy chwytak ma byc aktywny czy nie

	virtual void move_arm(lib::c_buffer &instruction) = 0; // przemieszczenie ramienia

	virtual void get_arm_position(bool read_hardware, lib::c_buffer &instruction) = 0; // odczytanie pozycji ramienia

	virtual void synchronise(); // synchronizacja robota
	virtual void servo_joints_and_frame_actualization_and_upload(void) = 0; // by Y

	void main_loop(); // main loop

	//! thread starting synchronization flag
	bool thread_started;

	//! thread starting synchronization mutex
	boost::mutex thread_started_mutex;

	//! thread starting synchronization condition variable
	boost::condition thread_started_cond;

	virtual void create_threads();

	void interpret_instruction(lib::c_buffer &instruction);
	// interpretuje otrzymana z ECP instrukcje;
	// wypelnaia struktury danych TRANSFORMATORa;
	// przygotowuje odpowiedz dla ECP

	// odczytanie numerow algorytmow i numerow zestawow ich parametrow
	void get_algorithms();

	virtual void get_controller_state(lib::c_buffer &instruction); // by Y

	bool is_power_on() const;

	void update_servo_current_motor_pos(double motor_position_increment, int i);
	void update_servo_current_motor_pos_abs(double abs_motor_position, int i);

	// ustalenie formatu odpowiedzi
	lib::REPLY_TYPE rep_type(const lib::c_buffer &instruction);

	// sprawdzenie czy jest to dopuszczalny rozkaz ruchu
	// przed wykonaniem synchronizacji robota
	bool pre_synchro_motion(lib::c_buffer &instruction) const;

	// Czy robot zsynchronizowany? // by Y - wziete z ecp
	bool is_synchronised(void) const;

	virtual servo_buffer* return_created_servo_buffer() = 0;

	virtual void master_order(MT_ORDER nm_task, int nm_tryb);

};
/************************ edp_irp6s_and_conv_effector ****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
