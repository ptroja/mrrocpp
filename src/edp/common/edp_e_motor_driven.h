// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_E_MOTOR_DRIVEN_H
#define __EDP_E_MOTOR_DRIVEN_H

#include <stdint.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "lib/mis_fun.h"

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

/*!
 * \class motor_driven_effector
 * \brief Base class of all EDP effectors using motors (e.g. robots)
 *
 * The class can be treated as multi variant shield. The derrived classes can optionally use servo_buffer (dedicated servo thread)
 * reader_buffer - dedicated reader thread, mt_tt_obj - dedicated thread to interpolate in task coordinates, e.g. force control in manipulators
 * vis_server - dedicated thread to sent joint position e.g. to visualisation processes,
 * sensor::force -- dedicated thread to measure force for the purpose of position force control of robotics manipulator
 * edp_vsp_obj - thread to sent data to VSP process (when the force sensor is used both as the prioceptor and exteroceptor)
 * *
 * \author yoyek
 */
class motor_driven_effector: public effector, public kinematics::common::kinematics_manager
{
protected:

#ifdef DOCENT_SENSOR
	void onReaderStarted();
	void onReaderStopped();
#endif

	/*!
	 * \brief The number of steps in the macrostep.
	 *
	 * Each step takes typically 1 to 2ms.
	 */
	uint16_t motion_steps;

	/*!
	 * \brief The number of steps between the SET and QUERY command.
	 *
	 * This number should be lower then motion_steps while the manipulator is moving to receive new command before the execution of the previous is finished.
	 */
	uint16_t value_in_step_no;

#ifdef DOCENT_SENSOR
	boost::function<void()> startedCallback_;
	bool startedCallbackRegistered_;
	boost::function<void()> stoppedCallback_;
	bool stoppedCallbackRegistered_;
#endif

	/*!
	 * \brief friend class of servo thread to handle the motion controllers
	 *
	 * It is used when the controllers loop is implemented in the EDP.
	 */
	friend class servo_buffer;

	/*!
	 * \brief method to set the outputs in the hardware commanded by the ECP
	 *
	 * It is done with usage of in_out_object, processed in interrupt handler.
	 */
	void set_outputs(const lib::c_buffer &instruction); // ustawienie wyjsc binarnych

	/*!
	 * \brief method to get the inputs from the hardware commanded by the ECP and then reply it to the ECP.
	 *
	 * It is done with usage of in_out_object, processed in interrupt handler.
	 */
	void get_inputs(lib::r_buffer *local_reply); // odczytanie wejsc binarnych

	/*!
	 * \brief method reset to zero the vectors of motor and joint position.
	 *
	 * number_of_servos should be previously set
	 */
	void reset_variables();

	/*!
	 * \brief method to extract desired_motor_position basing on the ECP instruction with motor desired motor position in absolute or relative variant.
	 *
	 * It also checks kinematic constrains of motor and equivalent joint position.
	 */
	void compute_motors(const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (silnikami)

	/*!
	 * \brief method to compute desired_motor_position basing on the ECP instruction with motor desired joint position in absolute or relative variant.
	 *
	 * It also checks kinematic constrains of motor and equivalent joint position.
	 */
	void compute_joints(const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (stawami)

	/*!
	 * \brief method to prepare command for servos (typically servo_buffer thread)
	 *
	 * It bases on desired_motor_position.
	 */
	void move_servos();

	lib::MotorArray servo_current_motor_pos; // Polozenia walow silnikow -// dla watku edp_servo    XXXX

	lib::MotorArray global_current_motor_pos; // Polozenia walow silnikow -// globalne dla procesu EDP  XXXX

	lib::JointArray servo_current_joints; // Wspolrzedne wewnetrzne -// dla watku EDP_SERVO   XXXXXX

	lib::JointArray global_current_joints; // Wspolrzedne wewnetrzne -// globalne dla procesu EDP   XXXXX

	boost::mutex edp_irp6s_effector_mutex; // mutex    XXXXXX

	lib::JointArray desired_joints; // Wspolrzedne wewnetrzne -
	// ostatnio obliczone (zadane) (w radianach)


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

	motor_driven_effector(lib::configurator &_config, lib::robot_name_t l_robot_name); // konstruktor
	virtual ~motor_driven_effector();

	virtual void set_rmodel(lib::c_buffer &instruction); // zmiana narzedzia

	virtual void get_rmodel(lib::c_buffer &instruction); // odczytanie narzedzia


	lib::controller_state_t controller_state_edp_buf; // do okreslenia stanu robota
	unsigned long step_counter;

	short number_of_servos; // by Y ilosc serwomechanizmow  XXX
	// w zaleznosci od tego czy chwytak ma byc aktywny czy nie

	virtual void move_arm(lib::c_buffer &instruction) = 0; // przemieszczenie ramienia
	void multi_thread_move_arm(lib::c_buffer &instruction);
	void single_thread_move_arm(lib::c_buffer &instruction);
	virtual void get_arm_position(bool read_hardware, lib::c_buffer &instruction) = 0; // odczytanie pozycji ramienia
	void get_arm_position_read_hardware_sb(); // odczytanie pozycji ramienia sprzetowo z sb
	void get_arm_position_set_reply_step(); // odczytanie pozycji ramienia sprzetowo z sb

	virtual void get_arm_position_get_arm_type_switch(lib::c_buffer &instruction); // odczytanie pozycji ramienia sprzetowo z sb

	virtual void synchronise(); // synchronizacja robota
	virtual bool servo_joints_and_frame_actualization_and_upload(void); // by Y

	void main_loop(); // main loop
	void pre_synchro_loop(STATE& next_state);
	void synchro_loop(STATE& next_state);
	void post_synchro_loop(STATE& next_state);

	void hi_create_threads();

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

	virtual servo_buffer* return_created_servo_buffer();

	virtual void master_order(MT_ORDER nm_task, int nm_tryb) = 0;
	void multi_thread_master_order(common::MT_ORDER nm_task, int nm_tryb);
	void single_thread_master_order(common::MT_ORDER nm_task, int nm_tryb);
};
/************************ edp_irp6s_and_conv_effector ****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
