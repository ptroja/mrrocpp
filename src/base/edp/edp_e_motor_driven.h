/*!
 * \file edp_e_motor_driven.h
 * \brief File containing the declaration of edp::common::motor_driven_effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */

#ifndef __EDP_E_MOTOR_DRIVEN_H
#define __EDP_E_MOTOR_DRIVEN_H

#include <stdint.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>

#include "base/lib/condition_synchroniser.h"
#include "base/kinematics/kinematics_manager.h"
#include "base/edp/in_out.h"
#include "base/edp/edp_effector.h"

//#ifdef DOCENT_SENSOR
#include <boost/function.hpp>
//#endif

namespace mrrocpp {
namespace edp {
namespace sensor {
class force;
}
namespace common {

// TODO: remove forward declarations
class servo_buffer;
class edp_vsp;
class manip_trans_t;
class reader_buffer;
class vis_server;

enum STATE
{
	GET_STATE, GET_SYNCHRO, SYNCHRO_TERMINATED, GET_INSTRUCTION, EXECUTE_INSTRUCTION, WAIT, WAIT_Q
};

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
class motor_driven_effector : public effector, public kinematics::common::kinematics_manager
{
protected:
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

	//#ifdef DOCENT_SENSOR
	boost::function <void()> startedCallback_;
	bool startedCallbackRegistered_;
	boost::function <void()> stoppedCallback_;
	bool stoppedCallbackRegistered_;
	//#endif

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
	void get_inputs(lib::r_buffer & local_reply); // odczytanie wejsc binarnych

	/*!
	 * \brief method reset to zero the vectors of motor and joint position.
	 *
	 * number_of_servos should be previously set
	 */
	virtual void reset_variables();

	/*!
	 * \brief method to extract desired_motor_position basing on the ECP instruction with motor desired motor position in absolute or relative variant.
	 *
	 * It also checks kinematic constrains of motor and equivalent joint position.
	 */
	void compute_motors(const lib::c_buffer &instruction);

	/*!
	 * \brief method to compute desired_motor_position basing on the ECP instruction with motor desired joint position in absolute or relative variant.
	 *
	 * It also checks kinematic constrains of motor and equivalent joint position.
	 */
	void compute_joints(const lib::c_buffer &instruction);

	/*!
	 * \brief method to prepare command for servos (typically servo_buffer thread)
	 *
	 * It bases on desired_motor_position.
	 */
	void move_servos();

	/*!
	 * \brief motor position  currently computed in the servo
	 *
	 * for the single step of servo control
	 */
	lib::MotorArray servo_current_motor_pos;

	/*!
	 * \brief joint position currently computed in the servo
	 *
	 * for the single step of servo control
	 */
	lib::JointArray servo_current_joints;

	/*!
	 * \brief mutex to handle data set and get of the motor and joint position
	 *
	 * It is also used for the frame in child manip_effector_class
	 */
	boost::mutex effector_mutex;

	/*!
	 * \brief desired joints position
	 *
	 * for the whole macrostep
	 */
	lib::JointArray desired_joints;

	/*!
	 * \brief current joints position
	 *
	 * for the whole macrostep
	 */
	lib::JointArray current_joints;

	/*!
	 * \brief desired motor position for the previous macrostep
	 *
	 * for the whole macrostep
	 */
	lib::MotorArray desired_motor_pos_old;

	/*!
	 * \brief desired motor position for the next macrostep
	 *
	 * for the whole macrostep
	 */
	lib::MotorArray desired_motor_pos_new;

	/*!
	 * \brief current motor position
	 *
	 * for the whole macrostep
	 */
	lib::MotorArray current_motor_pos;

public:

	/*!
	 * \brief method to read current joint position stored in global_current_joints
	 *
	 * It is used for the purpose of the visualisation thread
	 */
	void master_joints_read(double[]);
	//#ifdef DOCENT_SENSOR
	void registerReaderStartedCallback(boost::function <void()> startedCallback);
	void registerReaderStoppedCallback(boost::function <void()> stoppedCallback);
	void onReaderStarted();
	void onReaderStopped();
	//#endif

	/*!
	 * \brief object to store output and input data
	 *
	 * It is used for the purpose of governing of input data form the hardware
	 * and transmission of output data to the hardware
	 */
	in_out_buffer in_out_obj;

	/*!
	 * \brief to wait for servo_buffer load in servo
	 */
	lib::condition_synchroniser sb_loaded;

	/*!
	 * \brief object to handle measurements
	 *
	 * It is implemented as the thread that collects the measurement in cyclic buffer and then saving it to the text file.
	 */
	boost::shared_ptr <reader_buffer> rb_obj;

	/*!
	 * \brief object that interpolates the motion in dedicated thread
	 *
	 * It is used for the purpose of interpolation in the external coordinates (in manipulators) e.g. for the purpose of position-force control
	 */
	boost::shared_ptr <manip_trans_t> mt_tt_obj;

	/*!
	 * \brief object of servo buffer
	 *
	 * With motor controllers in dedicated thread.
	 */
	boost::shared_ptr <servo_buffer> sb;

	/*!
	 * \brief Object of visualization
	 *
	 * This is dedicated thread that transmits joints positions to visualisation process.
	 */
	boost::shared_ptr <vis_server> vis_obj;

	/*!
	 * \brief force object to collect force measurements.
	 *
	 * The force measurements are collected in dedicated thread. Then the influence of gravitational force is removed in the same thread.
	 */
	boost::shared_ptr <sensor::force> vs;

	/*!
	 * \brief class constructor
	 *
	 * The attributes are initialized here.
	 */
	motor_driven_effector(lib::configurator &_config, lib::robot_name_t l_robot_name);

	/*!
	 * \brief class destructor
	 *
	 * The dynamic objects are deleted here.
	 */
	virtual ~motor_driven_effector();

	/*!
	 * \brief method to set the robot model commanded by ECP
	 *
	 * The model consists of servo algorithms and kinematic models
	 */
	virtual void set_robot_model(const lib::c_buffer &instruction);

	/*!
	 * \brief method to get (read) the robot model
	 *
	 * The model consists of servo algorithms and kinematic models. Then it is sent to the ECP
	 */
	virtual void get_robot_model(lib::c_buffer &instruction);

	/*!
	 * \brief structure with attributes describing the initial state of the effector
	 *
	 * If it is synchronised, power is on etc.
	 */
	lib::controller_state_t controller_state_edp_buf;

	/*!
	 * \brief the current step number
	 *
	 * The step counter depends on the number of steps executed in controllers starting from the beginning of the EDP execution.
	 */
	unsigned long step_counter;

	/*!
	 * \brief the number of servos
	 *
	 * It is set by the specific robots.
	 */
	unsigned short number_of_servos;

	/*!
	 * \brief pure virtual method of move arm to be implemented in specific robot.
	 *
	 * The child robot should decide which of the two following variants will be used.
	 */
	virtual void move_arm(const lib::c_buffer &instruction) = 0;

	/*!
	 * \brief move arm in two thread version
	 *
	 * This variant does uses extra thread for motion interpolation purpose. Two representation are handled here: joints and motors
	 */
	void multi_thread_move_arm(const lib::c_buffer &instruction);

	/*!
	 * \brief move arm in single thread version
	 *
	 * This variant does not use extra thread for motion interpolation purpose. Two representation are handled here: joints and motors
	 */
	void single_thread_move_arm(const lib::c_buffer &instruction);

	/*!
	 * \brief method to get position of the arm in the one of the representation commandenf by the ECP
	 *
	 * Here this method is pure virtual because it is not s specific robot method and the common parts are defined in other methods.
	 */
	virtual void get_arm_position(bool read_hardware, lib::c_buffer &instruction) = 0;

	/*!
	 * \brief common part of get_arm method that get current arm position from hardware
	 *
	 * Typically it is taken from servo_buffer
	 */
	void get_arm_position_read_hardware_sb();

	/*!
	 * \brief commonly used part of the get_arm_position method
	 *
	 * it defines the execution for the joint and motor coordinates.
	 */
	virtual void get_arm_position_get_arm_type_switch(lib::c_buffer &instruction);

	/*!
	 * \brief method to synchronise robot
	 *
	 * it is impossible to move robot in absolute coordinates before synchronisation.
	 */
	virtual void synchronise();

	/*!
	 * \brief method to compute servo_current_motor_pos, servo_cuurent_joints_pos and surve_current_frame in child classes
	 *
	 * It is commanded in every step of motor control.
	 * The computer servo_frame is used e.g. for the purpose of removal of gravity force from the raw force measurement.
	 *
	 */
	virtual bool compute_servo_joints_and_frame(void);

	/*!
	 * \brief main loop of the EDP master thread.
	 *
	 * It is a sewuence of the three following small loops.
	 */
	void main_loop();

	/*!
	 * \brief loop of the system before EDP is being synchronised.
	 *
	 * It waits for the command to gover initial system state.
	 */
	void pre_synchro_loop(STATE& next_state);

	/*!
	 * \brief loop of the system after the initial state is sent to ECP.
	 *
	 * It waits for the synchronisation command. Sometimes robot is initially synchronised just after it is run. Then this loop is inactive.
	 */
	void synchro_loop(STATE& next_state);

	/*!
	 * \brief loop after system synchronisation.
	 *
	 * This is common loop of the system, used durring task execution.
	 */
	void post_synchro_loop(STATE& next_state);

	/*!
	 * \brief method to create threads other then EDP master thread.
	 *
	 * It implemented for the purpose of the specific EDP effector, choosing the suitable components (e.g. servo_buffer, transformation etc.)
	 */
	void hi_create_threads();

	/*!
	 * \brief method interpreting ECP command.
	 *
	 * It decides what kind of command specific methods will be  run using master_order method.
	 */
	void interpret_instruction(lib::c_buffer &instruction);

	/*!
	 * \brief Method checking the algorithms of the motor controllers
	 *
	 * It typically communicates with servo_buffer or other hardware libraries.
	 */
	void get_algorithms();

	/*!
	 * \brief The method checks the initial state of the controller.
	 *
	 * This method typically communicates with hardware to check if the robot is synchronised etc.
	 * It is reimplemented in the inherited classes
	 */
	virtual void get_controller_state(lib::c_buffer &instruction); // by Y

	/*!
	 * \brief The method checks if the hardware is on.
	 *
	 * It is suitable to detect some kind of system initialization failure.
	 */
	bool is_power_on() const;

	/*!
	 * \brief method to increments servo_current_motor_pos by its argument
	 *
	 * It is used by the servo_buffer
	 */
	void update_servo_current_motor_pos(double motor_position_increment, size_t i);

	/*!
	 * \brief method to set servo_current_motor_pos as its argument
	 *
	 * It is used by the servo_buffer
	 */
	void update_servo_current_motor_pos_abs(double abs_motor_position, size_t i);

	/*!
	 * \brief method that generates type of reply commanded by the ECP.
	 *
	 * It takes into account instruction commanded by the ECP. The rep_type is also used during ECP command interpretation and execution to gother valid information to be replied to ECP.
	 */
	lib::REPLY_TYPE rep_type(const lib::c_buffer &instruction);

	/*!
	 * \brief method that check if the ECP command is valid for the robot before it is synchronised.
	 *
	 * The valid methods are specific motion instruction (in motor coordinates) and synchronisation command.
	 */
	bool pre_synchro_motion(lib::c_buffer &instruction) const;

	/*!
	 * \brief Method informing if the robot is synchronised or not.
	 *
	 * Some of the robots are initially synchronised after trurn on, some needs extra synchronisation procedure.
	 */
	bool is_synchronised(void) const;

	/*!
	 * \brief method that returns pointer to object of servo_buffer thread. It is used in some robots.
	 *
	 * when the servo_buffer thread is used it have to be implemented in specific robot. Otherwise it is not needed.
	 */
	virtual servo_buffer* return_created_servo_buffer();

	/*!
	 * \brief pure virtual method to be implemented in specific effector.
	 *
	 * It decides which variant of master_order is used (single or multi thread)
	 */
	virtual void master_order(MT_ORDER nm_task, int nm_tryb) = 0;

	/*!
	 * \brief method running ECP command specific methods in two thread version
	 *
	 * It uses extra, dedicated transformation thread
	 */
	void multi_thread_master_order(common::MT_ORDER nm_task, int nm_tryb);

	/*!
	 * \brief method running ECP command specific methods in single thread version
	 *
	 * It does not use extra transformation thread
	 */
	void single_thread_master_order(common::MT_ORDER nm_task, int nm_tryb);

	lib::c_buffer instruction;
	lib::r_buffer reply;

EIGEN_MAKE_ALIGNED_OPERATOR_NEW};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
