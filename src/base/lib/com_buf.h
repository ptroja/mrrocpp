////////////////////////////////////////////////////////////////////////////////
/*
 *  Data structures for IPC.
 */
////////////////////////////////////////////////////////////////////////////////
#ifndef __COM_BUF_H
#define __COM_BUF_H

#include "base/lib/xdr/xdr_iarchive.hpp"
#include "base/lib/xdr/xdr_oarchive.hpp"

#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/nvp.hpp>

#include "base/lib/impconst.h"
#include "base/lib/typedefs.h"

#include "base/lib/mrmath/homog_matrix.h"
#include "base/lib/mrmath/ft_v_vector.h"

#include "base/lib/messip/messip.h"

namespace mrrocpp {
namespace lib {

typedef messip_channel_t * fd_client_t;
static const fd_client_t invalid_fd = NULL;
typedef messip_channel_t * fd_server_t;

//------------------------------------------------------------------------------
/*!
 *  Type of command sent from MP to ECP.
 *  NEXT_POSE to command continuous coordination
 */
enum MP_COMMAND
{
	START_TASK, PAUSE_TASK, RESUME_TASK, NEXT_POSE, END_MOTION, NEXT_STATE, STOP
};

//------------------------------------------------------------------------------
/*!
 *  Type of reply from ECP to the MP command.
 */
enum ECP_REPLY
{
	INCORRECT_MP_COMMAND, ERROR_IN_ECP, ECP_ACKNOWLEDGE, TASK_TERMINATED
};

//------------------------------------------------------------------------------
/*!
 *  Type of arm position definition.
 */
enum POSE_SPECIFICATION
{
	INVALID_END_EFFECTOR, FRAME, JOINT, MOTOR, PF_VELOCITY
};

//------------------------------------------------------------------------------
/*!
 *  Type of arm position definition on the ECP level.
 */
enum ECP_POSE_SPECIFICATION
{
	ECP_INVALID_END_EFFECTOR, ECP_XYZ_ANGLE_AXIS, ECP_XYZ_EULER_ZYZ, ECP_JOINT, ECP_MOTOR, ECP_PF_VELOCITY
};

//------------------------------------------------------------------------------
/*!
 *  Trajectory description for a chosen type of interpolation.
 *
 *  @warning  Enum type POSE_SPECIFICATION moved to the front
 *            of the file, because it is used in this structure.
 *            Connected with C_MOTOR and C_JOINT etc. - watch out for the indexes.
 *  @see      POSE_SPECIFICATION  C_MOTOR  C_JOINT.
 */
struct trajectory_description
{
	/*! Robot arm representation. */
	ECP_POSE_SPECIFICATION arm_type;
	/*! Number of interpolation nodes. */
	unsigned int interpolation_node_no;
	/*! Number of steps for a single internode. */
	int internode_step_no;
	/*! Step in which the read position is returned. */
	int value_in_step_no;
	/*! Coordinates increment table. */
	double coordinate_delta[lib::MAX_SERVOS_NR];
};

//------------------------------------------------------------------------------
/*!
 *  Types of processes in MRROC++.
 */
typedef enum _PROCESS_TYPE
{
	UNKNOWN_PROCESS_TYPE, EDP, ECP, MP, VSP, UI
} process_type_t;

//------------------------------------------------------------------------------
/*!
 *  Definitions for available values of set_type i get_type.
 *  @author yoyek
 */
#define CONTROLLER_STATE_DEFINITION                     0x08
#define ARM_DEFINITION                                  0x04
#define ROBOT_MODEL_DEFINITION                          0x02
#define OUTPUTS_DEFINITION                              0x01
#define NOTHING_DEFINITION                              0x00

//------------------------------------------------------------------------------
/*! Error numbers generated in EDP.	*/
#define OK                                      0x0000000000000000ULL

#define INVALID_INSTRUCTION_TYPE                0x0100000000000000ULL
#define INVALID_REPLY_TYPE                      0x0200000000000000ULL
#define INVALID_SET_ROBOT_MODEL_TYPE            0x0300000000000000ULL
#define INVALID_GET_ROBOT_MODEL_TYPE            0x0400000000000000ULL
#define ERROR_IN_ROBOT_MODEL_REQUEST            0x0500000000000000ULL
#define INVALID_HOMOGENEOUS_MATRIX              0x0600000000000000ULL

#define QUERY_EXPECTED                          0x1000000000000000ULL
#define QUERY_NOT_EXPECTED                      0x1100000000000000ULL
#define NO_VALID_END_EFFECTOR_POSE              0x1200000000000000ULL
#define INVALID_MOTION_TYPE                     0x1300000000000000ULL
#define INVALID_MOTION_PARAMETERS               0x1400000000000000ULL
#define INVALID_SET_END_EFFECTOR_TYPE           0x1500000000000000ULL
#define INVALID_GET_END_EFFECTOR_TYPE           0x1600000000000000ULL
#define STRANGE_GET_ARM_REQUEST                 0x1700000000000000ULL

//------------------------------------------------------------------------------
/*! Range exceeding error codes (nonfatal) - drive shaft. */
#define BEYOND_UPPER_LIMIT_AXIS_0               0x2000000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_1               0x2100000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_2               0x2200000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_3               0x2300000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_4               0x2400000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_5               0x2500000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_6               0x2600000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_7               0x2700000000000000ULL

#define BEYOND_LOWER_LIMIT_AXIS_0               0x2800000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_1               0x2900000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_2               0x2A00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_3               0x2B00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_4               0x2C00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_5               0x2D00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_6               0x2E00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_7               0x2F00000000000000ULL

//------------------------------------------------------------------------------
/*! Range exceeding error codes (nonfatal) - internal coordinates. */
#define BEYOND_UPPER_D0_LIMIT                   0x3000000000000000ULL
#define BEYOND_UPPER_THETA1_LIMIT               0x3100000000000000ULL
#define BEYOND_UPPER_THETA2_LIMIT               0x3200000000000000ULL
#define BEYOND_UPPER_THETA3_LIMIT               0x3300000000000000ULL
#define BEYOND_UPPER_THETA4_LIMIT               0x3400000000000000ULL
#define BEYOND_UPPER_THETA5_LIMIT               0x3500000000000000ULL
#define BEYOND_UPPER_THETA6_LIMIT               0x3600000000000000ULL
#define BEYOND_UPPER_THETA7_LIMIT               0x3700000000000000ULL

#define BEYOND_LOWER_D0_LIMIT                   0x3800000000000000ULL
#define BEYOND_LOWER_THETA1_LIMIT               0x3900000000000000ULL
#define BEYOND_LOWER_THETA2_LIMIT               0x3A00000000000000ULL
#define BEYOND_LOWER_THETA3_LIMIT               0x3B00000000000000ULL
#define BEYOND_LOWER_THETA4_LIMIT               0x3C00000000000000ULL
#define BEYOND_LOWER_THETA5_LIMIT               0x3D00000000000000ULL
#define BEYOND_LOWER_THETA6_LIMIT               0x3E00000000000000ULL
#define BEYOND_LOWER_THETA7_LIMIT               0x3F00000000000000ULL

#define OUT_OF_WORKSPACE                        0x4100000000000000ULL
#define SINGULAR_POSE                           0x4200000000000000ULL

//------------------------------------------------------------------------------
/*! Standard mathematical functions - argument out of domain. */
#define ACOS_DOMAIN_ERROR                       0x4300000000000000ULL
#define ASIN_DOMAIN_ERROR                       0x4400000000000000ULL
#define ATAN2_DOMAIN_ERROR                      0x4500000000000000ULL
#define SQRT_DOMAIN_ERROR                       0x4600000000000000ULL
#define UNKNOWN_MATH_ERROR                      0x4700000000000000ULL

#define UNKNOWN_INSTRUCTION                     0x4800000000000000ULL
#define NOT_IMPLEMENTED_YET                     0x4900000000000000ULL

#define NOT_YET_SYNCHRONISED                    0x5200000000000000ULL
#define ALREADY_SYNCHRONISED                    0x5300000000000000ULL
#define UNKNOWN_SYNCHRO_ERROR                   0x5400000000000000ULL
#define INVALID_KINEMATIC_MODEL_NO              0x5500000000000000ULL
#define INVALID_KINEMATIC_CORRECTOR_NO          0x5600000000000000ULL
#define EDP_UNIDENTIFIED_ERROR                  0x5700000000000000ULL

#define NOT_A_NUMBER_JOINT_VALUE_D0             0x6000000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA1         0x6100000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA2         0x6200000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA3         0x6300000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA4         0x6400000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA5         0x6500000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA6         0x6600000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA7         0x6700000000000000ULL

//------------------------------------------------------------------------------
/*! Errors detected by SERVO_GROUP. */
#define SERVO_ERROR_IN_PASSIVE_LOOP             0x0004000000000000ULL
/*! Wrong command from EDP_MASTER. */
#define UNIDENTIFIED_SERVO_COMMAND              0x0008000000000000ULL
#define SERVO_ERROR_IN_PHASE_1                  0x000C000000000000ULL
#define SERVO_ERROR_IN_PHASE_2                  0x0010000000000000ULL

//------------------------------------------------------------------------------
/*! Errors detected in servo synchronization. */
#define SYNCHRO_SWITCH_EXPECTED                 0x0014000000000000ULL
#define SYNCHRO_ERROR                           0x0018000000000000ULL
#define SYNCHRO_DELAY_ERROR                     0x001C000000000000ULL

//------------------------------------------------------------------------------
/*! Detailed errors generated by ECP and MP. */
#define INVALID_MP_COMMAND                       0x1ULL
#define INVALID_POSE_SPECIFICATION               0x2ULL
#define INVALID_ROBOT_MODEL_TYPE                 0x3ULL
#define ECP_ERRORS                               0x6ULL
#define INVALID_COMMAND_TO_EDP                   0x7ULL
#define ECP_UNIDENTIFIED_ERROR                   0x8ULL
#define MP_UNIDENTIFIED_ERROR                    0x9ULL
#define EDP_ERROR                                0xAULL
#define NON_EXISTENT_DIRECTORY                   0xBULL
#define NON_EXISTENT_FILE                        0xCULL
#define READ_FILE_ERROR                          0xDULL
#define NON_TRAJECTORY_FILE                      0xEULL
#define NON_COMPATIBLE_LISTS                     0xFULL
#define ECP_STOP_ACCEPTED                       0x10ULL
#define MAX_ACCELERATION_EXCEEDED               0x11ULL
#define MAX_VELOCITY_EXCEEDED                   0x12ULL
#define INVALID_TIME_SPECIFICATION              0x14ULL
#define INVALID_ECP_PULSE_IN_MP_START_ALL       0x16ULL
#define INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL     0x17ULL
#define INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL   0x18ULL

//------------------------------------------------------------------------------
/*! Detailed errors generated by ECP and MP to VSP - type SYSTEM_ERROR. */
#define CANNOT_SPAWN_VSP                        0x20ULL
#define CANNOT_LOCATE_DEVICE                    0x21ULL
#define CANNOT_READ_FROM_DEVICE                 0x22ULL
#define CANNOT_WRITE_TO_DEVICE                  0x23ULL
#define DEVICE_ALREADY_EXISTS                   0x24ULL

/*! Detailed errors generated by ECP and MP to VSP - other errors. */
#define VSP_UNIDENTIFIED_ERROR                  0x27ULL

/*! Detailed errors generated by ECP and MP to VSP - in a trajectory generator. */
#define DANGEROUS_FORCE_DETECTED                0x28ULL
#define SAVE_FILE_ERROR                         0x29ULL
#define NAME_ATTACH_ERROR                       0x2AULL

//------------------------------------------------------------------------------
/*! Detailed errors generated by VSP - type SYSTEM_ERROR. */
#define DISPATCH_ALLOCATION_ERROR                0x1ULL
#define DEVICE_EXISTS                            0x2ULL
#define DEVICE_CREATION_ERROR                    0x3ULL
#define DISPATCH_LOOP_ERROR                      0x4ULL

/*! Detailed errors generated by VSP - type FATAL_ERROR. */
#define SENSOR_NOT_CONFIGURED                    0x5ULL
#define READING_NOT_READY                        0x6ULL

/*! Detailed errors generated by VSP - type NON_FATAL_ERROR. */
#define INVALID_COMMAND_TO_VSP                   0x7ULL

//------------------------------------------------------------------------------
enum GRIPPER_STATE_ENUM
{
	GRIPPER_START_STATE,
	GRIPPER_EXPAND_STATE,
	GRIPPER_NARROW_STATE,
	GRIPPER_BLOCKED_AFTER_EXPAND_STATE,
	GRIPPER_BLOCKED_AFTER_NARROW_STATE,
	GRIPPER_BLOCKED_STATE
};

//------------------------------------------------------------------------------
enum INSTRUCTION_TYPE
{
	SET, GET, SET_GET, SYNCHRO, QUERY
};

//------------------------------------------------------------------------------
typedef enum _ROBOT_MODEL_SPECIFICATION
{
	TOOL_FRAME, ARM_KINEMATIC_MODEL, SERVO_ALGORITHM, FORCE_TOOL, FORCE_BIAS
} ROBOT_MODEL_SPECIFICATION;

//------------------------------------------------------------------------------
enum MOTION_TYPE
{
	ABSOLUTE, RELATIVE
};

//------------------------------------------------------------------------------
enum INTERPOLATION_TYPE
{
	MIM, //! motor interpolated motion
	TCIM
//! task coordinates interpolated motion
};

//------------------------------------------------------------------------------
enum REPLY_TYPE
{
	ERROR,
	ACKNOWLEDGE,
	SYNCHRO_OK,
	ARM,
	ROBOT_MODEL,
	INPUTS,
	ARM_ROBOT_MODEL,
	ARM_INPUTS,
	ROBOT_MODEL_INPUTS,
	ARM_ROBOT_MODEL_INPUTS,
	CONTROLLER_STATE
/*
 * TODO: would not be it easier to handle with the following?
 ERROR = 0,
 ACKNOWLEDGE = 0x01,
 SYNCHRO_OK = 0x02,
 ARM = 0x04,
 ROBOT_MODEL = 0x08,
 INPUTS = 0x10,
 ARM_ROBOT_MODEL = 0x20,
 ARM_INPUTS = 0x40,
 ROBOT_MODEL_INPUTS = 0x80,
 ARM_ROBOT_MODEL_INPUTS = 0x100,
 CONTROLLER_STATE = 0x200
 */
};

//------------------------------------------------------------------------------
/*! @todo Rename from "behavior". */
enum BEHAVIOUR_SPECIFICATION
{
	UNGUARDED_MOTION, GUARDED_MOTION, CONTACT
};

//------------------------------------------------------------------------------
/*! Structure for error codes. */
struct edp_error
{
	uint64_t error0;
	uint64_t error1;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & error0;
		ar & error1;
	}
};

//------------------------------------------------------------------------------
/*! robot_model */
struct robot_model_t
{
	//! Constructor set default discriminant type
	robot_model_t() :
			type(ROBOT_MODEL_SPECIFICATION(-1))
	{
	}

	/*! Tool definition type - setting. */
	ROBOT_MODEL_SPECIFICATION type;

	//----------------------------------------------------------
	struct
	{
		/*! Tool trihedron relative to the collar. */
		lib::Homog_matrix tool_frame;
	} tool_frame_def;
	//----------------------------------------------------------
	struct
	{
		/*! Parameter set number for the kinematic kinematic_model_with_tool. */
		uint8_t kinematic_model_no;
	} kinematic_model;
	//----------------------------------------------------------
	struct
	{
		/*! Numbers for the servo-regulation algorithms. */
		uint8_t servo_algorithm_no[lib::MAX_SERVOS_NR];
		/*! Parameter set numbers for the servo-regulation algorithms. */
		uint8_t servo_parameters_no[lib::MAX_SERVOS_NR];
	} servo_algorithm;
	//----------------------------------------------------------
	struct
	{
		double position[3]; // TODO: this should be a Eigen::Vector3f
		double weight;
	} force_tool;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & type;
		switch (type)
		{
			case TOOL_FRAME:
				ar & tool_frame_def.tool_frame;
				break;
			case ARM_KINEMATIC_MODEL:
				ar & kinematic_model.kinematic_model_no;
				break;
			case SERVO_ALGORITHM:
				ar & servo_algorithm.servo_algorithm_no;
				ar & servo_algorithm.servo_parameters_no;
				break;
			case FORCE_TOOL:
				ar & force_tool.position;
				ar & force_tool.weight;
				break;
			default:
				break;
		}
	}
};

//------------------------------------------------------------------------------
//                                  c_buffer
//------------------------------------------------------------------------------

typedef robot_model_t c_buffer_robot_model_t;

//------------------------------------------------------------------------------
/*! arm */
struct c_buffer_arm_t
{
	//----------------------------------------------------------
	struct
	{
		/*!  End's trihedron relative to the base system. */
		lib::Homog_matrix arm_frame;
		/*! XYZ + end's orientation relative to the base system. */
		double arm_coordinates[lib::MAX_SERVOS_NR];
		/*! Given torque. */
		double desired_torque[lib::MAX_SERVOS_NR];
		double inertia[6], reciprocal_damping[6];
		lib::Ft_vector force_xyz_torque_xyz;
		BEHAVIOUR_SPECIFICATION behaviour[6];
	} pf_def;
	//----------------------------------------------------------

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & pf_def.arm_frame; // if set_arm_type == FRAME
		ar & pf_def.arm_coordinates; // otherwise.
		ar & pf_def.desired_torque;
		ar & pf_def.inertia;
		ar & pf_def.reciprocal_damping;
		ar & pf_def.force_xyz_torque_xyz;
		ar & pf_def.behaviour;

	}
};

//------------------------------------------------------------------------------
struct c_buffer
{
	/*! Type of the instruction. */
	INSTRUCTION_TYPE instruction_type;
	/*! Type of the SET instruction. */
	uint8_t set_type;
	/*! Type of the GET instruction. */
	uint8_t get_type;
	/*! Tool definition type - reading. */
	ROBOT_MODEL_SPECIFICATION get_robot_model_type;
	/*! Definition type of the end-effector's given position. */
	POSE_SPECIFICATION set_arm_type;

	/*! Binary outputs values. */
	uint16_t output_values;

	/*! Type of interpolation. */
	INTERPOLATION_TYPE interpolation_type;

	/*! Type of motion - means of describing the shift. */
	MOTION_TYPE motion_type;
	/*! Number of steps for a given shift (macrostep). */
	uint16_t motion_steps;

	/*!
	 *  Number of steps for the 1st movemement phase.
	 *  Krok, w ktorym ma zostac przekazana informacja
	 *  o realizacji pierwszej fazy ruchu:
	 *  0 < value_in_step_no <= motion_steps + 1 .
	 *
	 *  Dla value_in_step_no = motion_steps
	 *  wiadomosc dotrze po zrealizowaniu makrokroku,
	 *  ale informacja o polozeniu bedzie dotyczyc
	 *  realizacji przedostatniego kroku makrokroku.
	 *
	 *  Dla value_in_step_no = motion_steps + 1
	 *  wiadomosc dotrze po zrealizowaniu jednego kroku
	 *  obiegu petli ruchu jalowego po zakonczeniu makrokroku,
	 *  ale informacja o polozeniu bedzie dotyczyc
	 *  realizacji calego makrokroku.
	 *
	 *  Dla value_in_step_no < motion_steps
	 *  wiadomosc dotrze przed zrealizowaniem makrokroku
	 *  i informacja o polozeniu bedzie dotyczyc
	 *  realizacji srodkowej fazy makrokroku.
	 */
	uint16_t value_in_step_no;
	c_buffer_robot_model_t robot_model;
	c_buffer_arm_t arm;

	//-----------------------------------------------------
	//                      METHODS
	//-----------------------------------------------------

	c_buffer(void); // by W odkomentowane
	/*!
	 *  Oczytac wejscia?
	 *  @todo Translate to English.
	 */
	bool is_get_controller_state() const;
	/*!
	 *  Oczytac wejscia?
	 *  @todo Translate to English.
	 */
	bool is_get_inputs() const;
	/*!
	 *  Odczytac narzedzie?
	 *  @todo Translate to English.
	 */
	bool is_get_robot_model() const;
	/*!
	 *  Odczytac polozenie ramienia?
	 *  @todo Translate to English.
	 */
	bool is_get_arm() const;
	/*!
	 *  Ustawic wyjscia?
	 *  @todo Translate to English.
	 */
	bool is_set_outputs() const;
	/*!
	 *  Zmienic narzedzie?
	 *  @todo Translate to English.
	 */
	bool is_set_robot_model() const;
	/*!
	 *  Zmienic polozenie ramienia?
	 *  @todo Translate to English.
	 */
	bool is_set_arm() const;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & instruction_type;
		ar & set_type;
		ar & get_type;
		ar & get_robot_model_type;
		ar & set_arm_type;
		ar & output_values;
		ar & interpolation_type;
		ar & motion_type;
		ar & motion_steps;
		ar & value_in_step_no;
		ar & robot_model;
		ar & arm;
	}
};

//------------------------------------------------------------------------------
//                                  r_buffer
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/*! robot_model */
typedef robot_model_t r_buffer_robot_model_t;

//------------------------------------------------------------------------------
struct controller_state_t
{
	//! Flag informing whether the robot is synchronized or not.
	bool is_synchronised;

	//! Flag telling whether the power is supplied to effector motors or not.
	bool is_power_on;

	//! Flag informing whewher robot is in fault state or not.
	bool robot_in_fault_state;

private:
	//! Give access to boost::serialization framework.
	friend class boost::serialization::access;

	//! Serialization of the data structure.
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & is_synchronised;
		ar & is_power_on;
		ar & robot_in_fault_state;
	}
};

//------------------------------------------------------------------------------
/*! arm */
struct r_buffer_arm_t
{
	/*!
	 *  Sposob  zdefiniowania polozenia zadanego koncowki.
	 *  @todo Translate to English.
	 */
	// POSE_SPECIFICATION type;
	struct
	{
		/*!
		 *  Macierz reprezentujaca koncowke wzgledem bazy manipulatora.
		 *  @todo Translate to English.
		 */
		lib::Homog_matrix arm_frame;

		double joint_coordinates[lib::MAX_SERVOS_NR];

		double motor_coordinates[lib::MAX_SERVOS_NR];

		lib::Ft_vector force_xyz_torque_xyz;
	} pf_def;

	/*!
	 *  Measured current macrostep statistics of particular axis
	 */
	struct
	{
		/*!
		 *  Average module
		 */
		unsigned short average_module[lib::MAX_SERVOS_NR];

		/*!
		 *  Minimum module
		 */
		unsigned short minimum_module[lib::MAX_SERVOS_NR];

		/*!
		 *  Maksimum module
		 */
		unsigned short maximum_module[lib::MAX_SERVOS_NR];

		/*!
		 *  Average square
		 */
		float average_square[lib::MAX_SERVOS_NR];

		/*!
		 *  Average cubic
		 */
		float average_cubic[lib::MAX_SERVOS_NR];

		/*!
		 *  energy
		 * For the whole macrostep sum of measured currents for current step multiplied by pwm for previous step
		 */
		float energy[lib::MAX_SERVOS_NR];

	} measured_current;

	/*!
	 *  State of grippers regutor
	 */
	int16_t gripper_reg_state;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		//	ar & type;

		ar & pf_def.arm_frame;
		ar & pf_def.joint_coordinates;
		ar & pf_def.motor_coordinates;

		ar & pf_def.force_xyz_torque_xyz;
		ar & gripper_reg_state;

		ar & measured_current.average_module;
		ar & measured_current.minimum_module;
		ar & measured_current.maximum_module;
		ar & measured_current.average_square;
		ar & measured_current.average_cubic;
		ar & measured_current.energy;

	}
};

//------------------------------------------------------------------------------
struct r_buffer_base
{
	/*! Type of the reply. */
	REPLY_TYPE reply_type;

	/*! Number of the error (if it occured). */
	edp_error error_no;

	//! Set default values
	r_buffer_base();

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & reply_type;
		ar & error_no;
	}
};

struct r_buffer : r_buffer_base
{
	/*!
	 *  Wartosci wejsc binarnych.
	 *  @todo Translate to English.
	 */
	uint16_t input_values;

	/*! Analog inputs. */
	uint8_t analog_input[8];

	controller_state_t controller_state;

	/*! Number of the servo step. */
	uint32_t servo_step;

	/*! Given values for PWM (Pulse-width modulation), usually unnecessary. */
	int16_t PWM_value[lib::MAX_SERVOS_NR];

	/*! Control currents, usually unnecessary. */
	int16_t current[lib::MAX_SERVOS_NR];

	r_buffer_robot_model_t robot_model;
	r_buffer_arm_t arm;

	//-----------------------------------------------------
	//                      METHODS
	//-----------------------------------------------------
	r_buffer(void);

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		// serialize base class information
		ar & boost::serialization::base_object <r_buffer_base>(*this);
		ar & input_values;
		ar & analog_input;
		ar & controller_state;
		ar & servo_step;
		ar & PWM_value;
		ar & current;
		// The following are unions... probably have to handle with boost::variant
		ar & robot_model;
		ar & arm;
	}
};

//------------------------------------------------------------------------------
/*!
 *\brief Buffer to store data for communication between processes with
 *\brief various data types and serialization
 */
class seter_geter_buffer_t
{
public:

	/*!
	 * \brief memory to store data
	 */
	uint32_t data[MP_2_ECP_SERIALIZED_DATA_SIZE / sizeof(uint32_t)];

	/*!
	 * \brief template method to put data into the memory (serialize)
	 */
	template <typename BUFFER_TYPE>
	void set(const BUFFER_TYPE & buffer)
	{
		xdr_oarchive <> oa;
		oa << buffer;

		//sprawdza wielkosc czy nie przekracza wielkosci bufora z assert
		assert(MP_2_ECP_SERIALIZED_DATA_SIZE > oa.getArchiveSize());

		// serializacja
		memcpy(data, oa.get_buffer(), oa.getArchiveSize());
	}

	/*!
	 * \brief template method to get data data from the memory (deserialize)
	 */
	template <typename BUFFER_TYPE>
	void get(BUFFER_TYPE & buffer) const
	{
		// deserializacja
		xdr_iarchive <> ia((const char *) data, (std::size_t) MP_2_ECP_SERIALIZED_DATA_SIZE);

		ia >> buffer;
	}

	/*!
	 * \brief template method to get data data from the memory (deserialize)
	 */
	template <typename BUFFER_TYPE>
	BUFFER_TYPE get() const
	{
		BUFFER_TYPE buffer_tmp;

		get(buffer_tmp);

		return buffer_tmp;
	}

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & data;
	}
};

//------------------------------------------------------------------------------
/*!
 *  Zlecenie zmiany stanu ECP skojarzone z NEXT_STATE.
 *  @todo Translate to English.
 */
struct ecp_next_state_t
{
	std::string next_state;

	int variant;

	seter_geter_buffer_t sg_buf;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & next_state;
		ar & variant;
		ar & sg_buf;
	}
};

//------------------------------------------------------------------------------
/*! MP to ECP command. */
template <class NEXT_STATE_T>
struct _MP_COMMAND_PACKAGE
{
	MP_COMMAND command;
	NEXT_STATE_T ecp_next_state;
	c_buffer instruction;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & command;
		switch (command)
		{
			case NEXT_STATE:
				ar & ecp_next_state;
				break;
			case NEXT_POSE:
				ar & instruction;
				break;
			default:
				break;
		}
	}
};

typedef struct _MP_COMMAND_PACKAGE <ecp_next_state_t> MP_COMMAND_PACKAGE;

//------------------------------------------------------------------------------
/*! ECP to MP reply. */
struct ECP_REPLY_PACKAGE
{
	ECP_REPLY reply;

	// TODO: this should be rather union, but it is not possible to union non-POD objects
	r_buffer reply_package;
	int variant;
	seter_geter_buffer_t sg_buf;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & reply;
		ar & reply_package;
		ar & variant;
		ar & sg_buf;
	}
};
// ------------------------------------------------------------------------

/**
 * @brief Empty data structure.
 */
struct empty_t
{
private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
	}
};

} // namespace lib
} // namespace mrrocpp

/**
 * @brief Spline type
 */
enum splineType
{
	linear = 1, cubic = 2, quintic = 3
};

#endif
