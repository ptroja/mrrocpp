////////////////////////////////////////////////////////////////////////////////
/*! \file     src/lib/com_buf.h
 *
 *  Data structures for IPC.
 *
 *  \author   tkornuta
 *  \date     2006-11-29
 *  \URL: https://segomo.elka.pw.edu.pl/svn/mrrocpp/base/trunk/include/lib/com_buf.h $
 *  $LastChangedRevision$
 *  $LastChangedDate$
 *  $LastChangedBy$
 *
 *  \todo <ul>
 *          <li>Translate to English where necessary.</li>
 *          <li>Write detailed comments.</li>
 *          <li>Suplement comments for those consts, variables and structures
 *              that are not commented at all.</li>
 *          <li>Clean up the commented fragments of code.</li>
 *        </ul>
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef __COM_BUF_H
#define __COM_BUF_H

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip.h"
#endif

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>

namespace mrrocpp {
namespace lib {

#define ECP_EDP_SERIALIZED_COMMAND_SIZE 1000
#define EDP_ECP_SERIALIZED_REPLY_SIZE 1000

//------------------------------------------------------------------------------
/*!
 *  Type of command sent from MP to ECP.
 */
enum MP_COMMAND
{
	INVALID_COMMAND, START_TASK, NEXT_POSE, END_MOTION, NEXT_STATE, STOP
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
 *  Reply types from UI to ECP and commands from UI (pressing a button).
 */
enum UI_TO_ECP_COMMAND
{
	INVALID_REPLY,
	NEXT,
	QUIT,
	ANSWER_YES,
	ANSWER_NO,
	FILE_LOADED,
	FILE_SAVED,

	/*! Commands from Force Control window. */
	FC_ADD_MACROSTEP,
	FC_CALIBRATE_SENSOR,
	FC_CHANGE_CONTROL,
	FC_MOVE_ROBOT,
	FC_SAVE_TRAJECTORY,
	FC_NEW_TRAJECTORY,
	FC_EXIT,
	FC_GET_DATA,

	/*! Commands from Trajectory Render window. */
	TR_LOAD_TRAJECTORY,
	TR_PAUSE_MOVE,
	TR_START_MOVE,
	TR_STOP_MOVE,
	TR_EXIT,
	TR_ZERO_POSITION,
	TR_SAVE_READINGS,
	TR_CALIBRATE_DIGITAL_SCALES_SENSOR,
	TR_CALIBRATE_FORCE_SENSOR,
	TR_TRY_MOVE_AGAIN,

	/*! Replies from the options window. */
	OPTION_ONE,
	OPTION_TWO,
	OPTION_THREE,
	OPTION_FOUR,

	/*!
	 *  Commands from the window
	 *  MAM_wnd_manual_moves_automatic_measures.
	 */
	MAM_START,
	MAM_STOP,
	MAM_CLEAR,
	MAM_SAVE,
	MAM_EXIT,
	MAM_CALIBRATE
};

//------------------------------------------------------------------------------
/*!
 *  Types of ECP to UI commands.
 */
enum ECP_TO_UI_COMMAND
{
	C_INVALID_END_EFFECTOR,
	C_FRAME,
	C_XYZ_ANGLE_AXIS,
	C_XYZ_EULER_ZYZ,
	C_JOINT,
	C_MOTOR,
	YES_NO,
	DOUBLE_NUMBER,
	INTEGER_NUMBER,
	SAVE_FILE,
	LOAD_FILE,
	MESSAGE,
	OPEN_FORCE_SENSOR_MOVE_WINDOW,
	OPEN_TRAJECTORY_REPRODUCE_WINDOW,
	TR_REFRESH_WINDOW,
	TR_DANGEROUS_FORCE_DETECTED,
	CHOOSE_OPTION,
	MAM_OPEN_WINDOW,
	MAM_REFRESH_WINDOW
};

//------------------------------------------------------------------------------
/*! Length of a message sent from ECP to MP or UI */
#define MSG_LENGTH 60

//------------------------------------------------------------------------------
/*!
 *  ECP to UI message.
 */
struct ECP_message
{
#ifndef USE_MESSIP_SRR
	msg_header_t hdr;
#endif
	/*! Type of message. */
	ECP_TO_UI_COMMAND ecp_message;
	/*! Robot name. */
	robot_name_t robot_name;
	/*! Number of options - from 2 to 4 - - for CHOOSE_OPTION mode. */
	uint8_t nr_of_options;

	//----------------------------------------------------------
#ifndef USE_MESSIP_SRR
	union
	{
#endif
		/*! A comment for the command. */
		char string[MSG_LENGTH];

		//------------------------------------------------------
		struct
		{
			double robot_position[lib::MAX_SERVOS_NR];
			double sensor_reading[lib::MAX_SERVOS_NR];
		}
		/*! Robot positions + Sensor readings. */
		RS;
		//------------------------------------------------------
		struct
		{
			double robot_position[lib::MAX_SERVOS_NR];
			double digital_scales_sensor_reading[6];
			double force_sensor_reading[6];
		}
		/*! Robot positions + 2 * (Sensor readings). */
		R2S;
		//------------------------------------------------------
		struct
		{
			double robot_position[lib::MAX_SERVOS_NR];
			double sensor_reading[6];
			int measure_number;
		}
		/*! Robot positions + Sensor readings + Measure number. */
		MAM;
#ifndef USE_MESSIP_SRR
	};
#endif

    //! Give access to boost::serialization framework
    friend class boost::serialization::access;

    //! Serialization of the data structure
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & ecp_message;
        ar & robot_name;
        ar & nr_of_options;
        /*
        ar & string;

        ar & RS.robot_position;
        ar & RS.sensor_reading;

        ar & R2S.robot_position;
        ar & R2S.digital_scales_sensor_reading;
        ar & R2S.force_sensor_reading;

        ar & MAM.robot_position;
        ar & MAM.sensor_reading;
        ar & MAM.measure_number;
        */
    }
};

//------------------------------------------------------------------------------
/*!
 *  UI to ECP reply.
 */
struct UI_reply
{
#ifndef USE_MESSIP_SRR
	msg_header_t hdr;
#endif
	UI_TO_ECP_COMMAND reply;
	int integer_number;
	double double_number;
	double coordinates[lib::MAX_SERVOS_NR];
	char path[80];
	char filename[20];

    //! Give access to boost::serialization framework
    friend class boost::serialization::access;

    //! Serialization of the data structure
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & reply;
        ar & integer_number;
        ar & double_number;
        ar & coordinates;
        ar & path;
        ar & filename;
    }
};

//------------------------------------------------------------------------------
/*!
 *  Message from UI to ECP.
 */
struct UI_ECP_message
{
#ifndef USE_MESSIP_SRR
	msg_header_t hdr;
#endif

	UI_TO_ECP_COMMAND command;

	union
	{
		/*! The name of the file. */
		char filename[100];
		/*! Time of the robot's motion. */
		int motion_time;
		/*! (axis - 1..6) && (+/- left/right). */
		short move_type;
		/*! Change of control type. */
		POSE_SPECIFICATION ps;
	};
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
/*! Error classes. */
typedef enum _ERROR_CLASS
{
	NEW_MESSAGE, SYSTEM_ERROR, FATAL_ERROR, NON_FATAL_ERROR
} error_class_t;

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
	INVALID, SET, GET, SET_GET, SYNCHRO, QUERY
};

//------------------------------------------------------------------------------
enum ROBOT_MODEL_SPECIFICATION
{
	INVALID_ROBOT_MODEL, TOOL_FRAME, ARM_KINEMATIC_MODEL, SERVO_ALGORITHM, FORCE_TOOL, FORCE_BIAS
};

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

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & error0;
		ar & error1;
	}
};

//------------------------------------------------------------------------------
enum SERVO_COMMAND
{
	MOVE, READ, SYNCHRONISE, SERVO_ALGORITHM_AND_PARAMETERS
};

//------------------------------------------------------------------------------
/*! Structure of a command from EDP_MASTER to SERVO_GROUP. */
struct edp_master_command
{
	/*! Code for the instruction sent to SERVO_GROUP. */
	SERVO_COMMAND instruction_code;
	union
	{
		//------------------------------------------------------
		struct
		{
			/*! Number of steps for a macrostep. */
			uint16_t number_of_steps;
			/*!
			 *  Number of steps after which the information about the previously
			 *  realized position is to be sent.
			 *  Information is sent to EDP_MASTER using READING_BUFFER.
			 *  After k steps SERVO has the position from the k-1 step.
			 */
			uint16_t return_value_in_step_no;
			/*! Length of a macrostep (given value of a macrostep - increase). */
			double macro_step[lib::MAX_SERVOS_NR];
			/*! Given absolute position at the end of a macrostep. */
			double abs_position[lib::MAX_SERVOS_NR];
		} move;
		//------------------------------------------------------
		struct
		{
			/*! Servo algorithm numbers. */
			uint8_t servo_algorithm_no[lib::MAX_SERVOS_NR];
			/*! Numbers fo servo algorithm parameters set. */
			uint8_t servo_parameters_no[lib::MAX_SERVOS_NR];
		} servo_alg_par;

	} parameters;
};

//------------------------------------------------------------------------------
/*! Structure of a reply from SERVO_GROUP to EDP_MASTER. */
struct servo_group_reply
{
	/*! Error code. */
	edp_error error;
	/*! Position increment of the motor shaft reached since the last reading. */
	double position[lib::MAX_SERVOS_NR];
	/*! Absolute position of the joints (in radians). */
	double abs_position[lib::MAX_SERVOS_NR];
	/*! Given values for PWM fill (Phase Wave Modulation) - (usualy unnecessary). */
	int16_t PWM_value[lib::MAX_SERVOS_NR];
	/*! Control current - (usualy unnecessary). */
	int16_t current[lib::MAX_SERVOS_NR];
	/*! Numbers for the regulation algorithms in use. */
	uint8_t algorithm_no[lib::MAX_SERVOS_NR];
	uint8_t algorithm_parameters_no[lib::MAX_SERVOS_NR];
	/*! Gripper regulator state. */
	short gripper_reg_state;
};

//------------------------------------------------------------------------------
//                                  c_buffer
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
/*! robot_model */
typedef
#ifndef USE_MESSIP_SRR
union
#else
struct
#endif
c_buffer_robot_model
{
	//----------------------------------------------------------
	struct
	{
		/*! Tool trihedron ralative to the collar. */
		frame_tab tool_frame;
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

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & tool_frame_def.tool_frame;
		ar & kinematic_model.kinematic_model_no;
		ar & servo_algorithm.servo_algorithm_no;
		ar & servo_algorithm.servo_parameters_no;
		ar & force_tool.position;
		ar & force_tool.weight;
	}
} c_buffer_robot_model_t;

//------------------------------------------------------------------------------
/*! arm */
typedef union c_buffer_arm
{
	//----------------------------------------------------------
	struct
	{
		/*!  End's trihedron ralative to the base system. */
		frame_tab arm_frame;
		/*! XYZ + end's orientation relative to the base system. */
		double arm_coordinates[lib::MAX_SERVOS_NR];
		/*! Given torque. */
		double desired_torque[lib::MAX_SERVOS_NR];
		double inertia[6], reciprocal_damping[6];
		double force_xyz_torque_xyz[6];
		BEHAVIOUR_SPECIFICATION behaviour[6];
	} pf_def;
	//----------------------------------------------------------
	struct
	{
		/*! text to speak */
		char text[lib::MAX_TEXT];
		/*! prosody of the text to speak */
		char prosody[lib::MAX_PROSODY];
	} text_def;
	//----------------------------------------------------------
	char serialized_command[ECP_EDP_SERIALIZED_COMMAND_SIZE];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & pf_def.arm_frame;
		ar & pf_def.arm_coordinates;
		ar & pf_def.desired_torque;
		ar & pf_def.inertia;
		ar & pf_def.reciprocal_damping;
		ar & pf_def.force_xyz_torque_xyz;
		ar & pf_def.arm_frame;
		ar & pf_def.behaviour;

//		ar & text_def.text;
//		ar & text_def.prosody;

///		ar & serialized_command;
	}
} c_buffer_arm_t;

//------------------------------------------------------------------------------
struct c_buffer
{
	/*! Type of the instruction. */
	INSTRUCTION_TYPE instruction_type;
	/*! Type of the SET instruction. */
	uint8_t set_type;
	/*! Type of the GET instruction. */
	uint8_t get_type;
	/*! Tool definition type - setting. */
	ROBOT_MODEL_SPECIFICATION set_robot_model_type;
	/*! Tool definition type - reading. */
	ROBOT_MODEL_SPECIFICATION get_robot_model_type;
	/*! Definition type of the end-effector's given position. */
	POSE_SPECIFICATION set_arm_type;
	/*! Definition type of the end-effector's read position. */
	POSE_SPECIFICATION get_arm_type;
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

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & instruction_type;
		ar & set_type;
		ar & get_type;
		ar & set_robot_model_type;
		ar & get_robot_model_type;
		ar & set_arm_type;
		ar & get_arm_type;
		ar & output_values;
		ar & interpolation_type;
		ar & motion_type;
		ar & motion_steps;
		ar & value_in_step_no;
		ar & robot_model;
		ar & arm;
	}

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
};

//------------------------------------------------------------------------------
//                                  r_buffer
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
/*! robot_model */
typedef
#ifndef USE_MESSIP_SRR
union
#else
struct
#endif
_r_buffer_robot_model
{
	//----------------------------------------------------------
	struct
	{
		/*!
		 *  Macierz reprezentujaca narzedzie wzgledem konca lancucha kinematycznego.
		 *  @todo Translate to English.
		 */
		frame_tab tool_frame;
	} tool_frame_def;
	//----------------------------------------------------------
	struct
	{
		/*!
		 *  Numer modelu kinematyki.
		 *  @todo Translate to English.
		 */
		uint8_t kinematic_model_no;
	} kinematic_model;
	//----------------------------------------------------------
	struct
	{
		/*!
		 *  Numery algorytmow serworegulacji.
		 *  @todo Translate to English.
		 */
		uint8_t servo_algorithm_no[lib::MAX_SERVOS_NR];
		/*!
		 *  Numery zestawu parametrow algorytmow serworegulacji.
		 *  @todo Translate to English.
		 */
		uint8_t servo_parameters_no[lib::MAX_SERVOS_NR];
	} servo_algorithm;
	//----------------------------------------------------------
	struct
	{
		double position[3];
		double weight;
	} force_tool;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & tool_frame_def.tool_frame;
		ar & kinematic_model.kinematic_model_no;
		ar & servo_algorithm.servo_algorithm_no;
		ar & servo_algorithm.servo_parameters_no;
		ar & force_tool.position;
		ar & force_tool.weight;
	}

} r_buffer_robot_model_t;

//------------------------------------------------------------------------------
typedef struct _controller_state_t
{
	/*! Is robot synchronised? */
	bool is_synchronised;
	/*!
	 *  Czy wzmacniacze mocy sa zasilane?
	 *  @todo Translate to English.
	 */
	bool is_power_on;
	/*!
	 *  Czy szafa jest waczona?
	 *  @todo Translate to English.
	 *        Change the "wardrobe" thing for God's sake !!!
	 */
	bool is_wardrobe_on;
	/*!
	 *  Czy wyzerowano sterowanie na silnikach po awarii sprzetowej?
	 *  @todo Translate to English.
	 */
	bool is_robot_blocked;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & is_synchronised;
		ar & is_power_on;
		ar & is_wardrobe_on;
		ar & is_robot_blocked;
	}
} controller_state_t;

//------------------------------------------------------------------------------
/*! arm */
typedef
#ifndef USE_MESSIP_SRR
union
#else
struct
#endif
r_buffer_arm
{
	struct
	{
		/*!
		 *  Macierz reprezentujaca koncowke wzgledem bazy manipulatora.
		 *  @todo Translate to English.
		 */
		frame_tab arm_frame;
		/*!
		 *  XYZ + orientacja koncowki wzgledem ukladu bazowego.
		 *  @todo Translate to English.
		 */
		double arm_coordinates[lib::MAX_SERVOS_NR];
		double force_xyz_torque_xyz[6];
		/*!
		 *  Stan w ktorym znajduje sie regulator chwytaka.
		 *  @todo Translate to English.
		 */
		short gripper_reg_state;
	} pf_def;
	//----------------------------------------------------------
	struct
	{
		/*!
		 *  Czy mowi?
		 *  @todo Translate to English.
		 */
		bool speaking;
	} text_def;

	char serialized_reply[EDP_ECP_SERIALIZED_REPLY_SIZE];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & pf_def.arm_frame;
		ar & pf_def.arm_coordinates;
		ar & pf_def.force_xyz_torque_xyz;
		ar & pf_def.gripper_reg_state;
		ar & text_def.speaking;
//		ar & serialized_reply;
	}
} r_buffer_arm_t;

//------------------------------------------------------------------------------
struct r_buffer
{
	/*! Type of the reply. */
	REPLY_TYPE reply_type;
	/*! Number of the error (if it occured). */
	edp_error error_no;
	/*!
	 *  Sposob zdefiniowania narzedzia przy jego odczycie.
	 *  @todo Translate to English.
	 */
	ROBOT_MODEL_SPECIFICATION robot_model_type;
	/*!
	 *  Sposob  zdefiniowania polozenia zadanego koncowki.
	 *  @todo Translate to English.
	 */
	POSE_SPECIFICATION arm_type;
	/*!
	 *  Wartosci wejsc binarnych.
	 *  @todo Translate to English.
	 */
	uint16_t input_values;
	/*! Analog input. */
	uint8_t analog_input[8];
	controller_state_t controller_state;
	/*! Number of the servo step. */
	unsigned long servo_step;
	/*! Given values for PWM fill (Phase Wave Modulation) - (usualy unnecessary). */
	int16_t PWM_value[lib::MAX_SERVOS_NR];
	/*! Control current - (usualy unnecessary). */
	int16_t current[lib::MAX_SERVOS_NR];
	r_buffer_robot_model_t robot_model;
	r_buffer_arm_t arm;

	//-----------------------------------------------------
	//                      METHODS
	//-----------------------------------------------------
	r_buffer(void); // W odkomentowane

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & reply_type;
		ar & error_no;
		ar & robot_model_type;
		ar & arm_type;
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
}
#ifndef USE_MESSIP_SRR
__attribute__((__packed__))
#endif
;
//------------------------------------------------------------------------------
/*! Target position for the mobile robot. */
class playerpos_goal_t
{
private:
	double x, y, t;

public:
	void forward(double length);
	void turn(double angle);
	void setGoal(double _x, double _y, double _z);

	double getX() const;
	double getY() const;
	double getT() const;

	playerpos_goal_t(double _x, double _y, double _t);
	playerpos_goal_t();
};

//------------------------------------------------------------------------------
/*!
 *  Zlecenie zmiany stanu ECP skojarzone z NEXT_STATE.
 *  @todo Translate to English.
 */
struct ecp_next_state_t
{
	char mp_2_ecp_next_state[MP_2_ECP_NEXT_STATE_STRING_SIZE];
	int mp_2_ecp_next_state_variant;
	char mp_2_ecp_next_state_string[MP_2_ECP_STRING_SIZE];

	/*! Target position for the mobile robot. */
	playerpos_goal_t playerpos_goal;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & mp_2_ecp_next_state;
		ar & mp_2_ecp_next_state_variant;
		ar & mp_2_ecp_next_state_string;
		// ar & playerpos_goal; // this is not needed at this moment
	}
};

//------------------------------------------------------------------------------
struct ecp_command_buffer
{
#ifndef USE_MESSIP_SRR
	/*! This is a message buffer, so it needs a message header */
	msg_header_t hdr;
#endif
	c_buffer instruction;

    //! Give access to boost::serialization framework
    friend class boost::serialization::access;

    //! Serialization of the data structure
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & instruction;
    }
};

//------------------------------------------------------------------------------
/*! MP to ECP command. */
struct MP_COMMAND_PACKAGE
{
#ifndef USE_MESSIP_SRR
	msg_header_t hdr;
#endif
	MP_COMMAND command;
	ecp_next_state_t ecp_next_state;
	c_buffer instruction;
	bool pulse_to_ecp_sent;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & command;
		ar & ecp_next_state;
		ar & instruction;
	}
};

//------------------------------------------------------------------------------
/*! ECP to MP reply. */
struct ECP_REPLY_PACKAGE
{
	ECP_REPLY reply;

	// TODO: this should be rather union, but it is not possible to union non-POD objects
	r_buffer reply_package;
	char recognized_command[ECP_2_MP_STRING_SIZE];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & reply;
		ar & reply_package;
		ar & recognized_command; // TODO: this should be handled in better way...
	}
};
// ------------------------------------------------------------------------


/**
 * @brief Empty data structure.
 */
typedef struct _empty
{
    //! Give access to boost::serialization framework
    friend class boost::serialization::access;

    //! Serialization of the data structure
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
    }
} empty_t;

/*
 // by Y
 inline void copy_frame(frame_tab destination_frame, frame_tab source_frame)
 {
 for (int   column = 0; column < 4; column++)
 for (int row = 0; row < 3; row++)
 destination_frame[column][row] = source_frame[column][row];
 }
 */

} // namespace lib
} // namespace mrrocpp

#endif
