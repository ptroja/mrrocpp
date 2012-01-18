/*
 * ecp_ui_msg.h
 *
 *  Created on: Jan 1, 2012
 *      Author: ptroja
 */

#ifndef ECP_UI_MSG_H_
#define ECP_UI_MSG_H_

#include <boost/serialization/serialization.hpp>

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

//------------------------------------------------------------------------------
/*!
 *  Types of ECP to UI commands.
 */
enum ECP_TO_UI_REQUEST
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
	MAM_REFRESH_WINDOW,

	//! Swarm-related entries
	PLAN_STEP_MODE
};

//------------------------------------------------------------------------------
/*!
 *  Reply types from UI to ECP and commands from UI (pressing a button).
 */
enum UI_TO_ECP_REPLY
{
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
	MAM_CALIBRATE,

	//! Swarm-related entries
	PLAN_PREV,
	PLAN_NEXT,
	PLAN_EXEC,
	PLAN_SAVE
};

//------------------------------------------------------------------------------
/*! Length of a message sent from ECP to MP or UI */
#define MSG_LENGTH 60

//! Swarm plan item types
enum PLAN_ITEM_TYPE {
	MBASE_AND_BENCH, PKM_AND_HEAD
};

//------------------------------------------------------------------------------
/*!
 *  ECP to UI message.
 */
struct ECP_message
{
	/*! Type of message. */
	ECP_TO_UI_REQUEST ecp_message;

	/*! Robot name. */
	robot_name_t robot_name;

	/*! Number of options - from 2 to 4 - - for CHOOSE_OPTION mode. */
	uint8_t nr_of_options;

	//----------------------------------------------------------

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
		int32_t measure_number;
	}
	/*! Robot positions + Sensor readings + Measure number. */
	MAM;

	//! XML string with current plan item
	std::string plan_item_string;

	//! Type of plan item
	PLAN_ITEM_TYPE plan_item_type;

	//! Default constructor
	ECP_message() {
		// Initialize members to make serialization work
		string[0] = '\0';
	}

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & ecp_message;

		switch (ecp_message)
		{
			case PLAN_STEP_MODE:
				ar & plan_item_type;
				ar & plan_item_string;
				break;
			default:
				ar & robot_name;
				ar & nr_of_options;

				ar & string;

				ar & RS.robot_position;
				ar & RS.sensor_reading;

				ar & R2S.robot_position;
				ar & R2S.digital_scales_sensor_reading;
				ar & R2S.force_sensor_reading;

				ar & MAM.robot_position;
				ar & MAM.sensor_reading;
				ar & MAM.measure_number;
				break;
		}
	}
};

//------------------------------------------------------------------------------
/*!
 *  UI to ECP reply.
 */
struct UI_reply
{
	UI_TO_ECP_REPLY reply;
	int32_t integer_number;
	double double_number;
	double coordinates[lib::MAX_SERVOS_NR];
	char path[80];
	char filename[20];

	//! Type of plan item
	PLAN_ITEM_TYPE plan_item_type;
	
	//! XML string with current plan item.
	std::string plan_item_string;

	//! Default constructor
	UI_reply() {
		path[0] = '\0';
		filename[0] = '\0';
	}

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & reply;

		switch (reply)
		{
			case PLAN_EXEC:
				ar & plan_item_type;
				ar & plan_item_string;
				break;
			case PLAN_PREV:
			case PLAN_NEXT:
			case PLAN_SAVE:
				break;
			default:
				ar & integer_number;
				ar & double_number;
				ar & coordinates;
				ar & path;
				ar & filename;
				break;
		}
	}
};

} // namespace lib
} // namespace mrrocpp

#endif /* ECP_UI_MSG_H_ */
