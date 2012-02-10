/*
 * sr_edp.cc
 *
 *  Created on: Sep 9, 2010
 *      Author: ptroja
 */

#include <cstdio>
#include <cstring>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include "base/lib/sr/sr_edp.h"
#include "base/edp/edp_exceptions.h"

namespace mrrocpp {
namespace lib {

sr_edp::sr_edp(process_type_t process_type, const std::string & process_name, const std::string & sr_name) :
	sr(process_type, process_name, sr_name)
{
}

void sr_edp::interpret(char * description, error_class_t message_type, uint64_t error_code0, uint64_t error_code1)
{
	uint64_t s_error; // zmienna pomocnicza
	char tbuf[1 + 1]; // bufor tymczasowy na liczby

	description[0] = '\0';
	switch (message_type)
	{
		case SYSTEM_ERROR: // interpretacja do funkcji:
			// message(int16_t message_type, uint64_t error_code, char *text)

			break;
		case FATAL_ERROR:
		case NON_FATAL_ERROR: // interpretacja do funkcji:
			// message(int16_t message_type, uint64_t error_code0, uint64_t error_code1)
			s_error = error_code0;
			for (unsigned int j = 0; j < lib::MAX_SERVOS_NR; j++) {
				if (s_error & 0x00000001) {
					sprintf(tbuf, "%1d", j + 1);
					strcpy(description, "SERVO_");
					strcat(description, tbuf);
					strcat(description, "_LOWER_LIMIT_SWITCH ");
				}
				s_error >>= 1;
				if (s_error & 0x00000001) {
					sprintf(tbuf, "%1d", j + 1);
					strcat(description, "SERVO_");
					strcat(description, tbuf);
					strcat(description, "_UPPER_LIMIT_SWITCH ");
				}
				s_error >>= 1;
				if (s_error & 0x00000001) {
					sprintf(tbuf, "%1d", j + 1);
					strcat(description, "SERVO_");
					strcat(description, tbuf);
					strcat(description, "_OVER_CURRENT ");
				}
				s_error >>= 1;
			} // end: for

			s_error = error_code0;

			switch (s_error & 0x003C000000000000ULL)
			{
				case OK:
					break; // OK
				case SERVO_ERROR_IN_PASSIVE_LOOP:
					strcat(description, "SERVO_ERROR_IN_PASSIVE_LOOP ");
					break;
				case UNIDENTIFIED_SERVO_COMMAND:
					strcat(description, "UNIDENTIFIED_SERVO_COMMAND ");
					break;
				case SERVO_ERROR_IN_PHASE_1:
					strcat(description, "SERVO_ERROR_IN_PHASE_1 ");
					break;
				case SERVO_ERROR_IN_PHASE_2:
					strcat(description, "SERVO_ERROR_IN_PHASE_2 ");
					break;
				case SYNCHRO_SWITCH_EXPECTED:
					strcat(description, "SYNCHRO_SWITCH_EXPECTED ");
					break;
				case SYNCHRO_ERROR:
					strcat(description, "SYNCHRO_ERROR ");
					break;
				case SYNCHRO_DELAY_ERROR:
					strcat(description, "SYNCHRO_DELAY_ERROR ");
					break;
				default:
					strcat(description, "UNIDENTIFIED_SERVO_ERROR ");
					break;
			}

			switch (s_error & 0xFF00000000000000ULL)
			{
				case OK:
					break;// OK
				case INVALID_INSTRUCTION_TYPE:
					strcat(description, "INVALID_INSTRUCTION_TYPE");
					break;
				case INVALID_REPLY_TYPE:
					strcat(description, "INerror_messageVALID_REPLY_TYPE");
					break;
				case INVALID_SET_ROBOT_MODEL_TYPE:
					strcat(description, "INVALID_SET_ROBOT_MODEL_TYPE");
					break;
				case INVALID_GET_ROBOT_MODEL_TYPE:
					strcat(description, "INVALID_GET_ROBOT_MODEL_TYPE");
					break;
				case ERROR_IN_ROBOT_MODEL_REQUEST:
					strcat(description, "ERROR_IN_ROBOT_MODEL_REQUEST");
					break;
				case INVALID_HOMOGENEOUS_MATRIX:
					strcat(description, "INVALID_HOMOGENEOUS_MATRIX");
					break;
				case QUERY_EXPECTED:
					strcat(description, "QUERY_EXPECTED");
					break;
				case QUERY_NOT_EXPECTED:
					strcat(description, "QUERY_NOT_EXPECTED");
					break;
				case NO_VALID_END_EFFECTOR_POSE:
					strcat(description, "NO_VALID_END_EFFECTOR_POSE");
					break;
				case INVALID_MOTION_TYPE:
					strcat(description, "INVALID_MOTION_TYPE");
					break;
				case INVALID_MOTION_PARAMETERS:
					strcat(description, "INVALID_MOTION_PARAMETERS");
					break;
				case INVALID_SET_END_EFFECTOR_TYPE:
					strcat(description, "INVALID_SET_END_EFFECTOR_TYPE");
					break;
				case INVALID_GET_END_EFFECTOR_TYPE:
					strcat(description, "INVALID_GET_END_EFFECTOR_TYPE");
					break;
				case STRANGE_GET_ARM_REQUEST:
					strcat(description, "STRANGE_GET_ARM_REQUEST");
					break;
				case BEYOND_UPPER_LIMIT_AXIS_0:
					strcat(description, "BEYOND_UPPER_LIMIT_AXIS_0");
					break;
				case BEYOND_UPPER_LIMIT_AXIS_1:
					strcat(description, "BEYOND_UPPER_LIMIT_AXIS_1");
					break;
				case BEYOND_UPPER_LIMIT_AXIS_2:
					strcat(description, "BEYOND_UPPER_LIMIT_AXIS_2");
					break;
				case BEYOND_UPPER_LIMIT_AXIS_3:
					strcat(description, "BEYOND_UPPER_LIMIT_AXIS_3");
					break;
				case BEYOND_UPPER_LIMIT_AXIS_4:
					strcat(description, "BEYOND_UPPER_LIMIT_AXIS_4");
					break;
				case BEYOND_UPPER_LIMIT_AXIS_5:
					strcat(description, "BEYOND_UPPER_LIMIT_AXIS_5");
					break;
				case BEYOND_UPPER_LIMIT_AXIS_6:
					strcat(description, "BEYOND_UPPER_LIMIT_AXIS_6");
					break;
				case BEYOND_UPPER_LIMIT_AXIS_7:
					strcat(description, "BEYOND_UPPER_LIMIT_AXIS_7");
					break;
				case BEYOND_LOWER_LIMIT_AXIS_0:
					strcat(description, "BEYOND_LOWER_LIMIT_AXIS_0");
					break;
				case BEYOND_LOWER_LIMIT_AXIS_1:
					strcat(description, "BEYOND_LOWER_LIMIT_AXIS_1");
					break;
				case BEYOND_LOWER_LIMIT_AXIS_2:
					strcat(description, "BEYOND_LOWER_LIMIT_AXIS_2");
					break;
				case BEYOND_LOWER_LIMIT_AXIS_3:
					strcat(description, "BEYOND_LOWER_LIMIT_AXIS_3");
					break;
				case BEYOND_LOWER_LIMIT_AXIS_4:
					strcat(description, "BEYOND_LOWER_LIMIT_AXIS_4");
					break;
				case BEYOND_LOWER_LIMIT_AXIS_5:
					strcat(description, "BEYOND_LOWER_LIMIT_AXIS_5");
					break;
				case BEYOND_LOWER_LIMIT_AXIS_6:
					strcat(description, "BEYOND_LOWER_LIMIT_AXIS_6");
					break;
				case BEYOND_LOWER_LIMIT_AXIS_7:
					strcat(description, "BEYOND_LOWER_LIMIT_AXIS_7");
					break;
				case BEYOND_UPPER_D0_LIMIT:
					strcat(description, "BEYOND_UPPER_Q1_LIMIT");
					break;
				case BEYOND_UPPER_THETA1_LIMIT:
					strcat(description, "BEYOND_UPPER_LEFT_LIMIT");
					break;
				case BEYOND_UPPER_THETA2_LIMIT:
					strcat(description, "BEYOND_UPPER_RIGHT_LIMIT");
					break;
				case BEYOND_UPPER_THETA3_LIMIT:
					strcat(description, "BEYOND_Q4_LIMIT_RANGE (BEYOND_UPPER_THETA3_LIMIT)");
					break;
				case BEYOND_UPPER_THETA4_LIMIT:
					strcat(description, "BEYOND_UPPER_Q5_LIMIT");
					break;
				case BEYOND_UPPER_THETA5_LIMIT:
					strcat(description, "BEYOND_UPPER_Q6_LIMIT");
					break;
				case BEYOND_UPPER_THETA6_LIMIT:
					strcat(description, "BEYOND_UPPER_Q7_LIMIT");
					break;
				case BEYOND_UPPER_THETA7_LIMIT:
					strcat(description, "BEYOND_UPPER_Q8_LIMIT");
					break;
				case BEYOND_LOWER_D0_LIMIT:
					strcat(description, "BEYOND_LOWER_Q1_LIMIT");
					break;
				case BEYOND_LOWER_THETA1_LIMIT:
					strcat(description, "BEYOND_LOWER_LEFT_LIMIT");
					break;
				case BEYOND_LOWER_THETA2_LIMIT:
					strcat(description, "BEYOND_LOWER_RIGHT_LIMIT");
					break;
				case BEYOND_LOWER_THETA3_LIMIT:
					strcat(description, "BEYOND_Q4_LIMIT_RANGE (BEYOND_LOWER_THETA3_LIMIT)");
					break;
				case BEYOND_LOWER_THETA4_LIMIT:
					strcat(description, "BEYOND_LOWER_Q5_LIMIT");
					break;
				case BEYOND_LOWER_THETA5_LIMIT:
					strcat(description, "BEYOND_LOWER_Q6_LIMIT");
					break;
				case BEYOND_LOWER_THETA6_LIMIT:
					strcat(description, "BEYOND_LOWER_Q7_LIMIT");
					break;
				case BEYOND_LOWER_THETA7_LIMIT:
					strcat(description, "BEYOND_LOWER_Q8_LIMIT");
					break;
				case OUT_OF_WORKSPACE:
					strcat(description, "OUT_OF_WORKSPACE");
					break;
				case SINGULAR_POSE:
					strcat(description, "SINGULAR_POSE");
					break;
				case ACOS_DOMAIN_ERROR:
					strcat(description, "ACOS_DOMAIN_ERROR");
					break;
				case ASIN_DOMAIN_ERROR:
					strcat(description, "ASIN_DOMAIN_ERROR");
					break;
				case ATAN2_DOMAIN_ERROR:
					strcat(description, "ATAN2_DOMAIN_ERROR");
					break;
				case SQRT_DOMAIN_ERROR:
					strcat(description, "SQRT_DOMAIN_ERROR");
					break;
				case UNKNOWN_MATH_ERROR:
					strcat(description, "UNKNOWN_MATH_ERROR");
					break;
				case UNKNOWN_INSTRUCTION:
					strcat(description, "UNKNOWN_INSTRUCTION");
					break;
				case NOT_IMPLEMENTED_YET:
					strcat(description, "NOT_IMPLEMENTED_YET");
					break;
				case NOT_YET_SYNCHRONISED:
					strcat(description, "NOT_YET_SYNCHRONISED");
					break;
				case ALREADY_SYNCHRONISED:
					strcat(description, "ALREADY_SYNCHRONISED");
					break;
				case UNKNOWN_SYNCHRO_ERROR:
					strcat(description, "UNKNOWN_SYNCHRO_ERROR");
					break;
				case INVALID_KINEMATIC_MODEL_NO:
					strcat(description, "INVALID_KINEMATIC_MODEL_NO");
					break;
				case EDP_UNIDENTIFIED_ERROR:
					strcat(description, "EDP_UNIDENTIFIED_ERROR");
					break;
				case NOT_A_NUMBER_JOINT_VALUE_D0:
					strcat(description, "NOT_A_NUMBER_JOINT_VALUE_D0");
					break;
				case NOT_A_NUMBER_JOINT_VALUE_THETA1:
					strcat(description, "NOT_A_NUMBER_JOINT_VALUE_THETA1");
					break;
				case NOT_A_NUMBER_JOINT_VALUE_THETA2:
					strcat(description, "NOT_A_NUMBER_JOINT_VALUE_THETA2");
					break;
				case NOT_A_NUMBER_JOINT_VALUE_THETA3:
					strcat(description, "NOT_A_NUMBER_JOINT_VALUE_THETA3");
					break;
				case NOT_A_NUMBER_JOINT_VALUE_THETA4:
					strcat(description, "NOT_A_NUMBER_JOINT_VALUE_THETA4");
					break;
				case NOT_A_NUMBER_JOINT_VALUE_THETA5:
					strcat(description, "NOT_A_NUMBER_JOINT_VALUE_THETA5");
					break;
				case NOT_A_NUMBER_JOINT_VALUE_THETA6:
					strcat(description, "NOT_A_NUMBER_JOINT_VALUE_THETA6");
					break;
				case NOT_A_NUMBER_JOINT_VALUE_THETA7:
					strcat(description, "NOT_A_NUMBER_JOINT_VALUE_THETA7");
					break;
				default:
					strcat(description, "UNIDENTIFIED_ERROR");
					break;
			}

			// analiza informacji zawartej w error_code1
			s_error = error_code1;
			for (unsigned int j = 0; j < lib::MAX_SERVOS_NR; j++) {
				if (s_error & 0x00000001) {
					sprintf(tbuf, "%1d", j + 1);
					strcat(description, "SERVO ");
					strcat(description, tbuf);
					strcat(description, " : UNIDENTIFIED_ALGORITHM_NO");
				}
				s_error >>= 1;
				if (s_error & 0x00000001) {
					sprintf(tbuf, "%1d", j + 1);
					strcat(description, "SERVO ");
					strcat(description, tbuf);
					strcat(description, " : UNIDENTIFIED_ALGORITHM_PARAMETERS_NO");
				}
				s_error >>= 1;
			}
			break;
		default:
			strcat(description, "EDP UNIDENTIFIED ERROR");
	}
}

} // namespace lib
} // namespace mrrocpp
