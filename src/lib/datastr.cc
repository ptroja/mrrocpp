/**
 * \file datastr.cc
 *
 * \date Oct 21, 2009
 * \author ptrojane
 *
 */

#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include "lib/datastr.h"

namespace mrrocpp {
namespace lib {

std::string toString(const double valArr[], int length) {
	std::ostringstream stm;
	for (int i = 0; i < length; i++) {
		if (i == 0)
			stm << valArr[i];
		else
			stm << "\t" << valArr[i];
	}

	//	std::cout<<stm.str()<<std::endl;
	return stm.str();
}

std::string toString(int numberOfPoses) {
	return boost::lexical_cast<std::string>(numberOfPoses);
}

std::string toString(lib::robot_name_t robot) {
	using namespace lib;
	switch (robot) {
	case ROBOT_IRP6OT_M:
		return std::string("ROBOT_IRP6OT_M");
	case ROBOT_IRP6P_M:
		return std::string("ROBOT_IRP6P_M");
	case ROBOT_IRP6OT_TFG:
		return std::string("ROBOT_IRP6OT_TFG");
	case ROBOT_IRP6P_TFG:
		return std::string("ROBOT_IRP6P_TFG");
	case ROBOT_CONVEYOR:
		return std::string("ROBOT_CONVEYOR");
	case ROBOT_SPEAKER:
		return std::string("ROBOT_SPEAKER");
	case ROBOT_IRP6_MECHATRONIKA:
		return std::string("ROBOT_IRP6_MECHATRONIKA");
	case ROBOT_ELECTRON:
		return std::string("ROBOT_ELECTRON");
	case ROBOT_FESTIVAL:
		return std::string("ROBOT_FESTIVAL");
	case ROBOT_HAND:
		return std::string("ROBOT_HAND");
	case ROBOT_SPEECHRECOGNITION:
		return std::string("ROBOT_SPEECHRECOGNITION");
	default:
		return std::string("ROBOT_UNDEFINED");
	}
}

std::string toString(lib::ECP_POSE_SPECIFICATION ps) {
	switch (ps) {
	case lib::ECP_XYZ_ANGLE_AXIS:
		return std::string("ECP_XYZ_ANGLE_AXIS");
	case lib::ECP_XYZ_EULER_ZYZ:
		return std::string("ECP_XYZ_EULER_ZYZ");
	case lib::ECP_MOTOR:
		return std::string("MOTOR");
	case lib::ECP_JOINT:
		return std::string("JOINT");
	default:
		return std::string("INVALID_END_EFFECTOR");
	}
}

//-----------------------------------------------------------------------------------------------------------

lib::robot_name_t returnProperRobot(const std::string & robotName) {
	if (robotName == "ROBOT_IRP6OT_M")
		return lib::ROBOT_IRP6OT_M;
	else if (robotName == "ROBOT_IRP6P_M")
		return lib::ROBOT_IRP6P_M;
	else if (robotName == "ROBOT_IRP6OT_TFG")
		return lib::ROBOT_IRP6OT_TFG;
	else if (robotName == "ROBOT_IRP6P_TFG")
		return lib::ROBOT_IRP6P_TFG;
	else if (robotName == "ROBOT_CONVEYOR")
		return lib::ROBOT_CONVEYOR;
	else if (robotName == "ROBOT_SPEAKER")
		return lib::ROBOT_SPEAKER;
	else if (robotName == "ROBOT_IRP6_MECHATRONIKA")
		return lib::ROBOT_IRP6_MECHATRONIKA;
	else if (robotName == "ROBOT_ELECTRON")
		return lib::ROBOT_ELECTRON;
	else if (robotName == "ROBOT_FESTIVAL")
		return lib::ROBOT_FESTIVAL;
	else if (robotName == "ROBOT_HAND")
		return lib::ROBOT_HAND;
	else if (robotName == "ROBOT_SPEECHRECOGNITION")
		return lib::ROBOT_SPEECHRECOGNITION;
	else
		return lib::ROBOT_UNDEFINED;
}

lib::ECP_POSE_SPECIFICATION returnProperPS(
		const std::string & poseSpecification) {
	if (poseSpecification == "ECP_XYZ_ANGLE_AXIS") {
		return lib::ECP_XYZ_ANGLE_AXIS;
	}
	if (poseSpecification == "ECP_XYZ_EULER_ZYZ") {
		return lib::ECP_XYZ_EULER_ZYZ;
	}
	if (poseSpecification == "MOTOR") {
		return lib::ECP_MOTOR;
	}
	if (poseSpecification == "JOINT") {
		return lib::ECP_JOINT;
	} else
		return lib::ECP_INVALID_END_EFFECTOR;
}

int setValuesInArray(double arrayToFill[], const std::string & dataString) {
	int index = 0;
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> sep(" \t");
	tokenizer tok(dataString, sep);

	for (tokenizer::iterator beg = tok.begin(); beg != tok.end(); ++beg) {
		arrayToFill[index++] = boost::lexical_cast<double>(std::string(*beg));
	}

	return index;
}

}
}
