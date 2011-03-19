/**
 * \file datastr.cc
 *
 * \author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * \brief Methods for MRROC++ data types/string conversion.
 */

#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include "base/lib/datastr.h"

namespace mrrocpp {
namespace lib {

std::string toString(const double valArr[], int length)
{
	std::ostringstream stm;
	for (int i = 0; i < length; i++) {
		if (i == 0)
			stm << valArr[i];
		else
			stm << "\t" << valArr[i];
	}

	return stm.str();
}

std::string toString(int numberOfPoses)
{
	return boost::lexical_cast <std::string>(numberOfPoses);
}

std::string toString(const lib::robot_name_t & robot)
{
	return robot;
}

std::string toString(lib::ECP_POSE_SPECIFICATION ps)
{
	switch (ps)
	{
		case lib::ECP_XYZ_ANGLE_AXIS:
			return std::string("ecp_XYZ_ANGLE_AXIS");
		case lib::ECP_XYZ_EULER_ZYZ:
			return std::string("ecp_XYZ_EULER_ZYZ");
		case lib::ECP_MOTOR:
			return std::string("MOTOR");
		case lib::ECP_JOINT:
			return std::string("JOINT");
		default:
			return std::string("INVALID_END_EFFECTOR");
	}
}

lib::robot_name_t returnProperRobot(const std::string & robotName)
{
	const lib::robot_name_t returned_robot(robotName);
	return returned_robot;
}

lib::ECP_POSE_SPECIFICATION returnProperPS(const std::string & poseSpecification)
{
	if (poseSpecification == "ecp_XYZ_ANGLE_AXIS") {
		return lib::ECP_XYZ_ANGLE_AXIS;
	} else if (poseSpecification == "ecp_XYZ_EULER_ZYZ") {
		return lib::ECP_XYZ_EULER_ZYZ;
	} else if (poseSpecification == "MOTOR") {
		return lib::ECP_MOTOR;
	} else if (poseSpecification == "JOINT") {
		return lib::ECP_JOINT;
	} else
		return lib::ECP_INVALID_END_EFFECTOR;
}

unsigned int setValuesInArray(double arrayToFill[], const std::string & dataString)
{
	int index = 0;
	typedef boost::tokenizer <boost::char_separator <char> > tokenizer;
	boost::char_separator <char> sep(" \t");
	tokenizer tok(dataString, sep);

	for (tokenizer::iterator beg = tok.begin(); beg != tok.end(); ++beg) {
		arrayToFill[index++] = boost::lexical_cast <double>(std::string(*beg));
	}

	return index;
}

}
}
