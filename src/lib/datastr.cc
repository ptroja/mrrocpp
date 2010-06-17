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

std::string toString(const double valArr[], int length)
{
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

std::string toString(int numberOfPoses)
{
	return boost::lexical_cast <std::string>(numberOfPoses);
}

std::string toString(lib::robot_name_t robot)
{
	using namespace lib;

	return robot;
}

std::string toString(lib::ECP_POSE_SPECIFICATION ps)
{
	switch (ps)
	{
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

lib::robot_name_t returnProperRobot(const std::string & robotName)
{

	lib::robot_name_t returned_robot = robotName;
	return returned_robot;
}

lib::ECP_POSE_SPECIFICATION returnProperPS(const std::string & poseSpecification)
{
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

int setValuesInArray(double arrayToFill[], const std::string & dataString)
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
