#include <iostream>
#include <cstring>
#include <cstdio>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include "State.h"
#include "base/lib/datastr.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"

#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/force_tool_change/ecp_mp_g_force_tool_change.h"
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/tff_nose_run/ecp_mp_g_tff_nose_run.h"
#include "generator/ecp/tff_rubik_face_rotate/ecp_mp_g_tff_rubik_face_rotate.h"

#include "generator/ecp/transparent/ecp_mp_g_transparent.h"
#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"
#include "generator/ecp/teach_in/ecp_mp_g_teach_in.h"
#include "generator/ecp/weight_measure/ecp_mp_g_weight_measure.h"

namespace mrrocpp {
namespace mp {
namespace common {

State::State() :
		numArgument(0)
{
}

void State::setStateID(const std::string & stateID)
{
	id = stateID;
}

const char* State::getStateID() const
{
	return id.c_str();
}

//-----------------------------------------------------------------------------------------------------------

void State::setNumArgument(const std::string & numArgument)
{
	this->numArgument = boost::lexical_cast <int>(numArgument);
}

//-----------------------------------------------------------------------------------------------------------

int State::getNumArgument() const
{
	return numArgument;
}

//-----------------------------------------------------------------------------------------------------------

void State::setType(const std::string & _type)
{
	type = _type;
}

//-----------------------------------------------------------------------------------------------------------

const std::string & State::getType() const
{
	return type;
}

//-----------------------------------------------------------------------------------------------------------

void State::setRobot(const std::string & _robot)
{
	this->robot = lib::returnProperRobot(_robot);
}

//-----------------------------------------------------------------------------------------------------------

lib::robot_name_t State::getRobot() const
{
	return robot;
}
//-----------------------------------------------------------------------------------------------------------
void State::setGeneratorType(const std::string & genType)
{
	//std::cout<<"######"<<genType<<std::endl;
	//std::cout<<strcmp(genType, (const char *)"ECP_GEN_TRANSPARENT")<<std::endl;
	//strcpy(this->generatorType, genType);
	if (genType == "ECP_GEN_TRANSPARENT")
		this->generatorType = ecp_mp::generator::ECP_GEN_TRANSPARENT;
	else if (genType == "ECP_GEN_TFF_NOSE_RUN")
		this->generatorType = ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN;
	else if (genType == "ECP_GEN_TEACH_IN")
		this->generatorType = ecp_mp::generator::ECP_GEN_TEACH_IN;
	else if (genType == "ECP_GEN_NEWSMOOTH")
		this->generatorType = ecp_mp::generator::ECP_GEN_NEWSMOOTH;
	else if (genType == "ECP_GEN_TFF_RUBIK_FACE_ROTATE")
		this->generatorType = ecp_mp::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE;
	else if (genType == "ECP_GEN_TFF_GRIPPER_APPROACH")
		this->generatorType = ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH;
	else if (genType == "ECP_GEN_BIAS_EDP_FORCE")
		this->generatorType = ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE;
	else if (genType == "ECP_ST_BIAS_EDP_FORCE")
		this->generatorType = ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE;
	else if (genType == "ECP_GEN_WEIGHT_MEASURE")
		this->generatorType = ecp_mp::generator::ECP_GEN_WEIGHT_MEASURE;
	else if (genType == "ECP_TOOL_CHANGE_GENERATOR")
		this->generatorType = ecp_mp::generator::ECP_GEN_FORCE_TOOL_CHANGE;
	// TODO: unknown generatorType handler should throw an exception
}

//----------------------------------------------------------------------------------------------------------

std::string State::getGeneratorType() const
{
	return generatorType;
}

//----------------------------------------------------------------------------------------------------------

void State::setStringArgument(const std::string & trajFilePath)
{
	stringArgument = trajFilePath;
}

//----------------------------------------------------------------------------------------------------------

const std::string & State::getStringArgument() const
{
	return stringArgument;
}

//----------------------------------------------------------------------------------------------------------

void State::setTransition(const std::string & cond, const std::string & target, lib::configurator &_config)
{
	Transition *tempTr = new Transition(cond, target, _config);
	stateTransitions.push_back(*tempTr);
}

//----------------------------------------------------------------------------------------------------------

void State::setProperTransitionResult(bool result)
{
	for (std::list <Transition>::iterator it = stateTransitions.begin(); it != stateTransitions.end(); ++it) {
		if (((*it).getConditionDescription()) == "stateOperationResult")
			(*it).setConditionResult(result);
	}
}

//----------------------------------------------------------------------------------------------------------

const std::list <Transition> & State::getTransitions() const
{
	return stateTransitions;
}

//----------------------------------------------------------------------------------------------------------
const char * State::returnNextStateID(StateHeap &sh)
{
	for (std::list <Transition>::iterator it = stateTransitions.begin(); it != stateTransitions.end(); ++it) {
		if ((*it).getConditionResult()) {
			const char * str = (*it).getTargetID(sh);
			return str;
		}
	}
	// to avoid lock
	return "_STOP_";
}
//----------------------------------------------------------------------------------------------------------

void State::showStateContent() const
{
	std::cout << id << std::endl << type << std::endl << robot << std::endl << generatorType << std::endl; //<<stringArgument<<std::endl;
	if (robotSet.is_initialized()) {
		std::cout << "\nFirst set count: " << robotSet->firstSet.size() << " = ";
		BOOST_FOREACH(const lib::robot_name_t & name, robotSet->firstSet)
				{
					std::cout << name << "; ";
				}
		std::cout << std::endl;

		std::cout << "\nSecond set count: " << robotSet->secondSet.size() << " = ";
		BOOST_FOREACH(const lib::robot_name_t & name, robotSet->secondSet)
				{
					std::cout << name << "; ";
				}
		std::cout << std::endl;
	}
	std::cout << "Transitions count: " << stateTransitions.size() << std::endl;
	for (std::list <Transition>::const_iterator it = stateTransitions.begin(); it != stateTransitions.end(); ++it) {
		std::cout << "----- Transition ------" << std::endl;
		(*it).showContent();
	}
}
} // namespace common
} // namespace mp
} // namespace mrrocpp

