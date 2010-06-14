#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "lib/datastr.h"
#include "State.h"
#include "ecp_mp/task/ecp_mp_st_bias_edp_force.h"
#include "ecp_mp/task/ecp_mp_st_tff_nose_run.h"
#include "ecp_mp/common/generator/ecp_mp_g_bias_edp_force.h"
#include "ecp_mp/common/generator/ecp_mp_g_transparent.h"
#include "ecp_mp/common/generator/ecp_mp_g_smooth.h"
#include "ecp_mp/common/generator/ecp_mp_g_teach_in.h"
#include "ecp_mp/common/generator/ecp_mp_g_force.h"
#include "ecp_mp/speaker/generator/ecp_mp_g_speak.h"
#include "ecp_mp/task/ecp_mp_st_gripper_opening.h"

namespace mrrocpp {
namespace mp {
namespace common {

State::State()
{
	numArgument = 0;
	robotSet = NULL;
	stateTransitions = new std::list <Transition>();
}
//-----------------------------------------------------------------------------------------------------------
State::State(const State &state)
{
	this->numArgument = state.numArgument;
	this->id = state.id;
	this->type = state.type;
	this->stringArgument = state.stringArgument;
	robot = state.robot;
	generatorType = state.generatorType;
	if (state.robotSet)
		this->robotSet = new RobotSets(*(state.robotSet));
	else
		robotSet = NULL;
	this->stateTransitions = new std::list <Transition>(*(state.stateTransitions));
}

//-----------------------------------------------------------------------------------------------------------

State::~State()
{
	if (stateTransitions)
		delete stateTransitions;
	if (robotSet)
		delete robotSet;
}

//-----------------------------------------------------------------------------------------------------------
State::RobotSets::RobotSets()
{
	firstSetCount = 0;
	secondSetCount = 0;
	firstSet = NULL;
	secondSet = NULL;
}
//-----------------------------------------------------------------------------------------------------------
State::RobotSets::RobotSets(const RobotSets &robotSets)
{
	this->firstSetCount = robotSets.firstSetCount;
	this->secondSetCount = robotSets.secondSetCount;
	this->firstSet = new lib::robot_name_t[firstSetCount];
	for (int i = 0; i < firstSetCount; i++)
		this->firstSet[i] = robotSets.firstSet[i];
	this->secondSet = new lib::robot_name_t[secondSetCount];
	for (int i = 0; i < secondSetCount; i++)
		this->secondSet[i] = robotSets.secondSet[i];
}
//-----------------------------------------------------------------------------------------------------------
State::RobotSets::~RobotSets()
{
	if (firstSet)
		delete[] firstSet;
	if (secondSet)
		delete[] secondSet;
}
//-----------------------------------------------------------------------------------------------------------

void State::setStateID(const std::string & stateID)
{
	id = stateID;
}

const char* State::getStateID() const
{
	return id.c_str();
}

//-----------------------------------------------------------------------------------------------------------

void State::setNumArgument(const char *numArgument)
{
	this->numArgument = atoi(numArgument);
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

const char * State::getType() const
{
	return type.c_str();
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
		this->generatorType = ecp_mp::common::generator::ECP_GEN_TRANSPARENT;
	else if (genType == "ECP_GEN_TFF_NOSE_RUN")
		this->generatorType = ecp_mp::common::generator::ECP_GEN_TFF_NOSE_RUN;
	else if (genType == "ECP_ST_TFF_NOSE_RUN")
		this->generatorType = ecp_mp::task::ECP_ST_TFF_NOSE_RUN;
	else if (genType == "ECP_GEN_TEACH_IN")
		this->generatorType = ecp_mp::common::generator::ECP_GEN_TEACH_IN;
	else if (genType == "ECP_GEN_SMOOTH")
		this->generatorType = ecp_mp::common::generator::ECP_GEN_SMOOTH;
	else if (genType == "ECP_GEN_TFF_RUBIK_GRAB")
		this->generatorType = ecp_mp::common::generator::ECP_GEN_TFF_RUBIK_GRAB;
	else if (genType == "ECP_GEN_TFF_RUBIK_FACE_ROTATE")
		this->generatorType = ecp_mp::common::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE;
	else if (genType == "ECP_GEN_TFF_GRIPPER_APPROACH")
		this->generatorType = ecp_mp::common::generator::ECP_GEN_TFF_GRIPPER_APPROACH;
	else if (genType == "ECP_ST_GRIPPER_OPENING")
		this->generatorType = ecp_mp::task::ECP_ST_GRIPPER_OPENING;
	else if (genType == "ECP_GEN_BIAS_EDP_FORCE")
		this->generatorType = ecp_mp::common::generator::ECP_GEN_BIAS_EDP_FORCE;
	else if (genType == "ECP_ST_BIAS_EDP_FORCE")
		this->generatorType = ecp_mp::task::ECP_ST_BIAS_EDP_FORCE;
	else if (genType == "ECP_WEIGHT_MEASURE_GENERATOR")
		this->generatorType = ecp_mp::common::generator::ECP_WEIGHT_MEASURE_GENERATOR;
	else if (genType == "ECP_TOOL_CHANGE_GENERATOR")
		this->generatorType = ecp_mp::common::generator::ECP_TOOL_CHANGE_GENERATOR;
	else
		this->generatorType = ecp_mp::speaker::generator::ECP_GEN_SPEAK;
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

const char* State::getStringArgument() const
{
	return stringArgument.c_str();
}

//----------------------------------------------------------------------------------------------------------

void State::setTransition(const char *cond, const char *target, lib::configurator &_config)
{
	Transition *tempTr = new Transition(cond, target, _config);
	stateTransitions->push_back(*tempTr);
}

//----------------------------------------------------------------------------------------------------------

void State::setProperTransitionResult(bool result)
{
	for (std::list <Transition>::iterator it = stateTransitions->begin(); it != stateTransitions->end(); ++it) {
		if (((*it).getConditionDescription()) == "stateOperationResult")
			(*it).setConditionResult(result);
	}
}

//----------------------------------------------------------------------------------------------------------

std::list <Transition> * State::getTransitions() const
{
	return stateTransitions;
}

//----------------------------------------------------------------------------------------------------------
const char * State::returnNextStateID(StateHeap &sh)
{
	for (std::list <Transition>::iterator it = stateTransitions->begin(); it != stateTransitions->end(); ++it) {
		if ((*it).getConditionResult())
			return (*it).getTargetID(sh);
	}
	// to avoid lock
	return "_STOP_";
}
//----------------------------------------------------------------------------------------------------------

void State::showStateContent() const
{
	std::cout << id << std::endl << type << std::endl << robot << std::endl << generatorType << std::endl;//<<stringArgument<<std::endl;
	if (robotSet != NULL) {
		std::cout << "\nFirst set count: " << robotSet->firstSetCount << " = ";
		for (int i = 0; i < robotSet->firstSetCount; i++)
			std::cout << robotSet->firstSet[i] << "; ";
		std::cout << "\nSecond set count: " << robotSet->secondSetCount << " = ";
		for (int i = 0; i < robotSet->secondSetCount; i++)
			std::cout << robotSet->secondSet[i] << "; ";
		std::cout << std::endl;
	}
	std::cout << "Transitions count: " << stateTransitions->size() << std::endl;
	for (std::list <Transition>::iterator it = stateTransitions->begin(); it != stateTransitions->end(); ++it) {
		std::cout << "----- Transition ------" << std::endl;
		(*it).showContent();
	}
}
} // namespace common
} // namespace mp
} // namespace mrrocpp


