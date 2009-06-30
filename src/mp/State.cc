#include <mp/State.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

namespace mrrocpp {
namespace mp {
namespace common {


State::State()
{
	numArgument = 0;
	id = NULL;
	type =  NULL;
	stringArgument =  NULL;
	robotSet = NULL;
	stateTransitions = new std::list<Transition>();
}
//-----------------------------------------------------------------------------------------------------------
State::State(const State &state)
{
	int size;
	id = NULL;
	type =  NULL;
	stringArgument =  NULL;
	robotSet = NULL;
	this->numArgument = state.numArgument;
	if(state.id != NULL)
	{
		size = strlen(state.id) + 1;
		this->id = new char[size];
		strcpy(this->id, state.id);
	}
	if(state.type != NULL)
	{
		size = strlen(state.type) + 1;
		this->type =  new char[size];
		strcpy(this->type, state.type);
	}
	if(state.stringArgument != NULL)
	{
		size = strlen(state.stringArgument) + 1;
		this->stringArgument =  new char[size];
		strcpy(this->stringArgument, state.stringArgument);
	}
	robot = state.robot;
	generatorType = state.generatorType;
	if(state.robotSet != NULL )
		this->robotSet = new RobotSets(*(state.robotSet));
	this->stateTransitions = new std::list<Transition>(*(state.stateTransitions));
}

//-----------------------------------------------------------------------------------------------------------

State::~State()
{
	if(id)
		delete[] id;
	if(type)
		delete[] type;
	if(stringArgument)
		delete[] stringArgument;
	delete stateTransitions;
	if(robotSet)
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
	this->firstSet = new lib::ROBOT_ENUM[firstSetCount];
	for(int i = 0; i<firstSetCount; i++)
		this->firstSet[i] = robotSets.firstSet[i];
	this->secondSet = new lib::ROBOT_ENUM[secondSetCount];
	for(int i = 0; i<secondSetCount; i++)
		this->secondSet[i] = robotSets.secondSet[i];
}
//-----------------------------------------------------------------------------------------------------------
State::RobotSets::~RobotSets()
{
	if(firstSet)
		delete[] firstSet;
	if(secondSet)
		delete[] secondSet;
}
//-----------------------------------------------------------------------------------------------------------

void State::setStateID(const char *stateID)
{
	int size = strlen(stateID) + 1;
	id = new char[size];
	strcpy(this->id, stateID);
}

const char* State::getStateID() const
{
	return id;
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

void State::setType(const char *type)
{
	int size = strlen(type) + 1;
	this->type =  new char[size];
	strcpy(this->type, type);
}

//-----------------------------------------------------------------------------------------------------------

const char * State::getType() const
{
	return type;
}

//-----------------------------------------------------------------------------------------------------------

lib::ROBOT_ENUM State::returnProperRobot(char * robotName)
{
	if(strcmp(robotName, "ROBOT_IRP6_ON_TRACK") == 0)
		return lib::ROBOT_IRP6_ON_TRACK;
	else if(strcmp(robotName, "ROBOT_IRP6_POSTUMENT") == 0)
		return lib::ROBOT_IRP6_POSTUMENT;
	else if(strcmp(robotName, "ROBOT_CONVEYOR") == 0)
		return lib::ROBOT_CONVEYOR;
	else if(strcmp(robotName, "ROBOT_SPEAKER") == 0)
		return lib::ROBOT_SPEAKER;
	else if(strcmp(robotName, "ROBOT_IRP6_MECHATRONIKA") == 0)
		return lib::ROBOT_IRP6_MECHATRONIKA;
	else if(strcmp(robotName, "ROBOT_ELECTRON") == 0)
		return lib::ROBOT_ELECTRON;
	else if(strcmp(robotName, "ROBOT_FESTIVAL") == 0)
		return lib::ROBOT_FESTIVAL;
	else if(strcmp(robotName, "ROBOT_HAND") == 0)
		return lib::ROBOT_HAND;
	else if(strcmp(robotName, "ROBOT_SPEECHRECOGNITION") == 0)
		return lib::ROBOT_SPEECHRECOGNITION;
	else
		return lib::ROBOT_UNDEFINED;
}

//-----------------------------------------------------------------------------------------------------------

void State::setRobot(char *robot)
{
	this->robot = returnProperRobot(robot);
	//std::cout<<">>>>>>>>"<<robot<<"##"<<std::endl;
	//std::cout<<strcmp(robot, "ROBOT_IRP6_ON_TRACK")<<std::endl;
	//strcpy(this->robot, robot);
/*	if(strcmp(robot, "ROBOT_IRP6_ON_TRACK") == 0)
		this->robot = lib::ROBOT_IRP6_ON_TRACK;
	else if(strcmp(robot, "ROBOT_IRP6_POSTUMENT") == 0)
		this->robot = lib::ROBOT_IRP6_POSTUMENT;
	else if(strcmp(robot, "ROBOT_CONVEYOR") == 0)
		this->robot = lib::ROBOT_CONVEYOR;
	else if(strcmp(robot, "ROBOT_SPEAKER") == 0)
		this->robot = lib::ROBOT_SPEAKER;
	else if(strcmp(robot, "ROBOT_IRP6_MECHATRONIKA") == 0)
		this->robot = lib::ROBOT_IRP6_MECHATRONIKA;
	else if(strcmp(robot, "ROBOT_ELECTRON") == 0)
		this->robot = lib::ROBOT_ELECTRON;
	else if(strcmp(robot, "ROBOT_FESTIVAL") == 0)
		this->robot = lib::ROBOT_FESTIVAL;
	else if(strcmp(robot, "ROBOT_HAND") == 0)
		this->robot = lib::ROBOT_HAND;
	else if(strcmp(robot, "ROBOT_SPEECHRECOGNITION") == 0)
		this->robot = lib::ROBOT_SPEECHRECOGNITION;
	else
		this->robot = lib::ROBOT_UNDEFINED;
*/
}

//-----------------------------------------------------------------------------------------------------------

lib::ROBOT_ENUM State::getRobot() const
{
	return robot;
}
//-----------------------------------------------------------------------------------------------------------
void State::setGeneratorType(char *genType)
{
	//std::cout<<"######"<<genType<<std::endl;
	//std::cout<<strcmp(genType, (const char *)"ECP_GEN_TRANSPARENT")<<std::endl;
	//strcpy(this->generatorType, genType);
	if(strcmp(genType, "ECP_GEN_TRANSPARENT") == 0)
		this->generatorType = ecp_mp::task::ECP_GEN_TRANSPARENT;
	else if(strcmp(genType, "ECP_GEN_TFF_NOSE_RUN") == 0)
		this->generatorType = ecp_mp::task::ECP_GEN_TFF_NOSE_RUN;
	else if(strcmp(genType, "ECP_GEN_TEACH_IN") == 0)
		this->generatorType = ecp_mp::task::ECP_GEN_TEACH_IN;
	else if(strcmp(genType, "ECP_GEN_SMOOTH") == 0)
		this->generatorType = ecp_mp::task::ECP_GEN_SMOOTH;
	else if(strcmp(genType, "ECP_GEN_TFF_RUBIK_GRAB") == 0)
		this->generatorType = ecp_mp::task::ECP_GEN_TFF_RUBIK_GRAB;
	else if(strcmp(genType, "ECP_GEN_TFF_RUBIK_FACE_ROTATE") == 0)
		this->generatorType = ecp_mp::task::ECP_GEN_TFF_RUBIK_FACE_ROTATE;
	else if(strcmp(genType, "ECP_GEN_TFF_GRIPPER_APPROACH") == 0)
		this->generatorType = ecp_mp::task::ECP_GEN_TFF_GRIPPER_APPROACH;
	else if(strcmp(genType, "RCSC_GRIPPER_OPENING") == 0)
		this->generatorType = ecp_mp::task::RCSC_GRIPPER_OPENING;
	else if(strcmp(genType, "ECP_GEN_BIAS_EDP_FORCE") == 0)
		this->generatorType = ecp_mp::task::ECP_GEN_BIAS_EDP_FORCE;
	else if(strcmp(genType, "ECP_WEIGHT_MEASURE_GENERATOR") == 0)
		this->generatorType = ecp_mp::task::ECP_WEIGHT_MEASURE_GENERATOR;
	else if(strcmp(genType, "ECP_TOOL_CHANGE_GENERATOR") == 0)
		this->generatorType = ecp_mp::task::ECP_TOOL_CHANGE_GENERATOR;
	else
		this->generatorType = ecp_mp::task::ECP_GEN_SPEAK;
}

//----------------------------------------------------------------------------------------------------------

ecp_mp::task::STATE_MACHINE_ECP_STATES State::getGeneratorType() const
{
	return generatorType;
}

//----------------------------------------------------------------------------------------------------------

void State::setStringArgument(char* trajFilePath)
{
	int size = strlen(trajFilePath) + 1;
	stringArgument =  new char[size];
	strcpy(this->stringArgument, trajFilePath);
}

//----------------------------------------------------------------------------------------------------------

char* State::getStringArgument() const
{
	return stringArgument;
}

//----------------------------------------------------------------------------------------------------------

void State::setTransition(char *cond, char *target, lib::configurator &_config)
{
	Transition *tempTr = new Transition(cond, target, _config);
	stateTransitions->push_back(*tempTr);
}

//----------------------------------------------------------------------------------------------------------

void State::setProperTransitionResult(bool result)
{
	for(std::list<Transition>::iterator it = stateTransitions->begin(); it != stateTransitions->end(); ++it)
	{
		if(!strcmp((*it).getConditionDescription(), "stateOperationResult"))
			(*it).setConditionResult(result);
	}
}

//----------------------------------------------------------------------------------------------------------

std::list<Transition> * State::getTransitions() const
{
	return stateTransitions;
}

//----------------------------------------------------------------------------------------------------------
const char * State::returnNextStateID(StateHeap &sh)
{
	for(std::list<Transition>::iterator it = stateTransitions->begin(); it != stateTransitions->end(); ++it)
	{
		if((*it).getConditionResult())
			return (*it).getTargetID(sh);
	}
	// to avoid lock
	return "_STOP_";
}
//----------------------------------------------------------------------------------------------------------

void State::showStateContent() const
{
	std::cout<<id<<std::endl<<type<<std::endl<<robot<<std::endl<<generatorType<<std::endl;//<<stringArgument<<std::endl;
	if(robotSet != NULL)
	{
		std::cout<<"\nFirst set count: "<<robotSet->firstSetCount<<" = ";
		for(int i=0;i<robotSet->firstSetCount;i++)
			std::cout<<robotSet->firstSet[i]<<"; ";
		std::cout<<"\nSecond set count: "<<robotSet->secondSetCount<<" = ";
		for(int i=0;i<robotSet->secondSetCount;i++)
			std::cout<<robotSet->secondSet[i]<<"; ";
		std::cout<<std::endl;
	}
	std::cout<<"Transitions count: "<<stateTransitions->size()<<std::endl;
	for(std::list<Transition>::iterator it = stateTransitions->begin(); it != stateTransitions->end(); ++it)
	{
		std::cout<<"----- Transition ------"<<std::endl;
		(*it).showContent();
	}
}
} // namespace common
} // namespace mp
} // namespace mrrocpp


