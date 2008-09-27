

#include <mp/State.h>
#include <iostream>
#include <string.h>

State::State()
{
	name = new char[40];
	type =  new char[40];
	//robot =  new char[40];
	//generatorType =  new char[40];
	trajectoryFilePath =  new char[40];
}
//-----------------------------------------------------------------------------------------------------------
State::State(const State &state)
{
	name = new char[40];
	type =  new char[40];
	//robot =  new char[40];
	//generatorType =  new char[40];
	trajectoryFilePath =  new char[40];
	strcpy(this->name, state.name);
	strcpy(this->type, state.type);
	//strcpy(this->robot, state.robot);
	robot = state.robot;
	//strcpy(this->generatorType, state.generatorType);
	generatorType = state.generatorType;
	strcpy(this->trajectoryFilePath, state.trajectoryFilePath);
}

//-----------------------------------------------------------------------------------------------------------

State::~State()
{
	delete[] name;
	delete[] type;
	//delete[] robot;
	//delete[] generatorType;
	delete[] trajectoryFilePath;
}


//-----------------------------------------------------------------------------------------------------------

void State::setName(char *name)
{
	strcpy(this->name, name);
}

char* State::getName() const
{
	return name;
}

//-----------------------------------------------------------------------------------------------------------

void State::setType(char *type)
{
	strcpy(this->type, type);
}

//-----------------------------------------------------------------------------------------------------------

char* State::getType() const
{
	return type;
}

//-----------------------------------------------------------------------------------------------------------

void State::setRobot(char *robot)
{
	//std::cout<<">>>>>>>>"<<robot<<"##"<<std::endl;
	//std::cout<<strcmp(robot, (const char *)"ROBOT_IRP6_ON_TRACK")<<std::endl;
	//strcpy(this->robot, robot);
	if(strcmp(robot, (const char *)"ROBOT_IRP6_ON_TRACK") == 0)
		this->robot = ROBOT_IRP6_ON_TRACK;
	else if(strcmp(robot, (const char *)"ROBOT_IRP6_POSTUMENT") == 0)
		this->robot = ROBOT_IRP6_POSTUMENT;
	else if(strcmp(robot, (const char *)"ROBOT_CONVEYOR") == 0)
		this->robot = ROBOT_CONVEYOR;
	else if(strcmp(robot, (const char *)"ROBOT_SPEAKER") == 0)
		this->robot = ROBOT_SPEAKER;
	else if(strcmp(robot, (const char *)"ROBOT_IRP6_MECHATRONIKA") == 0)
		this->robot = ROBOT_IRP6_MECHATRONIKA;
	else if(strcmp(robot, (const char *)"ROBOT_ELECTRON") == 0)
		this->robot = ROBOT_ELECTRON;
	else if(strcmp(robot, (const char *)"ROBOT_FESTIVAL") == 0)
		this->robot = ROBOT_FESTIVAL;
	else if(strcmp(robot, (const char *)"ROBOT_HAND") == 0)
		this->robot = ROBOT_HAND;
	else if(strcmp(robot, (const char *)"ROBOT_SPEECHRECOGNITION") == 0)
		this->robot = ROBOT_SPEECHRECOGNITION;
	else
		this->robot = ROBOT_UNDEFINED;	
}

//-----------------------------------------------------------------------------------------------------------

ROBOT_ENUM State::getRobot() const
{
	return robot;
}
//-----------------------------------------------------------------------------------------------------------
void State::setGeneratorType(char *genType)
{
	//std::cout<<"######"<<genType<<std::endl;
	//std::cout<<strcmp(genType, (const char *)"ECP_GEN_TRANSPARENT")<<std::endl;
	//strcpy(this->generatorType, genType);
	if(strcmp(genType, (const char *)"ECP_GEN_TRANSPARENT") == 0)
		this->generatorType = ECP_GEN_TRANSPARENT;	
	else if(strcmp(genType, (const char *)"ECP_GEN_TFF_NOSE_RUN") == 0)
		this->generatorType = ECP_GEN_TFF_NOSE_RUN;	
	else if(strcmp(genType, (const char *)"ECP_GEN_TEACH_IN") == 0)
		this->generatorType = ECP_GEN_TEACH_IN;	
	else if(strcmp(genType, (const char *)"ECP_GEN_SMOOTH") == 0)
		this->generatorType = ECP_GEN_SMOOTH;	
	else if(strcmp(genType, (const char *)"ECP_GEN_TFF_RUBIK_GRAB") == 0)
		this->generatorType = ECP_GEN_TFF_RUBIK_GRAB;	
	else if(strcmp(genType, (const char *)"ECP_GEN_TFF_RUBIK_FACE_ROTATE") == 0)
		this->generatorType = ECP_GEN_TFF_RUBIK_FACE_ROTATE;	
	else if(strcmp(genType, (const char *)"ECP_GEN_TFF_GRIPPER_APPROACH") == 0)
		this->generatorType = ECP_GEN_TFF_GRIPPER_APPROACH;	
	else if(strcmp(genType, (const char *)"RCSC_GRIPPER_OPENING") == 0)
		this->generatorType = RCSC_GRIPPER_OPENING;	
	else if(strcmp(genType, (const char *)"ECP_GEN_BIAS_EDP_FORCE") == 0)
		this->generatorType = ECP_GEN_BIAS_EDP_FORCE;	
	else if(strcmp(genType, (const char *)"ECP_WEIGHT_MEASURE_GENERATOR") == 0)
		this->generatorType = ECP_WEIGHT_MEASURE_GENERATOR;	
	else if(strcmp(genType, (const char *)"ECP_TOOL_CHANGE_GENERATOR") == 0)
		this->generatorType = ECP_TOOL_CHANGE_GENERATOR;	
	else 
		this->generatorType = ECP_GEN_SPEAK;	
}

//----------------------------------------------------------------------------------------------------------

STATE_MACHINE_ECP_STATES State::getGeneratorType() const
{
	return generatorType;
}

//----------------------------------------------------------------------------------------------------------

void State::setTrajectoryFilePath(char* trajFilePath)
{
	strcpy(this->trajectoryFilePath, trajFilePath);
}

//----------------------------------------------------------------------------------------------------------

char* State::getTrajectoryFilePath() const
{
	return trajectoryFilePath;
}

//----------------------------------------------------------------------------------------------------------

void State::showStateContent() const
{
	std::cout<<name<<std::endl<<type<<std::endl<<robot<<std::endl<<generatorType<<std::endl<<trajectoryFilePath<<std::endl;
}
