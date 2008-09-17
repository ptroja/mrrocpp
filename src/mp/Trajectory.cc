
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "mp/Trajectory.h"

Trajectory::Trajectory()
{
	trjName = new char[80];
	trjPoses	= new std::list<Pose>();
}

Trajectory::Trajectory(char *numOfPoses, char *trajectoryName, char *poseSpecification)
{
	trjName = new char[80];
	strcpy(trjName, trajectoryName);
	this->numOfPoses = (uint64_t)atoi(numOfPoses);
	poseSpec = returnProperPS(poseSpecification);
	trjPoses	= new std::list<Pose>();
	
}

Trajectory::Trajectory(const Trajectory &trajectory)
{
	trjName = new char[80];
	strcpy(trjName, trajectory.trjName);
	numOfPoses = trajectory.numOfPoses;
	poseSpec = trajectory.poseSpec;
	trjPoses = new std::list<Pose>(trajectory.trjPoses->begin(), trajectory.trjPoses->end());
}

Trajectory::~Trajectory()
{
	delete []trjName;
	delete trjPoses;
}

void Trajectory::setName(char *trjName)
{
	strcpy(this->trjName, trjName);
}

char * Trajectory::getName() const
{
	return trjName;
}

POSE_SPECIFICATION Trajectory::returnProperPS(char *poseSpecification)
{
	if ( !strcmp(poseSpecification, (const char *)"MOTOR") )
	{	return MOTOR;	}
	if ( !strcmp(poseSpecification, (const char *)"JOINT") ) 
	{	return JOINT;	}
	if ( !strcmp(poseSpecification, (const char *)"XYZ_ANGLE_AXIS") ) 
	{	return XYZ_ANGLE_AXIS;	}
	if ( !strcmp(poseSpecification, (const char *)"XYZ_EULER_ZYZ") ) 
	{	return XYZ_EULER_ZYZ;	}
	else
		return INVALID_END_EFFECTOR;
}

void Trajectory::setValuesInArray(double arrayToFill[], char *dataString)
{
	int index = 0;
	char *value;
	char *toSplit = strdup(dataString);
	
	value = strtok(toSplit, " \t");
	arrayToFill[index++] = atof(value);
	while((value = strtok(NULL, " \t"))!=NULL)
		arrayToFill[index++] = atof(value);
}

void Trajectory::createNewPose()
{
	actPose = new Pose();
}

void Trajectory::addPoseToTrajectory()
{
	trjPoses->push_back(*actPose);
}

void Trajectory::setNumOfPoses(uint64_t numOfPoses)
{
	this->numOfPoses = numOfPoses;
}

uint64_t Trajectory::getNumberOfPoses() const
{
	return numOfPoses;
}

void Trajectory::setPoseSpecification(char *poseSpecification)
{
	poseSpec = returnProperPS(poseSpecification);
}

POSE_SPECIFICATION Trajectory::getPoseSpecification() const
{
	return poseSpec;
}

void Trajectory::setStartVelocities(char *startVelocities)
{
	setValuesInArray(actPose->startVelocity, startVelocities);
}

double * Trajectory::getStartVelocities() const
{
	return actPose->startVelocity;
}

void Trajectory::setEndVelocities(char *endVelocities)
{
	setValuesInArray(actPose->endVelocity, endVelocities);
}

double * Trajectory::getEndVelocities() const
{
	return actPose->endVelocity;
}

void Trajectory::setVelocities(char *Velocities)
{
	setValuesInArray(actPose->velocity, Velocities);
}

double * Trajectory::getVelocities() const
{
	return actPose->velocity;
}

void Trajectory::setAccelerations(char *accelerations)
{
	setValuesInArray(actPose->accelerations, accelerations);
}

double * Trajectory::getAccelerations() const
{
	return actPose->accelerations;
}

void Trajectory::setCoordinates(char *cCoordinates)
{
	setValuesInArray(actPose->coordinates, cCoordinates);
}

double * Trajectory::getCoordinates() const
{
	return actPose->coordinates;
}

void Trajectory::showTime()
{
	std::list<Pose>::iterator it;
	printf("Nazwa: %s, PoseSpec: %d, NumOfPoses: %d\n", trjName, poseSpec, numOfPoses);
	for(it=trjPoses->begin(); it!=trjPoses->end(); ++it)
	{
		printf("%f %f %f %f %f %f %f %f \n", (*it).startVelocity[0], (*it).startVelocity[1], (*it).startVelocity[2], (*it).startVelocity[3], 
				(*it).startVelocity[4], (*it).startVelocity[5], (*it).startVelocity[6], (*it).startVelocity[7]);
		printf("%f %f %f %f %f %f %f %f \n", (*it).endVelocity[0], (*it).endVelocity[1], (*it).endVelocity[2], (*it).endVelocity[3], 
				(*it).endVelocity[4], (*it).endVelocity[5], (*it).endVelocity[6], (*it).endVelocity[7]);
		printf("%f %f %f %f %f %f %f %f \n", (*it).velocity[0], (*it).velocity[1], (*it).velocity[2], (*it).velocity[3], 
				(*it).velocity[4], (*it).velocity[5], (*it).velocity[6], (*it).velocity[7]);
		printf("%f %f %f %f %f %f %f %f \n", (*it).accelerations[0], (*it).accelerations[1], (*it).accelerations[2], (*it).accelerations[3], 
				(*it).accelerations[4], (*it).accelerations[5], (*it).accelerations[6], (*it).accelerations[7]);
		printf("%f %f %f %f %f %f %f %f \n***\n\n", (*it).coordinates[0], (*it).coordinates[1], (*it).coordinates[2], (*it).coordinates[3], 
				(*it).coordinates[4], (*it).coordinates[5], (*it).coordinates[6], (*it).coordinates[7]);
	}

}

std::list<Trajectory::Pose> * Trajectory::getPoses()
{
	return trjPoses;
}
	
