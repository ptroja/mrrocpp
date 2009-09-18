
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include "ecp_mp/Trajectory.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {


Trajectory::Trajectory()
{
	trjPoses = new std::list<ecp_mp::common::smooth_trajectory_pose>();
	trjPoses2 = new std::list<ecp_mp::common::smooth2_trajectory_pose>(); //for smooth2
}

Trajectory::Trajectory(const char *numOfPoses, const char *trajectoryID, const char *poseSpecification)
{
	strcpy(trjID, trajectoryID);
	this->numOfPoses = (uint64_t)atoi(numOfPoses);
	poseSpec = returnProperPS(poseSpecification);
	trjPoses = new std::list<ecp_mp::common::smooth_trajectory_pose>();
	trjPoses2 = new std::list<ecp_mp::common::smooth2_trajectory_pose>(); //for smooth2

}

Trajectory::Trajectory(const Trajectory &trajectory)
{
	strcpy(trjID, trajectory.trjID);
	numOfPoses = trajectory.numOfPoses;
	poseSpec = trajectory.poseSpec;
	trjPoses = new std::list<ecp_mp::common::smooth_trajectory_pose>(trajectory.trjPoses->begin(), trajectory.trjPoses->end());
	trjPoses2 = new std::list<ecp_mp::common::smooth2_trajectory_pose>(trajectory.trjPoses2->begin(), trajectory.trjPoses2->end()); //for smooth2
}

Trajectory::~Trajectory()
{
	delete trjPoses;
	delete trjPoses2;
}

void Trajectory::setTrjID(const char *trjID)
{
	strcpy(this->trjID, trjID);
}

const char * Trajectory::getTrjID() const
{
	return trjID;
}

lib::POSE_SPECIFICATION Trajectory::returnProperPS(const std::string & poseSpecification)
{
	if (poseSpecification == "MOTOR")
	{	return lib::MOTOR;	}
	if (poseSpecification == "JOINT")
	{	return lib::JOINT;	}
	if (poseSpecification == "XYZ_ANGLE_AXIS")
	{	return lib::XYZ_ANGLE_AXIS;	}
	if (poseSpecification == "XYZ_EULER_ZYZ")
	{	return lib::XYZ_EULER_ZYZ;	}
	else
		return lib::INVALID_END_EFFECTOR;
}

int Trajectory::setValuesInArray(double arrayToFill[], const char *dataString)
{
	int index = 0;
	char *value;
	char *toSplit = strdup(dataString);

	value = strtok(toSplit, " \t");
	arrayToFill[index++] = atof(value);
	while((value = strtok(NULL, " \t"))!=NULL)
		arrayToFill[index++] = atof(value);

	free(toSplit);

	return index;
}

const char * Trajectory::toString(double valArr[], int length)
{
	char * afterConv = new char[length];
	for(int i=0; i<length; i++)
	{
		if(i==0)
			sprintf(afterConv, "%g", valArr[i]);
		else
			sprintf(afterConv, "%s\t%g", afterConv, valArr[i]);
	}
//	std::cout<<afterConv<<std::endl;
	return afterConv;
}

const char * Trajectory::toString(int numberOfPoses)
{
	char * numStr = new char[10];
	sprintf(numStr, "%d", numberOfPoses);

	return numStr;
}

const char * Trajectory::returnRobotName(lib::ROBOT_ENUM robot)
{
	using namespace lib;
	switch (robot)
	{
		case ROBOT_IRP6_ON_TRACK:
			return "ROBOT_IRP6_ON_TRACK";
		case ROBOT_IRP6_POSTUMENT:
			return "ROBOT_IRP6_POSTUMENT";
		case ROBOT_CONVEYOR:
			return "ROBOT_CONVEYOR";
		case ROBOT_SPEAKER:
			return "ROBOT_SPEAKER";
		case ROBOT_IRP6_MECHATRONIKA:
			return "ROBOT_IRP6_MECHATRONIKA";
		case ROBOT_ELECTRON:
			return "ROBOT_ELECTRON";
		case ROBOT_FESTIVAL:
			return "ROBOT_FESTIVAL";
		case ROBOT_HAND:
			return "ROBOT_HAND";
		case ROBOT_SPEECHRECOGNITION:
			return "ROBOT_SPEECHRECOGNITION";
		default:
			return "ROBOT_UNDEFINED";
	}
}

const char * Trajectory::toString(lib::POSE_SPECIFICATION ps)
{
	if ( ps == lib::MOTOR )
	{	return "MOTOR";	}
	if ( ps == lib::JOINT )
	{	return "JOINT";	}
	if ( ps == lib::XYZ_ANGLE_AXIS )
	{	return "XYZ_ANGLE_AXIS";	}
	if ( ps == lib::XYZ_EULER_ZYZ )
	{	return "XYZ_EULER_ZYZ";	}
	else
		return "INVALID_END_EFFECTOR";
}

void Trajectory::writeTrajectoryToXmlFile(const char *fileName, lib::POSE_SPECIFICATION ps, std::list<ecp_mp::common::smooth_trajectory_pose> &poses)
{
	char * file = new char[80];
	int posCount = poses.size();
	xmlDocPtr doc;
	xmlNodePtr tree, subtree;
	std::list<ecp_mp::common::smooth_trajectory_pose>::iterator it;

	doc = xmlNewDoc((const xmlChar *) "1.0");

	doc->children = xmlNewDocNode(doc, NULL, (const xmlChar *) "Trajectory", NULL);
	xmlSetProp(doc->children, (const xmlChar *) "coordinateType", (const xmlChar *) Trajectory::toString(ps));
	xmlSetProp(doc->children, (const xmlChar *) "numOfPoses", (const xmlChar *) Trajectory::toString(posCount));
	for(it = poses.begin(); it != poses.end(); ++it)
	{
		tree = xmlNewChild(doc->children, NULL, (const xmlChar *) "Pose", NULL);
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"StartVelocity", (const xmlChar *)Trajectory::toString((*it).v_p, MAX_SERVOS_NR));
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"EndVelocity", (const xmlChar *)Trajectory::toString((*it).v_k, MAX_SERVOS_NR));
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Velocity", (const xmlChar *)Trajectory::toString((*it).v, MAX_SERVOS_NR));
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Accelerations", (const xmlChar *)Trajectory::toString((*it).a, MAX_SERVOS_NR));
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Coordinates", (const xmlChar *)Trajectory::toString((*it).coordinates, MAX_SERVOS_NR));
	}
	sprintf(file, "%s%s", fileName, ".xml");

	xmlKeepBlanksDefault(0);
	xmlSaveFormatFile(file, doc, 1);
	printf("-->  File \"%s\" was saved to XML file\n", fileName);
}

void Trajectory::writeTrajectoryToXmlFile2(const char *fileName, lib::POSE_SPECIFICATION ps, std::list<ecp_mp::common::smooth2_trajectory_pose> &poses)
{
	char * file = new char[80];
	int posCount = poses.size();
	xmlDocPtr doc;
	xmlNodePtr tree, subtree;
	std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator it;

	doc = xmlNewDoc((const xmlChar *) "1.0");

	doc->children = xmlNewDocNode(doc, NULL, (const xmlChar *) "Trajectory", NULL);
	xmlSetProp(doc->children, (const xmlChar *) "coordinateType", (const xmlChar *) Trajectory::toString(ps));
	xmlSetProp(doc->children, (const xmlChar *) "numOfPoses", (const xmlChar *) Trajectory::toString(posCount));
	for(it = poses.begin(); it != poses.end(); ++it)
	{
		tree = xmlNewChild(doc->children, NULL, (const xmlChar *) "Pose", NULL);
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Velocity", (const xmlChar *)Trajectory::toString((*it).v, MAX_SERVOS_NR));
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Accelerations", (const xmlChar *)Trajectory::toString((*it).a, MAX_SERVOS_NR));
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Coordinates", (const xmlChar *)Trajectory::toString((*it).coordinates, MAX_SERVOS_NR));
	}
	sprintf(file, "%s%s", fileName, ".xml");

	xmlKeepBlanksDefault(0);
	xmlSaveFormatFile(file, doc, 1);
	printf("-->  File \"%s\" was saved to XML file\n", fileName);
}

void Trajectory::createNewPose()
{
	actPose = new ecp_mp::common::smooth_trajectory_pose();
	actPose->arm_type = this->poseSpec;
}

void Trajectory::createNewPose2() //for smooth2
{
	actPose2 = new ecp_mp::common::smooth2_trajectory_pose();
	actPose2->arm_type = this->poseSpec;
}

void Trajectory::addPoseToTrajectory()
{
	trjPoses->push_back(*actPose);
}

void Trajectory::addPoseToTrajectory2()//for smooth2
{
	trjPoses2->push_back(*actPose2);
}

void Trajectory::setNumOfPoses(unsigned int numOfPoses)
{
	this->numOfPoses = numOfPoses;
}

unsigned int Trajectory::getNumberOfPoses() const
{
	return numOfPoses;
}

void Trajectory::setPoseSpecification(const char *poseSpecification)
{
	poseSpec = returnProperPS(poseSpecification);
}

lib::POSE_SPECIFICATION Trajectory::getPoseSpecification() const
{
	return poseSpec;
}

void Trajectory::setStartVelocities(const char *startVelocities)
{
	setValuesInArray(actPose->v_p, startVelocities);
}

double * Trajectory::getStartVelocities() const
{
	return actPose->v_p;
}

void Trajectory::setEndVelocities(const char *endVelocities)
{
	setValuesInArray(actPose->v_k, endVelocities);
}

double * Trajectory::getEndVelocities() const
{
	return actPose->v_k;
}

void Trajectory::setVelocities(const char *Velocities)
{
	setValuesInArray(actPose->v, Velocities);
}

double * Trajectory::getVelocities() const
{
	return actPose->v;
}

void Trajectory::setAccelerations(const char *accelerations)
{
	setValuesInArray(actPose->a, accelerations);
}

double * Trajectory::getAccelerations() const
{
	return actPose->a;
}

void Trajectory::setVelocities2(const char *Velocities)//for smooth2
{
	setValuesInArray(actPose2->v, Velocities);
}

double * Trajectory::getVelocities2() const//for smooth2
{
	return actPose2->v;
}

void Trajectory::setAccelerations2(const char *accelerations)//for smooth2
{
	setValuesInArray(actPose2->a, accelerations);
}

double * Trajectory::getAccelerations2() const//for smooth2
{
	return actPose2->a;
}

void Trajectory::setCoordinates(const char *cCoordinates)
{
	setValuesInArray(actPose->coordinates, cCoordinates);
}

double * Trajectory::getCoordinates() const
{
	return actPose->coordinates;
}

void Trajectory::setCoordinates2(const char *cCoordinates)//for smooth2
{
	setValuesInArray(actPose2->coordinates, cCoordinates);
}

double * Trajectory::getCoordinates2() const//for smooth2
{
	return actPose2->coordinates;
}

void Trajectory::showTime()
{
	std::list<ecp_mp::common::smooth_trajectory_pose>::iterator it;
	printf("Nazwa: %s, PoseSpec: %d, NumOfPoses: %u\n", trjID, poseSpec, numOfPoses);
	for(it=trjPoses->begin(); it!=trjPoses->end(); ++it)
	{
		printf("%f %f %f %f %f %f %f %f\n", (*it).v_p[0], (*it).v_p[1], (*it).v_p[2], (*it).v_p[3],
				(*it).v_p[4], (*it).v_p[5], (*it).v_p[6], (*it).v_p[7]);
		printf("%f %f %f %f %f %f %f %f\n", (*it).v_k[0], (*it).v_k[1], (*it).v_k[2], (*it).v_k[3],
				(*it).v_k[4], (*it).v_k[5], (*it).v_k[6], (*it).v_k[7]);
		printf("%f %f %f %f %f %f %f %f\n", (*it).v[0], (*it).v[1], (*it).v[2], (*it).v[3],
				(*it).v[4], (*it).v[5], (*it).v[6], (*it).v[7]);
		printf("%f %f %f %f %f %f %f %f\n", (*it).a[0], (*it).a[1], (*it).a[2], (*it).a[3],
				(*it).a[4], (*it).a[5], (*it).a[6], (*it).a[7]);
		printf("%f %f %f %f %f %f %f %f\n***\n\n", (*it).coordinates[0], (*it).coordinates[1], (*it).coordinates[2], (*it).coordinates[3],
				(*it).coordinates[4], (*it).coordinates[5], (*it).coordinates[6], (*it).coordinates[7]);
	}
}

void Trajectory::showTime2()//for smooth2
{
	std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator it;
	printf("Nazwa: %s, PoseSpec: %d, NumOfPoses: %u\n", trjID, poseSpec, numOfPoses);
	for(it=trjPoses2->begin(); it!=trjPoses2->end(); ++it)
	{
		printf("%f %f %f %f %f %f %f %f\n", (*it).v[0], (*it).v[1], (*it).v[2], (*it).v[3],
				(*it).v[4], (*it).v[5], (*it).v[6], (*it).v[7]);
		printf("%f %f %f %f %f %f %f %f\n", (*it).a[0], (*it).a[1], (*it).a[2], (*it).a[3],
				(*it).a[4], (*it).a[5], (*it).a[6], (*it).a[7]);
		printf("%f %f %f %f %f %f %f %f\n***\n\n", (*it).coordinates[0], (*it).coordinates[1], (*it).coordinates[2], (*it).coordinates[3],
				(*it).coordinates[4], (*it).coordinates[5], (*it).coordinates[6], (*it).coordinates[7]);
	}
}

std::list<ecp_mp::common::smooth_trajectory_pose> * Trajectory::getPoses()
{
	return new std::list<ecp_mp::common::smooth_trajectory_pose>(*trjPoses);
}

std::list<ecp_mp::common::smooth2_trajectory_pose> * Trajectory::getPoses2()//for smooth2
{
	return new std::list<ecp_mp::common::smooth2_trajectory_pose>(*trjPoses2);
}

} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp


