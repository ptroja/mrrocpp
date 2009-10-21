#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <string>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include "lib/datastr.h"
#include "ecp_mp/Trajectory.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {


Trajectory::Trajectory()
{
}

Trajectory::Trajectory(const char *numOfPoses, const std::string & trajectoryID, const char *poseSpecification)
{
	trjID = trajectoryID;
	this->numOfPoses = (uint64_t)atoi(numOfPoses);
	poseSpec = lib::returnProperPS(poseSpecification);
}

Trajectory::Trajectory(const Trajectory &trajectory)
{
	trjID = trajectory.trjID;
	numOfPoses = trajectory.numOfPoses;
	poseSpec = trajectory.poseSpec;
	trjPoses = trajectory.trjPoses;
	trjPoses2 = trajectory.trjPoses2;
}

void Trajectory::setTrjID(const char *trjID)
{
	this->trjID = trjID;
}

const char * Trajectory::getTrjID() const
{
	return trjID.c_str();
}

void Trajectory::writeTrajectoryToXmlFile(const char *fileName, lib::POSE_SPECIFICATION ps, std::list<ecp_mp::common::smooth_trajectory_pose> &poses)
{
	int posCount = poses.size();
	xmlDocPtr doc;
	xmlNodePtr tree, subtree;
	std::list<ecp_mp::common::smooth_trajectory_pose>::iterator it;

	doc = xmlNewDoc((const xmlChar *) "1.0");

	doc->children = xmlNewDocNode(doc, NULL, (const xmlChar *) "Trajectory", NULL);
	xmlSetProp(doc->children, (const xmlChar *) "coordinateType", (const xmlChar *) lib::toString(ps).c_str());
	xmlSetProp(doc->children, (const xmlChar *) "numOfPoses", (const xmlChar *) lib::toString(posCount).c_str());
	for(it = poses.begin(); it != poses.end(); ++it)
	{
		tree = xmlNewChild(doc->children, NULL, (const xmlChar *) "Pose", NULL);
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"StartVelocity", (const xmlChar *)lib::toString((*it).v_p, MAX_SERVOS_NR).c_str());
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"EndVelocity", (const xmlChar *)lib::toString((*it).v_k, MAX_SERVOS_NR).c_str());
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Velocity", (const xmlChar *)lib::toString((*it).v, MAX_SERVOS_NR).c_str());
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Accelerations", (const xmlChar *)lib::toString((*it).a, MAX_SERVOS_NR).c_str());
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Coordinates", (const xmlChar *)lib::toString((*it).coordinates, MAX_SERVOS_NR).c_str());
	}
	std::string file(fileName);
	file+=".xml";

	xmlKeepBlanksDefault(0);
	xmlSaveFormatFile(file.c_str(), doc, 1);
	printf("-->  File \"%s\" was saved to XML file\n", fileName);
}

void Trajectory::writeTrajectoryToXmlFile2(const char *fileName, lib::POSE_SPECIFICATION ps, std::list<ecp_mp::common::smooth2_trajectory_pose> &poses)
{
	int posCount = poses.size();
	xmlDocPtr doc;
	xmlNodePtr tree, subtree;
	std::list<ecp_mp::common::smooth2_trajectory_pose>::iterator it;

	doc = xmlNewDoc((const xmlChar *) "1.0");

	doc->children = xmlNewDocNode(doc, NULL, (const xmlChar *) "Trajectory", NULL);
	xmlSetProp(doc->children, (const xmlChar *) "coordinateType", (const xmlChar *) lib::toString(ps).c_str());
	xmlSetProp(doc->children, (const xmlChar *) "numOfPoses", (const xmlChar *) lib::toString(posCount).c_str());
	for(it = poses.begin(); it != poses.end(); ++it)
	{
		tree = xmlNewChild(doc->children, NULL, (const xmlChar *) "Pose", NULL);
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Velocity", (const xmlChar *)lib::toString((*it).v, MAX_SERVOS_NR).c_str());
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Accelerations", (const xmlChar *)lib::toString((*it).a, MAX_SERVOS_NR).c_str());
		subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Coordinates", (const xmlChar *)lib::toString((*it).coordinates, MAX_SERVOS_NR).c_str());
	}
	std::string file(fileName);
	file+=".xml";

	xmlKeepBlanksDefault(0);
	xmlSaveFormatFile(file.c_str(), doc, 1);
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
	trjPoses.push_back(*actPose);
}

void Trajectory::addPoseToTrajectory2()//for smooth2
{
	trjPoses2.push_back(*actPose2);
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
	poseSpec = lib::returnProperPS(poseSpecification);
}

lib::POSE_SPECIFICATION Trajectory::getPoseSpecification() const
{
	return poseSpec;
}

void Trajectory::setStartVelocities(const char *startVelocities)
{
	lib::setValuesInArray(actPose->v_p, startVelocities);
}

double * Trajectory::getStartVelocities() const
{
	return actPose->v_p;
}

void Trajectory::setEndVelocities(const char *endVelocities)
{
	lib::setValuesInArray(actPose->v_k, endVelocities);
}

double * Trajectory::getEndVelocities() const
{
	return actPose->v_k;
}

void Trajectory::setVelocities(const char *Velocities)
{
	lib::setValuesInArray(actPose->v, Velocities);
}

double * Trajectory::getVelocities() const
{
	return actPose->v;
}

void Trajectory::setAccelerations(const char *accelerations)
{
	lib::setValuesInArray(actPose->a, accelerations);
}

double * Trajectory::getAccelerations() const
{
	return actPose->a;
}

void Trajectory::setVelocities2(const char *Velocities)//for smooth2
{
	lib::setValuesInArray(actPose2->v, Velocities);
}

double * Trajectory::getVelocities2() const//for smooth2
{
	return actPose2->v;
}

void Trajectory::setAccelerations2(const char *accelerations)//for smooth2
{
	lib::setValuesInArray(actPose2->a, accelerations);
}

double * Trajectory::getAccelerations2() const//for smooth2
{
	return actPose2->a;
}

void Trajectory::setCoordinates(const char *cCoordinates)
{
	lib::setValuesInArray(actPose->coordinates, cCoordinates);
}

double * Trajectory::getCoordinates() const
{
	return actPose->coordinates;
}

void Trajectory::setCoordinates2(const char *cCoordinates)//for smooth2
{
	lib::setValuesInArray(actPose2->coordinates, cCoordinates);
}

double * Trajectory::getCoordinates2() const//for smooth2
{
	return actPose2->coordinates;
}

void Trajectory::showTime()
{
	std::list<ecp_mp::common::smooth_trajectory_pose>::iterator it;
	printf("Nazwa: %s, PoseSpec: %d, NumOfPoses: %u\n", trjID.c_str(), poseSpec, numOfPoses);
	for(it=trjPoses.begin(); it!=trjPoses.end(); ++it)
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
	printf("Nazwa: %s, PoseSpec: %d, NumOfPoses: %u\n", trjID.c_str(), poseSpec, numOfPoses);
	for(it=trjPoses2.begin(); it!=trjPoses2.end(); ++it)
	{
		printf("%f %f %f %f %f %f %f %f\n", (*it).v[0], (*it).v[1], (*it).v[2], (*it).v[3],
				(*it).v[4], (*it).v[5], (*it).v[6], (*it).v[7]);
		printf("%f %f %f %f %f %f %f %f\n", (*it).a[0], (*it).a[1], (*it).a[2], (*it).a[3],
				(*it).a[4], (*it).a[5], (*it).a[6], (*it).a[7]);
		printf("%f %f %f %f %f %f %f %f\n***\n\n", (*it).coordinates[0], (*it).coordinates[1], (*it).coordinates[2], (*it).coordinates[3],
				(*it).coordinates[4], (*it).coordinates[5], (*it).coordinates[6], (*it).coordinates[7]);
	}
}

std::list<ecp_mp::common::smooth_trajectory_pose> & Trajectory::getPoses()
{
	return trjPoses;
}

std::list<ecp_mp::common::smooth2_trajectory_pose> & Trajectory::getPoses2()//for smooth2
{
	return trjPoses2;
}

} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp


