/*!
 * @file
 * @brief File contains Trajectory class definition
 * @author lorenzo, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */


#include <cstdio>
#include <cstring>

#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include "base/lib/datastr.h"
#include "base/ecp_mp/Trajectory.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {

Trajectory::Trajectory()
{
}

Trajectory::Trajectory(const std::string & numOfPoses, const std::string & trajectoryID, const std::string & poseSpecification)
{
	trjID = trajectoryID;
	this->numOfPoses = boost::lexical_cast <unsigned int>(numOfPoses);
	poseSpec = lib::returnProperPS(poseSpecification);
}

Trajectory::Trajectory(const Trajectory &trajectory)
{
	trjID = trajectory.trjID;
	numOfPoses = trajectory.numOfPoses;
	poseSpec = trajectory.poseSpec;
	//trjPoses = trajectory.trjPoses;
}

void Trajectory::setTrjID(const std::string & trjID)
{
	this->trjID = trjID;
}

std::string Trajectory::getTrjID() const
{
	return trjID;
}
//TODO do poprawki, zmiana na newsmooth
/*void Trajectory::writeTrajectoryToXmlFile(const std::string & fileName, lib::POSE_SPECIFICATION ps, const std::list <
		ecp_mp::common::smooth_trajectory_pose> &poses)
{
	int posCount = poses.size();

	xmlDocPtr doc = xmlNewDoc((const xmlChar *) "1.0");

	doc->children = xmlNewDocNode(doc, NULL, (const xmlChar *) "Trajectory", NULL);
	xmlSetProp(doc->children, (const xmlChar *) "coordinateType", (const xmlChar *) lib::toString(ps).c_str());
	xmlSetProp(doc->children, (const xmlChar *) "numOfPoses", (const xmlChar *) lib::toString(posCount).c_str());

	std::list <ecp_mp::common::smooth_trajectory_pose>::const_iterator it;
	for (it = poses.begin(); it != poses.end(); ++it) {
		//		xmlNodePtr tree = xmlNewChild(doc->children, NULL, (const xmlChar *) "Pose", NULL);
		//xmlNodePtr subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Velocity", (const xmlChar *)lib::toString((*it).v, lib::MAX_SERVOS_NR).c_str());
		//subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Accelerations", (const xmlChar *)lib::toString((*it).a, lib::MAX_SERVOS_NR).c_str());
		//subtree = xmlNewChild(tree, NULL, (const xmlChar *)"Coordinates", (const xmlChar *)lib::toString((*it).coordinates, lib::MAX_SERVOS_NR).c_str());
	}
	std::string file(fileName);
	file += ".xml";

	xmlKeepBlanksDefault(0);
	xmlSaveFormatFile(file.c_str(), doc, 1);
	std::cout << "-->  File " << fileName << " was saved to XML file" << std::endl;
}*/

void Trajectory::createNewPose()
{
	//actPose = new ecp_mp::common::smooth_trajectory_pose();
	//actPose.arm_type = this->poseSpec;
}

void Trajectory::addPoseToTrajectory()
{
	//trjPoses.push_back(actPose);
}

void Trajectory::setNumOfPoses(unsigned int numOfPoses)
{
	this->numOfPoses = numOfPoses;
}

unsigned int Trajectory::getNumberOfPoses() const
{
	return numOfPoses;
}

void Trajectory::setPoseSpecification(const std::string & poseSpecification)
{
	poseSpec = lib::returnProperPS(poseSpecification);
}

lib::ECP_POSE_SPECIFICATION Trajectory::getPoseSpecification() const
{
	return poseSpec;
}

void Trajectory::setVelocities(const std::string & Velocities)
{
	//lib::setValuesInArray(actPose.v, Velocities);
}

const double* Trajectory::getVelocities() const
{
	//return actPose.v;
	return NULL;
}

void Trajectory::setAccelerations(const std::string & accelerations)
{
	//lib::setValuesInArray(actPose.a, accelerations);
}

const double* Trajectory::getAccelerations() const
{
	return NULL;
	//return actPose.a;
}

void Trajectory::setCoordinates(const std::string & cCoordinates)
{
	//lib::setValuesInArray(actPose.coordinates, cCoordinates);
}

const double* Trajectory::getCoordinates() const
{
	return NULL;
	//return actPose.coordinates;
}

void Trajectory::showTime()
{
	/*std::list <ecp_mp::common::smooth_trajectory_pose>::const_iterator it;
	printf("Nazwa: %s, PoseSpec: %d, NumOfPoses: %u\n", trjID.c_str(), poseSpec, numOfPoses);
	for (it = trjPoses.begin(); it != trjPoses.end(); ++it) {
		printf("%f %f %f %f %f %f %f %f\n", (*it).v[0], (*it).v[1], (*it).v[2], (*it).v[3], (*it).v[4], (*it).v[5], (*it).v[6], (*it).v[7]);
		printf("%f %f %f %f %f %f %f %f\n", (*it).a[0], (*it).a[1], (*it).a[2], (*it).a[3], (*it).a[4], (*it).a[5], (*it).a[6], (*it).a[7]);
		printf("%f %f %f %f %f %f %f %f\n***\n\n", (*it).coordinates[0], (*it).coordinates[1], (*it).coordinates[2], (*it).coordinates[3], (*it).coordinates[4], (*it).coordinates[5], (*it).coordinates[6], (*it).coordinates[7]);
	}*/
}

/*std::list <ecp_mp::common::smooth_trajectory_pose> & Trajectory::getPoses()
{
	return trjPoses;
}*/

} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp


