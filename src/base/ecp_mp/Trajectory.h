#if !defined (_TRAJECTORY_H_)
#define _TRAJECTORY_H_

/*!
 * @file
 * @brief File contains Trajectory class declaration
 * @author mkisiel, jstocka, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

#include <list>
#include <string>

//#include "base/ecp_mp/smooth_trajectory_pose.h"

#include <boost/serialization/utility.hpp>

namespace mrrocpp {
namespace ecp_mp {
namespace common {

class Trajectory
{
public:
	Trajectory();
			Trajectory(const std::string & numOfPoses, const std::string & trajectoryName, const std::string & poseSpecification);
	Trajectory(const Trajectory &trajectory);

	//static void writeTrajectoryToXmlFile(const std::string & fileName, lib::POSE_SPECIFICATION ps, const std::list <
	//		ecp_mp::common::smooth_trajectory_pose> &poses);//for smooth
	void createNewPose();
	void addPoseToTrajectory();

	void setTrjID(const std::string & trjID);
	std::string getTrjID() const;

	void setNumOfPoses(unsigned int numOfPoses);
	unsigned int getNumberOfPoses() const;

	void setPoseSpecification(const std::string & poseSpecification);
	lib::ECP_POSE_SPECIFICATION getPoseSpecification() const;

	void setStartVelocities(const std::string & startVelocities);
	double getStartVelocities() const;

	void setEndVelocities(const std::string & endVelocities);
	double getEndVelocities() const;

	void setVelocities(const std::string & Velocities);
	const double* getVelocities() const;

	void setAccelerations(const std::string & accelerations);
	const double* getAccelerations() const;

	void setCoordinates(const std::string & cCoordinates);
	const double* getCoordinates() const;

	void showTime();

	//std::list <ecp_mp::common::smooth_trajectory_pose> & getPoses();

private:
	std::string trjID;
	unsigned int numOfPoses;
	lib::ECP_POSE_SPECIFICATION poseSpec;
	//ecp_mp::common::smooth_trajectory_pose actPose;
	//std::list <ecp_mp::common::smooth_trajectory_pose> trjPoses;

	// boost serialization methods
	friend class boost::serialization::access;

	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_NVP(trjID);
		ar & BOOST_SERIALIZATION_NVP(numOfPoses);
		ar & BOOST_SERIALIZATION_NVP(poseSpec);
		//ar & BOOST_SERIALIZATION_NVP(trjPoses);
	}
};

} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp


#endif
