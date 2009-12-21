
#if !defined (_TRAJECTORY_H_)
#define _TRAJECTORY_H_

#include <list>
#include <string>

#include "lib/impconst.h"
#include "lib/com_buf.h"
//#include "ecp_mp/smooth_trajectory_pose.h"
#include "ecp_mp/smooth_trajectory_pose.h"

#include <boost/serialization/utility.hpp>

namespace mrrocpp {
namespace ecp_mp {
namespace common {


class Trajectory
{
	public:
		Trajectory();
		Trajectory(const char *numOfPoses, const std::string & trajectoryName, const char *poseSpecification);
		Trajectory(const Trajectory &trajectory);

		//static void writeTrajectoryToXmlFile(const char *fileName, lib::POSE_SPECIFICATION ps, std::list<ecp_mp::common::smooth_trajectory_pose> &poses);
		static void writeTrajectoryToXmlFile2(const char *fileName, lib::POSE_SPECIFICATION ps, std::list<ecp_mp::common::smooth_trajectory_pose> &poses);//for smooth
		//void createNewPose();
		void createNewPose2();//for smooth
		//void addPoseToTrajectory();
		void addPoseToTrajectory2();//for smooth

		void setTrjID(const char *trjID);
		const char * getTrjID() const;

		void setNumOfPoses(unsigned int numOfPoses);
		unsigned int getNumberOfPoses() const;

		void setPoseSpecification(const char *poseSpecification);
		lib::ECP_POSE_SPECIFICATION getPoseSpecification() const;

		void setStartVelocities(const char *startVelocities);
		double *getStartVelocities() const;

		void setEndVelocities(const char *endVelocities);
		double *getEndVelocities() const;

		//void setVelocities(const char *Velocities);
		//double *getVelocities() const;

		//void setAccelerations(const char *accelerations);
		//double *getAccelerations() const;

		void setVelocities2(const char *Velocities);//for smooth
		double *getVelocities2() const;//for smooth

		void setAccelerations2(const char *accelerations);//for smooth
		double *getAccelerations2() const;//for smooth

		//void setCoordinates(const char *cCoordinates);//for smooth
		//double *getCoordinates() const;//for smooth

		void setCoordinates2(const char *cCoordinates);//for smooth
		double *getCoordinates2() const;//for smooth

		//void showTime();
		void showTime2();//for smooth

		//std::list<ecp_mp::common::smooth_trajectory_pose> & getPoses();
		std::list<ecp_mp::common::smooth_trajectory_pose> & getPoses2();//for smooth

	private:
		std::string trjID;
		unsigned int numOfPoses;
		lib::ECP_POSE_SPECIFICATION poseSpec;
		//ecp_mp::common::smooth_trajectory_pose *actPose;
		ecp_mp::common::smooth_trajectory_pose *actPose2;//for smooth
		//std::list<ecp_mp::common::smooth_trajectory_pose> trjPoses;
		std::list<ecp_mp::common::smooth_trajectory_pose> trjPoses2;//for smooth

		// boost serialization methods
		friend class boost::serialization::access;

		template<class Archive>
		    void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(trjID);
			ar & BOOST_SERIALIZATION_NVP(numOfPoses);
			ar & BOOST_SERIALIZATION_NVP(poseSpec);
			//ar & BOOST_SERIALIZATION_NVP(trjPoses);
//			ar & BOOST_SERIALIZATION_NVP(trjPoses2);
		}
};

} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp



#endif
