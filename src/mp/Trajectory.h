
#if !defined (_TRAJECTORY_H_)
#define _TRAJECTORY_H_

#include <list>

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "ecp/common/ecp_smooth_taught_in_pose.h"

namespace mrrocpp {
namespace mp {
namespace common {


class Trajectory
{
	public:
		Trajectory();
		Trajectory(const char *numOfPoses, const char *trajectoryName, const char *poseSpecification);
		Trajectory(const Trajectory &trajectory);
		~Trajectory();

		static int setValuesInArray(double arrayToFill[], const char *dataString);
		static lib::POSE_SPECIFICATION returnProperPS(const char *poseSpecification);
		static const char * toString(double valArr[], int length);
		static const char * toString(int numberOfPoses);
		static const char * toString(lib::POSE_SPECIFICATION ps);
		static const char * returnRobotName(lib::ROBOT_ENUM robot);

		static bool writeTrajectoryToXmlFile(const char *fileName, lib::POSE_SPECIFICATION ps, std::list<ecp::common::ecp_smooth_taught_in_pose> &poses);
		
		void createNewPose();
		void addPoseToTrajectory();

		void setTrjID(const char *trjID);
		const char * getTrjID() const;

		void setNumOfPoses(uint64_t numOfPoses);
		uint64_t getNumberOfPoses() const;

		void setPoseSpecification(const char *poseSpecification);
		lib::POSE_SPECIFICATION getPoseSpecification() const;

		void setStartVelocities(const char *startVelocities);
		double *getStartVelocities() const;

		void setEndVelocities(const char *endVelocities);
		double *getEndVelocities() const;

		void setVelocities(const char *Velocities);
		double *getVelocities() const;

		void setAccelerations(const char *accelerations);
		double *getAccelerations() const;

		void setCoordinates(const char *cCoordinates);
		double *getCoordinates() const;

		void showTime();

		std::list<ecp::common::ecp_smooth_taught_in_pose> * getPoses();

	private:
		char trjID[80];
		uint64_t numOfPoses;		
		lib::POSE_SPECIFICATION poseSpec;
		ecp::common::ecp_smooth_taught_in_pose *actPose;
		std::list<ecp::common::ecp_smooth_taught_in_pose> *trjPoses;
};

} // namespace common
} // namespace mp
} // namespace mrrocpp



#endif
