
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
		Trajectory(char *numOfPoses, char *trajectoryName, char *poseSpecification);
		Trajectory(const Trajectory &trajectory);
		~Trajectory();

		static int setValuesInArray(double arrayToFill[], char *dataString);
		static lib::POSE_SPECIFICATION returnProperPS(char *poseSpecification);
		static const char * toString(double valArr[], int length);
		static const char * toString(int numberOfPoses);
		static const char * toString(lib::POSE_SPECIFICATION ps);
		static const char * returnRobotName(ROBOT_ENUM robot);

		static bool writeTrajectoryToXmlFile(char *fileName, lib::POSE_SPECIFICATION ps, std::list<ecp::common::ecp_smooth_taught_in_pose> &poses);
		
		void createNewPose();
		void addPoseToTrajectory();
		void setTrjID(char *trjID);
		char * getTrjID() const;
		void setNumOfPoses(uint64_t numOfPoses);
		uint64_t getNumberOfPoses() const;
		void setPoseSpecification(char *poseSpecification);
		lib::POSE_SPECIFICATION getPoseSpecification() const;
		void setStartVelocities(char *startVelocities);
		double *getStartVelocities() const;
		void setEndVelocities(char *endVelocities);
		double *getEndVelocities() const;
		void setVelocities(char *Velocities);
		double *getVelocities() const;
		void setAccelerations(char *accelerations);
		double *getAccelerations() const;
		void setCoordinates(char *cCoordinates);
		double *getCoordinates() const;
		void showTime();

		std::list<ecp::common::ecp_smooth_taught_in_pose> * getPoses();

	private:
		char * trjID;
		uint64_t numOfPoses;		
		lib::POSE_SPECIFICATION poseSpec;
		ecp::common::ecp_smooth_taught_in_pose *actPose;
		std::list<ecp::common::ecp_smooth_taught_in_pose> *trjPoses;
};

} // namespace common
} // namespace mp
} // namespace mrrocpp



#endif
