
#if !defined (_TRAJECTORY_H_)
#define _TRAJECTORY_H_

#include <list>

#include "common/impconst.h"
#include "common/com_buf.h"
#include "ecp/common/ecp_smooth_taught_in_pose.h"

class Trajectory
{
	public:
		Trajectory();
		Trajectory(char *numOfPoses, char *trajectoryName, char *poseSpecification);
		Trajectory(const Trajectory &trajectory);
		~Trajectory();

		static void setValuesInArray(double arrayToFill[], char *dataString);
		static POSE_SPECIFICATION returnProperPS(char *poseSpecification);
		static char * toString(double valArr[], int length);
		static char * toString(int numberOfPoses);
		static char * toString(POSE_SPECIFICATION ps);
		static char * returnRobotName(ROBOT_ENUM robot);

		static bool writeTrajectoryToXmlFile(char *fileName, POSE_SPECIFICATION ps, std::list<ecp_smooth_taught_in_pose> &poses);
		
		void createNewPose();
		void addPoseToTrajectory();
		void setName(char *trjName);
		char * getName() const;
		void setNumOfPoses(uint64_t numOfPoses);
		uint64_t getNumberOfPoses() const;
		void setPoseSpecification(char *poseSpecification);
		POSE_SPECIFICATION getPoseSpecification() const;
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

		std::list<ecp_smooth_taught_in_pose> * getPoses();

	private:
		char * trjName;
		uint64_t numOfPoses;		
		POSE_SPECIFICATION poseSpec;
		ecp_smooth_taught_in_pose *actPose;
		std::list<ecp_smooth_taught_in_pose> *trjPoses;
};


#endif
