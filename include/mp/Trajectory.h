
#if !defined (_TRAJECTORY_H_)
#define _TRAJECTORY_H_

#include <list>

#include "common/impconst.h"
#include "common/com_buf.h"

class Trajectory
{
	public:
		typedef struct {
			double startVelocity[MAX_SERVOS_NR];
			double endVelocity[MAX_SERVOS_NR];
			double velocity[MAX_SERVOS_NR];
			double accelerations[MAX_SERVOS_NR];
			double coordinates[MAX_SERVOS_NR];
		} Pose;
		struct cmp_str{
			bool operator()(char const *a, char const *b)
			{	return std::strcmp(a,b)<0;	}
		};

	public:
		Trajectory();
		Trajectory(char *numOfPoses, char *trajectoryName, char *poseSpecification);
		Trajectory(const Trajectory &trajectory);
		~Trajectory();

		static void setValuesInArray(double arrayToFill[], char *dataString);
		static POSE_SPECIFICATION returnProperPS(char *poseSpecification);
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

		std::list<Pose> * getPoses();

	private:
		char * trjName;
		uint64_t numOfPoses;		
		POSE_SPECIFICATION poseSpec;
		Pose *actPose;
		std::list<Pose> *trjPoses;
};


#endif
