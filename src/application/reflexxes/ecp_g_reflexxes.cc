#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/ecp/ecp_robot.h"
#include "base/lib/sr/srlib.h"
#include "ecp_g_reflexxes.h"

#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

reflexxes::reflexxes(common::task::task& _ecp_task) :
		common::generator::generator(_ecp_task)
{
	// Setup RML objects.
	RML = new ReflexxesAPI(the_robot->number_of_servos, lib::EDP_STEP*10);
	IP = new RMLPositionInputParameters(the_robot->number_of_servos);
	OP = new RMLPositionOutputParameters(the_robot->number_of_servos);
}

reflexxes::~reflexxes()
{
	// Destroy RML objects.
	delete OP;
	delete IP;
	delete RML;
}

void reflexxes::set_goal_pose(const double goal_joint_coordinates[])
{
	// Copy goal pose to internal cache.
	for(int i = 0; i < the_robot->number_of_servos; ++i) {
		goal[i] = goal_joint_coordinates[i];
	}
}

void reflexxes::showOTG(int verbose) const
{
	if(verbose > 0) {
		fprintf(stderr, "-------------------------------------------------------\n");
		fprintf(stderr, "General information:\n\n");

		fprintf(stderr, "The execution time of the current trajectory is %.3lf seconds.\n", OP->GetSynchronizationTime());

		if (OP->IsTrajectoryPhaseSynchronized()) {
			fprintf(stderr, "The current trajectory is phase-synchronized.\n");
		} else {
			fprintf(stderr, "The current trajectory is time-synchronized.\n");
		}
		if (OP->WasACompleteComputationPerformedDuringTheLastCycle()) {
			fprintf(stderr, "The trajectory was computed during the last computation cycle.\n");
		} else {
			fprintf(stderr, "The input values did not change, and a new computation of the trajectory parameters was not required.\n");
		}
	}

	if(verbose > 1) {
		// Get number of joints.
		const int NUMBER_OF_DOFS = the_robot->number_of_servos;

		int i, j;

		fprintf(stderr, "-------------------------------------------------------\n");
		fprintf(stderr, "New state of motion:\n\n");

		fprintf(stderr, "New position/pose vector                  : ");
		for (j = 0; j < NUMBER_OF_DOFS; j++) {
			fprintf(stderr, "%10.3lf ", OP->NewPositionVector->VecData[j]);
		}
		fprintf(stderr, "\n");
		fprintf(stderr, "New velocity vector                       : ");
		for (j = 0; j < NUMBER_OF_DOFS; j++) {
			fprintf(stderr, "%10.3lf ", OP->NewVelocityVector->VecData[j]);
		}
		fprintf(stderr, "\n");
		fprintf(stderr, "New acceleration vector                   : ");
		for (j = 0; j < NUMBER_OF_DOFS; j++) {
			fprintf(stderr, "%10.3lf ", OP->NewAccelerationVector->VecData[j]);
		}
		fprintf(stderr, "\n");
		fprintf(stderr, "-------------------------------------------------------\n");
		fprintf(stderr, "Extremes of the current trajectory:\n");

		for (i = 0; i < NUMBER_OF_DOFS; i++) {
			fprintf(stderr, "\n");
			fprintf(stderr, "Degree of freedom                         : %d\n", i);
			fprintf(stderr, "Minimum position                          : %10.3lf\n", OP->MinPosExtremaPositionVectorOnly->VecData[i]);
			fprintf(stderr, "Time, at which the minimum will be reached: %10.3lf\n", OP->MinExtremaTimesVector->VecData[i]);
			fprintf(stderr, "Position/pose vector at this time         : ");
			for (j = 0; j < NUMBER_OF_DOFS; j++) {
				fprintf(stderr, "%10.3lf ", OP->MinPosExtremaPositionVectorArray[i]->VecData[j]);
			}
			fprintf(stderr, "\n");
			fprintf(stderr, "Velocity vector at this time              : ");
			for (j = 0; j < NUMBER_OF_DOFS; j++) {
				fprintf(stderr, "%10.3lf ", OP->MinPosExtremaVelocityVectorArray[i]->VecData[j]);
			}
			fprintf(stderr, "\n");
			fprintf(stderr, "Acceleration vector at this time          : ");
			for (j = 0; j < NUMBER_OF_DOFS; j++) {
				fprintf(stderr, "%10.3lf ", OP->MinPosExtremaAccelerationVectorArray[i]->VecData[j]);
			}
			fprintf(stderr, "\n");
			fprintf(stderr, "Maximum position                          : %10.3lf\n", OP->MaxPosExtremaPositionVectorOnly->VecData[i]);
			fprintf(stderr, "Time, at which the maximum will be reached: %10.3lf\n", OP->MaxExtremaTimesVector->VecData[i]);
			fprintf(stderr, "Position/pose vector at this time         : ");
			for (j = 0; j < NUMBER_OF_DOFS; j++) {
				fprintf(stderr, "%10.3lf ", OP->MaxPosExtremaPositionVectorArray[i]->VecData[j]);
			}
			fprintf(stderr, "\n");
			fprintf(stderr, "Velocity vector at this time              : ");
			for (j = 0; j < NUMBER_OF_DOFS; j++) {
				fprintf(stderr, "%10.3lf ", OP->MaxPosExtremaVelocityVectorArray[i]->VecData[j]);
			}
			fprintf(stderr, "\n");
			fprintf(stderr, "Acceleration vector at this time          : ");
			for (j = 0; j < NUMBER_OF_DOFS; j++) {
				fprintf(stderr, "%10.3lf ", OP->MaxPosExtremaAccelerationVectorArray[i]->VecData[j]);
			}
			fprintf(stderr, "\n");
		}
		fprintf(stderr, "-------------------------------------------------------\n");
	}
}

bool reflexxes::first_step()
{
	sr_ecp_msg.message("first_step()");

	the_robot->ecp_command.instruction_type = lib::GET;

	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.set_type = ARM_DEFINITION;

	the_robot->ecp_command.set_arm_type = lib::JOINT;
	the_robot->ecp_command.get_arm_type = lib::JOINT;

	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.interpolation_type = lib::MIM;
	the_robot->ecp_command.motion_steps = 10;
	the_robot->ecp_command.value_in_step_no = 8;

	// Setup parameters.
	for (unsigned int i = 0; i < the_robot->number_of_servos; ++i) {
		// Stand-still conditions.
		IP->CurrentVelocityVector->VecData[i] = 0.0;
		IP->CurrentAccelerationVector->VecData[i] = 0.0;

		// Limit values.
		IP->MaxVelocityVector->VecData[i] = M_PI/20.0;
		IP->MaxAccelerationVector->VecData[i] = M_PI/5.0;
		IP->MaxJerkVector->VecData[i] = 100.0;

		// Take the axis into account when synchronizing motion.
		IP->SelectionVector->VecData[i] = true;

		IP->TargetPositionVector->VecData[i] = goal[i];
		IP->TargetVelocityVector->VecData[i] = 0.0;
	}

	// Initially there is no good data about current pose.
	fresh = false;
	display = true;

    // ********************************************************************
    // Setting the flag for time- and phase-synchronization:
    //
    //  - RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION for
    //    time-synchronization
    //  - RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE for
    //    phase-synchronization
    //
    // Please feel free to change this flag to see the difference in the
    // behavior of the algorithm.

    Flags.SynchronizationBehavior	=	RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;

	return true;
}

bool reflexxes::next_step()
{
	if (check_and_null_trigger()) {
		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji.
	the_robot->ecp_command.instruction_type = lib::SET_GET;

	if(!fresh) {
		fresh = true;

		for (unsigned int i = 0; i < the_robot->number_of_servos; ++i) {
			IP->CurrentPositionVector->VecData[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
			the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
			printf("%f\t", the_robot->reply_package.arm.pf_def.arm_coordinates[i]);
		}
		printf("\n");

	    if (IP->CheckForValidity())
	    {
	        fprintf(stderr, "Input values are valid!\n");
	    }
	    else
	    {
	        fprintf(stderr, "Input values are INVALID!\n");

	        return false;
	    }
	}

	// Call the OTG algorithm.
	ResultValue = RML->RMLPosition(*IP, OP, Flags);

    // Display trajectory parameters.
    if (display) {
    	showOTG(1);

    	display = false;
    }

	if (ResultValue < 0) {
		fprintf(stderr, "An error occurred (%d).\n", ResultValue);

		if(ResultValue != ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION){
			return false;
		}
	}

	// ****************************************************************
	// Feed the output values of the current control cycle back to
	// input values of the next control cycle

	*IP->CurrentPositionVector = *OP->NewPositionVector;
	*IP->CurrentVelocityVector = *OP->NewVelocityVector;
	*IP->CurrentAccelerationVector = *OP->NewAccelerationVector;

	// Setup command.
	for (unsigned int i = 0; i < the_robot->number_of_servos; i++) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = OP->NewPositionVector->VecData[i];
	}

	return (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED);
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
