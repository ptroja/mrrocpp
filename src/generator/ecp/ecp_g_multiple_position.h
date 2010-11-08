/**
 * @file
 * @brief Contains declarations and definitions of the methods of multiple_position class.
 * @author rtulwin
 * @ingroup generators
 */

#ifndef _MULTIPLE_POSITION_H_
#define _MULTIPLE_POSITION_H_

#include <cstdio>

#include "base/ecp/ecp_robot.h"
#include "base/lib/trajectory_pose/trajectory_pose.h"
#include "generator/ecp/ecp_g_get_position.h"
#include "base/ecp/ecp_generator.h"
#include "generator/ecp/velocity_profile_calculator/velocity_profile.h"
#include "generator/ecp/trajectory_interpolator/trajectory_interpolator.h"
#include "base/lib/mrmath/mrmath.h"

#include <vector>

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief Base class for the motion generators interpolating between fixed trajectory points.
 *
 * @author rtulwin
 * @ingroup generators
 */
template <class Pos, class Inter, class Calc>
class multiple_position : public generator
{
protected:

	/**
	 * Vector of positions (vector of velocity profiles).
	 */
	std::vector <Pos> pose_vector;
	/**
	 * Position vector iterator.
	 */
	typename std::vector <Pos>::iterator pose_vector_iterator;
	/**
	 * Vector of coordinates.
	 */
	std::vector <std::vector <double> > coordinate_vector;
	/**
	 * Coordinate vector iterator.
	 */
	std::vector <std::vector <double> >::iterator coordinate_vector_iterator;
	/**
	 * Temporary iterator used mainly to iterate through a single position which is always of type vector<double>.
	 */
	std::vector <double>::iterator tempIter;
	/**
	 * Type of the commanded motion (absolute or relative)
	 */
	lib::MOTION_TYPE motion_type;
	/**
	 * Velocity profile calculator.
	 */
	Calc vpc;
	/**
	 * Trajectory interpolator.
	 */
	Inter inter;
	/**
	 * Number of axes for a given robot in used representation.
	 */
	int axes_num;
	/**
	 * Type of the used representation.
	 */
	lib::ECP_POSE_SPECIFICATION pose_spec;
	/**
	 * Set to true if trajectory was calculated (list of positions contains all of the information needed to start the interpolation).
	 */
	bool calculated;
	/**
	 * Set to true if list of coordinates was filled in.
	 */
	bool interpolated;
	/**
	 * Time of a single macrostep.
	 */
	double mc;
	/**
	 * Number of steps in a single macrostep.
	 */
	int nmc;
	/**
	 * If true, debug information is shown.
	 */
	bool debug;
	/**
	 * Set to true if trajectory was specified in angle axis absolute coordinates and the interpolation is performed on poses transformed into relative vectors.
	 */
	bool angle_axis_absolute_transformed_into_relative;

	//--------- VELOCITY AND ACCELERATION VECTORS ---------
	/**
	 * Standard velocity in joint coordinates.
	 */
	std::vector <double> joint_velocity;
	/**
	 * Maximal velocity in joint coordinates.
	 */
	std::vector <double> joint_max_velocity;
	/**
	 * Standard velocity in motor coordinates.
	 */
	std::vector <double> motor_velocity;
	/**
	 * Maximal velocity in motor coordinates.
	 */
	std::vector <double> motor_max_velocity;
	/**
	 * Standard velocity in euler zyz coordinates.
	 */
	std::vector <double> euler_zyz_velocity;
	/**
	 * Maximal velocity in euler zyz coordinates.
	 */
	std::vector <double> euler_zyz_max_velocity;
	/**
	 * Standard velocity in angle axis coordinates.
	 */
	std::vector <double> angle_axis_velocity;
	/**
	 * Maximal velocity in angle axis coordinates.
	 */
	std::vector <double> angle_axis_max_velocity;
	/**
	 * Standard acceleration in joint coordinates.
	 */
	std::vector <double> joint_acceleration;
	/**
	 * Maximal acceleration in joint coordinates.
	 */
	std::vector <double> joint_max_acceleration;
	/**
	 * Standard acceleration in motor coordinates.
	 */
	std::vector <double> motor_acceleration;
	/**
	 * Maximal acceleration in motor coordinates.
	 */
	std::vector <double> motor_max_acceleration;
	/**
	 * Standard acceleration in euler zyz coordinates.
	 */
	std::vector <double> euler_zyz_acceleration;
	/**
	 * Maximal acceleration in euler zyz coordinates.
	 */
	std::vector <double> euler_zyz_max_acceleration;
	/**
	 * Standard acceleration in angle axis coordinates.
	 */
	std::vector <double> angle_axis_acceleration;
	/**
	 * Maximal acceleration in angle axis coordinates.
	 */
	std::vector <double> angle_axis_max_acceleration;
	//--------- VELOCITY AND ACCELERATION VECTORS END ---------

	/**
	 * Temporary homog matrix.
	 */
	lib::Homog_matrix homog_matrix;
	/**
	 * Sets up the start position vector of the first position in the trajectory chain.
	 */
	void get_initial_position()
	{
		pose_vector_iterator = pose_vector.begin();
		if (motion_type == lib::ABSOLUTE) {
			get_position * get_pos = new get_position(ecp_t, pose_spec, axes_num); //generator used to get the actual position of the robot
			get_pos->Move();

			//---------------- DEGUG --------------------

			if (debug) {
				printf("actual position vector, size: %d\n", get_pos->get_position_vector().size());
				for (int j = 0; j < get_pos->get_position_vector().size(); j++) {
					printf("%f\t", get_pos->get_position_vector()[j]);
				}
				printf("\n");
				flushall();
			}

			//------------------ DEBUG END ---------------

			pose_vector_iterator->start_position = get_pos->get_position_vector();//get actual position of the robot
			delete get_pos;
		} else if (motion_type == lib::RELATIVE) {
			pose_vector_iterator->start_position = std::vector <double>(axes_num, 0);
		} else {
			sr_ecp_msg.message("Wrong motion type");
			throw ECP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);//TODO change the second argument
		}
	}
	/**
	 * Creates the vectors containning the information about the maximal and typical velocities for each representation.
	 */
	virtual void create_velocity_vectors(int axes_num) = 0;
	/**
	 * Performs interpolation of the trajectory. Fills in the coordinates vector.
	 * @return true if interpolation was successful
	 */
	bool interpolate()
	{

		sr_ecp_msg.message("Interpolating...");

		if (!calculated) {
			sr_ecp_msg.message("Cannot perform interpolation. Trajectory not calculated.");
			return false;
		}

		coordinate_vector.clear();
		coordinate_vector_iterator = coordinate_vector.begin();
		pose_vector_iterator = pose_vector.begin();

		int i; //loop counter

		bool trueFlag = true;//flag set to false if interpolation is not successful at some point

		if (motion_type == lib::ABSOLUTE) {
			for (i = 0; i < pose_vector.size(); i++) {//interpolate trajectory, fill in the coordinate list
				if (inter.interpolate_absolute_pose(pose_vector_iterator, coordinate_vector, mc) == false) {
					trueFlag = false;
				}
				pose_vector_iterator++;
			}
		} else if (motion_type == lib::RELATIVE && angle_axis_absolute_transformed_into_relative == true) {
			for (i = 0; i < pose_vector.size(); i++) {//interpolate trajectory, fill in the coordinate list
				if (inter.interpolate_angle_axis_absolute_pose_transformed_into_relative(pose_vector_iterator, coordinate_vector, mc) == false) {
					trueFlag = false;
				}
				pose_vector_iterator++;
			}
			set_absolute();
		} else if (motion_type == lib::RELATIVE) {
			for (i = 0; i < pose_vector.size(); i++) {//interpolate trajectory, fill in the coordinate list
				if (inter.interpolate_relative_pose(pose_vector_iterator, coordinate_vector, mc) == false) {
					trueFlag = false;
				}
				pose_vector_iterator++;
			}
		} else {
			sr_ecp_msg.message("Wrong motion type");
			throw ECP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);//TODO change the second argument
		}

		return trueFlag;
	}
	/**
	 * Calculates trajectory.
	 * @return true if calculation was successful.
	 */
	virtual bool calculate() = 0;
	/**
	 * Method used to print list of positions.
	 */
	virtual void print_pose_vector() = 0;
	/**
	 * Method used to print list of coordinates.
	 */
	virtual void print_coordinate_vector()
	{
		coordinate_vector_iterator = coordinate_vector.begin();
		printf("coordinate_vector_size: %d\n", coordinate_vector.size());
		for (int i = 0; i < coordinate_vector.size(); i++) {
			tempIter = (*coordinate_vector_iterator).begin();
			printf("%d:\t", (i + 1));
			for (tempIter = (*coordinate_vector_iterator).begin(); tempIter != (*coordinate_vector_iterator).end(); tempIter++) {
				printf(" %f\t", *tempIter);
			}
			coordinate_vector_iterator++;
			printf("\n");
		}
		flushall();
	}

public:
	/**
	 * Constructor.
	 */
	multiple_position(common::task::task& _ecp_task) :
		generator(_ecp_task)
	{
		debug = false;
		angle_axis_absolute_transformed_into_relative = false;
		motion_type = lib::ABSOLUTE;
		nmc = 10;
		mc = nmc * lib::EDP_STEP;

	}
	/**
	 * Destructor.
	 */
	virtual ~multiple_position()
	{

	}
	/**
	 * Set debug variable.
	 */
	void set_debug(bool debug)
	{
		this->debug = debug;
	}
	/**
	 * Implementation of the first_step method.
	 */
	bool first_step()
	{

		if (debug) {
			printf("\n##################################### first_step ####################################\n");
			flushall();
		}

		if (!calculated || !interpolated) {
			sr_ecp_msg.message("Trajectory not inerpolated");
			reset();
			return false;
		}

		the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
		the_robot->ecp_command.instruction.motion_steps = nmc;
		the_robot->ecp_command.instruction.value_in_step_no = nmc - 2;
		the_robot->communicate_with_edp = false;

		if (motion_type == lib::RELATIVE) {
			the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
		} else if (motion_type == lib::ABSOLUTE) {
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		} else {
			sr_ecp_msg.message("Wrong motion type");
			throw ECP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);//TODO change the second argument
		}

		switch (pose_spec)
		{
			case lib::ECP_XYZ_EULER_ZYZ:
				the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
				if (motion_type == lib::RELATIVE) {
					the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
					for (int i = 0; i < axes_num; i++) {
						the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
					}
				} else {
					the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
				}
				break;
			case lib::ECP_XYZ_ANGLE_AXIS:
				the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
				if (motion_type == lib::RELATIVE) {
					the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
					for (int i = 0; i < axes_num; i++) {
						the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
					}
				} else {
					the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
				}
				break;
			case lib::ECP_MOTOR:
				the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
				the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
				break;
			case lib::ECP_JOINT:
				the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
				the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
				break;
			default:
				reset();
				throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}

		coordinate_vector_iterator = coordinate_vector.begin();
		sr_ecp_msg.message("Moving...");
		return true;
	}
	/**
	 * Definition of the next_step method.
	 */
	bool next_step()
	{

		if (debug) {
			//printf("\n##################################### next_step ####################################\n");
			flushall();
		}

		if (coordinate_vector_iterator == coordinate_vector.end()) {
			sr_ecp_msg.message("Motion finished");
			reset();//reset the generator, set generated and calculated flags to false, flush coordinate and pose lists
			return false;
		}

		int i;//loop counter

		if (coordinate_vector.empty()) {

			sr_ecp_msg.message("No coordinates generated");
			reset();
			return false;
		}

		the_robot->communicate_with_edp = true;//turn on the communication with EDP
		the_robot->ecp_command.instruction.instruction_type = lib::SET;

		double coordinates[axes_num];

		switch (pose_spec)
		{

			case lib::ECP_JOINT:

				tempIter = (*coordinate_vector_iterator).begin();
				for (i = 0; i < axes_num; i++) {
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = *tempIter;
					if (debug) {
						printf("%f\t", *tempIter);
					}
					tempIter++;

				}
				if (debug) {
					printf("\n");
					flushall();
				}
				break;

			case lib::ECP_MOTOR:

				tempIter = (*coordinate_vector_iterator).begin();
				for (i = 0; i < axes_num; i++) {
					the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = *tempIter;
					if (debug) {
						printf("%f\t", *tempIter);
					}
					tempIter++;

				}
				if (debug) {
					printf("\n");
					flushall();
				}
				break;

			case lib::ECP_XYZ_EULER_ZYZ:

				i = 0;

				for (tempIter = (*coordinate_vector_iterator).begin(); tempIter != (*coordinate_vector_iterator).end(); tempIter++) {
					coordinates[i] = *tempIter;
					if (debug) {
						printf("%f\t", *tempIter);
					}
					i++;
				}

				if (debug) {
					printf("\n");
					flushall();
				}

				homog_matrix.set_from_xyz_euler_zyz(lib::Xyz_Euler_Zyz_vector(coordinates));
				homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

				break;

			case lib::ECP_XYZ_ANGLE_AXIS:

				i = 0;

				for (tempIter = (*coordinate_vector_iterator).begin(); tempIter != (*coordinate_vector_iterator).end(); tempIter++) {
					coordinates[i] = *tempIter;
					if (debug) {
						printf("%f\t", *tempIter);
					}
					i++;
				}

				if (debug) {
					printf("\n");
					flushall();
				}

				homog_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(coordinates));
				homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

				break;

			default:
				reset();
				throw ECP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
		}// end:switch

		coordinate_vector_iterator++;

		return true;
	}
	/**
	 * Performs calculation of the trajectory and interpolation. Fills in pose_vector and coordinate_vector.
	 * @return true if the calculation was successful
	 */
	virtual bool calculate_interpolate()
	{

		if (debug) {
			printf("\n##################################### calculate_interpolate ####################################\n");
			flushall();
		}

		if (pose_vector.empty()) {
			sr_ecp_msg.message("No loaded poses");
			return false;
		}

		get_initial_position();

		calculated = calculate();

		if (debug) {
			print_pose_vector();
		}

		interpolated = interpolate();

		if (debug) {
			print_coordinate_vector();
		}

		return calculated && interpolated;
	}
	/**
	 * Sets the number of axes in which the generator will move the robot.
	 */
	virtual void set_axes_num(int axes_num)
	{
		this->axes_num = axes_num;
		create_velocity_vectors(axes_num);
	}
	/**
	 * Sets the relative type of motion.
	 */
	void set_relative(void)
	{
		motion_type = lib::RELATIVE;
	}
	/**
	 * Sets the absolute type of motion.
	 */
	void set_absolute(void)
	{
		motion_type = lib::ABSOLUTE;
	}
	/**
	 * Clears vectors of positions and coordinates. Sets %calculated and %interpolated flags to false;
	 */
	virtual void reset()
	{
		pose_vector.clear();
		coordinate_vector.clear();
		calculated = false;
		interpolated = false;
		angle_axis_absolute_transformed_into_relative = false;
		sr_ecp_msg.message("Generator reset");
	}
	/**
	 * Detection of possible jerks. Method scans the vector of coordinates (after interpolation) and checks if the allowed acceleration was not exceeded.
	 * @param max_acc maximal allowed acceleration
	 * @return -1 if the trajectory was not interpolated before, 0 if the jerks were not detected, if yes, the number of the coordinate where the jerk was detected is returned
	 */
    virtual int detect_jerks(double max_acc)
	{
    	if (debug) {
    		printf("##################################### detect_jerks #####################################\n");
		}

    	if (!interpolated) {
    		return -1;
    	}

		coordinate_vector_iterator = coordinate_vector.begin();
		std::vector<double> temp1 = pose_vector.begin()->start_position;
		std::vector<double> temp2 = (*coordinate_vector_iterator);

		int i, j;//loop counters

		for (i = 0; i < axes_num; i++) {
			if (motion_type == lib::ABSOLUTE) {
				if ((2*fabs(temp2[i]-temp1[i]))/(mc*mc) > max_acc) {
					sr_ecp_msg.message("Possible jerk detected!");
					if (debug) {
						printf("Jerk detected in coordinates: 1\t axis: %d\n",i);
						//printf("acc: %f\n", (2*fabs(temp2[i]-temp1[i]))/(mc*mc));
						flushall();
					}
					return 1; //jerk in the first macrostep
				}
			} else if (motion_type == lib::RELATIVE) {
				if ((2*fabs(temp2[i]))/(mc*mc) > max_acc) {
					sr_ecp_msg.message("Possible jerk detected!");
					if (debug) {
						printf("Jerk detected in coordinates: 1\t axis: %d\n",i);
						//printf("acc: %f\n", (2*fabs(temp2[i]))/(mc*mc));
						flushall();
					}
					return 1; //jerk in the first macrostep
				}
			} else {
				sr_ecp_msg.message("Wrong motion type");
				throw ECP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);//TODO change the second argument
			}
		}

		coordinate_vector_iterator++;

		for (i = 1; i < coordinate_vector.size(); i++) {

			j = 0;
			for (tempIter = (*coordinate_vector_iterator).begin(); tempIter != (*coordinate_vector_iterator).end(); tempIter++) {
				if (motion_type == lib::ABSOLUTE) {
					if (fabs((fabs(temp1[j] - temp2[j])/mc) - (fabs(temp2[j] - *tempIter)/mc)) / mc  > max_acc) {
						sr_ecp_msg.message("Possible jerk detected!");
						if (debug) {
							printf("Jerk detected in coordinates: %d\t axis: %d\n", i+1, j);
							//printf("acc: %f\n", (fabs((fabs(temp1[j] - temp2[j])/mc) - (fabs(temp2[j] - *tempIter)/mc)) / mc));
							flushall();
						}
						return i+1;
					}
				} else if (motion_type == lib::RELATIVE) {
					if (fabs((fabs(temp2[j])/mc) - (fabs(*tempIter)/mc)) / mc  > max_acc) {
						sr_ecp_msg.message("Possible jerk detected!");
						if (debug) {
							printf("Jerk detected in coordinates: %d\t axis: %d\n", i+1, j);
							//printf("acc: %f\n", (fabs((fabs(temp2[j])/mc) - (fabs(*tempIter)/mc)) / mc));
							flushall();
						}
						return i+1;
					}
				} else {
					sr_ecp_msg.message("Wrong motion type");
					throw ECP_error(lib::NON_FATAL_ERROR, ECP_ERRORS);//TODO change the second argument
				}

				j++;
			}

			temp1 = temp2;
			temp2 = *coordinate_vector_iterator;

			coordinate_vector_iterator++;
		}

		flushall();

		return 0;
	}
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _MULTIPLE_POSITION_H_ */
