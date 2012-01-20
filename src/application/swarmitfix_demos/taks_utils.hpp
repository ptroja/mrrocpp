/*!
 * @file taks_utils.hpp
 * @brief Classes utilized in the task execution.
 *
 * @date Jan 20, 2012
 * @author tkornuta
 */

#ifndef TASK_UTILS_HPP_
#define TASK_UTILS_HPP_

#include "robot/sbench/dp_sbench.h"

/*!
 * @brief Stores data related to rotation of PKM around the leg.
 * @author tkornuta
 */
struct pkm_leg_rotation
{
public:
	/*!
	 * Leg number [1,2,3].
	 */
	unsigned char leg;

	/*!
	 * Rotation in external values (-6,-5,...,5,6).
	 */
	char rotation;

	/*!
	 * Default constructor.
	 */
	pkm_leg_rotation() :
			leg(0), rotation(0)
	{
	}

	/*!
	 * Sets row and column.
	 */
	pkm_leg_rotation(unsigned char leg_, char rotation_) :
			leg(leg_), rotation(rotation_)
	{
	}

	/*!
	 * Returns the rotation description.
	 */
	std::string get_description() const
	{
		std::stringstream name;
		name << "rotation around " << (int) leg << " by " << (int) rotation;
		return name.str();
	}
};

/*!
 * Stores data utilized for execution of one SMB rotation around leg with control of the power from the bench.
 */
class power_smb_move
{
public:
	/*!
	 * Start pose of the agent on the bench.
	 */
	lib::sbench::bench_pose start_pose;

	/*!
	 * Final pose of the agent on the bench.
	 */
	lib::sbench::bench_pose final_pose;

	/*!
	 * Rotation performed by PKM around given leg in order to get to the final pose.
	 */
	pkm_leg_rotation rotation_leg;

	/*!
	 * Pin used in both start and final poses - the one will be powered during rotation.
	 * It is computed automatically during the object construction.
	 */
	lib::sbench::pin rotation_pin;

	/*!
	 * Default constructor.
	 */
	/*	power_smb_move() : start_pose(), final_pose(), rotation_leg()
	 { }*/
	/*!
	 * Initializes the start and final poses, as well as the leg rotation. Sets the rotation pin.
	 */
	power_smb_move(const lib::sbench::bench_pose & start_pose_, const lib::sbench::bench_pose & final_pose_, const pkm_leg_rotation & rotation_leg_) :
			start_pose(start_pose_), final_pose(final_pose_), rotation_leg(rotation_leg_)
	{
		// Find rotation pin.
		validate_rotation_pin();
	}

	/*!
	 * Returns the power move description.
	 */
	std::string get_description() const
	{
		std::stringstream name;
		name << start_pose.get_description() << " -> " << rotation_leg.get_description() << " -> "
				<< final_pose.get_description();
		return name.str();
	}

private:
	/*
	 * Validates rotation pin.
	 */
	void validate_rotation_pin()
	{
		// Compare start and final pose pins, taking into consideration the leg that we are going to rotate around.
		// Assure that the pin was found.
		if ((final_pose.pins[rotation_leg.leg - 1].column != start_pose.pins[rotation_leg.leg - 1].column)
				|| (final_pose.pins[rotation_leg.leg - 1].row != start_pose.pins[rotation_leg.leg - 1].row)) {
			std::cerr << get_description() << std::endl;
			assert(0);
		}
		// Rotation pin found.
		rotation_pin = start_pose.pins[rotation_leg.leg - 1];
	}
};

#endif /* TASK_UTILS_HPP_ */
