/*!
 * @file taks_utils.hpp
 * @brief Classes utilized in the task execution.
 *
 * @date Jan 20, 2012
 * @author tkornuta
 */


#ifndef PLAN_HPP_
#define PLAN_HPP_

/*!
 * @brief Structure representing one bench pin.
 * @author tkornuta
 */
struct pin {
public:
	/*!
	 * Pin row.
	 */
	unsigned char row;

	/*!
	 * Pin column.
	 */
	unsigned char column;

	/*!
	 * Default constructor.
	 */
	pin() : row(0), column(0)
		{ }

	/*!
	 * Sets row and column.
	 */
	pin(unsigned char row_, unsigned char column_) : row(row_), column(column_)
		{ }

	/*!
	 * Return the pin description in Row (arabic) - Column (roman) form.
	 */
	std::string get_name() const {
		std::stringstream name;
		name << (int) row;
		name <<  "-";
		switch(column){
			case 1:
				name << "I";
				break;
			case 2:
				name << "II";
				break;
			case 3:
				name << "III";
				break;
			case 4:
				name << "IV";
				break;
			case 5:
				name << "V";
				break;
			case 6:
				name << "VI";
				break;
			case 7:
				name << "VII";
				break;
			default:
				name << "?";
				break;
		}
		return name.str();
	}

};

/*!
 * @brief Stores data related to one rotation around the leg.
 * @author tkornuta
 */
struct leg_rotation {
public:
	/*!
	 * SMB leg.
	 */
	unsigned char leg;

	/*!
	 * SMB leg.
	 */
	char rotation;

	/*!
	 * Default constructor.
	 */
	leg_rotation() : leg(0), rotation(0)
		{ }

	/*!
	 * Sets row and column.
	 */
	leg_rotation(unsigned char leg_, char rotation_) : leg(leg_), rotation(rotation_)
		{ }

	/*!
	 * Return the description.
	 */
	std::string get_name() const {
		std::stringstream name;
		name << (int)leg << " -> " << (int)rotation;
		return name.str();
	}
};


struct power_clean_pose {
	/*!
	 * Pin around we rotate (bench enumeration).
	 */
	pin rotation_pin;

	/*!
	 * First pin on which we will stand.
	 */
	pin desired_pin1;

	/*!
	 * Second pin on which we will stand.
	 */
	pin desired_pin2;

	/*!
	 * Rotation performed in order to get to the pose.
	 */
	leg_rotation rotation;

	std::string get_name() const {
		return rotation_pin.get_name() + " | " + desired_pin1.get_name() + " | " + desired_pin2.get_name();
	}

};





#endif /* PLAN_HPP_ */
