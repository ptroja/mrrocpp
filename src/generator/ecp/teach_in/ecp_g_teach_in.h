#if !defined(_ECP_TEACH_IN_GENERATOR_H)
#define  _ECP_TEACH_IN_GENERATOR_H

/*!
 * @file
 * @brief File contains teach_in generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include <list>

#include "ecp_mp_g_teach_in.h"
#include "base/ecp_mp/ecp_ui_msg.h"
#include "base/ecp/ecp_generator.h"
#include "ecp_taught_in_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * @brief generator to move though the list of stored positions (poses)
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup generators
 */
class teach_in : public common::generator::generator
{

protected:
	/**
	 * @brief pose list data type
	 */
	typedef std::list <ecp_taught_in_pose> pose_list_t;

	/**
	 * @brief pose list
	 */
	pose_list_t pose_list;

	/**
	 * @brief pose list iterator
	 */
	pose_list_t::iterator pose_list_iterator;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	teach_in(common::task::task& _ecp_task);

	/**
	 * @brief Destructor
	 */
	virtual ~teach_in(void);

	/**
	 * @brief teaches trajectory
	 * communicates with UI to store the list of positions and interpose motion duration
	 * @param ps coordinates type of position (pose)
	 * @param msg message to display in UI
	 */
	void teach(lib::ECP_POSE_SPECIFICATION ps, const char* msg);

	/**
	 * @brief loads trajectory from file set by operator in UI
	 * @return operation success status
	 */
	bool load_file_from_ui();

	/**
	 * @brief loads trajectory from file of given path
	 * @param file_name file path
	 * @return operation success status
	 */
	bool load_file_with_path(const std::string & file_name);

	/**
	 * @brief save trajectory to file set by operator in UI
	 * @param ps coordinates type of position (pose)
	 */
	void save_file(lib::ECP_POSE_SPECIFICATION ps);

	/**
	 * @brief clears (flushes) pose list
	 */
	void flush_pose_list(void);

	/**
	 * @brief moves iterator to the beginning of the pose list
	 */
	void initiate_pose_list(void);

	/**
	 * @brief increments pose list iterator
	 * initially it checks if the pose list end is reached
	 */
	void next_pose_list_ptr(void);

	/**
	 * @brief returns pose pointed by the current pose list iterator
	 * @param tip ecp_taught_in_pose reference to return value
	 */
	const ecp_taught_in_pose & get_pose(void) const;

	/**
	 * @brief returns pose coordinates pointed by the current pose list iterator
	 * @param next_pose coordinates vector to return value
	 */
	void get_next_pose(double next_pose[lib::MAX_SERVOS_NR]);

	/**
	 * @brief sets pose pointed by the current pose list iterator
	 * @param ps coordinates type of position (pose)
	 * @param motion_time motion duration
	 * @param coordinates position (pose) coordinates
	 * @param extra_info optional additional information stored with position
	 */
	void
	set_pose(lib::ECP_POSE_SPECIFICATION ps, double motion_time, double coordinates[lib::MAX_SERVOS_NR], int extra_info =
			0);

	/**
	 * @brief checks if pose_list_iterator reached the pose list end
	 * @return check status
	 */
	bool is_pose_list_element(void) const;

	/**
	 * @brief checks if pose_list_iterator points last element of the pose list
	 * @return check status
	 */
	bool is_last_list_element(void) const;

	/**
	 * @brief sets pose as the pose list head
	 * @param ps coordinates type of position (pose)
	 * @param motion_time motion duration
	 * @param coordinates position (pose) coordinates
	 * @param extra_info optional additional information stored with position
	 */
	void
	create_pose_list_head(lib::ECP_POSE_SPECIFICATION ps, double motion_time, const double coordinates[lib::MAX_SERVOS_NR], int extra_info =
			0);

	/**
	 * @brief sets pose of the element pointed by pose list iterator and the increments iterator
	 * @param ps coordinates type of position (pose)
	 * @param motion_time motion duration
	 * @param coordinates position (pose) coordinates
	 * @param extra_info optional additional information stored with position
	 */
	void
	insert_pose_list_element(lib::ECP_POSE_SPECIFICATION ps, double motion_time, const double coordinates[lib::MAX_SERVOS_NR], int extra_info =
			0);

	/**
	 * @brief checks number of pose list elements
	 * @return number of pose list elements
	 */
	std::size_t pose_list_length(void) const;

	virtual bool first_step();

	virtual bool next_step();

	/**
	 * @brief converts ECP_POSE_SPECIFICATION to ECP_TO_UI_COMMAND
	 * @param ps coordinates type of position (pose)
	 * @return equivalent ECP_TO_UI_COMMAND
	 */
	lib::ECP_TO_UI_REQUEST convert(lib::ECP_POSE_SPECIFICATION ps) const;

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TEACH_IN_GENERATOR_H */
