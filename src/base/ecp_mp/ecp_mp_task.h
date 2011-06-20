#if !defined(_ECP_MP_TASK_H)
#define _ECP_MP_TASK_H

/*!
 * @file
 * @brief File contains ecp_mp base task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>

#include <libxml/tree.h>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp_mp/ecp_mp_typedefs.h"
#include "base/lib/agent/Agent.h"

#include "base/lib/trajectory_pose/trajectory_pose.h"
#include "base/lib/trajectory_pose/bang_bang_trajectory_pose.h"
#include "base/ecp_mp/Trajectory.h"
#include "base/lib/agent/Agent.h"

/**
 * @brief Container type for storing cc_t internal agent memory boost::any objects.
 *
 * @ingroup ecp_mp
 */
typedef std::map <std::string, boost::any> cc_t;

/**
 * @brief Type for Items from cc_t container.
 *
 * @ingroup ecp_mp
 */
typedef cc_t::value_type cc_item_t;

namespace mrrocpp {
namespace ecp_mp {
namespace task {

/*!
 * @brief Base class of all ecp and mp tasks
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp_mp
 */
class task : protected Agent
{
public:
	/**
	 * @brief Container type for storing trajectory objects.
	 */
	typedef std::map <const char *, ecp_mp::common::Trajectory> trajectories_t;

	typedef std::map <std::string, std::pair<std::vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose *>, lib::MOTION_TYPE> > bang_trajectories_map;

	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	task(lib::configurator &_config);

	/**
	 * @brief Destructor
	 */
	virtual ~task();

	/**
	 * @brief the map of sensors
	 */
	sensors_t sensor_m;

	/**
	 * @brief the map of the shared memory (ccj buffer)
	 */
	cc_t cc_m;

	/**
	 * @brief the map of transmitters
	 */
	transmitters_t transmitter_m;

	/**
	 * @brief pointer to the SR communication object
	 */
	static boost::shared_ptr<lib::sr_ecp> sr_ecp_msg; // TODO: rename from _ecp_ (?!)

	/**
	 * @brief configurator object reference
	 */
	lib::configurator &config;

	/**
	 * @brief UI communication channel descriptor
	 */
	lib::fd_client_t UI_fd;

	/**
	 * @brief path to mrrocpp directory structure with network node prefix
	 */
	const std::string mrrocpp_network_path;

	/**
	 * @brief operator bool decision (Yes/No) request through UI
	 * @param question string with question to display
	 * @return bool decision
	 */
	bool operator_reaction(const char* question);

	/**
	 * @brief operator decision (1/2/3...) request through UI
	 * @param question string with question to display
	 * @param nr_of_options_input number of options to choose
	 * @return uint8_t decision
	 *
	 */
	uint8_t choose_option(const char* question, uint8_t nr_of_options_input);

	/**
	 * @brief operator integer decision request through UI
	 * @param question string with question to display
	 * @return int decision
	 */
	int input_integer(const char* question);

	/**
	 * @brief operator double decision request through UI
	 * @param question string with question to display
	 * @return double decision
	 */
	double input_double(const char* question);

	/**
	 * @brief message send to display in UI
	 * @param message string to display
	 * @return communication status
	 */
	bool show_message(const char* message);

	/**
	 * @brief calls initiate reading method for sensor from sensor map
	 * @param _sensor_m stl map of sensors
	 */
	void all_sensors_initiate_reading(sensors_t & _sensor_m);

	/**
	 * @brief calls get reading method for sensor from sensor map
	 * @param _sensor_m stl map of sensors
	 */
	void all_sensors_get_reading(sensors_t & _sensor_m);

	//ecp_mp::common::Trajectory * createTrajectory(xmlNodePtr actNode, xmlChar *stateID);

	/**
	 * @brief creates trajectory
	 * @param fileName actNode xml_node
	 * @param stateID task state id
	 * @return Trajectory pointer
	 */
	std::pair<std::vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose *>, lib::MOTION_TYPE> createTrajectory2(xmlNodePtr actNode, xmlChar *stateID, int axes_num);

	/**
	 * @brief loads trajectory
	 * @param fileName file that stores trajectory
	 * @param robot_name_t robot associated with trajectory
	 * @return trajectories_t pointer
	 */
	bang_trajectories_map * loadTrajectories(const char * fileName, lib::robot_name_t propRobot, int axes_num);
};

/*!
 * @brief ECP_MP main task error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp_mp
 */
class ECP_MP_main_error
{
public:
	/**
	 * @brief error number
	 */
	const lib::error_class_t error_class;

	/**
	 * @brief error number
	 */
	const uint64_t mp_error;

	/**
	 * @brief constructor
	 * @param err0 error class
	 * @param err1 error number
	 * @param file file_name
	 * @param line line number
	 */
	ECP_MP_main_error(lib::error_class_t err0, uint64_t err1, const char *file, int line) :
		error_class(err0), mp_error(err1)
	{
		fprintf(stderr, "ecp_MP_main_error @ %s:%d\n", file, line);
	}
#define ECP_MP_main_error(e0,e1)	ECP_MP_main_error((e0),(e1), __FILE__, __LINE__)
};

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _ECP_MP_TASK_H */
