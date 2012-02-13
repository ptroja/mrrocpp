#ifndef ECP_MP_ROBOT_H_
#define ECP_MP_ROBOT_H_

#include <boost/utility.hpp>

#include "base/lib/impconst.h"

/*!
 * @file
 * @brief File contains ecp_mp base robot declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

namespace mrrocpp {
namespace ecp_mp {

/*!
 * @brief Base class of all ecp and mp robots
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp_mp
 */
class robot : private boost::noncopyable
{
public:
	/**
	 * @brief Unique robot name
	 */
	const lib::robot_name_t robot_name; // by Y - nazwa robota (track, postument etc.)

	/**
	 * @brief Constructor
	 * @param _robot_name Unique robot name.
	 */
	robot(const lib::robot_name_t & _robot_name);

	virtual ~robot()
	{

	}
};

} // namespace ecp_mp
} // namespace mrrocpp


#endif
