#if !defined(_TRANSMITTER_H)
#define _TRANSMITTER_H

/*!
 * @file
 * @brief File contains ecp_mp transmitter base class declaration
 * @author ptroja, Warsaw University of Technology
 *
 * @ingroup ecp_mp
 */

#include "base/lib/impconst.h"
#include "base/lib/sr/sr_ecp.h"

namespace mrrocpp {
namespace ecp_mp {
namespace task {
// XXX Forward declaration
class task;
}
namespace transmitter {

/*!
 * @brief Base abstract class of all ecp_mp transmitters
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp_mp
 */
class transmitter_base
{
public:
	/**
	 * @brief Unique transmitter name
	 */
	const lib::TRANSMITTER_t transmitter_name; // nazwa czujnika z define w impconst.h

protected:
	/**
	 * @brief sr communication object reference
	 */
	lib::sr_ecp &sr_ecp_msg;

public:
	/**
	 * @brief Constructor
	 * @param _transmitter_name Unique transmitter name.
	 * @param _section_name configuration section name
	 * @param _ecp_mp_object ecp_mp task object reference
	 */
			transmitter_base(lib::TRANSMITTER_t _transmitter_name, const std::string & _section_name, task::task& _ecp_mp_object);

	/**
	 * @brief Destructor
	 */
	virtual ~transmitter_base();

	/**
	 * @brief reads data
	 */
	virtual bool t_read(bool wait);

	/**
	 * @brief writes data
	 */
	virtual bool t_write(void);

};

/*!
 * @brief ECP_MP transmitter_error error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp_mp
 */
class transmitter_error
{
public:
	/**
	 * @brief error number
	 */
	const lib::error_class_t error_class;

	/**
	 * @brief constructor
	 * @param err_cl error class
	 */
	transmitter_error(lib::error_class_t err_cl);

};

/*!
 * @brief Template ecp_mp transmitter class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp_mp
 */
template <typename TO_VA, typename FROM_VA>
class transmitter : public transmitter_base
{
public:
	/**
	 * @brief Template output buffer
	 */
	TO_VA to_va;

	/**
	 * @brief Template input buffer
	 */
	FROM_VA from_va;

	/**
	 * @brief Constructor
	 * @param _transmitter_name Unique transmitter name.
	 * @param _section_name configuration section name
	 * @param _ecp_mp_object ecp_mp task object reference
	 */
	transmitter(lib::TRANSMITTER_t _transmitter_name, const std::string & _section_name, task::task& _ecp_mp_object) :
		transmitter_base(_transmitter_name, _section_name, _ecp_mp_object)
	{
	}
};

} // namespace transmitter


} // namespace ecp_mp
} // namespace mrrocpp

#endif /* _TRANSMITTER_H */
