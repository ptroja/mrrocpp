#if !defined(__SBENCH_DATA_PORT_H)
#define __SBENCH_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include <string>
#include <boost/serialization/serialization.hpp>
#include "const_sbench.h"
#include "../../base/lib/com_buf.h"

namespace mrrocpp {
namespace edp {
namespace sbench {
class effector;
}
}

namespace lib {
namespace sbench {

/*!
 * @brief SwarmItFix bench pins voltage activation command data port
 * @ingroup sbench
 */
const std::string COMMAND_DATA_VOLTAGE_PORT = "SBENCH_COMMAND_VOLTAGE_DATA_PORT";

/*!
 * @brief SwarmItFix bench pins preasure activation command data port
 * @ingroup sbench
 */
const std::string COMMAND_DATA_PREASURE_PORT = "SBENCH_COMMAND_PREASURE_DATA_PORT";

/*!
 * @brief SwarmItFix sbench status data request port
 * @ingroup sbench
 */
const std::string REPLY_DATA_REQUEST_PORT = "SBENCH_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix Head EDP command buffer variant
 * @ingroup shead
 */
enum CBUFFER_VARIANT
{
	VOLTAGE, PREASURE
};

/*!
 * @brief SwarmItFix bench pins state typedef
 * @ingroup sbench
 */

class pins_buffer
{
	friend class mrrocpp::edp::sbench::effector;

private:
	bool pins_state[NUM_OF_PINS];

public:

	pins_buffer();

	void set_zeros();

	void set_value(int row, int column, int value);
	bool get_value(int row, int column);

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & pins_state;
	}

};

/*!
 * @brief SwarmItFix Bench EDP command buffer
 * @ingroup sbench
 */
struct cbuffer
{
	//! Variant of the command
	CBUFFER_VARIANT variant;

	pins_buffer voltage_buf;
	pins_buffer preasure_buf;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & variant;
		ar & voltage_buf;
		ar & preasure_buf;
	}

};

/*!
 * @brief SwarmItFix Bench EDP command buffer
 * @ingroup sbench
 */
struct c_buffer : lib::c_buffer
{
	cbuffer sbench;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <lib::c_buffer>(*this);
		ar & sbench;
	}

};

/*!
 * @brief SwarmItFix Bench EDP reply buffer
 * @ingroup sbench
 */
struct rbuffer : lib::r_buffer
{
	pins_buffer voltage_buf;
	pins_buffer preasure_buf;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & voltage_buf;
		ar & preasure_buf;
	}

};

struct r_buffer : lib::r_buffer
{
	rbuffer sbench;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		// serialize base class informationZ
		ar & boost::serialization::base_object <lib::r_buffer>(*this);
		ar & sbench;
	}

};

} // namespace sbench
}
}

#endif
