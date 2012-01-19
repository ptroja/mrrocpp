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
 * @brief SwarmItFix bench pins preasure (cleaning) activation command data port
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
	POWER_SUPPLY, CLEANING
};

/*!
 * @brief Bench pins state.
 * @ingroup sbench
 */

class bench_state
{
private:
	friend class mrrocpp::edp::sbench::effector;

	bool pins_state[NUM_OF_PINS];

protected:
	int translation_table[8][8];

public:

	bench_state();

	//! clears translation table
	void set_all_off();

	//! sets the value due to the translation table
	void set_value(int row, int column, const bool value);

	//! Sets value on (1) on given row and column.
	void set_on(int row, int column);

	//! Sets value off (0) on given row and column.
	void set_off(int row, int column);

	//! gets the value due to the translation table
	bool get_value(int row, int column) const;

	//! checks if any value in translation table is doubled
	bool is_any_doubled_value() const;

private:
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
 * @brief Power supply state.
 * @ingroup sbench
 */

class power_supply_state : public bench_state
{
public:
	power_supply_state();

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <bench_state>(*this);
	}

};


/*!
 * @brief SwarmItFix cleaning activation state.
 * @ingroup sbench
 */
class cleaning_state : public bench_state
{
public:

	cleaning_state();

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <bench_state>(*this);
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

	power_supply_state voltage_buf;
	cleaning_state preasure_buf;

private:
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
 * @brief SwarmItFix Bench EDP command buffer derrived from c_buffer to be used in ecp edp communication
 * @ingroup sbench
 */
struct c_buffer : lib::c_buffer
{
	//! sbench specific field
	cbuffer sbench;

private:
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
	power_supply_state voltage_buf;
	cleaning_state preasure_buf;

private:
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

/*!
 * @brief SwarmItFix Bench EDP reply buffer derrived from r_buffer to be used in ecp edp communication
 * @ingroup sbench
 */

struct r_buffer : lib::r_buffer
{
	//! sbench specific field
	rbuffer sbench;

private:
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
