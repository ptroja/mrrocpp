#if !defined(_ECP_MP_ST_TFF_NOSE_RUN_H)
#define _ECP_MP_ST_TFF_NOSE_RUN_H

/*!
 * @file
 * @brief File contains ECP_ST_TFF_NOSE_RUN definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup subtasks
 */

#include "../base/lib/com_buf.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sub_task {

/**
 * @brief Enum to define mp to ecp communication variants
 */
enum communication_type
{
	no_data = 0, behaviour_specification = 1
};

class behaviour_specification_data_type
{
public:

	lib::BEHAVIOUR_SPECIFICATION behaviour[6];

	void set_compliance(bool x, bool y, bool z, bool ax, bool ay, bool az);

	behaviour_specification_data_type();
	behaviour_specification_data_type(bool x, bool y, bool z, bool ax, bool ay, bool az);

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		// serialize base class information
		ar & behaviour;
	}

};

/*!
 * @brief tff nose run ecp subtask label
 */
const std::string ECP_ST_TFF_NOSE_RUN = "ECP_ST_TFF_NOSE_RUN";

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif
