#if !defined(_ECP_MP_G_TFF_NOSE_RUN_H)
#define _ECP_MP_G_TFF_NOSE_RUN_H

/*!
 * @file
 * @brief File contains tff nose run label definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "../../../base/lib/com_buf.h"

namespace mrrocpp {
namespace ecp_mp {
namespace generator {

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

};

/*!
 * @brief tff nose run generator label
 */
const std::string ECP_GEN_TFF_NOSE_RUN = "ECP_GEN_TFF_NOSE_RUN";

} // namespace generator
} // namespace ecp_mp
} // namespace mrrocpp

#endif
