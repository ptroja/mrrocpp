#if !defined(__SBENCH_DATA_PORT_H)
#define __SBENCH_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include <string>
#include "const_sbench.h"

namespace mrrocpp {
namespace lib {
namespace sbench {

/*!
 * @brief SwarmItFix bench pins activation command data port
 * @ingroup sbench
 */
const std::string COMMAND_DATA_PORT = "SBENCH_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix sbench status data request port
 * @ingroup sbench
 */
const std::string REPLY_DATA_REQUEST_PORT = "SBENCH_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix bench pins state typedef
 * @ingroup sbench
 */

class pins_buffer
{
public:
	pins_buffer();

	void set_zeros();

	pins_buffer & operator =(const pins_buffer &); // operator przypisania

	bool pins_state[NUM_OF_PINS];
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Head EDP command buffer
 * @ingroup sbench
 */
struct cbuffer
{
	pins_buffer pins_buf;
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Head EDP reply buffer
 * @ingroup sbench
 */
struct rbuffer
{
	pins_buffer pins_buf;
}__attribute__((__packed__));

} // namespace sbench
}
}

#endif
