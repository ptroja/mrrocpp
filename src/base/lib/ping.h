/*!
 * @file ping
 * @brief C++ ping implementation in linux for ui-qt utilization
 *
 * @author Tomasz Winiarski
 *
 * @ingroup LIB
 */

#ifndef __PING_H
#define __PING_H

#include <string>

namespace mrrocpp {
namespace lib {

bool ping(const std::string & target);

} // namespace lib
} // namespace mrrocpp

#endif
