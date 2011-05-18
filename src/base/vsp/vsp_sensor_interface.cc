/*!
 * @file
 * @brief File containing methods of the base sensor_interface class.
 * @author ptrojane <piotr.trojanek@gmail.com>, Warsaw University of Technology
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup VSP
 */

#include <boost/shared_ptr.hpp>

#include "base/lib/sr/srlib.h"
#include "base/vsp/vsp_sensor_interface.h"

namespace mrrocpp {
namespace vsp {
namespace common {

sensor_interface::sensor_interface(lib::configurator &_config) :
	is_sensor_configured(false), is_reading_ready(false), config(_config),
	mrrocpp_network_path(config.return_mrrocpp_network_path())
{
	// SR localization -  initialization of the VSP-UI communication.
	sr_msg = (boost::shared_ptr<lib::sr_vsp>) new lib::sr_vsp(lib::VSP, config.value <std::string> ("resourceman_attach_point"), config.get_sr_attach_point(););

	sr_msg->message("Communication with SR ready");
}

void sensor_interface::wait_for_event(void)
{
}

sensor_interface::~sensor_interface()
{
	sr_msg->message("VSP  terminated");
}

} // namespace common
} // namespace vsp
} // namespace mrrocpp

