/*!
 * @file
 * @brief File containing methods of the base sensor_interface class.
 * @author ptrojane <piotr.trojanek@gmail.com>, Warsaw University of Technology
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup VSP
 */

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
	sr_msg = new lib::sr_vsp(lib::VSP, config.value <std::string> ("resourceman_attach_point"), config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", lib::UI_SECTION), true);

	sr_msg->message("Communication with SR ready");
}

void sensor_interface::wait_for_event(void)
{
}

sensor_interface::~sensor_interface()
{
	sr_msg->message("VSP  terminated");
	delete sr_msg;
}

} // namespace common
} // namespace vsp
} // namespace mrrocpp

