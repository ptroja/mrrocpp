/*!
 * @file
 * @brief File contains template class definition for a single thread communication ports
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup lib
 */

#include <boost/foreach.hpp>

#include "single_thread_port.h"

namespace mrrocpp {
namespace lib {

single_thread_port_interface::single_thread_port_interface(const std::string & _name, single_thread_port_manager & _port_manager) :
		name(_name), new_data(false)
{
	_port_manager.add_port(this);
}

const std::string & single_thread_port_interface::get_name() const
{
	return name;
}

void single_thread_port_manager::add_port(single_thread_port_interface* port_iface)
{
	single_thread_port_map[port_iface->get_name()] = port_iface;
}

void single_thread_port_manager::clear_data_ports()
{
	// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
	BOOST_FOREACH(const single_thread_port_interface_pair_t & port_node, single_thread_port_map)
			{
				port_node.second->clear_all_flags();
			}
}

}
}

