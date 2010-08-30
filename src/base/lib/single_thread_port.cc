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

single_thread_port_interface::single_thread_port_interface(std::string _name, single_thread_port_manager & _port_manager) :
	name(_name), new_data(false)
{
	_port_manager.add_port(this);
}

std::string single_thread_port_interface::get_name()
{
	return name;
}

single_thread_port_manager::single_thread_port_manager()
{
}

void single_thread_port_manager::add_port(single_thread_port_interface* single_thread_port_inter)
{
	single_thread_port_map[single_thread_port_inter->get_name()] = single_thread_port_inter;
}

void single_thread_port_manager::clear_data_ports()
{
	single_thread_port_interface_t single_thread_port_map_tmp;

	// przepisanie mapy robotow do skomunikowania na wersje tymczasowa
BOOST_FOREACH(const single_thread_port_interface_pair_t & port_node, single_thread_port_map_tmp)
{	port_node.second->clear_all_flags();
}

}

}
}

