/**
 * \file single_thread_port.h
 *
 * \date 2010
 * \author yoyek
 *
 * \brief Template class for single robot communication port
 */

#include <string>
#include <map>

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

}
}

