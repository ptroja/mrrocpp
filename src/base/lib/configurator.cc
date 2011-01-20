/**
 * \file configurator.cc
 *
 * \author Piotr Trojanek <piotr.trojanek@gmail.com>
 * \author Tomasz Winiarski <tomrobotics@gmail.com>
 *
 * \brief Definitions of configurator.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <cerrno>
#include <unistd.h>
#include <sys/wait.h>
#include <iostream>
#include <strings.h>
#include <sys/utsname.h>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <stdexcept>

//#include <boost/algorithm/string/trim.hpp>
//#include <boost/algorithm/string/classification.hpp>
#include <stdexcept>

#include <string>

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip_dataport.h"
#include "base/lib/config_types.h"
#else
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#endif

#if defined(__QNXNTO__)
#include <process.h>
#include <spawn.h>
#include <sys/netmgr.h>
#endif /* __QNXNTO__ */

#include "base/lib/impconst.h"
#include "base/lib/configurator.h"
#include "base/lib/typedefs.h"

namespace mrrocpp {
namespace lib {

// Konstruktor obiektu - konfiguratora.
configurator::configurator(const std::string & _node, const std::string & _dir, const std::string & _ini_file, const std::string & _section_name, const std::string & _session_name) :
	node(_node), dir(_dir), ini_file(_ini_file), session_name(_session_name), section_name(_section_name)
{
	if (uname(&sysinfo) == -1) {
		perror("uname");
	}

	/*mrrocpp_network_path = "/net/";
	mrrocpp_network_path += node;
	mrrocpp_network_path += dir;*/
	mrrocpp_network_path = "../";

#ifdef USE_MESSIP_SRR
	if ((ch = messip::port_connect(CONFIGSRV_CHANNEL_NAME)) == NULL) {
	}
	assert(ch);
#else
	file_location = get_config_file_path();
	common_file_location = get_common_config_file_path();

	read_property_tree_from_file(file_pt, file_location);
	read_property_tree_from_file(common_file_pt, common_file_location);
#endif /* USE_MESSIP_SRR */
}

#ifndef USE_MESSIP_SRR
void configurator::read_property_tree_from_file(boost::property_tree::ptree & pt, const std::string & file)
{
	try {
		if (boost::filesystem::extension(file) == ".ini") {
			boost::property_tree::read_ini(file, pt);
		} else if (boost::filesystem::extension(file) == ".xml") {
			boost::property_tree::read_xml(file, pt);
		} else {
			throw std::logic_error("unknown config file extension");
		}
	} catch (boost::property_tree::ptree_error & e) {
		std::cerr << e.what() << std::endl;
	}
}
#endif

void configurator::change_config_file(const std::string & _ini_file)
{
#ifdef USE_MESSIP_SRR
	config_query_t query, reply;

	query.key = _ini_file;
	query.flag = true;

	{
		boost::mutex::scoped_lock l(access_mutex);

		messip::port_send(this->ch,
				0, 0,
				query, reply);
	}

	if(!reply.flag) {
		// TODO: throw
		std::cerr << "change_config_file to " << _ini_file << " failed" << std::endl;
	}
#else
	boost::mutex::scoped_lock l(access_mutex);

	ini_file = _ini_file;

	file_location = get_config_file_path();
	common_file_location = get_common_config_file_path();

	read_property_tree_from_file(file_pt, file_location);
	read_property_tree_from_file(common_file_pt, common_file_location);
#endif /* USE_MESSIP_SRR */
}

bool configurator::check_config(const std::string & key) const
{
	return (exists(key.c_str()) && value <int> (key));
}

int configurator::return_node_number(const std::string & node_name_l)
{

	return ND_LOCAL_NODE;

}

std::string configurator::return_attach_point_name(config_path_type_t _type, const char* _key, const char* __section_name) const
{
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();
	std::string name;

	if (_type == CONFIG_RESOURCEMAN_LOCAL) {
		name = "/dev/";
		name += value <std::string> (_key, _section_name);
		name += session_name;

	} else if (_type == CONFIG_RESOURCEMAN_GLOBAL) {
		name = "/net/";
		name += value <std::string> ("node_name", _section_name);
		name += "/dev/";
		name += value <std::string> (_key, _section_name);
		name += session_name;

	} else if (_type == CONFIG_SERVER) {
		name = value <std::string> (_key, _section_name);
		name += session_name;

	} else {
		fprintf(stderr, "Nieznany argument w metodzie configuratora return_attach_point_name\n");
		throw;
	}

	// Zwrocenie atach_point'a.
	return (name);
}

#ifndef USE_MESSIP_SRR
std::string configurator::get_config_file_path() const
{
	std::string value(mrrocpp_network_path);
	//value += "configs/";
	value += "../";
	value += ini_file;

	return value;
}

std::string configurator::get_common_config_file_path() const
{
	std::string value(mrrocpp_network_path);
	value += "../configs/common.ini";

	return value;
}
#endif

std::string configurator::return_default_reader_measures_path() const
{
	std::string path(mrrocpp_network_path);
	path += "../msr/";

	return path;
}

std::string configurator::return_mrrocpp_network_path() const
{
	return mrrocpp_network_path;
}

bool configurator::exists(const char* _key, const char* __section_name) const
{
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();

	try {
		value <std::string> (_key, _section_name);
	} catch (boost::property_tree::ptree_error & e) {
		return false;
	}

	return true;
}

pid_t configurator::process_spawn(const std::string & _section_name)
{

	std::string spawned_program_name = value <std::string> ("program_name", _section_name);
	std::string spawned_node_name = value <std::string> ("node_name", _section_name);

	std::string rsh_spawn_node;

	if (spawned_node_name == sysinfo.nodename) {
		rsh_spawn_node = "localhost";
	} else {
		rsh_spawn_node = spawned_node_name;

		std::string opendir_path("/net/");
		opendir_path += rsh_spawn_node;

		if (access(opendir_path.c_str(), R_OK) != 0) {
			printf("spawned node absent: %s\n", opendir_path.c_str());
			//throw std::logic_error("spawned node absent: " + opendir_path);
		}
	}

	bool use_ssh;

	// Use SSH ?
	if (exists("use_ssh", _section_name)) {
		use_ssh = value <bool> ("use_ssh", _section_name);
	} else {
		use_ssh = false;
	}

	const char * rsh_cmd = (use_ssh) ? "ssh" : "rsh";

	// Sciezka do binariow.
	char bin_path[PATH_MAX];
	if (exists("binpath", _section_name)) {
		std::string _bin_path = value <std::string> ("binpath", _section_name);
		strcpy(bin_path, _bin_path.c_str());
		if (strlen(bin_path) && bin_path[strlen(bin_path) - 1] != '/') {
			strcat(bin_path, "/");
		}

	} else {
		snprintf(bin_path, sizeof(bin_path), "/net/%s%sbin/", node.c_str(), dir.c_str());
	}

	std::string opendir_path(bin_path);
	opendir_path += spawned_program_name;

	if (access(opendir_path.c_str(), R_OK) != 0) {
		printf("spawned program absent: %s\n", opendir_path.c_str());
		throw std::logic_error("spawned program absent: " + opendir_path);
	}

	pid_t child_pid = vfork();

	if (child_pid == 0) {

		//ewentualne dodatkowe argumenty wywolania np. przekierowanie na konsole
		std::string asa;
		if (exists("additional_spawn_argument", lib::UI_SECTION)) {
			asa = value <std::string> ("additional_spawn_argument", lib::UI_SECTION);
		}

		char process_path[PATH_MAX];
		char *ui_host = getenv("UI_HOST");
		snprintf(process_path, sizeof(process_path), "cd %s; UI_HOST=%s %s%s %s %s %s %s %s %s", bin_path, ui_host ? ui_host : "", bin_path, spawned_program_name.c_str(), node.c_str(), dir.c_str(), ini_file.c_str(), _section_name.c_str(), session_name.length() ? session_name.c_str() : "\"\"", asa.c_str());

		// create new session for separation of signal delivery
		if (setsid() == (pid_t) - 1) {
			perror("setsid()");
		}

		if (exists("username", _section_name)) {
			std::string username = value <std::string> ("username", _section_name);

			//fprintf(stderr, "rsh -l %s %s \"%s\"\n", username.c_str(), rsh_spawn_node.c_str(), process_path);
			if (!use_ssh) {
				execlp(rsh_cmd, rsh_cmd, "-l", username.c_str(), rsh_spawn_node.c_str(), process_path, NULL);
			} else {
				execlp(rsh_cmd, rsh_cmd, "-t", "-l", username.c_str(), rsh_spawn_node.c_str(), process_path, NULL);
			}
		} else {
			//			printf("rsh %s \"%s\"\n", rsh_spawn_node.c_str(), process_path);

			//			fprintf(stderr,
			//					"bin_path ->%s<-\n"
			//					"ui_host ->%s<-\n"
			//					"spawned_program_name ->%s<-\n"
			//					"node ->%s<-\n"
			//					"dir ->%s<-\n"
			//					"ini_file ->%s<-\n"
			//					"_section_name ->%s<-\n"
			//					"session_name ->%s<-\n"
			//					"asa ->%s<-\n",
			//					bin_path, ui_host ? ui_host : "",
			//					spawned_program_name.c_str(),
			//					node.c_str(), dir.c_str(), ini_file.c_str(), _section_name,
			//					session_name.length() ? session_name.c_str() : "\"\"",
			//					asa.c_str()
			//			);
			//fprintf(stderr, "rsh %s \"%s\"\n", rsh_spawn_node.c_str(), process_path);
			if (!use_ssh) {
				execlp(rsh_cmd, rsh_cmd, rsh_spawn_node.c_str(), process_path, NULL);
			} else {
				execlp(rsh_cmd, rsh_cmd, "-t", rsh_spawn_node.c_str(), process_path, NULL);
			}
		}

	} else if (child_pid > 0) {
		printf("child %d created\n", child_pid);
	} else {
		perror("vfork()");
	}

	return child_pid;

}

#ifdef USE_MESSIP_SRR
configurator::~configurator()
{
	messip::port_disconnect(ch);
}
#endif /* USE_MESSIP_SRR */

} // namespace lib
} // namespace mrrocpp
