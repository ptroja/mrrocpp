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

#include "base/lib/messip/messip_dataport.h"
#include "base/lib/config_types.h"



#include "base/lib/impconst.h"
#include "base/lib/configurator.h"
#include "base/lib/typedefs.h"

namespace mrrocpp {
namespace lib {

// Konstruktor obiektu - konfiguratora.
configurator::configurator(const std::string & _node, const std::string & _dir, const std::string & _section_name) :
	node(_node), dir(_dir), section_name(_section_name)
{
	if (uname(&sysinfo) == -1) {
		perror("uname");
	}

	/*mrrocpp_network_path = "/net/";
	 mrrocpp_network_path += node;
	 mrrocpp_network_path += dir;*/
	mrrocpp_network_path = "../";

	if ((ch = messip::port_connect(CONFIGSRV_CHANNEL_NAME)) == NULL) {
	}
	assert(ch);
}

void configurator::change_config_file(const std::string & _ini_file)
{
	config_query_t query, reply;

	query.key = _ini_file;
	query.flag = true;

	{
		boost::mutex::scoped_lock l(access_mutex);

		messip::port_send(this->ch, 0, 0, query, reply);
	}

	if (!reply.flag) {
		// TODO: throw
		std::cerr << "change_config_file to " << _ini_file << " failed" << std::endl;
	}
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

	} else if (_type == CONFIG_RESOURCEMAN_GLOBAL) {
		name = "/net/";
		name += value <std::string> ("node_name", _section_name);
		name += "/dev/";
		name += value <std::string> (_key, _section_name);

	} else if (_type == CONFIG_SERVER) {
		name = value <std::string> (_key, _section_name);

	} else {
		fprintf(stderr, "Nieznany argument w metodzie configuratora return_attach_point_name\n");
		throw;
	}

	// Zwrocenie atach_point'a.
	return (name);
}

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
	const std::string program_name = value <std::string> ("program_name", _section_name);

	std::string rsh_spawn_node;

	if (!exists("node_name", _section_name)) {
		rsh_spawn_node = "localhost";
	} else {

		std::string spawned_node_name = value <std::string> ("node_name", _section_name);

		if (spawned_node_name == sysinfo.nodename) {
			rsh_spawn_node = "localhost";
		} else {
			rsh_spawn_node = spawned_node_name;

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

		if (_bin_path == std::string("current")) {
			char* cwd;
			char buff[PATH_MAX + 1];

			cwd = getcwd(buff, PATH_MAX + 1);
			if (cwd == NULL) {
				perror("Blad cwd w configurator");
			}
			strcpy(bin_path, cwd);
		} else {
			strcpy(bin_path, _bin_path.c_str());

		}

	} else {

		char* cwd;
		char buff[PATH_MAX + 1];

		cwd = getcwd(buff, PATH_MAX + 1);
		if (cwd == NULL) {
			perror("Blad cwd w configurator");
		}
		strcpy(bin_path, cwd);

	}

	if (strlen(bin_path) && bin_path[strlen(bin_path) - 1] != '/') {
		strcat(bin_path, "/");
	}

	std::string opendir_path(bin_path);
	opendir_path += program_name;

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
		snprintf(process_path, sizeof(process_path), "cd %s; UI_HOST=%s %s%s %s %s %s %s", bin_path, ui_host ? ui_host : "", bin_path, program_name.c_str(), node.c_str(), dir.c_str(), _section_name.c_str(), asa.c_str());

		// create new session for separation of signal delivery
		if (setsid() == (pid_t) - 1) {
			perror("setsid()");
		}

		std::string username;

		if (!exists("username", _section_name)) {
			username = getenv("USER");
		} else {
			username = value <std::string> ("username", _section_name);
		}

		if ((rsh_spawn_node == "localhost") && (username == getenv("USER"))) {
			snprintf(process_path, sizeof(process_path), "%s%s", bin_path, program_name.c_str());
			chdir(bin_path);
			execlp(process_path, program_name.c_str(), node.c_str(), dir.c_str(), _section_name.c_str(), asa.c_str(), NULL);
		} else {
			if (!use_ssh) {
				execlp(rsh_cmd, rsh_cmd, "-l", username.c_str(), rsh_spawn_node.c_str(), process_path, NULL);
			} else {
				execlp(rsh_cmd, rsh_cmd, "-t", "-l", username.c_str(), rsh_spawn_node.c_str(), process_path, NULL);
			}
		}

	} else if (child_pid > 0) {
		printf("child %d created\n", child_pid);
	} else {
		perror("vfork()");
	}

	return child_pid;
}

configurator::~configurator()
{
	messip::port_disconnect(ch);
}

} // namespace lib
} // namespace mrrocpp
