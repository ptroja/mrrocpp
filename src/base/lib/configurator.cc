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
#if defined(PROCESS_SPAWN_SPAWN)
#include "base/lib/y_spawn.h"
#endif
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

	mrrocpp_network_path = "/net/";
	mrrocpp_network_path += node;
	mrrocpp_network_path += dir;

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
		if(boost::filesystem::extension(file) == ".ini") {
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
#if defined(PROCESS_SPAWN_RSH)
	return ND_LOCAL_NODE;
#else
	return netmgr_strtond(node_name_l.c_str(), NULL);
#endif
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
	value += ini_file;

	return value;
}

std::string configurator::get_common_config_file_path() const
{
	std::string value(mrrocpp_network_path);
	value += "configs/common.ini";

	return value;
}
#endif

std::string configurator::return_default_reader_measures_path() const
{
	std::string path(mrrocpp_network_path);
	path += "msr/";

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
		value<std::string>(_key, _section_name);
	} catch (boost::property_tree::ptree_error & e) {
		return false;
	}

	return true;
}

pid_t configurator::process_spawn(const std::string & _section_name)
{
#if defined(PROCESS_SPAWN_RSH)

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

	// Sciezka do binariow.
	char bin_path[PATH_MAX];
	if (exists("binpath", _section_name)) {
		std::string _bin_path = value<std::string>("binpath", _section_name);
		strcpy(bin_path, _bin_path.c_str());
		if(strlen(bin_path) && bin_path[strlen(bin_path)-1] != '/') {
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
		if (setsid() == (pid_t) -1) {
			perror("setsid()");
		}

		if (exists("username", _section_name)) {
			std::string username = value <std::string> ("username", _section_name);

			//			fprintf(stderr, "rsh -l %s %s \"%s\"\n", username.c_str(), rsh_spawn_node.c_str(), process_path);

			execlp("rsh", "rsh", "-l", username.c_str(), rsh_spawn_node.c_str(), process_path, NULL);
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

			execlp("rsh", "rsh", rsh_spawn_node.c_str(), process_path, NULL);
		}

	} else if (child_pid > 0) {
		printf("child %d created\n", child_pid);
	} else {
		perror("vfork()");
	}

	return child_pid;
#endif
#if defined(PROCESS_SPAWN_SPAWN)
	// Identyfikator stworzonego procesu.
	int child_pid;
	// Deskryptor pliku.
	int fd;

	// printf("_section_name: %s,\n",_section_name);

	// Parametry stworzonego procesu.
	struct inheritance inherit;
	inherit.flags = SPAWN_SETGROUP;
	inherit.pgroup = SPAWN_NEWPGROUP;

	// Sciezka do binariow.
	int size = 1 + strlen("/net/") + strlen(node) +strlen(dir) + strlen("bin/");
	char * bin_path = new char[size];
	strcpy(bin_path,"/net/");
	strcat(bin_path, node);
	strcat(bin_path, dir);
	strcat(bin_path,"bin/");

	// Zlozenie lokalizacji odpalanego y_spawn_process
	size = 1 + strlen(bin_path) + strlen("y_spawn_process");
	char* spawn_process = new char[size];
	strcpy(spawn_process, bin_path);
	strcat(spawn_process,"y_spawn_process");

	// cout<<"spawn_process: "<<spawn_process<<endl;

	const int fd_map[] = {0, 1, 2};
	//printf("conf a\n");
	// Argumenty wywolania procesu.
	char *child_arg[3];
	child_arg[0]=spawn_process;
	child_arg[1]=(char*)"NET_SPAWN";
	child_arg[2]=NULL;
	//printf("conf b: %s, %s\n", child_arg[0], child_arg[1]);
	// Odpalenie y_spawn_process.
	if ((child_pid=spawn( child_arg[0], 3, fd_map, &inherit, child_arg, NULL)) ==-1)
	{
		fprintf( stderr, "Spawn of y_spawn_process failed (from PID %d): %s\n", getpid(), strerror(errno));
		// sleep(1000);
		return -1;
	}
	// Proba komunikacji z procesem odpalajacym inne procesy.
	short tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia
	while((fd = name_open(child_arg[1], 0))<0)
	if((tmp++)<lib::CONNECT_RETRY)
	delay(lib::CONNECT_DELAY);
	else {
		fprintf( stderr, "Cannot open y_spawn_process.\n");
		return -1;
	}
	//printf("conf 1\n");
	// Wiadomosci odbierane i wysylane.
	my_data_t input;
	my_reply_data_t output;
	// Parametry wywolania procesu.
	input.hdr.type=0;
	input.msg_type=1;
	// Odczytanie nazwy odpalanego pliku.
	char * spawned_program_name = value<std::string>("program_name", _section_name);
	char * spawned_node_name = value<std::string>("node_name", _section_name);

	// printf("spawned_node_name:%s\n", spawned_node_name);

	strcpy(input.node_name, spawned_node_name);
	strcpy(input.program_name_and_args, spawned_program_name);
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, node.c_str());
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, dir.c_str());
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, ini_file.c_str());
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, _section_name.c_str());
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, session_name.c_str());
	strcpy(input.binaries_path, bin_path);

	// cout<<"config_spawn: "<<input.node_name<<endl;
	// cout<<"config_spawn: "<<input.program_name_and_args<<endl;
	// cout<<"config_spawn: "<<input.binaries_path<<endl;
	// Wyslanie polecenia odpalenia procesu.
	if (MsgSend(fd, &input, sizeof(input), &output, sizeof(output))<0)
	{
		fprintf(stderr, "Send to y_spawn_process failed.\n");
		return -1;
	}

	//printf("conf 2\n");
	// Zamkniecie pliku.
	name_close(fd);
	// Zwolnienie pamieci.
	delete [] spawned_program_name;
	delete [] spawned_node_name;
	delete [] bin_path;
	delete [] spawn_process;
	// cout<<"Elo return"<<endl;
	waitpid(child_pid, NULL, WEXITED);
	// Zwrocenie wyniku.
	return output.pid;
#endif
}

#ifdef USE_MESSIP_SRR
configurator::~configurator()
{
	messip::port_disconnect(ch);
}
#endif /* USE_MESSIP_SRR */

} // namespace lib
} // namespace mrrocpp
