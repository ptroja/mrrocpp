// -------------------------------------------------------------------------
// Plik:			configurator.cc
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Plik zawiera definicje matod klasy lib::configurator - obsluga konfiguracji z pliku INI.
// Autor:		tkornuta
// Data:		10.11.2005
// -------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <sys/wait.h>
#include <iostream>
#include <strings.h>
#include <sys/utsname.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#include <string>

#if defined(__QNXNTO__)
#include <process.h>
#include <spawn.h>
#include <sys/netmgr.h>
#endif /* __QNXNTO__ */

#include "lib/impconst.h"
#include "lib/configurator.h"
#if defined(PROCESS_SPAWN_SPAWN)
#include "lib/y_spawn.h"
#endif
#include "lib/messip/messip_dataport.h"
#include "lib/config_types.h"
#include "lib/typedefs.h"

namespace mrrocpp {
namespace lib {

// Konstruktor obiektu - konfiguratora.
configurator::configurator (
		const std::string & _node,
		const std::string & _dir,
		const std::string & _ini_file,
		const std::string & _section_name,
		const std::string & _session_name) :
	node(_node),
	dir(_dir),
	ini_file(_ini_file),
	session_name(_session_name),
	section_name(_section_name)
{
	if( uname( &sysinfo ) == -1 ) {
		perror( "uname" );
	}

	mrrocpp_network_path = "/net/";
	mrrocpp_network_path += node;
	mrrocpp_network_path += dir;

#ifdef USE_MESSIP_SRR
	if ((ch = messip::port_connect(CONFIGSRV_CHANNEL_NAME)) == NULL) {
	}
	assert(ch);
#else
	file_location = return_ini_file_path();
	common_file_location = return_common_ini_file_path();
#endif /* USE_MESSIP_SRR */
}// : configurator

void configurator::change_ini_file (const std::string & _ini_file)
{
#ifdef USE_MESSIP_SRR
	configfile_t configfile;
	snprintf(configfile, sizeof(configfile), "%s", _ini_file.c_str());

	boost::mutex::scoped_lock l(file_mutex);

	messip::port_send_sync(this->ch,
			CONFIG_CHANGE_INI_FILE, 0,
			configfile);
#else
	boost::mutex::scoped_lock l(file_mutex);

	ini_file =_ini_file;

	file_location = return_ini_file_path();
	common_file_location = return_common_ini_file_path();
#endif /* USE_MESSIP_SRR */
}

// Zwraca numer wezla.
int configurator::return_node_number(const std::string & node_name_l)
{
#if defined(PROCESS_SPAWN_RSH)
	return ND_LOCAL_NODE;
#else
	return netmgr_strtond(node_name_l.c_str(), NULL);
#endif
}// : return_node_number


// Zwraca attach point'a dla serwerow.
std::string configurator::return_attach_point_name (config_path_type_t _type, const char* _key, const char* __section_name)
{
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();
	std::string name;

	if (_type == CONFIG_RESOURCEMAN_LOCAL)
	{
		name = "/dev/";
		name += return_string_value(_key, _section_name);
		name += session_name;

	} else if (_type == CONFIG_RESOURCEMAN_GLOBAL)
	{
		name = "/net/";
		name += return_string_value("node_name", _section_name);
		name += "/dev/";
		name += return_string_value(_key, _section_name);
		name += session_name;

	} else if (_type == CONFIG_SERVER)
	{
		name = return_string_value(_key, _section_name);
		name += session_name;

	} else {
		fprintf(stderr, "Nieznany argument w metodzie configuratora return_attach_point_name\n");
		throw;
	}

	// Zwrocenie atach_point'a.
	return (name);
}// : return_created_resourceman_attach_point_name

#ifndef USE_MESSIP_SRR
// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
std::string configurator::return_ini_file_path()
{
	std::string value(mrrocpp_network_path);
	value += "configs/";
	value += ini_file;

	return value;
}

// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego z konfiguracja domyslna (common.ini)
std::string configurator::return_common_ini_file_path()
{
	std::string value(mrrocpp_network_path);
	value += "configs/common.ini";

	return value;
}
#endif

// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
std::string configurator::return_default_reader_measures_path()
{
	std::string path(mrrocpp_network_path);
	path += "msr/";

	return path;
}

// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
std::string configurator::return_mrrocpp_network_path()
{
	return mrrocpp_network_path;
}

// Zwraca czy dany klucz istnieje
bool configurator::exists(const char* _key, const char* __section_name)
{
#ifdef USE_MESSIP_SRR
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();

	query_t query;
	snprintf(query.key, sizeof(query.key), "%s", _key);
	snprintf(query.section, sizeof(query.section), "%s", _section_name);

	bool value;

	boost::mutex::scoped_lock l(file_mutex);

	messip::port_send(this->ch,
			CONFIG_EXISTS, 0,
			query, value);

	return value;
#else
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();
	int value;
	struct Config_Tag configs[] = {
			// Pobierane pole.
			{ (char *) _key, Int_Tag, &value},
			// Pole konczace.
			{ NULL , Error_Tag, NULL }
	};

	boost::mutex::scoped_lock l(file_mutex);

	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			// Zwolnienie pamieci.

			return false;
		}
	}

	return true;
#endif /* USE_MESSIP_SRR */
}

// Zwraca wartosc (int) dla klucza.
int configurator::return_int_value(const char* _key, const char* __section_name)
{
#ifdef USE_MESSIP_SRR
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();

	query_t query;
	snprintf(query.key, sizeof(query.key), "%s", _key);
	snprintf(query.section, sizeof(query.section), "%s", _section_name);

	int value = 0;

	boost::mutex::scoped_lock l(file_mutex);

	messip::port_send(this->ch,
			CONFIG_RETURN_INT_VALUE, 0,
			query, value);

	return value;
#else
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();
	// Zwracana zmienna.
	int value = 0;
	struct Config_Tag configs[] = {
			// Pobierane pole.
			{ (char *) _key, Int_Tag, &value},
			// Pole konczace.
			{ NULL , Error_Tag, NULL }
	};

	// Odczytanie zmiennej.
	boost::mutex::scoped_lock l(file_mutex);

	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			fprintf(stderr, "Blad input_config() w return_int_value file_location:%s, _section_name:%s, _key:%s\n", file_location.c_str(), _section_name, _key);
		}
	}

	// 	throw ERROR

	// Zwrocenie wartosci.

	return value;
#endif /* USE_MESSIP_SRR */
}// : return_int_value


// Zwraca wartosc (double) dla klucza.
double configurator::return_double_value(const char* _key, const char*__section_name)
{
#ifdef USE_MESSIP_SRR
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();

	query_t query;
	snprintf(query.key, sizeof(query.key), "%s", _key);
	snprintf(query.section, sizeof(query.section), "%s", _section_name);

	double value = 0;

	boost::mutex::scoped_lock l(file_mutex);

	messip::port_send(this->ch,
			CONFIG_RETURN_INT_VALUE, 0,
			query, value);

	return value;
#else
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();
	// Zwracana zmienna.
	double value;
	struct Config_Tag configs[] = {
			// Pobierane pole.
			{ (char *) _key, Double_Tag, &value},
			// Pole konczace.
			{ NULL , Error_Tag, NULL }
	};

	// Odczytanie zmiennej.
	boost::mutex::scoped_lock l(file_mutex);
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			fprintf(stderr, "Blad input_config() w return_double_value file_location:%s, _section_name:%s, _key:%s\n",
					file_location.c_str(), _section_name, _key);
		}
	}

	// 	throw ERROR

	// Zwrocenie wartosci.
	return value;
#endif /* USE_MESSIP_SRR */
}// : return_int_value


// Zwraca wartosc (char*) dla klucza.
std::string configurator::return_string_value(const char* _key, const char*__section_name)
{
#ifdef USE_MESSIP_SRR
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();

	query_t query;

	snprintf(query.key, sizeof(query.key), "%s", _key);
	snprintf(query.section, sizeof(query.section), "%s", _section_name);

	char value[255];

	boost::mutex::scoped_lock l(file_mutex);

	messip::port_send(this->ch, CONFIG_RETURN_STRING_VALUE, 0,
			query, value);

	// Zwrocenie wartosci.
	return std::string(value);
#else
	const char *_section_name = (__section_name) ? __section_name : section_name.c_str();
	// Zwracana zmienna.
	char tmp[200];
	struct Config_Tag configs[] = {
			// Pobierane pole.
			{ (char *) _key, String_Tag, tmp},
			// Pole konczace.
			{ NULL , Error_Tag, NULL }
	};

	// Odczytanie zmiennej.
	boost::mutex::scoped_lock l(file_mutex);
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			fprintf(stderr, "Blad input_config() w return_string_value file_location:%s, _section_name:%s, _key:%s\n",
					file_location.c_str(), _section_name, _key);
		}
	}

	// 	throw ERROR

	// Zwrocenie wartosci.
	return std::string(tmp);
#endif /* USE_MESSIP_SRR */
}// : return_string_value


pid_t configurator::process_spawn(const std::string & _section_name) {
#if defined(PROCESS_SPAWN_RSH)
	pid_t child_pid = vfork();

	if (child_pid == 0) {

		std::string spawned_program_name = return_string_value("program_name", _section_name);
		std::string spawned_node_name = return_string_value("node_name", _section_name);

		std::string rsh_spawn_node;

		if (spawned_node_name == sysinfo.nodename)
		{
			rsh_spawn_node = "localhost";
		} else {
			rsh_spawn_node = spawned_node_name;
		}

		// Sciezka do binariow.
		char bin_path[PATH_MAX];
		if (exists("binpath", _section_name)) {
			std::string _bin_path = return_string_value("binpath", _section_name);
			strcpy(bin_path, _bin_path.c_str());
			if(strlen(bin_path) && bin_path[strlen(bin_path)-1] != '/') {
				strcat(bin_path, "/");
			}

		} else {
			snprintf(bin_path, sizeof(bin_path), "/net/%s%sbin/",
					node.c_str(), dir.c_str());
		}

		//ewentualne dodatkowe argumenty wywolania np. przekierowanie na konsole
		std::string asa;
		if (exists("additional_spawn_argument", UI_SECTION)) {
			asa = return_string_value("additional_spawn_argument", UI_SECTION);
		}

		char process_path[PATH_MAX];
		char *ui_host = getenv("UI_HOST");
		snprintf(process_path, sizeof(process_path), "cd %s; UI_HOST=%s %s%s %s %s %s %s %s %s",
				bin_path, ui_host ? ui_host : "",
				bin_path, spawned_program_name.c_str(),
				node.c_str(), dir.c_str(), ini_file.c_str(), _section_name.c_str(),
				session_name.length() ? session_name.c_str() : "\"\"", asa.c_str()
		);

		// create new session for separation of signal delivery
		/*
		if(setsid() == (pid_t) -1) {
			perror("setsid()");
		}
		*/

		if (exists("username", _section_name)) {
			std::string username = return_string_value("username", _section_name);

//			fprintf(stderr, "rsh -l %s %s \"%s\"\n", username.c_str(), rsh_spawn_node.c_str(), process_path);

			execlp("rsh",
					"rsh",
					"-l", username.c_str(),
					rsh_spawn_node.c_str(),
					process_path,
					NULL);
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

			execlp("rsh",
					"rsh",
					rsh_spawn_node.c_str(),
					process_path,
					NULL);
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

	const int fd_map[] = { 0, 1, 2};
	//printf("conf a\n");
	// Argumenty wywolania procesu.
	char *child_arg[3];
	child_arg[0]=spawn_process;
	child_arg[1]=(char*)"NET_SPAWN";
	child_arg[2]=NULL;
	//printf("conf b: %s, %s\n", child_arg[0], child_arg[1]);
	// Odpalenie y_spawn_process.
	if ((child_pid=spawn( child_arg[0], 3, fd_map,  &inherit, child_arg, NULL)) ==-1)
	{
		fprintf( stderr, "Spawn of y_spawn_process failed (from PID %d): %s\n", getpid(), strerror(errno));
		// sleep(1000);
		return -1;
	}
	// Proba komunikacji z procesem odpalajacym inne procesy.
	short tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia
	while((fd = name_open(child_arg[1], 0))<0)
		if((tmp++)<CONNECT_RETRY)
			delay(CONNECT_DELAY);
		else{
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
		char * spawned_program_name = return_string_value("program_name", _section_name);
		char * spawned_node_name = return_string_value("node_name", _section_name);

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
}// : spawn

configurator::~configurator() {
#ifdef USE_MESSIP_SRR
	messip::port_disconnect(ch);
#endif /* USE_MESSIP_SRR */
}

} // namespace lib
} // namespace mrrocpp
