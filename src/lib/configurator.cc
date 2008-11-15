// -------------------------------------------------------------------------
// Plik:			configurator.cc
// System:	QNX/MRROCPP  v. 6.3
// Opis:		Plik zawiera definicje matod klasy configurator - obsluga konfiguracji z pliku INI.
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


#if defined(__QNXNTO__)
#include <process.h>
#include <spawn.h>
#include <sys/netmgr.h>
#endif /* __QNXNTO__ */

#include "common/impconst.h"
#include "lib/configurator.h"
#include "lib/y_spawn.h"
#include "messip/messip.h"
#include "lib/config_types.h"

// Konstruktor obiektu - konfiguratora.
configurator::configurator (const char* _node, const char* _dir, const char* _ini_file, const char* _section_name,
		const char* _session_name)

{
	assert(_node);
	assert(_dir);
	assert(_ini_file);
	assert(_section_name);
	assert(_session_name);

	node = strdup(_node);
	dir = strdup(_dir);
	ini_file = strdup(_ini_file);
	section_name = strdup(_section_name); 
	session_name = strdup(_session_name);

	pthread_mutex_init(&mutex, NULL );

	int size = 1 + strlen("/net/") + strlen(node) + strlen(dir);
	mrrocpp_network_path = new char[size];
	// Stworzenie sciezki do pliku.
	sprintf(mrrocpp_network_path, "/net/%s%s", node, dir);

#ifdef USE_MESSIP_SRR
	printf("messip_channel_connect()");
	if ((ch = messip_channel_connect(NULL, CONFIGSRV_CHANNEL_NAME, MESSIP_NOTIMEOUT)) == NULL) {
		perror("messip_channel_connect()");
	}
	assert(ch);
#else
	file_location = return_ini_file_path();
	common_file_location = return_common_ini_file_path();	
#endif /* USE_MESSIP_SRR */
}// : configurator

void configurator::change_ini_file (const char* _ini_file)
{
#ifdef USE_MESSIP_SRR
	config_msg_t config_msg;
	snprintf(config_msg.configfile, sizeof(config_msg.configfile), "%s", _ini_file);
	int32_t answer;

	lock_mutex();

	messip_send(this->ch, CONFIG_CHANGE_INI_FILE, 0,
			&config_msg, sizeof(config_msg),
			&answer, NULL, 0,
			MESSIP_NOTIMEOUT);

	unlock_mutex();
#else
	lock_mutex();
	if (ini_file) free(ini_file);
	ini_file = strdup(_ini_file);

	delete [] file_location;	
	delete [] common_file_location;	

	file_location = return_ini_file_path();
	common_file_location = return_common_ini_file_path();

	unlock_mutex();
#endif /* USE_MESSIP_SRR */
}

int	configurator::lock_mutex() // zajecie mutex'a
{
	return pthread_mutex_lock( &mutex );
}

int	configurator::unlock_mutex() // zwolnienie mutex'a
{
	return pthread_mutex_unlock( &mutex );
}


// Zwraca numer wezla.
int configurator::return_node_number(const char* node_name_l)
{
#if defined(PROCESS_SPAWN_RSH)
#warning configurator::return_node_number by RSH
	return ND_LOCAL_NODE;
#else
	return netmgr_strtond(node_name_l, NULL);
#endif
}// : return_node_number


// Zwraca attach point'a dla serwerow.
char* configurator::return_attach_point_name (const int _type, const char* _key, const char* __section_name)
{
	const char *_section_name = (__section_name) ? __section_name : section_name;
	char * name = NULL;

	if (_type == CONFIG_RESOURCEMAN_LOCAL)
	{
		// Odczytanie zmiennych z INI.
		char* attach_point = return_string_value(_key, _section_name);
		// Obliczenie dlugosci nazwy.
		int size = 1 + strlen("/dev/") + strlen(attach_point)  + strlen(session_name);

		// Przydzielenie pamieci.
		name = new char[size];
		// Stworzenie nazwy.
		sprintf(name, "/dev/%s%s", attach_point, session_name);

		// Zwalniam pamiec.
		delete [] attach_point;
	} else if (_type == CONFIG_RESOURCEMAN_GLOBAL)
	{
		// Odczytanie zmiennych z INI.
		char* node_name = return_string_value("node_name", _section_name);
		char* attach_point = return_string_value(_key, _section_name);

		// Obliczenie dlugosci nazwy.
		int size = 1 + strlen("/net/") + strlen(node_name) + strlen("/dev/") + strlen(attach_point)  + strlen(session_name);
		// Przydzielenie pamieci.
		name = new char[size];
		// Stworzenie nazwy.
		sprintf(name, "/net/%s/dev/%s%s", node_name, attach_point, session_name);

		// Zwalniam pamiec.
		delete [] node_name;
		delete [] attach_point;
	} else if (_type == CONFIG_SERVER)
	{
		char* attach_point = return_string_value(_key, _section_name);

		// Obliczenie dlugosci nazwy.
		int size = 1 + strlen(attach_point) + strlen(session_name);

		// Przydzielenie pamieci.
		name = new char[size];

		// Stworzenie nazwy.
		name = new char[size];
		sprintf(name, "%s%s", attach_point, session_name);

		// Zwalniam pamiec.
		delete [] attach_point;
	} else {
		printf("Nieznany argument w metodzie configuratora return_attach_point_name\n");
	}

	// Zwrocenie atach_point'a.	
	return name;
}// : return_created_resourceman_attach_point_name

#ifndef USE_MESSIP_SRR
// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
char* configurator::return_ini_file_path()
{
	int size = 1 + strlen(mrrocpp_network_path) + strlen("configs/") + strlen(ini_file);
	char * value = new char[size];
	// Stworzenie sciezki do pliku.
	sprintf(value, "%sconfigs/%s", mrrocpp_network_path, ini_file);

	return value;
}

// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego z konfiguracja domyslna (common.ini)
char* configurator::return_common_ini_file_path()
{
	int size = 1 + strlen(mrrocpp_network_path) + strlen("configs/common.ini");
	char * value = new char[size];
	// Stworzenie sciezki do pliku.
	sprintf(value, "%sconfigs/common.ini", mrrocpp_network_path);

	return value;
}
#endif

// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
char* configurator::return_default_reader_measures_path()
{
	int size = 1 + strlen(mrrocpp_network_path) + strlen("pomiary/");
	char * value = new char[size];
	// Stworzenie sciezki do pliku.
	strcpy(value, mrrocpp_network_path);
	sprintf(value, "%spomiary/", mrrocpp_network_path);

	return value;
}

// Zwraca wartosc (char*) dla sciezki do pliku konfiguracyjnego.
char* configurator::return_mrrocpp_network_path()
{
	int size = 1 + strlen(mrrocpp_network_path);
	char * value = new char[size];
	// Stworzenie sciezki do pliku.
	strcpy(value, mrrocpp_network_path);

	return value;
}

// Zwraca czy dany klucz istnieje
bool configurator::exists(const char* _key, const char* __section_name)
{
#ifdef USE_MESSIP_SRR
	const char *_section_name = (__section_name) ? __section_name : section_name;

	config_msg_t config_msg;
	snprintf(config_msg.key, sizeof(config_msg.key), "%s", _key);
	snprintf(config_msg.section, sizeof(config_msg.section), "%s", _section_name);
	int32_t answer;

	bool value;

	lock_mutex();

	messip_send(this->ch, CONFIG_EXISTS, 0,
			&config_msg, sizeof(config_msg),
			&answer, &value, sizeof(value),
			MESSIP_NOTIMEOUT);

	unlock_mutex();

	return value;
#else
	const char *_section_name = (__section_name) ? __section_name : section_name;
	int value;
	struct Config_Tag configs[] = {
			// Pobierane pole.
			{ (char *) _key, Int_Tag, &value},
			// Pole konczace.
			{ NULL , Error_Tag, NULL }
	};

	lock_mutex();
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			unlock_mutex();
			// Zwolnienie pamieci.


			return false;
		}
	}
	unlock_mutex();

	return true;	
#endif /* USE_MESSIP_SRR */
}

// Zwraca wartosc (int) dla klucza.
int configurator::return_int_value(const char* _key, const char* __section_name)
{
#ifdef USE_MESSIP_SRR
	const char *_section_name = (__section_name) ? __section_name : section_name;

	config_msg_t config_msg;
	snprintf(config_msg.key, sizeof(config_msg.key), "%s", _key);
	snprintf(config_msg.section, sizeof(config_msg.section), "%s", _section_name);
	int32_t answer;

	int value;

	lock_mutex();

	messip_send(this->ch, CONFIG_RETURN_INT_VALUE, 0,
			&config_msg, sizeof(config_msg),
			&answer, &value, sizeof(value),
			MESSIP_NOTIMEOUT);

	unlock_mutex();

	return value;
#else
	const char *_section_name = (__section_name) ? __section_name : section_name;
	// Zwracana zmienna.
	int value;
	struct Config_Tag configs[] = {
			// Pobierane pole.
			{ (char *) _key, Int_Tag, &value},
			// Pole konczace.
			{ NULL , Error_Tag, NULL }
	};


	// Odczytanie zmiennej.
	lock_mutex();
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			printf("Blad input_config() w return_int_value file_location:%s, _section_name:%s, _key:%s\n", file_location, _section_name, _key);
		}
	}
	unlock_mutex();

	// 	throw ERROR

	// Zwrocenie wartosci.

	return value;
#endif /* USE_MESSIP_SRR */
}// : return_int_value


// Zwraca wartosc (double) dla klucza.
double configurator::return_double_value(const char* _key, const char*__section_name)
{
#ifdef USE_MESSIP_SRR
	const char *_section_name = (__section_name) ? __section_name : section_name;

	config_msg_t config_msg;
	snprintf(config_msg.key, sizeof(config_msg.key), "%s", _key);
	snprintf(config_msg.section, sizeof(config_msg.section), "%s", _section_name);
	int32_t answer;

	double value;

	lock_mutex();

	messip_send(this->ch, CONFIG_RETURN_INT_VALUE, 0,
			&config_msg, sizeof(config_msg),
			&answer, &value, sizeof(value),
			MESSIP_NOTIMEOUT);

	unlock_mutex();

	return value;
#else
	const char *_section_name = (__section_name) ? __section_name : section_name;
	// Zwracana zmienna.
	double value;
	struct Config_Tag configs[] = {
			// Pobierane pole.
			{ (char *) _key, Double_Tag, &value},
			// Pole konczace.
			{ NULL , Error_Tag, NULL }
	};


	// Odczytanie zmiennej.
	lock_mutex();
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			printf("Blad input_config() w return_double_value file_location:%s, _section_name:%s, _key:%s\n", 
					file_location, _section_name, _key);
		}
	}
	unlock_mutex();

	// 	throw ERROR

	// Zwrocenie wartosci.
	return value;
#endif /* USE_MESSIP_SRR */
}// : return_int_value


// Zwraca wartosc (char*) dla klucza.
char* configurator::return_string_value(const char* _key, const char*__section_name)
{
#ifdef USE_MESSIP_SRR
	const char *_section_name = (__section_name) ? __section_name : section_name;

	config_msg_t config_msg;
	snprintf(config_msg.key, sizeof(config_msg.key), "%s", _key);
	snprintf(config_msg.section, sizeof(config_msg.section), "%s", _section_name);
	int32_t answer;

	char value[255];

	lock_mutex();

	messip_send(this->ch, CONFIG_RETURN_STRING_VALUE, 0,
			&config_msg, sizeof(config_msg),
			&answer, &value, sizeof(value),
			MESSIP_NOTIMEOUT);

	unlock_mutex();

	//printf("configurator::return_string_value(%s, %s) = %s\n", _key, _section_name, value);

	// Przepisanie wartosci.
	int size = 1 + strlen(value);
	char * _value = new char [size];
	strcpy(_value, value);

	// Zwrocenie wartosci.
	return _value;
#else
	const char *_section_name = (__section_name) ? __section_name : section_name;
	// Zwracana zmienna.
	char tmp[200];
	struct Config_Tag configs[] = {
			// Pobierane pole.
			{ (char *) _key, String_Tag, tmp},
			// Pole konczace.
			{ NULL , Error_Tag, NULL }
	};		

	// Odczytanie zmiennej.
	lock_mutex();
	if (input_config(file_location, configs, _section_name)<1) {
		if (input_config(common_file_location, configs, _section_name)<1) {
			printf("Blad input_config() w return_string_value file_location:%s, _section_name:%s, _key:%s\n",
					file_location, _section_name, _key);
		}
	}
	unlock_mutex();
	// 	throw ERROR
	// Przepisanie wartosci.
	int size = 1 + strlen(tmp);
	char * value = new char [size];
	strcpy(value, tmp);

	// Zwrocenie wartosci.
	return value;
#endif /* USE_MESSIP_SRR */
}// : return_string_value


pid_t configurator::process_spawn(const char*_section_name) {
#if defined(PROCESS_SPAWN_RSH)
#warning configurator::process_spawn by RSH
	pid_t child_pid = vfork();

	if (child_pid == 0) {

		char * spawned_program_name = return_string_value("program_name", _section_name);
		char * spawned_node_name = return_string_value("node_name", _section_name);

		// Sciezka do binariow.
		char bin_path[PATH_MAX];
		if (exists("binpath", _section_name)) {
			char * _bin_path = return_string_value("binpath", _section_name);
			strcpy(bin_path, _bin_path);
			if(strlen(bin_path) && bin_path[strlen(bin_path)-1] != '/') {
				strcat(bin_path, "/");
			}
			delete [] _bin_path;
		} else {
			snprintf(bin_path, sizeof(bin_path), "/net/%s%sbin/",
					node, dir);
		}

		char process_path[PATH_MAX];
		snprintf(process_path, sizeof(process_path), "cd %s; UI_HOST=%s %s%s %s %s %s %s %s",
				bin_path, getenv("UI_HOST"),
				bin_path, spawned_program_name,
				node, dir, ini_file, _section_name
				, session_name
		);

		delete [] spawned_program_name;

		if (exists("username", _section_name)) {
			char * username = return_string_value("username", _section_name);

			printf("rsh -l %s %s \"%s\"\n", username, spawned_node_name, process_path);

			execlp("rsh",
					"rsh",
					"-l", username,
					spawned_node_name,
					process_path,
					NULL);

			delete [] username;
		} else {
			printf("rsh %s \"%s\"\n", spawned_node_name, process_path);

			execlp("rsh",
					"rsh",
					spawned_node_name,
					process_path,
					NULL);
		}

		delete [] spawned_node_name;

	} else if (child_pid > 0) {
		printf("child %d created\n", child_pid);
	} else {
		perror("vfork()");
	}

	return child_pid;
#endif
#if defined(PROCESS_SPAWN_YRSH)
	// Deskryptor pliku.

	struct utsname sysinfo;
	// printf("_section_name: %s,\n",_section_name);

	name_attach_t *my_attach;	// by Y
	my_data_t msg;
	int rcvid;
	bool wyjscie=false; // okresla stan procesu: 0 -przed komunikacja, 1 - po komunikacji
	_msg_info* info;
	info = new  _msg_info;

	// Sciezka do binariow.
	char * bin_path;
	int size = 1 + strlen("/net/") + strlen(node) +strlen(dir) + strlen("bin/");
	bin_path = new char[size];
	strcpy(bin_path,"/net/");
	strcat(bin_path, node);
	strcat(bin_path, dir);
	strcat(bin_path,"bin/");


	// printf("conf 1\n");
	// Wiadomosci odbierane i wysylane.
	my_data_t input;
	my_reply_data_t output;
	// Parametry wywolania procesu.
	input.hdr.type=0;
	input.msg_type=1;
	// Odczytanie nazwy odpalanego pliku.
	char * spawned_program_name = return_string_value("program_name", _section_name);
	char * spawned_node_name = return_string_value("node_name", _section_name);
	if( uname( &sysinfo ) == -1 ) {
		perror( "uname" );
	}


	if (strcmp(sysinfo.nodename,spawned_node_name) == 0)
	{
		strcpy(input.node_name, "localhost");
	} else
	{
		strcpy(input.node_name, spawned_node_name);
	}

	time_t time_of_day;
	char rsp_attach[20];


	time_of_day = time( NULL );
	strftime( rsp_attach, 8, "rsp%H%M%S", localtime( &time_of_day ) );

	// printf("spawned_node_name:%s\n", spawned_node_name);


	strcpy(input.program_name_and_args, spawned_program_name);
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, node);
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, dir);
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, ini_file);
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, _section_name);
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, session_name);
	strcat(input.program_name_and_args, " ");
	strcat(input.program_name_and_args, rsp_attach);
	strcpy(input.binaries_path, bin_path);

	// Zwolnienie pamieci.
	delete [] spawned_program_name;
	delete [] spawned_node_name;
	delete [] bin_path;
	
	char rsh_cmd[PATH_MAX];
		snprintf(rsh_cmd, PATH_MAX, "rsh %s %s%s&",
				input.node_name, input.binaries_path, input.program_name_and_args);

	if ((my_attach = name_attach(NULL, rsp_attach, NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
		printf("process_spawn: blad name_attach\n");
	}

	
	system(rsh_cmd);

	
	while (!wyjscie)
	{
		//printf("spawn_prc 3\n");

		rcvid = MsgReceive(my_attach->chid, &msg, sizeof(msg), info);

		if (rcvid == -1) {/* Error condition, exit */
			perror("blad receive w rsh spawn\n");
			break;
		}

		if (rcvid == 0) /* Pulse received */
		{
			//	printf("1\n");
			switch (msg.hdr.code) {
			case _PULSE_CODE_DISCONNECT:
				ConnectDetach(msg.hdr.scoid);
				wyjscie=true;
			case _PULSE_CODE_UNBLOCK:
				break;
			default:
				break;
			}
			continue;
		}

		/* A QNX IO message received, reject */
		if (msg.hdr.type >= _IO_BASE && msg.hdr.type <= _IO_MAX) {
			MsgReply(rcvid, EOK, 0, 0);
			output.pid = info->pid;
			//	printf("Child pid:%d\n", output.pid);
			continue;
		}
		//printf("3\n");
		MsgReply(rcvid, EOK, 0, 0);
	} // // end while



	name_detach(my_attach, 0);
	// Zwrocenie wyniku.  	
	return output.pid;
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
	char * bin_path;
	int size = 1 + strlen("/net/") + strlen(node) +strlen(dir) + strlen("bin/");
	bin_path = new char[size];
	strcpy(bin_path,"/net/");
	strcat(bin_path, node);
	strcat(bin_path, dir);
	strcat(bin_path,"bin/");

	// Zlozenie lokalizacji odpalanego y_spawn_process
	char* spawn_process;
	size = 1 + strlen(bin_path) + strlen("y_spawn_process");
	spawn_process = new char[size];
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
		};
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
		strcat(input.program_name_and_args, node);
		strcat(input.program_name_and_args, " ");
		strcat(input.program_name_and_args, dir);
		strcat(input.program_name_and_args, " ");
		strcat(input.program_name_and_args, ini_file);
		strcat(input.program_name_and_args, " ");
		strcat(input.program_name_and_args, _section_name);
		strcat(input.program_name_and_args, " ");
		strcat(input.program_name_and_args, session_name);
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

int configurator::answer_to_y_rsh_spawn(const char* rsh_spl)
{
	int spawn_fd = name_open(rsh_spl, NAME_FLAG_ATTACH_GLOBAL);
	if (spawn_fd <0)
	{
		printf("Blad spawn name_open\n");
	} else
	{
		name_close(spawn_fd);
	}

	return spawn_fd;

}

configurator::~configurator() {
	free(node);
	free(dir);
	free(ini_file);
	free(section_name);
	free(session_name);
	delete [] mrrocpp_network_path;
#ifdef USE_MESSIP_SRR
	messip_channel_disconnect(ch, MESSIP_NOTIMEOUT);
#else
	delete [] file_location;	
	delete [] common_file_location;	
#endif /* USE_MESSIP_SRR */
}
