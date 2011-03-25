// ------------------------------------------------------------------------
//                                  edp.cc
//
// EDP_MASTER Effector Driver Master Process
// Driver dla robota IRp-6 na torze - metody: class edp_irp6s_robot
//
// Ostatnia modyfikacja: styczen 2005
// -------------------------------------------------------------------------

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <cerrno>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <fstream>

#include <boost/shared_ptr.hpp>

#include "base/lib/messip/messip_dataport.h"

#include "base/lib/mis_fun.h"
#include "edp_shell.h"

namespace mrrocpp {
namespace edp {
namespace common {

/*--------------------------------------------------------------------------*/
shell::shell(lib::configurator &_config) :
	config(_config), hardware_busy_file_fullpath(""), my_pid(0)
{
	my_pid = getpid();
}

shell::~shell()
{
}

/*--------------------------------------------------------------------------*/
bool shell::detect_hardware_busy()
{

	// obsluga mechanizmu sygnalizacji zajetosci sprzetu

	const std::string hardware_busy_attach_point = config.value <std::string> ("hardware_busy_attach_point");

	hardware_busy_file_fullpath = "/tmp/.";

	hardware_busy_file_fullpath += hardware_busy_attach_point + ".pid";

	FILE * fp;

	if (access(hardware_busy_file_fullpath.c_str(), R_OK) != 0) {

		std::cerr << "initialize_communication nie moglem odczytac: " << hardware_busy_file_fullpath << std::endl;

		// utworz plik i wstaw do niego pid

		fp = fopen(hardware_busy_file_fullpath.c_str(), "w");
		if (fp) {
			fclose(fp);
		} else {
			return false;
		}

		if (chmod(hardware_busy_file_fullpath.c_str(), S_IRGRP | S_IROTH | S_IRUSR | S_IWGRP | S_IWOTH | S_IWUSR) != 0) {
			perror("chmod() error");
			return false;
		}

		std::ofstream outfile(hardware_busy_file_fullpath.c_str(), std::ios::out);
		if (!outfile.good()) {
			std::cerr << hardware_busy_file_fullpath << std::endl;
			perror("because of");
			return false;
		} else {
			outfile << my_pid;
		}
		outfile.close();

	} else {
		pid_t file_pid;

		{
			std::fstream infile(hardware_busy_file_fullpath.c_str(), std::ios::in);
			if (!infile.good()) {
				std::cerr << hardware_busy_file_fullpath << std::endl;
				perror("because of");
				return false;
			} else {
				infile >> file_pid;
			}
		}

		std::stringstream ss(std::stringstream::in | std::stringstream::out);

		ss << "/proc/" << file_pid;
		ss.str().c_str();

		std::cerr << ss.str() << std::endl;
		// jesli nie ma procesu
		if (access(ss.str().c_str(), R_OK) != 0) {
			// usun plik
			if (remove(hardware_busy_file_fullpath.c_str()) != 0) {
				perror("Error deleting file");
				return false;
			} else {
				puts("File successfully deleted");
			}
			// utworz plik
			fp = fopen(hardware_busy_file_fullpath.c_str(), "w");
			if (fp) {
				fclose(fp);
			} else {
				return false;
			}
			// wypelnij plik pidem edp
			std::ofstream outfile(hardware_busy_file_fullpath.c_str(), std::ios::out);
			if (!outfile.good()) {
				std::cerr << hardware_busy_file_fullpath << std::endl;
				perror("because of");
				return false;
			} else {
				outfile << my_pid;
			}
			outfile.close();
		} else {
			// juz jest EDP
			fprintf(stderr, "edp: hardware busy\n");
			return false;
		}

	}

	return true;
}

bool shell::close_hardware_busy_file()
{

	if (access(hardware_busy_file_fullpath.c_str(), R_OK) == 0) {

		std::cerr << "close_hardware_busy_file odczytałem: " << hardware_busy_file_fullpath << std::endl;

		pid_t file_pid;

		// utworz plik i wstaw do niego pid

		{
			std::fstream infile(hardware_busy_file_fullpath.c_str(), std::ios::in);
			if (!infile.good()) {
				std::cerr << "infile " << hardware_busy_file_fullpath << std::endl;
				perror("because of");
			} else {
				infile >> file_pid;
			}
		}
		if (file_pid == my_pid) {
			// usun plik
			if (remove(hardware_busy_file_fullpath.c_str()) != 0) {
				perror("Error deleting file");
			} else {
				puts("File successfully deleted");
			}
		} else {
			std::cerr << "Another EDP was running" << std::endl;
		}
	} else {
		std::cerr << "close_hardware_busy_file nie mogle odczytac: " << hardware_busy_file_fullpath << std::endl;
	}

	return true;
}

} // namespace common
} // namespace edp
}
// namespace mrrocpp

