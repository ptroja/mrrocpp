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
#include <fcntl.h>
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
		config(_config), hardware_busy_file_fullpath(""), my_pid(getpid())
{
	/* Lokalizacja procesu wywietlania komunikatow SR */
	msg =
			(boost::shared_ptr <lib::sr_edp>) new lib::sr_edp(lib::EDP, config.robot_name, config.get_sr_attach_point().c_str());
}

shell::~shell()
{
	close_hardware_busy_file();
}

/*--------------------------------------------------------------------------*/
bool shell::detect_hardware_busy()
{
	// obsluga mechanizmu sygnalizacji zajetosci sprzetu

	const std::string hardware_busy_attach_point = config.get_edp_hardware_busy_file();

	hardware_busy_file_fullpath = "/tmp/.";

	hardware_busy_file_fullpath += hardware_busy_attach_point + ".pid";

	if (access(hardware_busy_file_fullpath.c_str(), R_OK) != 0) {

		std::cerr << "initialize_communication nie moglem odczytac: " << hardware_busy_file_fullpath << std::endl;

		// utworz plik i wstaw do niego pid
		return create_hardware_busy_file();

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

		std::cerr << ss.str() << std::endl;

		// jesli nie ma procesu
		if (access(ss.str().c_str(), R_OK) != 0) {
			// usun plik
			if (remove(hardware_busy_file_fullpath.c_str()) != 0) {
				perror("Error deleting file");
				return false;
			} else {
				//	puts("File successfully deleted");
			}

			// utworz plik i wstaw do niego pid
			return create_hardware_busy_file();

		} else {
			// juz jest EDP
			fprintf(stderr, "edp: hardware busy\n");
			return false;
		}
	}

	return true;
}

bool shell::create_hardware_busy_file()
{
	FILE * fp;
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
	return true;
}

void shell::close_hardware_busy_file()
{
	if (access(hardware_busy_file_fullpath.c_str(), R_OK) == 0) {

		//	std::cerr << "close_hardware_busy_file odczytałem: " << hardware_busy_file_fullpath << std::endl;

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
				//	puts("File successfully deleted");
			}
		} else {
			std::cerr << "Another EDP was running" << std::endl;
		}
	} else {
		std::cerr << "close_hardware_busy_file nie mogle odczytac: " << hardware_busy_file_fullpath << std::endl;
	}
}

} // namespace common
} // namespace edp
}
// namespace mrrocpp

