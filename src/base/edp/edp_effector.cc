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
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <fstream>

#include <boost/shared_ptr.hpp>

#include "base/lib/messip/messip_dataport.h"

#include "base/lib/mis_fun.h"
#include "base/edp/edp_effector.h"

namespace mrrocpp {
namespace edp {
namespace common {

/*--------------------------------------------------------------------------*/
effector::effector(lib::configurator &_config, lib::robot_name_t l_robot_name) :
	hardware_busy_file_fullpath(""), robot_name(l_robot_name), config(_config), robot_test_mode(true)
{
	/* Lokalizacja procesu wywietlania komunikatow SR */
	msg
			= (boost::shared_ptr <lib::sr_edp>) new lib::sr_edp(lib::EDP, config.value <std::string> ("resourceman_attach_point").c_str(), config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "sr_attach_point", lib::UI_SECTION).c_str());

	if (config.exists(lib::ROBOT_TEST_MODE.c_str())) {
		robot_test_mode = config.value <int> (lib::ROBOT_TEST_MODE);
	}

	if (robot_test_mode) {
		msg->message("Robot test mode activated");
	}

	my_pid = getpid();
}

effector::~effector()
{
}

/*--------------------------------------------------------------------------*/
bool effector::detect_hardware_busy()
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

		std::string system_command_string;

		system_command_string = "chmod 757 " + hardware_busy_file_fullpath;
		system(system_command_string.c_str());

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

/*--------------------------------------------------------------------------*/
bool effector::initialize_communication()
{

	const std::string
			server_attach_point(config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point"));

	// nawiazywanie komunikacji

	std::string full_path_to_server_attach_point("/dev/name/global/");
	full_path_to_server_attach_point += server_attach_point;

	// sprawdzenie czy nie jest juz zarejestrowany server EDP
	if (access(full_path_to_server_attach_point.c_str(), R_OK) == 0) {
		fprintf(stderr, "edp already exists() failed: %s\n", strerror(errno));
		return false;
	}

	/* Ustawienie priorytetu procesu */

	lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 2);

	server_attach = messip::port_create(server_attach_point);

	if (server_attach == NULL) {
		msg->message(lib::SYSTEM_ERROR, errno, "edp: resmg failed to attach");
		fprintf(stderr, "name_attach() failed: %s\n", strerror(errno));
		return false;
	}

	msg->message("edp loaded");

	return true;
}

bool effector::close_hardware_busy_file()
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

void effector::establish_error(lib::r_buffer_base & reply, uint64_t err0, uint64_t err1)
{
	reply.reply_type = lib::ERROR;
	reply.error_no.error0 = err0;
	reply.error_no.error1 = err1;
}

void effector::instruction_deserialization()
{
}

void effector::reply_serialization()
{
}

} // namespace common
} // namespace edp
}
// namespace mrrocpp

