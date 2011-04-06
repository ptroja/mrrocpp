//
// READER - watek do buforowania danych pomiarowych i ich zapisu do pliku
// Date: maj 2006
//

#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <csignal>
#include <cerrno>
#include <sys/wait.h>
#include <sys/types.h>

#include "base/lib/messip/messip_dataport.h"

#include <cerrno>
#include <pthread.h>
#include <ctime>

#include <boost/scoped_array.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mis_fun.h"
#include "base/edp/edp_e_motor_driven.h"
#include "base/edp/reader.h"

namespace mrrocpp {
namespace edp {
namespace common {

reader_config::reader_config() :
	step(false), measure_time(false), servo_mode(false)
{
	for (std::size_t i = 0; i < lib::MAX_SERVOS_NR; ++i) {
		desired_inc[i] = false;
		current_inc[i] = false;
		pwm[i] = false;
		uchyb[i] = false;
		abs_pos[i] = false;
		current_joints[i] = false;
	}

	for (int i = 0; i < 6; ++i) {
		force[i] = false;
		desired_force[i] = false;
		filtered_force[i] = false;
		desired_cartesian_position[i] = false;
		real_cartesian_position[i] = false;
		real_cartesian_vel[i] = false;
		real_cartesian_acc[i] = false;
	}
}

reader_buffer::reader_buffer(motor_driven_effector &_master) :
	new_data(false), master(_master), write_csv(true)
{
	thread_id = boost::thread(boost::bind(&reader_buffer::operator(), this));
}

reader_buffer::~reader_buffer()
{
	// TODO: stop (interrupt?) the thread
	//thread_id.interrupt();
	//thread_id.join(); // join it
}

void reader_buffer::operator()()
{
	uint64_t nr_of_samples; // maksymalna liczba pomiarow
	bool start; // shall we start the reader?
	bool stop; // shall we stop the reader?

	bool ui_trigger = false; // specjalny puls z UI

	int file_counter = 0;
	time_t time_of_day;
	char file_name[50];
	char file_date[40];
	char config_file_with_dir[80];

	// czytanie konfiguracji
	std::string reader_meassures_dir;

	if (master.config.exists("reader_meassures_dir")) {
		reader_meassures_dir = master.config.value <std::string> ("reader_meassures_dir", lib::UI_SECTION);
	} else {
		reader_meassures_dir = master.config.return_default_reader_measures_path();
	}

	std::string robot_filename = master.config.value <std::string> ("reader_attach_point");

	if (master.config.exists("reader_samples"))
		nr_of_samples = master.config.value <int> ("reader_samples");
	else
		nr_of_samples = 1000;

	reader_cnf.step = 1;
	reader_cnf.servo_mode = master.config.check_config("servo_tryb");
	reader_cnf.measure_time = master.config.check_config("measure_time");

	char tmp_string[50];

	for (int j = 0; j < master.number_of_servos; j++) {

		sprintf(tmp_string, "desired_inc_%d", j);
		reader_cnf.desired_inc[j] = master.config.check_config(tmp_string);

		sprintf(tmp_string, "current_inc_%d", j);
		reader_cnf.current_inc[j] = master.config.check_config(tmp_string);

		sprintf(tmp_string, "pwm_%d", j);
		reader_cnf.pwm[j] = master.config.check_config(tmp_string);

		sprintf(tmp_string, "uchyb_%d", j);
		reader_cnf.uchyb[j] = master.config.check_config(tmp_string);

		sprintf(tmp_string, "abs_pos_%d", j);
		reader_cnf.abs_pos[j] = master.config.check_config(tmp_string);

		sprintf(tmp_string, "current_joints_%d", j);
		reader_cnf.current_joints[j] = master.config.check_config(tmp_string);

		if (j < 6) {
			sprintf(tmp_string, "force_%d", j);
			reader_cnf.force[j] = master.config.check_config(tmp_string);

			sprintf(tmp_string, "desired_force_%d", j);
			reader_cnf.desired_force[j] = master.config.check_config(tmp_string);

			sprintf(tmp_string, "filtered_force_%d", j);
			reader_cnf.filtered_force[j] = master.config.check_config(tmp_string);

			sprintf(tmp_string, "desired_cartesian_position_%d", j);
			reader_cnf.desired_cartesian_position[j] = master.config.check_config(tmp_string);

			sprintf(tmp_string, "real_cartesian_position_%d", j);
			reader_cnf.real_cartesian_position[j] = master.config.check_config(tmp_string);

			sprintf(tmp_string, "real_cartesian_vel_%d", j);
			reader_cnf.real_cartesian_vel[j] = master.config.check_config(tmp_string);

			sprintf(tmp_string, "real_cartesian_acc_%d", j);
			reader_cnf.real_cartesian_acc[j] = master.config.check_config(tmp_string);
		}
	}

	// ustawienie priorytetu watku
	lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 10);

	// NOTE: readed buffer has to be allocated on heap (using "new" operator) due to huge size
	// boost::scoped_array takes care of deallocating in case of exception
	boost::circular_buffer <reader_data> reader_buf(nr_of_samples);

	//	fprintf(stderr, "reader buffer size %lluKB\n", nr_of_samples*sizeof(reader_data)/1024);

	// by Y komuniakicja pomiedzy ui i reader'em rozwiazalem poprzez pulsy
	// powolanie kanalu komunikacyjnego do odbioru pulsow sterujacych

	lib::fd_server_t my_attach;


		if ((my_attach = messip::port_create(
								master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "reader_attach_point")))
				== NULL) {

		perror("Failed to attach pulse chanel for READER");
		master.msg->message("Failed to attach pulse chanel for READER");
		//  throw MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}

	// GLOWNA PETLA Z OCZEKIWANIEM NA ZLECENIE POMIAROW
	for (;;) {
		// ustawienie priorytetu watku
		lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 10);

		// ustawienie priorytetu watku
		lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 10);

		start = false; // okresla czy odebrano juz puls rozpoczecia pomiarow

		// dopoki nie przyjdzie puls startu
		while (!start) {
			int32_t type, subtype;
			int rcvid = messip::port_receive_pulse(my_attach, type, subtype);

			//std::cerr << "pulse received: " << rcvid << " " << type << " " << subtype << std::endl;

			if (rcvid == MESSIP_MSG_NOREPLY) {
				if (type == READER_START) {
					start = true;
				}
			}

		}

		master.msg->message("measures started");

		lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY + 1);

		// dopoki nie przyjdzie puls stopu
		do {
			//	master.msg->message("measure 1");
			new_data = false;
			// sekcja krytyczna odczytu danych pomiarowych dla biezacego kroku
			{
				boost::mutex::scoped_lock lock(reader_mutex);

				while (!new_data) {
					// czekamy na opuszcenie semafora przez watek EDP_SERVO (co mikrokrok)
					cond.wait(lock);
				}

				step_data.ui_trigger = ui_trigger;

				// przepisanie danych dla biezacego kroku do bufora lokalnego reader
				reader_buf.push_back(step_data);
			}

			//	master.msg->message("measure 2");

			// warunkowy odbior pulsu (o ile przyszedl)
			stop = false;
			ui_trigger = false;

			int32_t type, subtype;
			int rcvid = messip::port_receive_pulse(my_attach, type, subtype, 0);

			//std::cerr << "pulse received: " << rcvid << " " << type << " " << subtype << std::endl;

			if (rcvid == MESSIP_MSG_NOREPLY) {
				if (type == READER_STOP) {
					stop = true;
					master.onReaderStopped();
				} else if (type == READER_TRIGGER) {
					ui_trigger = true;
				}
			}

		} while (!stop); // dopoki nie przyjdzie puls stopu

		lib::set_thread_priority(pthread_self(), 1);// Najnizszy priorytet podczas proby zapisu do pliku
		master.msg->message("measures stopped");

		// przygotowanie nazwy pliku do ktorego beda zapisane pomiary
		time_of_day = time(NULL);
		strftime(file_date, 40, "%Y-%m-%d_%H-%M-%S", localtime(&time_of_day));

		sprintf(file_name, "/%s_%s_pomiar-%d.csv", file_date, robot_filename.c_str(), ++file_counter);
		strcpy(config_file_with_dir, reader_meassures_dir.c_str());

		strcat(config_file_with_dir, file_name);

		std::ofstream outfile(config_file_with_dir, std::ios::out);
		if (!outfile.good()) // jesli plik nie instnieje
		{
			std::cerr << "Cannot open file: " << file_name << '\n';
			perror("because of");
			master.msg->message("cannot open destination file");
			// TODO: throw
		} else { // jesli plik istnieje

			if (write_csv) {
				write_header_csv(outfile);
			} else {
				write_header_old_format(outfile);
			}

			// TODO: sprawdzenie czy bufor byl przepelniony i odpowiednie
			// przygotowanie granic bufora przy zapi sie do pliku

			// dla calego horyzontu pomiarow

			while (!reader_buf.empty()) {
				// zapis pomiarow z biezacego kroku do pliku
				// printf("edp %f\n", reader_buf.front().desired_cartesian_position[1]);

				reader_data & data = reader_buf.front();

				if (write_csv) {
					write_data_csv(outfile, data);
				} else {
					write_data_old_format(outfile, data);
				}

				reader_buf.pop_front();
			}

			master.msg->message("file writing is finished");
		}
	} // end: for (;;)
}

void reader_buffer::write_header_old_format(std::ofstream& outfile)
{
	// does nothing
}

void reader_buffer::write_data_old_format(std::ofstream& outfile, const reader_data & data)
{
	outfile << data.step << " ";
	if (reader_cnf.measure_time)
		outfile << ((int) (data.measure_time.tv_nsec / 1000000)) << " ";
	if (reader_cnf.servo_mode)
		outfile << (data.servo_mode ? "1" : "0") << " ";
	for (int j = 0; j < master.number_of_servos; j++) {
		if (reader_cnf.desired_inc[j])
			outfile << data.desired_inc[j] << " ";
		if (reader_cnf.current_inc[j])
			outfile << data.current_inc[j] << " ";
		if (reader_cnf.pwm[j])
			outfile << data.pwm[j] << " ";
		if (reader_cnf.uchyb[j])
			outfile << data.uchyb[j] << " ";
		if (reader_cnf.abs_pos[j])
			outfile << data.abs_pos[j] << " ";
	}

	outfile << "j: ";

	for (int j = 0; j < master.number_of_servos; j++) {
		if (reader_cnf.current_joints[j])
			outfile << data.current_joints[j] << " ";
	}

	outfile << "f: ";

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.force[j])
			outfile << data.force[j] << " ";
		if (reader_cnf.desired_force[j])
			outfile << data.desired_force[j] << " ";
		if (reader_cnf.filtered_force[j])
			outfile << data.filtered_force[j] << " ";
	}

	outfile << "k: ";

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.desired_cartesian_position[j])
			outfile << data.desired_cartesian_position[j] << " ";
	}

	outfile << "r: ";

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.real_cartesian_position[j])
			outfile << data.real_cartesian_position[j] << " ";
	}

	outfile << "v: ";

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.real_cartesian_vel[j])
			outfile << data.real_cartesian_vel[j] << " ";
	}

	outfile << "a: ";

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.real_cartesian_acc[j])
			outfile << data.real_cartesian_acc[j] << " ";
	}

	outfile << "t: " << data.ui_trigger;

	outfile << '\n';
}

void reader_buffer::write_header_csv(std::ofstream& outfile)
{
	outfile << "step;";
	if (reader_cnf.measure_time)
		outfile << "measure_time_sec;measure_time_nsec;";
	if (reader_cnf.servo_mode)
		outfile << "servo_mode;";
	for (int j = 0; j < master.number_of_servos; j++) {
		if (reader_cnf.desired_inc[j])
			outfile << "desired_inc[" << j << "];";
		if (reader_cnf.current_inc[j])
			outfile << "current_inc[" << j << "];";
		if (reader_cnf.pwm[j])
			outfile << "pwm[" << j << "];";
		if (reader_cnf.uchyb[j])
			outfile << "uchyb[" << j << "];";
		if (reader_cnf.abs_pos[j])
			outfile << "abs_pos[" << j << "];";
	}

	for (int j = 0; j < master.number_of_servos; j++) {
		if (reader_cnf.current_joints[j])
			outfile << "current_joints[" << j << "];";
	}

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.force[j])
			outfile << "force[" << j << "];";
		if (reader_cnf.desired_force[j])
			outfile << "desired_force[" << j << "];";
		if (reader_cnf.filtered_force[j])
			outfile << "filtered_force[" << j << "];";
	}

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.desired_cartesian_position[j])
			outfile << "desired_cartesian_position[" << j << "];";
	}

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.real_cartesian_position[j])
			outfile << "real_cartesian_position[" << j << "];";
	}

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.real_cartesian_vel[j])
			outfile << "real_cartesian_vel[" << j << "];";
	}

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.real_cartesian_acc[j])
			outfile << "real_cartesian_acc[" << j << "];";
	}

	outfile << "ui_trigger\n";
}

void reader_buffer::write_data_csv(std::ofstream& outfile, const reader_data & data)
{
	outfile << data.step << ";";
	if (reader_cnf.measure_time)
		outfile << data.measure_time.tv_sec << ";" << data.measure_time.tv_nsec << ";";
	if (reader_cnf.servo_mode)
		outfile << (data.servo_mode ? "1" : "0") << ";";
	for (int j = 0; j < master.number_of_servos; j++) {
		if (reader_cnf.desired_inc[j])
			outfile << data.desired_inc[j] << ";";
		if (reader_cnf.current_inc[j])
			outfile << data.current_inc[j] << ";";
		if (reader_cnf.pwm[j])
			outfile << data.pwm[j] << ";";
		if (reader_cnf.uchyb[j])
			outfile << data.uchyb[j] << ";";
		if (reader_cnf.abs_pos[j])
			outfile << data.abs_pos[j] << ";";
	}

	for (int j = 0; j < master.number_of_servos; j++) {
		if (reader_cnf.current_joints[j])
			outfile << data.current_joints[j] << ";";
	}

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.force[j])
			outfile << data.force[j] << ";";
		if (reader_cnf.desired_force[j])
			outfile << data.desired_force[j] << ";";
		if (reader_cnf.filtered_force[j])
			outfile << data.filtered_force[j] << ";";
	}

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.desired_cartesian_position[j])
			outfile << data.desired_cartesian_position[j] << ";";
	}

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.real_cartesian_position[j])
			outfile << data.real_cartesian_position[j] << ";";
	}

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.real_cartesian_vel[j])
			outfile << data.real_cartesian_vel[j] << ";";
	}

	for (int j = 0; j < 6; j++) {
		if (reader_cnf.real_cartesian_acc[j])
			outfile << data.real_cartesian_acc[j] << ";";
	}

	outfile << data.ui_trigger << '\n';
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
