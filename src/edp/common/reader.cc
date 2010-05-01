//
// READER - watek do buforowania danych pomiarowych i ich zapisu do pliku
// Date: maj 2006
//

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <sys/wait.h>
#include <sys/types.h>
#if !defined(USE_MESSIP_SRR)
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/netmgr.h>
#else
#include "messip_dataport.h"
#endif
#include <errno.h>
#include <pthread.h>
#include <time.h>

#include <boost/scoped_array.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "edp/common/edp_e_motor_driven.h"
#include "edp/common/reader.h"

namespace mrrocpp {
namespace edp {
namespace common {

reader_buffer::reader_buffer(motor_driven_effector &_master) :
	master (_master), new_data(false)
{
	thread_id = new boost::thread(boost::bind(&reader_buffer::operator(), this));
}

reader_buffer::~reader_buffer()
{
	// TODO: stop (interrupt?) the thread
	//thread_id->interrupt();
	//thread_id->join(); // join it
	//delete thread_id; // delete a pointer
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
		reader_meassures_dir = master.config.value<std::string>("reader_meassures_dir", UI_SECTION);
	} else {
		reader_meassures_dir = master.config.return_default_reader_measures_path();
	}

	std::string robot_filename = master.config.value<std::string>("reader_attach_point");

	if (master.config.exists("reader_samples"))
		nr_of_samples = master.config.value<int>("reader_samples");
	else
		nr_of_samples = 1000;

	reader_cnf.step = 1;
	reader_cnf.servo_mode = master.config.check_config("servo_tryb");
	reader_cnf.msec = master.config.check_config("msec");

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

	// NOTE: readed buffer has to be allocated on heap (using "new" operator) due to huge size
	// boost::scoped_array takes care of deallocating in case of exception
	boost::circular_buffer<reader_data> reader_buf(nr_of_samples);

//	fprintf(stderr, "reader buffer size %lluKB\n", nr_of_samples*sizeof(reader_data)/1024);

	// by Y komuniakicja pomiedzy ui i reader'em rozwiazalem poprzez pulsy
	// powolanie kanalu komunikacyjnego do odbioru pulsow sterujacych
#if !defined(USE_MESSIP_SRR)
	name_attach_t *my_attach; // nazwa kanalu komunikacyjnego

	if ((my_attach = name_attach(NULL, master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "reader_attach_point").c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
#else
	messip_channel_t *my_attach;

	if ((my_attach = messip::port_create(
			master.config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "reader_attach_point")))
			== NULL) {
#endif
		perror("Failed to attach pulse chanel for READER");
		master.msg->message("Failed to attach pulse chanel for READER");
		//  throw MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}

	// GLOWNA PETLA Z OCZEKIWANIEM NA ZLECENIE POMIAROW
	for (;;) {

		// ustawienie priorytetu watku
		lib::set_thread_priority(pthread_self(), MAX_PRIORITY-10);

		start = false; // okresla czy odebrano juz puls rozpoczecia pomiarow

		// dopoki nie przyjdzie puls startu
		while (!start) {
#if !defined(USE_MESSIP_SRR)
			_pulse_msg ui_msg;// wiadomosc z ui

			int rcvid = MsgReceive(my_attach->chid, &ui_msg, sizeof(ui_msg), NULL);

			if (rcvid == -1) {/* Error condition, exit */
				perror("blad receive w reader");
				break;
			}

			if (rcvid == 0) {/* Pulse received */
				//  printf("reader puls\n");
				switch (ui_msg.hdr.code) {
					case _PULSE_CODE_DISCONNECT:
					ConnectDetach(ui_msg.hdr.scoid);
					break;
					case _PULSE_CODE_UNBLOCK:
					break;
					default:
					if (ui_msg.hdr.code==READER_START) { // odebrano puls start
						start = true;
//#ifdef DOCENT_SENSOR
						master.onReaderStarted();
//#endif
					}
				}
				continue;
			}

			/* A QNX IO message received, reject */
			if (ui_msg.hdr.type >= _IO_BASE && ui_msg.hdr.type <= _IO_MAX) {
				MsgReply(rcvid, EOK, 0, 0);
				continue;
			}

			/* A message (presumable ours) received, handle */
			fprintf(stderr, "reader server receive strange message of type: %d\n", ui_msg.data);
			MsgReply(rcvid, EOK, 0, 0);
			rcvid = MsgReceive(my_attach->chid, &ui_msg, sizeof(ui_msg), NULL);
#else
			int32_t type, subtype;
			int rcvid = messip::port_receive_pulse(my_attach, type, subtype);

			if (rcvid >= 0) {
				if (type == READER_START) {
					start = true;
				}
			}
#endif
		}

		master.msg->message("measures started");

		lib::set_thread_priority(pthread_self(), MAX_PRIORITY+1);

		// dopoki nie przyjdzie puls stopu
		do {
			// sekcja krytyczna odczytu danych pomiarowych dla biezacego kroku
			{
				boost::mutex::scoped_lock lock(reader_mutex);

				while(!new_data) {
					// czekamy na opuszcenie semafora przez watek EDP_SERVO (co mikrokrok)
					cond.wait(lock);
				}

				step_data.ui_trigger = ui_trigger;

				// przepisanie danych dla biezacego kroku do bufora lokalnego reader
				reader_buf.push_back(step_data);
			}

			// warunkowy odbior pulsu (o ile przyszedl)
			stop = false;
			ui_trigger = false;

#if !defined(USE_MESSIP_SRR)
			_pulse_msg ui_msg;// wiadomosc z ui

			// sprawdzamy pulsu stopu
			if(TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE, NULL, NULL, NULL) == -1) {
				perror("TimerTimeout()");
			}

			int rcvid = MsgReceive(my_attach->chid, &ui_msg, sizeof(ui_msg), NULL);

			if (rcvid == -1 && errno != ETIMEDOUT) {/* Error condition, exit */
				perror("reader::MsgReceive()");
			}

			if (rcvid == 0) {/* Pulse received */
				// printf("reader puls\n");
				switch (ui_msg.hdr.code) {
					case _PULSE_CODE_DISCONNECT:
					ConnectDetach(ui_msg.hdr.scoid);
					break;
					case _PULSE_CODE_UNBLOCK:
					break;
					default:
					if (ui_msg.hdr.code==READER_STOP) {
						stop = true; // dostalismy puls STOP
//#ifdef DOCENT_SENSOR
						master.onReaderStopped();
//#endif
					} else if (ui_msg.hdr.code==READER_TRIGGER) {
						ui_trigger = true; // dostalismy puls TRIGGER
					}

				}
			}

			if (rcvid> 0) {
				/* A QNX IO message received, reject */
				if (ui_msg.hdr.type >= _IO_BASE && ui_msg.hdr.type <= _IO_MAX) {
					MsgReply(rcvid, EOK, 0, 0);
				} else {
					/* A message (presumable ours) received, handle */
					printf("reader server receive strange message of type: %d\n", ui_msg.data);
					MsgReply(rcvid, EOK, 0, 0);
				}
			}
#else
			int32_t type, subtype;
			int rcvid = messip::port_receive_pulse(my_attach, type, subtype, 0);

			if (rcvid >= 0) {
				if (type == READER_STOP) {
					stop = true;
				} else if (type == READER_TRIGGER) {
					ui_trigger = true;
				}
			}
#endif
		} while (!stop); // dopoki nie przyjdzie puls stopu

		lib::set_thread_priority(pthread_self(), 1);// Najnizszy priorytet podczas proby zapisu do pliku
		master.msg->message("measures stopped");

		// przygotowanie nazwy pliku do ktorego beda zapisane pomiary
		time_of_day = time(NULL);
		strftime(file_date, 40, "%g%m%d_%H-%M-%S", localtime(&time_of_day));

		sprintf(file_name, "/%s_%s_pomiar-%d", file_date, robot_filename.c_str(), ++file_counter);
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

			// TODO: sprawdzenie czy bufor byl przepelniony i odpowiednie
			// przygotowanie granic bufora przy zapi sie do pliku

			// dla calego horyzontu pomiarow

			while (!reader_buf.empty()) {
				// zapis pomiarow z biezacego kroku do pliku
				// printf("EDP %f\n", reader_buf.front().desired_cartesian_position[1]);

				reader_data & data = reader_buf.front();

				outfile << data.step << " ";
				if (reader_cnf.msec)
					outfile << data.msec << " ";
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

				reader_buf.pop_front();
			} // end for(i = 0; i < msr_counter; i++)

			master.msg->message("file writing is finished");
		}
	} // end: for (;;)
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
