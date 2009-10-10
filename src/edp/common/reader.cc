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
#include <errno.h>
#include <sys/wait.h>
#include <sys/types.h>
#if !defined(USE_MESSIP_SRR)
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/netmgr.h>
#else
#include "lib/messip/messip.h"
#endif
#include <errno.h>
#include <pthread.h>
#include <time.h>

#include <boost/scoped_array.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "edp/common/edp_e_manip_and_conv.h"

namespace mrrocpp {
namespace edp {
namespace common {

void * manip_and_conv_effector::reader_thread_start(void* arg)
{
	return static_cast<manip_and_conv_effector*> (arg)->reader_thread(arg);
}

void * manip_and_conv_effector::reader_thread(void* arg)
{
	uint64_t k;
	uint64_t nr_of_samples; // maksymalna liczba pomiarow
	uint64_t msr_nr; // numer pomiaru
	int przepelniony; // czy bufor byl przepelniony
	int msr_counter; // liczba pomiarow, ktore maja byc zapisane do pliku
	uint64_t e; // kod bledu systemowego
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

	if (config.exists("reader_meassures_dir")) {
		reader_meassures_dir = config.return_string_value("reader_meassures_dir", "[ui]");
	} else {
		reader_meassures_dir = config.return_default_reader_measures_path();
	}

	std::string robot_filename = config.return_string_value("reader_attach_point");

	if (config.exists("reader_samples"))
		nr_of_samples = config.return_int_value("reader_samples");
	else
		nr_of_samples = 1000;

	rb_obj.reader_cnf.step = 1;
	rb_obj.reader_cnf.servo_mode = check_config("servo_tryb");
	rb_obj.reader_cnf.msec = check_config("msec");

	char tmp_string[50];

	for (int j = 0; j < MAX_SERVOS_NR; j++) {

		sprintf(tmp_string, "desired_inc_%d", j);
		rb_obj.reader_cnf.desired_inc[j] = check_config(tmp_string);

		sprintf(tmp_string, "current_inc_%d", j);
		rb_obj.reader_cnf.current_inc[j] = check_config(tmp_string);

		sprintf(tmp_string, "pwm_%d", j);
		rb_obj.reader_cnf.pwm[j] = check_config(tmp_string);

		sprintf(tmp_string, "uchyb_%d", j);
		rb_obj.reader_cnf.uchyb[j] = check_config(tmp_string);

		sprintf(tmp_string, "abs_pos_%d", j);
		rb_obj.reader_cnf.abs_pos[j] = check_config(tmp_string);

		sprintf(tmp_string, "current_joints_%d", j);
		rb_obj.reader_cnf.current_joints[j] = check_config(tmp_string);

		if (j < 6) {
			sprintf(tmp_string, "force_%d", j);
			rb_obj.reader_cnf.force[j] = check_config(tmp_string);

			sprintf(tmp_string, "desired_force_%d", j);
			rb_obj.reader_cnf.desired_force[j] = check_config(tmp_string);

			sprintf(tmp_string, "filtered_force_%d", j);
			rb_obj.reader_cnf.filtered_force[j] = check_config(tmp_string);

			sprintf(tmp_string, "current_cartesian_position_%d", j);
			rb_obj.reader_cnf.current_cartesian_position[j] = check_config(tmp_string);

			sprintf(tmp_string, "real_cartesian_position_%d", j);
			rb_obj.reader_cnf.real_cartesian_position[j] = check_config(tmp_string);

			sprintf(tmp_string, "real_cartesian_vel_%d", j);
			rb_obj.reader_cnf.real_cartesian_vel[j] = check_config(tmp_string);

			sprintf(tmp_string, "real_cartesian_acc_%d", j);
			rb_obj.reader_cnf.real_cartesian_acc[j] = check_config(tmp_string);
		}
	}

	// ustawienie priorytetu watku
	lib::set_thread_priority(pthread_self(), MAX_PRIORITY-10);

	// alokacja pamieci pod lokalny bufor z pomiarami

	// NOTE: readed buffer has to be allocated on heap (using "new" operator) due to huge size
	// boost::scoped_array takes care of deallocating in case of exception
	boost::scoped_array<reader_data> r_measptr (new reader_data[nr_of_samples]);
//	fprintf(stderr, "reader buffer size %lluKB\n", nr_of_samples*sizeof(reader_data)/1024);

	// by Y komuniakicja pomiedzy ui i reader'em rozwiazalem poprzez pulsy
	// powolanie kanalu komunikacyjnego do odbioru pulsow sterujacych
#if !defined(USE_MESSIP_SRR)
	name_attach_t *my_attach; // nazwa kanalu komunikacyjnego

	if ((my_attach = name_attach(NULL, config.return_attach_point_name(lib::configurator::CONFIG_SERVER, "reader_attach_point").c_str(), NAME_FLAG_ATTACH_GLOBAL)) == NULL) {
#else
	messip_channel_t *my_attach;

	if ((my_attach = messip_channel_create(NULL, config.return_attach_point_name(lib::configurator::CONFIG_SERVER,
			"reader_attach_point").c_str(), MESSIP_NOTIMEOUT, 0)) == NULL) {
#endif
		e = errno;
		perror("Failed to attach pulse chanel for READER\n");
		msg->message("Failed to attach pulse chanel for READER");
		//  throw MP_main_error(lib::SYSTEM_ERROR, (uint64_t) 0);
	}

	// GLOWNA PETLA Z OCZEKIWANIEM NA ZLECENIE POMIAROW
	for (;;) {

		msr_nr = 0; // wyzerowanie liczby pomiarow
		przepelniony = 0; // bufor narazie nie jest przepelniony

		start = false; // okresla czy odebrano juz puls rozpoczecia pomiarow

		// dopoki nie przyjdzie puls startu
		while (!start) {
#if !defined(USE_MESSIP_SRR)
			_pulse_msg ui_msg;// wiadomosc z ui

			int rcvid = MsgReceive(my_attach->chid, &ui_msg, sizeof(ui_msg), NULL);

			if (rcvid == -1) {/* Error condition, exit */
				perror("blad receive w reader\n");
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
#ifdef DOCENT_SENSOR
						onReaderStarted();
#endif
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
			int rcvid = messip_receive(my_attach, &type, &subtype, NULL, 0, MESSIP_NOTIMEOUT);

			if (rcvid >= 0) {
				if (type == READER_START) {
					start = true;
				}
			}
#endif
		}

		msg->message("measures started");

		lib::set_thread_priority(pthread_self(), MAX_PRIORITY+1);

		rb_obj.reader_wait_for_new_step();
		// dopoki nie przyjdzie puls stopu
		do {
			// czekamy na opuszcenie semafora przez watek EDP_SERVO (co mikrokrok)
			rb_obj.reader_wait_for_new_step();

			// sekcja krytyczna odczytu danych pomiarowych dla biezacego kroku
			rb_obj.lock_mutex();

			rb_obj.step_data.ui_trigger = ui_trigger;

			// printf("EDPX: %f\n", rb_obj.step_data.current_cartesian_position[1]);
			// przepisanie danych dla biezacego kroku do bufora lokalnego reader
			memcpy(&(r_measptr[msr_nr]), &rb_obj.step_data, sizeof(reader_data));

			rb_obj.unlock_mutex();

			// wykrycie przepelnienia
			if ((++msr_nr) >= nr_of_samples) {
				msr_nr = 0;
				przepelniony = 1;
			}

			// warunkowy odbior pulsu (o ile przyszedl)
			stop = false;
			ui_trigger = false;

#if !defined(USE_MESSIP_SRR)
			_pulse_msg ui_msg;// wiadomosc z ui

			struct sigevent stop_event; // do oblugi pulsu stopu

			stop_event.sigev_notify = SIGEV_UNBLOCK;
			TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE, &stop_event, NULL, NULL); // czekamy na odbior pulsu stopu
			int rcvid = MsgReceive(my_attach->chid, &ui_msg, sizeof(ui_msg), NULL);

			if (rcvid == -1) {/* Error condition, exit */
				// perror("blad receive w reader\n");
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
#ifdef DOCENT_SENSOR
						onReaderStopped();
#endif
					} else if (ui_msg.hdr.code==READER_TRIGGER) {
						ui_trigger = true; // dostaliśmy puls TRIGGER
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
			int rcvid = messip_receive(my_attach, &type, &subtype, NULL, 0, 0);

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
		msg->message("measures stopped");

		// przygotowanie nazwy pliku do ktorego beda zapisane pomiary
		time_of_day = time(NULL);
		strftime(file_date, 40, "%g%m%d_%H-%M-%S", localtime(&time_of_day));

		sprintf(file_name, "/%s_%s_pomiar-%d", file_date, robot_filename.c_str(), ++file_counter);
		strcpy(config_file_with_dir, reader_meassures_dir.c_str());

		strcat(config_file_with_dir, file_name);

        std::ofstream outfile(config_file_with_dir, std::ios::out);
		if (!outfile) // jesli plik nie instnieje
		{
			std::cerr << "Cannot open file: " << file_name << '\n';
			perror("because of");
			msg->message("cannot open destination file");
		} else { // jesli plik istnieje

			// sprawdzenie czy bufor byl przepelniony i odpowiednie przygotowanie granic bufora przy zapi sie do pliku
			if (przepelniony) {
				k = msr_nr;
				msr_counter = nr_of_samples;
			} else {
				k = 0;
				msr_counter = msr_nr;
			}

			// dla calego horyzontu pomiarow

			for (int i = 0; i < msr_counter; i++) {

				if (k == nr_of_samples)
					k = 0;

				// zapis pomiarow z biezacego kroku do pliku
				// printf("EDP %f\n", r_measptr[k].current_cartesian_position[1]);

				outfile << r_measptr[k].step << " ";
				if (rb_obj.reader_cnf.msec)
					outfile << r_measptr[k].msec << " ";
				if (rb_obj.reader_cnf.servo_mode)
					outfile << (r_measptr[k].servo_mode ? "1" : "0") << " ";

				for (int j = 0; j < MAX_SERVOS_NR; j++) {
					if (rb_obj.reader_cnf.desired_inc[j])
						outfile << r_measptr[k].desired_inc[j] << " ";
					if (rb_obj.reader_cnf.current_inc[j])
						outfile << r_measptr[k].current_inc[j] << " ";
					if (rb_obj.reader_cnf.pwm[j])
						outfile << r_measptr[k].pwm[j] << " ";
					if (rb_obj.reader_cnf.uchyb[j])
						outfile << r_measptr[k].uchyb[j] << " ";
					if (rb_obj.reader_cnf.abs_pos[j])
						outfile << r_measptr[k].abs_pos[j] << " ";
				}

				outfile << "j: ";

				for (int j = 0; j < MAX_SERVOS_NR; j++) {
					if (rb_obj.reader_cnf.current_joints[j])
						outfile << r_measptr[k].current_joints[j] << " ";
				}

				outfile << "f: ";

				for (int j = 0; j < 6; j++) {
					if (rb_obj.reader_cnf.force[j])
						outfile << r_measptr[k].force[j] << " ";
					if (rb_obj.reader_cnf.desired_force[j])
						outfile << r_measptr[k].desired_force[j] << " ";
					if (rb_obj.reader_cnf.filtered_force[j])
						outfile << r_measptr[k].filtered_force[j] << " ";
				}

				outfile << "k: ";

				for (int j = 0; j < 6; j++) {
					if (rb_obj.reader_cnf.current_cartesian_position[j])
						outfile << r_measptr[k].current_cartesian_position[j] << " ";
				}

				outfile << "r: ";

				for (int j = 0; j < 6; j++) {
					if (rb_obj.reader_cnf.real_cartesian_position[j])
						outfile << r_measptr[k].real_cartesian_position[j] << " ";
				}

				outfile << "v: ";

				for (int j = 0; j < 6; j++) {
					if (rb_obj.reader_cnf.real_cartesian_vel[j])
						outfile << r_measptr[k].real_cartesian_vel[j] << " ";
				}

				outfile << "a: ";

				for (int j = 0; j < 6; j++) {
					if (rb_obj.reader_cnf.real_cartesian_acc[j])
						outfile << r_measptr[k].real_cartesian_acc[j] << " ";
				}

				outfile << "t: " << r_measptr[k].ui_trigger;

				outfile << '\n';

				k++;
			} // end for(i = 0; i < msr_counter; i++)

			// zamkniecie pliku
			outfile.close();
			msg->message("file writing is finished");
		}

		lib::set_thread_priority(pthread_self(), MAX_PRIORITY-10);

	} // end: for (;;)

	return NULL;
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
