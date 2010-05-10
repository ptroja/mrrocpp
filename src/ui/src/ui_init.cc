// ------------------------------------------------------------------------
// Proces:		UI
// Plik:			ui_init.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:
// Autor:		twiniarski/ ostatnie zmiany tkornuta
// Data:		14.03.2006
// ------------------------------------------------------------------------

/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>

#include <string.h>
#include <signal.h>
#include <dirent.h>

#include <sys/dispatch.h>

#include <fcntl.h>
#include <string.h>
#include <process.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <iostream>
#include <fstream>

#include <pthread.h>
#include <errno.h>

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "ui/ui_const.h"
#include "ui/ui.h"
#include "lib/configurator.h"
#include "lib/mis_fun.h"
#include "ui/ui_ecp.h"
#include "lib/robot_consts/conveyor_const.h"
#if defined(USE_MESSIP_SRR)
#include <messip.h>
#endif

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
#include <Pt.h>
#include <Ph.h>

extern ui_msg_def ui_msg;

lib::configurator* config;

ui_state_def ui_state;

std::ofstream *log_file_outfile;

/* Przechwycenie sygnalu */
void catch_signal(int sig) {
	int status;
	pid_t child_pid;

	// print a message
	fprintf(stderr, "UI: %s\n", strsignal(sig));

	switch (sig) {
	case SIGINT:
		UI_close();
		break;
	case SIGALRM:
		break;
	case SIGSEGV:
		signal(SIGSEGV, SIG_DFL);
		break;
	case SIGCHLD:
		child_pid = waitpid(-1, &status, 0);

		if (child_pid == -1) {
			int e = errno;
			perror("UI: waitpid()");
		} else if (child_pid == 0) {
			fprintf(stderr, "UI: no child exited\n");
		} else {
			//fprintf(stderr, "UI: child %d...\n", child_pid);
			if (WIFEXITED(status)) {
				fprintf(stderr,
						"UI: child %d exited normally with status %d\n",
						child_pid, WEXITSTATUS(status));
			}
			if (WIFSIGNALED(status)) {
#ifdef WCOREDUMP
				if (WCOREDUMP(status)) {
					fprintf(
							stderr,
							"UI: child %d terminated by signal %d (core dumped)\n",
							child_pid, WTERMSIG(status));
				} else
#endif /* WCOREDUMP */
				{
					fprintf(stderr, "UI: child %d terminated by signal %d\n",
							child_pid, WTERMSIG(status));
				}
			}
			if (WIFSTOPPED(status)) {
				fprintf(stderr, "UI: child %d stopped\n", child_pid);
			}
			if (WIFCONTINUED(status)) {
				fprintf(stderr, "UI: child %d resumed\n", child_pid);
			}
		}
		break;
	default:
		fprintf(stderr, "UI: unknown signal (%d)\n", sig);
	} // end: switch
}

void UI_close(void) {
	printf("UI CLOSING\n");
	delay(100);// czas na ustabilizowanie sie edp
	ui_state.ui_state = 2;// funcja OnTimer dowie sie ze aplikacja ma byc zamknieta
}

int init(PtWidget_t *link_instance, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	/* eliminate 'unreferenced' warnings */
	link_instance = link_instance, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_STARTING);

	struct utsname sysinfo;
	char* cwd;
	char buff[PATH_MAX + 1];

	signal(SIGINT, &catch_signal);// by y aby uniemozliwic niekontrolowane zakonczenie aplikacji ctrl-c z kalwiatury
	signal(SIGALRM, &catch_signal);
	signal(SIGSEGV, &catch_signal);
#ifdef PROCESS_SPAWN_RSH
	signal(SIGCHLD, &catch_signal);
#endif /* PROCESS_SPAWN_RSH */

	lib::set_thread_priority(pthread_self(), MAX_PRIORITY - 6);

	config = NULL;
	ui_msg.ui = NULL;
	ui_state.ui_state = 1;// ui working

	ui_state.irp6_on_track.edp.state = -1; // edp nieaktywne
	ui_state.irp6_on_track.edp.last_state = -1; // edp nieaktywne
	ui_state.irp6_on_track.ecp.trigger_fd = -1;
	ui_state.irp6_on_track.edp.section_name = EDP_IRP6_ON_TRACK_SECTION;
	ui_state.irp6_on_track.ecp.section_name = ECP_IRP6_ON_TRACK_SECTION;

	ui_state.irp6ot_tfg.edp.state = -1; // edp nieaktywne
	ui_state.irp6ot_tfg.edp.last_state = -1; // edp nieaktywne
	ui_state.irp6ot_tfg.ecp.trigger_fd = -1;
	ui_state.irp6ot_tfg.edp.section_name = EDP_IRP6OT_TFG_SECTION;
	ui_state.irp6ot_tfg.ecp.section_name = ECP_IRP6OT_TFG_SECTION;

	ui_state.irp6_postument.edp.state = -1; // edp nieaktywne
	ui_state.irp6_postument.edp.last_state = -1; // edp nieaktywne
	ui_state.irp6_postument.ecp.trigger_fd = -1;
	ui_state.irp6_postument.edp.section_name = EDP_IRP6_POSTUMENT_SECTION;
	ui_state.irp6_postument.ecp.section_name = ECP_IRP6_POSTUMENT_SECTION;

	ui_state.irp6p_tfg.edp.state = -1; // edp nieaktywne
	ui_state.irp6p_tfg.edp.last_state = -1; // edp nieaktywne
	ui_state.irp6p_tfg.ecp.trigger_fd = -1;
	ui_state.irp6p_tfg.edp.section_name = EDP_IRP6P_TFG_SECTION;
	ui_state.irp6p_tfg.ecp.section_name = ECP_IRP6P_TFG_SECTION;

	ui_state.speaker.edp.state = -1; // edp nieaktywne
	ui_state.speaker.edp.last_state = -1; // edp nieaktywne
	ui_state.speaker.ecp.trigger_fd = -1;
	ui_state.speaker.edp.section_name = EDP_SPEAKER_SECTION;
	ui_state.speaker.ecp.section_name = ECP_SPEAKER_SECTION;

	ui_state.conveyor.edp.state = -1; // edp nieaktywne
	ui_state.conveyor.edp.last_state = -1; // edp nieaktywne
	ui_state.conveyor.ecp.trigger_fd = -1;
	ui_state.conveyor.edp.section_name = EDP_CONVEYOR_SECTION;
	ui_state.conveyor.ecp.section_name = ECP_CONVEYOR_SECTION;

	ui_state.irp6_mechatronika.edp.state = -1; // edp nieaktywne
	ui_state.irp6_mechatronika.edp.last_state = -1; // edp nieaktywne
	ui_state.irp6_mechatronika.ecp.trigger_fd = -1;
	ui_state.irp6_mechatronika.edp.section_name = EDP_IRP6_MECHATRONIKA_SECTION;
	ui_state.irp6_mechatronika.ecp.section_name = ECP_IRP6_MECHATRONIKA_SECTION;

	ui_state.spkm.edp.state = -1; // edp nieaktywne
	ui_state.spkm.edp.last_state = -1; // edp nieaktywne
	ui_state.spkm.ecp.trigger_fd = -1;
	ui_state.spkm.edp.section_name = EDP_SPKM_SECTION;
	ui_state.spkm.ecp.section_name = ECP_SPKM_SECTION;

	ui_state.bird_hand.edp.state = -1; // edp nieaktywne
	ui_state.bird_hand.edp.last_state = -1; // edp nieaktywne
	ui_state.bird_hand.ecp.trigger_fd = -1;
	ui_state.bird_hand.edp.section_name = EDP_BIRD_HAND_SECTION;
	ui_state.bird_hand.ecp.section_name = ECP_BIRD_HAND_SECTION;

	ui_state.smb.edp.state = -1; // edp nieaktywne
	ui_state.smb.edp.last_state = -1; // edp nieaktywne
	ui_state.smb.ecp.trigger_fd = -1;
	ui_state.smb.edp.section_name = EDP_SMB_SECTION;
	ui_state.smb.ecp.section_name = ECP_SMB_SECTION;

	ui_state.shead.edp.state = -1; // edp nieaktywne
	ui_state.shead.edp.last_state = -1; // edp nieaktywne
	ui_state.shead.ecp.trigger_fd = -1;
	ui_state.shead.edp.section_name = EDP_SHEAD_SECTION;
	ui_state.shead.ecp.section_name = ECP_SHEAD_SECTION;

	ui_state.file_window_mode = FSTRAJECTORY; // uczenie
	ui_state.all_edps = UI_ALL_EDPS_NONE_EDP_LOADED;
	ui_state.mp.state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	ui_state.mp.last_state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	ui_state.mp.pid = -1;
	ui_state.is_task_window_open = false;// informacja czy okno zadanai jest otwarte
	ui_state.is_process_control_window_open = false;// informacja czy okno sterowania procesami jest otwarte
	ui_state.process_control_window_renew = true;
	ui_state.is_file_selection_window_open = false;
	ui_state.is_wind_irp6ot_int_open = false;
	ui_state.is_wind_irp6p_int_open = false;
	ui_state.is_wind_irp6m_int_open = false;
	ui_state.is_wind_polycrank_int_open = false;
	ui_state.is_wind_irp6ot_inc_open = false;
	ui_state.is_wind_irp6p_inc_open = false;
	ui_state.is_wind_irp6m_inc_open = false;
	ui_state.is_wind_polycrank_inc_open = false;
	ui_state.is_wind_irp6ot_xyz_euler_zyz_open = false;
	ui_state.is_wind_irp6p_xyz_euler_zyz_open = false;
	ui_state.is_wind_irp6m_xyz_euler_zyz_open = false;
	ui_state.is_wind_irp6ot_xyz_angle_axis_open = false;
	ui_state.is_wind_irp6p_xyz_angle_axis_open = false;
	ui_state.is_wind_irp6m_xyz_angle_axis_open = false;
	ui_state.is_wind_irp6ot_xyz_angle_axis_ts_open = false;
	ui_state.is_wind_irp6p_xyz_angle_axis_ts_open = false;
	ui_state.is_wind_irp6m_xyz_angle_axis_ts_open = false;
	ui_state.is_wind_irp6ot_xyz_euler_zyz_ts_open = false;
	ui_state.is_wind_irp6p_xyz_euler_zyz_ts_open = false;
	ui_state.is_wind_irp6m_xyz_euler_zyz_ts_open = false;
	ui_state.is_teaching_window_open = false;
	ui_state.is_wind_conveyor_moves_open = false;
	ui_state.is_wind_irp6ot_tfg_moves_open = false;
	ui_state.is_wind_irp6p_tfg_moves_open = false;
	ui_state.is_wind_irp6ot_kinematic_open = false;
	ui_state.is_wind_irp6p_kinematic_open = false;
	ui_state.is_wind_irp6m_kinematic_open = false;
	ui_state.is_wind_speaker_play_open = false;

	ui_state.is_wind_irp6ot_xyz_aa_relative_open = false;
	ui_state.is_wind_irp6p_xyz_aa_relative_open = false;

	ui_state.is_wind_irp6ot_servo_algorithm_open = false;
	ui_state.is_wind_irp6p_servo_algorithm_open = false;
	ui_state.is_wind_irp6ot_tfg_servo_algorithm_open = false;
	ui_state.is_wind_irp6p_tfg_servo_algorithm_open = false;
	ui_state.is_wind_irp6m_servo_algorithm_open = false;
	ui_state.is_wind_conv_servo_algorithm_open = false;

	ui_state.is_mp_and_ecps_active = false;
	// ui_state.is_any_edp_active = false;


	ui_state.irp6_on_track.edp.is_synchronised = false;
	ui_state.irp6_postument.edp.is_synchronised = false;
	ui_state.irp6ot_tfg.edp.is_synchronised = false;
	ui_state.irp6p_tfg.edp.is_synchronised = false;
	ui_state.conveyor.edp.is_synchronised = false;
	ui_state.spkm.edp.is_synchronised = false;
	ui_state.smb.edp.is_synchronised = false;
	ui_state.shead.edp.is_synchronised = false;
	ui_state.speaker.edp.is_synchronised = false;
	ui_state.irp6_mechatronika.edp.is_synchronised = false;

	// ustalenie katalogow UI

	if (uname(&sysinfo) == -1) {
		perror("uname");
	}

	cwd = getcwd(buff, PATH_MAX + 1);
	if (cwd == NULL) {
		perror("Blad cwd w UI");
	}

	ui_state.ui_node_name = sysinfo.nodename;
	ui_state.is_sr_thread_loaded = false;

	ui_state.binaries_local_path = cwd;
	ui_state.mrrocpp_local_path = cwd;
	ui_state.mrrocpp_local_path.erase(ui_state.mrrocpp_local_path.length() - 3);// kopiowanie lokalnej sciezki bez "bin" - 3 znaki
	ui_state.binaries_network_path = "/net/";
	ui_state.binaries_network_path += ui_state.ui_node_name;
	ui_state.binaries_network_path += ui_state.binaries_local_path;
	ui_state.binaries_network_path += "/";// wysylane jako argument do procesow potomnych (mp_m i dalej)
	// printf( "system name  : %s\n", ui_state.binaries_network_path);

	// sciezka dla okna z wyborem pliku podczas wybor trajektorii dla uczenia
	ui_state.teach_filesel_fullpath = "/net/";
	ui_state.teach_filesel_fullpath += ui_state.ui_node_name;
	ui_state.teach_filesel_fullpath += ui_state.mrrocpp_local_path;
	//	ui_state.teach_filesel_fullpath += "trj";
	// 	printf("abba: %s\n", ui_state.teach_filesel_fullpath);

	// sciezka dla okna z wyborem pliku z trajektoria podczas wyboru pliku konfiguracyjnego
	ui_state.config_file_fullpath = "/net/";
	ui_state.config_file_fullpath += ui_state.ui_node_name;
	ui_state.config_file_fullpath += ui_state.mrrocpp_local_path;
	//	ui_state.config_file_fullpath += "configs";

	// printf ("Remember to create gns server\n");

	// pierwsze zczytanie pliku konfiguracyjnego (aby pobrac nazwy dla pozostalych watkow UI)
	if (get_default_configuration_file_name() >= 1) // zczytaj nazwe pliku konfiguracyjnego
	{
		initiate_configuration();
		// sprawdza czy sa postawione gns's i ew. stawia je
		// uwaga serwer musi byc wczesniej postawiony
		check_gns();
	} else {
		printf("Blad manage_default_configuration_file\n");
		PtExit(EXIT_SUCCESS);
	}

	create_threads();

	// Zablokowanie domyslnej obslugi sygnalu SIGINT w watkach UI_SR i UI_COMM


	// kolejne zczytanie pliku konfiguracyjnego
	if (get_default_configuration_file_name() == 1) // zczytaj nazwe pliku konfiguracyjnego
	{
		reload_whole_configuration();

	} else {
		printf("Blad manage_default_configuration_file\n");
		PtExit(EXIT_SUCCESS);
	}

	// inicjacja pliku z logami sr
	check_gns();

	time_t time_of_day;
	char file_date[50];
	char log_file_with_dir[100];
	char file_name[50];

	time_of_day = time(NULL);
	strftime(file_date, 40, "%g%m%d_%H-%M-%S", localtime(&time_of_day));

	sprintf(file_name, "/%s_sr_log", file_date);

	// 	strcpy(file_name,"/pomiar.p");
	strcpy(log_file_with_dir, "../logs/");

	if (access(log_file_with_dir, R_OK) != 0) {
		mkdir(log_file_with_dir, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH
				| S_IXOTH);
	}

	strcat(log_file_with_dir, file_name);

	log_file_outfile = new std::ofstream(log_file_with_dir, std::ios::out);

	if (!(*log_file_outfile)) {
		std::cerr << "Cannot open file: " << file_name << '\n';
		perror("because of");
	}

	manage_interface();

	return (Pt_CONTINUE);

}
