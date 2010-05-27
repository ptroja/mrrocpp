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

#include "ui/ui_class.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "ui/ui_const.h"
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

Ui ui;

ui_state_def ui_state;

/* Przechwycenie sygnalu */
void catch_signal(int sig) {
	int status;
	pid_t child_pid;

	// print a message
	fprintf(stderr, "UI: %s\n", strsignal(sig));

	switch (sig) {
	case SIGINT:
		ui.UI_close();
		break;
	case SIGALRM:
		break;
	case SIGSEGV:
		signal(SIGSEGV, SIG_DFL);
		break;
	case SIGCHLD:
		child_pid = waitpid(-1, &status, 0);

		if (child_pid == -1) {
			//	int e = errno;
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

int init(PtWidget_t *link_instance, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	/* eliminate 'unreferenced' warnings */
	link_instance = link_instance, apinfo = apinfo, cbinfo = cbinfo;

	ui.irp6p_tfg.state.edp.state = -1; // edp nieaktywne
	ui.irp6p_tfg.state.edp.last_state = -1; // edp nieaktywne
	ui.irp6p_tfg.state.ecp.trigger_fd = -1;
	ui.irp6p_tfg.state.edp.section_name = EDP_IRP6P_TFG_SECTION;
	ui.irp6p_tfg.state.ecp.section_name = ECP_IRP6P_TFG_SECTION;

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

	ui_state.is_wind_irp6m_int_open = false;
	ui_state.is_wind_polycrank_int_open = false;

	ui_state.is_wind_irp6m_inc_open = false;
	ui_state.is_wind_polycrank_inc_open = false;

	ui_state.is_wind_irp6m_xyz_euler_zyz_open = false;

	ui_state.is_wind_irp6m_xyz_angle_axis_open = false;

	ui_state.is_wind_irp6m_xyz_angle_axis_ts_open = false;

	ui_state.is_wind_irp6m_xyz_euler_zyz_ts_open = false;

	ui_state.is_wind_conveyor_moves_open = false;

	ui.irp6p_tfg.is_wind_irp6p_tfg_moves_open = false;

	ui_state.is_wind_irp6m_kinematic_open = false;
	ui_state.is_wind_speaker_play_open = false;

	ui.irp6p_tfg.is_wind_irp6p_tfg_servo_algorithm_open = false;
	ui_state.is_wind_irp6m_servo_algorithm_open = false;
	ui_state.is_wind_conv_servo_algorithm_open = false;

	ui.irp6p_tfg.state.edp.is_synchronised = false;
	ui_state.conveyor.edp.is_synchronised = false;
	ui_state.spkm.edp.is_synchronised = false;
	ui_state.smb.edp.is_synchronised = false;
	ui_state.shead.edp.is_synchronised = false;
	ui_state.speaker.edp.is_synchronised = false;
	ui_state.irp6_mechatronika.edp.is_synchronised = false;

	// some variables initialization
	ui.init();

	return (Pt_CONTINUE);

}
