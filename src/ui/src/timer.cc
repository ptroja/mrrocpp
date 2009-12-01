/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>// Y&7
#include <time.h>
#include <iostream>
#include <fstream>

#include "lib/srlib.h"
// #include "ecp/common/ecp.h"
#include "lib/com_buf.h"
#include "ui/ui.h"


/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern int busy;
extern ui_sr_buffer* ui_sr_obj;
extern ui_msg_def ui_msg;
extern ui_state_def ui_state;
extern std::ofstream *log_file_outfile;

int
OnTimer( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

		//fprintf(stderr, "OnTimer()\n");

	#define CHECK_SPEAKER_STATE_ITER 10 // co ile iteracji ma byc sprawdzony stan speakera

	static int closing_delay_counter; // do odliczania czasu do zamkniecia aplikacji
	static int Iteration_counter = 0; // licznik uruchomienia fukcji


	Iteration_counter++;

	if ((Iteration_counter%CHECK_SPEAKER_STATE_ITER)==0) {
		if (ui_state.is_wind_speaker_play_open) // otworz okno
		{
			speaker_check_state(widget, apinfo, cbinfo);
		}
	}

	if (ui_sr_obj->check_new_msg() == 0) { // by Y jesli mamy co wypisywac
	ui_sr_obj->lock_mutex();
// 	printf("timer\n");
	int attributes_mask;
	PtMultiTextAttributes_t attr;

	// char buffer[ 80 ];

	char current_line[400];

	do { // dopoki mamy co wypisywac

		ui_sr_obj->reader_buf_position++;
		ui_sr_obj->reader_buf_position %= UI_SR_BUFFER_LENGHT;


		snprintf(current_line, 100, "%-10s",
				ui_sr_obj->message_buffer[ui_sr_obj->reader_buf_position].host_name);
		strcat(current_line, "  ");
		strftime( current_line+12, 100, "%H:%M:%S",
				localtime( &ui_sr_obj->message_buffer[ui_sr_obj->reader_buf_position].ts.tv_sec ));
		sprintf(current_line+20, ".%03d   ",
				ui_sr_obj->message_buffer[ui_sr_obj->reader_buf_position].ts.tv_nsec/1000000);

		switch (ui_sr_obj->message_buffer[ui_sr_obj->reader_buf_position].process_type) {
			case lib::EDP:
				strcat(current_line, "EDP: ");
				break;
			case lib::ECP:
				strcat(current_line, "ECP: ");
				break;
			case lib::MP:
				// printf("MP w ontimer\n");
				strcat(current_line, "MP:  ");
				break;
			case lib::VSP:
				strcat(current_line, "VSP: ");
				break;
			case lib::UI:
				strcat(current_line, "UI:  ");
				break;
			default:
				strcat(current_line, "???: ");
				continue;
		} // end: switch (message_buffer[reader_buf_position].process_type)

		// FIXME: ?
		ui_sr_obj->message_buffer[ui_sr_obj->reader_buf_position].process_type = lib::UNKNOWN_PROCESS_TYPE;

		char process_name_buffer[NAME_LENGTH+1];
		snprintf(process_name_buffer, sizeof(process_name_buffer), "%-21s",
				ui_sr_obj->message_buffer[ui_sr_obj->reader_buf_position].process_name);

		strcat(current_line,process_name_buffer);

		switch (ui_sr_obj->message_buffer[ui_sr_obj->reader_buf_position].message_type) {
			case lib::FATAL_ERROR:
				strcat(current_line, "FATAL_ERROR:     ");
				attr.text_color=Pg_RED;
				break;
			case lib::NON_FATAL_ERROR:
				strcat(current_line, "NON_FATAL_ERROR: ");
				attr.text_color=Pg_BLUE;
				break;
			case lib::SYSTEM_ERROR:
				// printf("SYSTEM ERROR W ONTIMER\n");
				// Informacja do UI o koniecznosci zmiany stanu na INITIAL_STATE
				strcat(current_line, "SYSTEM_ERROR:    ");
				attr.text_color=Pg_PURPLE;
				break;
			case lib::NEW_MESSAGE:
				strcat(current_line, "MESSAGE:         ");
				attr.text_color=Pg_BLACK;
				break;
			default:
				strcat(current_line, "UNKNOWN ERROR:   ");
				attr.text_color=Pg_YELLOW;
		}; // end: switch (message.message_type)

		strcat( current_line, ui_sr_obj->message_buffer[ui_sr_obj->reader_buf_position].description);
		strcat( current_line, "\n" );
		// 	printf("c_l W ONT: %s\n",current_line);
		// delay(1000);
		// 	attr.text_color=Pg_DBLUE;

		attributes_mask=Pt_MT_TEXT_COLOR;
		PtMultiTextModifyText(ABW_PtMultiText_sr_window, NULL, NULL, -1, current_line, strlen(current_line), &attr, attributes_mask);

	    (*log_file_outfile) << current_line;

	} 	while (ui_sr_obj->writer_buf_position!=ui_sr_obj->reader_buf_position);

		(*log_file_outfile).flush();

		ui_sr_obj->unlock_mutex();
	}

	if (ui_state.ui_state==2) {// jesli ma nastapic zamkniecie z aplikacji
		set_ui_state_notification(UI_N_EXITING);
	// 	printf("w ontimer 2\n");
		closing_delay_counter=20;// opoznienie zamykania
		ui_state.ui_state=3;
// 		delay(5000);
		MPslay(widget,apinfo,cbinfo);
		ui_msg.ui->message("closing");
	} else if (ui_state.ui_state==3) {// odliczanie
	// 	printf("w ontimer 3\n");
		if ((--closing_delay_counter)<=0)
			ui_state.ui_state=4;
	} else if (ui_state.ui_state==4) {// jesli ma nastapic zamkniecie aplikacji
 	//	printf("w ontimer 4\n");
		closing_delay_counter=20;// opoznienie zamykania
		ui_state.ui_state=5;
		EDP_all_robots_slay (widget, apinfo, cbinfo);
	} else if (ui_state.ui_state==5) {// odlcizanie do zamnkiecia
 	//	printf("w ontimer 5\n");
		if ((--closing_delay_counter)<=0)
			ui_state.ui_state=6;
	} else if (ui_state.ui_state==6) {// zakonczenie aplikacji
	      (*log_file_outfile).close();
	      delete log_file_outfile;
		printf("UI CLOSED\n");
		PtExit( EXIT_SUCCESS );
	}  else {
		if (busy)
		{
			set_ui_state_notification(UI_N_BUSY);
		} else
		{
			set_ui_state_notification(UI_N_READY);

		}
	}

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return( Pt_CONTINUE );

}
