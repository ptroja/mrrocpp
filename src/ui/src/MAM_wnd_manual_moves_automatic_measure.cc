// ------------------------------------------------------------------------
// Proces:		UI
// Plik:           wndTrajectoryReproduceEvents.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		wndTrajectoryReproduce -> okno do wykonywania 
//				ruchow recznych robotem oraz zbierania odczytow z linialow.
// Autor:		tkornuta
// Data:		16.03.2006
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/

/* Standard headers */
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>

#include "ui/src/ui.h"

/* MRROC++ headers */
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "ui/src/ui_class.h"
#include "ui/src/ui_ecp.h"

#include "base/lib/sr/srlib.h"

// Konfigurator.
#include "base/lib/configurator.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern ui::common::Interface interface;

// Tryb debugowania.
// #define MAMDEBUG

// Wiadomosc wysylana do ECP.
extern lib::UI_ECP_message ui_ecp_msg;
// Rozkaz przeslany z ECP.


// Zmienna konfiguracyjna.


// PID ECP.
static int ECPfd;
// Komenda wysylana z okna FileDialog po wcisnieciu accept.
extern uint8_t FDCommand;

int MAM_btn_start_measures(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef MAMDEBUG
	printf("MAM_btn_start_measures\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> start pomiarow.
	ui_ecp_msg.command = lib::MAM_START;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("MAM_btn_start_measures: Send to ECP failed");
	} else {
		// Ustawienie przyciskow.
		SetButtonState(ABW_MAM_btn_start_measures, false);
		SetButtonState(ABW_MAM_btn_stop_measures, true);
		SetButtonState(ABW_MAM_btn_clear_measures, false);
		SetButtonState(ABW_MAM_btn_save_measures, false);
		SetButtonState(ABW_MAM_btn_calibrate_scales, false);
		SetButtonState(ABW_MAM_btn_exit, false);
		// Odswiezenie okna.
		PtDamageWidget(ABW_MAM_wnd_manual_moves_automatic_measures);
	}
	return (Pt_CONTINUE);
}//: MAM_btn_start_measures


int MAM_btn_stop_measures(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef MAMDEBUG
	printf("MAM_btn_stop_measures\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> kalibracja czujnika.
	ui_ecp_msg.command = lib::MAM_STOP;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("MAM_btn_stop_measures: Send to ECP failed");
	} else {
		// Ustawienie przyciskow.
		SetButtonState(ABW_MAM_btn_start_measures, true);
		SetButtonState(ABW_MAM_btn_stop_measures, false);
		SetButtonState(ABW_MAM_btn_clear_measures, true);
		SetButtonState(ABW_MAM_btn_save_measures, true);
		SetButtonState(ABW_MAM_btn_calibrate_scales, true);
		SetButtonState(ABW_MAM_btn_exit, true);
		// Odswiezenie okna.
		PtDamageWidget(ABW_MAM_wnd_manual_moves_automatic_measures);
	}
	return (Pt_CONTINUE);
}//: MAM_btn_stop_measures


int MAM_btn_clear_measures(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef MAMDEBUG
	printf("MAM_btn_clear_measures\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> kalibracja czujnika.
	ui_ecp_msg.command = lib::MAM_CLEAR;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("MAM_btn_clear_measures: Send to ECP failed");
	} else {
		// Ustawienie przyciskow.
		SetButtonState(ABW_MAM_btn_start_measures, true);
		SetButtonState(ABW_MAM_btn_stop_measures, false);
		SetButtonState(ABW_MAM_btn_clear_measures, false);
		SetButtonState(ABW_MAM_btn_save_measures, false);
		SetButtonState(ABW_MAM_btn_calibrate_scales, true);
		SetButtonState(ABW_MAM_btn_exit, true);
		// Odswiezenie okna.
		PtDamageWidget(ABW_MAM_wnd_manual_moves_automatic_measures);
	}
	return (Pt_CONTINUE);
}//: MAM_btn_clear_measures


int MAM_btn_save_measures(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef MAMDEBUG
	printf("MAM_btn_save_measures\n");
#endif
	// Komenda wysylana z okna FileDialog po wcisnieciu accept.
	FDCommand = lib::MAM_SAVE;
	// Stworzenie okna wndFileLocation.
	ApCreateModule(ABM_wndFileLocation, widget, cbinfo);
	return (Pt_CONTINUE);
}//: MAM_btn_save_measures

int MAM_btn_calibrate(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef MAMDEBUG
	printf("MAM_btn_calibrate\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> kalibracja czujnika.
	ui_ecp_msg.command = lib::MAM_CALIBRATE;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("MAM_btn_calibrate: Send to ECP failed");
	} else {
		// Ustawienie przyciskow.
		SetButtonState(ABW_MAM_btn_start_measures, true);
		SetButtonState(ABW_MAM_btn_stop_measures, false);
		SetButtonState(ABW_MAM_btn_clear_measures, true);
		SetButtonState(ABW_MAM_btn_save_measures, true);
		SetButtonState(ABW_MAM_btn_calibrate_scales, true);
		SetButtonState(ABW_MAM_btn_exit, true);
		// Odswiezenie okna.
		PtDamageWidget(ABW_MAM_wnd_manual_moves_automatic_measures);
	}
	return (Pt_CONTINUE);
}//: MAM_btn_calibrate

int MAM_btn_exit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef MAMDEBUG
	printf("MAM_btn_exit\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP.
	ui_ecp_msg.command = lib::MAM_EXIT;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("MAM_btn_exit: Send to ECP failed");
	}
	// Zamkniecie polaczenia.
	name_close(ECPfd);
	// Zamkniecie okna.
	PtDestroyWidget(ABW_MAM_wnd_manual_moves_automatic_measures);
	return (Pt_CONTINUE);
}//: MAM_btn_exit


int MAM_tmr_connect(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef MAMDEBUG
	printf("MAM_tmr_connect: Create connection.\n");
#endif
	// Nazwa polacznia.
	std::string attach_point =
			interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "attach_point", "[ecp_ui_channel]");
	//    tmp_name =interface.config->return_attach_point_name	(CONFIG_SERVER, "ecp_chan_attach_point", lib::UI_SECTION);

#ifdef MAMDEBUG
	printf("MAM_tmr_connect: %s\n", tmp_name);
#endif
	// Otworzenie polaczenia.
	if ((ECPfd = name_open(attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == -1) {
		perror("MAM_tmr_connect: Connect to ECP failed");
		return EXIT_FAILURE;
	}
	return (Pt_CONTINUE);
} // end: MAM_tmr_connect


int MAM_refresh_window(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef MAMDEBUG
	printf("MAM_refresh_window\n");
#endif
	// Bufor uzywany do przetwarzania polozanie/odczytow do stringow.
	char tmp_buffer[20];
	// Wypisanie pozycji robota w oknie.
	// Zerowa os.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.robot_position[0]);
	PtSetResource(ABW_MAM_edt_arm0, Pt_ARG_TEXT_STRING, tmp_buffer, 0);

	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.robot_position[1]);
	PtSetResource(ABW_MAM_edt_arm1, Pt_ARG_TEXT_STRING, tmp_buffer, 0);

	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.robot_position[2]);
	PtSetResource(ABW_MAM_edt_arm2, Pt_ARG_TEXT_STRING, tmp_buffer, 0);

	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.robot_position[3]);
	PtSetResource(ABW_MAM_edt_arm3, Pt_ARG_TEXT_STRING, tmp_buffer, 0);

	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.robot_position[4]);
	PtSetResource(ABW_MAM_edt_arm4, Pt_ARG_TEXT_STRING, tmp_buffer, 0);

	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.robot_position[5]);
	PtSetResource(ABW_MAM_edt_arm5, Pt_ARG_TEXT_STRING, tmp_buffer, 0);

	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.robot_position[6]);
	PtSetResource(ABW_MAM_edt_arm6, Pt_ARG_TEXT_STRING, tmp_buffer, 0);

	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.robot_position[7]);
	PtSetResource(ABW_MAM_edt_arm7, Pt_ARG_TEXT_STRING, tmp_buffer, 0);

	// Wypisanie odczytow czujnika zlozonego z linialow w oknie.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.sensor_reading[0]);
	PtSetResource(ABW_MAM_edt_scale_reading0, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.sensor_reading[1]);
	PtSetResource(ABW_MAM_edt_scale_reading1, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.sensor_reading[2]);
	PtSetResource(ABW_MAM_edt_scale_reading2, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.sensor_reading[3]);
	PtSetResource(ABW_MAM_edt_scale_reading3, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.sensor_reading[4]);
	PtSetResource(ABW_MAM_edt_scale_reading4, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.sensor_reading[5]);
	PtSetResource(ABW_MAM_edt_scale_reading5, Pt_ARG_TEXT_STRING, tmp_buffer, 0);

	// Wypisanie numeru makrokroku.
	sprintf(tmp_buffer, "%i", interface.ui_ecp_obj->ecp_to_ui_msg.MAM.measure_number);
	PtSetResource(ABW_MAM_edt_measure_point_number, Pt_ARG_TEXT_STRING,
			tmp_buffer, 0);

	return (Pt_CONTINUE);
}

