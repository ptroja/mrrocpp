// ------------------------------------------------------------------------
// Proces:		UI
// Plik:            wndTrajectoryReproduceEvents.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		wndTrajectoryReproduce -> okno do wykonywania trajektorii
// 				wraz z odczytami z linialow
// Autor:		tkornuta
// Data:		01.03.2005
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/

/* Standard headers */
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>

/* MRROC++ headers */
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "ui/src/ui.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"

#include "ui/src/ui_class.h"
#include "ui/src/ui_ecp.h"
// Konfigurator.
#include "base/lib/configurator.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

// Tryb debugowania.
// #define TRDEBUG
extern ui::common::Interface interface;
// Wiadomosc wysylana do ECP.
extern lib::UI_ECP_message ui_ecp_msg;
// Rozkaz przeslany z ECP.


// Numer makrokroku.
int current_macrostep_number;
// PID ECP.
extern int ECPfd;
// Komenda wysylana z okna FileDialog po wcisnieciu accept.
extern uint8_t FDCommand;

void SetButtonState(PtWidget_t *widget, short active)
{
	// Przydzielenie pamieci.

	// Aktywacja przycisku.
	if (active) {
		PtSetResource(widget, Pt_ARG_FLAGS, Pt_FALSE, Pt_BLOCKED | Pt_GHOST);
		PtSetResource(widget, Pt_ARG_FLAGS, Pt_TRUE, Pt_SELECTABLE);
		PtDamageWidget(widget);
		// Deaktywacja przycisku.
	} else {
		PtSetResource(widget, Pt_ARG_FLAGS, Pt_TRUE, Pt_BLOCKED | Pt_GHOST);
		PtSetResource(widget, Pt_ARG_FLAGS, Pt_FALSE, Pt_SELECTABLE);
		PtDamageWidget(widget);
	}
	// Ustawienie stanu.
}

int TRbtnStart(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRbtnStart\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> kalibracja czujnika.
	ui_ecp_msg.command = lib::TR_START_MOVE;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("TRbtnPause: Send to ECP failed");
	} else {
		// Ustawienie przyciskow.
		SetButtonState(ABW_TRbtnPositionZero, false);
		SetButtonState(ABW_TRbtnStart, false);
		SetButtonState(ABW_TRbtnPause, true);
		SetButtonState(ABW_TRbtnStop, true);
		SetButtonState(ABW_TRbtnDSSCalibrate, false);
		SetButtonState(ABW_TRbtnFSCalibrate, false);
		// Odswiezenie okna.
		PtDamageWidget(ABW_wndTrajectoryReproduce);
	}
	return (Pt_CONTINUE);
}

int TRbtnPause(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRbtnPause\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> kalibracja czujnika.
	ui_ecp_msg.command = lib::TR_PAUSE_MOVE;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("TRbtnPause: Send to ECP failed");
	} else {
		// Ustawienie przyciskow.
		SetButtonState(ABW_TRbtnPositionZero, false);
		SetButtonState(ABW_TRbtnStart, true);
		SetButtonState(ABW_TRbtnPause, false);
		SetButtonState(ABW_TRbtnStop, true);
		// Odswiezenie okna.
		PtDamageWidget(ABW_wndTrajectoryReproduce);
	}
	return (Pt_CONTINUE);
}

int TRbtnStop(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRbtnStop\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> kalibracja czujnika.
	ui_ecp_msg.command = lib::TR_STOP_MOVE;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("TRbtnPause: Send to ECP failed");
	} else {
		// Ustawienie przyciskow.
		SetButtonState(ABW_TRbtnPositionZero, true);
		SetButtonState(ABW_TRbtnStart, false);
		SetButtonState(ABW_TRbtnPause, false);
		SetButtonState(ABW_TRbtnStop, false);
		SetButtonState(ABW_TRbtnLoadTrajectory, true);
		SetButtonState(ABW_TRbtnSaveAll, true);
		SetButtonState(ABW_TRbtnDSSCalibrate, false);
		SetButtonState(ABW_TRbtnFSCalibrate, false);
		// Odswiezenie okna.
		PtDamageWidget(ABW_wndTrajectoryReproduce);
	}
	return (Pt_CONTINUE);
}

int TRbtnExit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("btnExit\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP.
	ui_ecp_msg.command = lib::TR_EXIT;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("btnExit: Send to ECP failed");
	}
	// Zamkniecie polaczenia.
	name_close(ECPfd);
	// Zamkniecie okna.
	PtDestroyWidget(ABW_wndTrajectoryReproduce);
	return (Pt_CONTINUE);
}

int TRbtnPositionZero(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
	char tmp_buffer[20];
#ifdef TRDEBUG
	printf("TRbtnPositionZero\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// ustawienie typu wiadomosci
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> Osiagniecie zerowej pozycji.
	ui_ecp_msg.command = lib::TR_ZERO_POSITION;
	// printf("TRbtnPositionZero: sending %d \n", ui_ecp_msg.command);
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("TRbtnPositionZero: Send to ECP failed");
	} else {
		// Zerowy numer makrokroku.
		current_macrostep_number = 0;
		sprintf(tmp_buffer, "%i", current_macrostep_number);
		PtSetResource(ABW_TRedtMacrostepNumber, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
		// Zerowanie polozenia robota.
		PtSetResource(ABW_TRedtArm0, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtArm1, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtArm2, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtArm3, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtArm4, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtArm5, Pt_ARG_TEXT_STRING, " ", 0);
		// Zerowanie odczytow z suwmiarek.
		PtSetResource(ABW_TRedtScaleReading0, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtScaleReading1, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtScaleReading2, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtScaleReading3, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtScaleReading4, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtScaleReading5, Pt_ARG_TEXT_STRING, " ", 0);
		// Zerowanie odczytow z czujnika sily.
		PtSetResource(ABW_TRedtForceReading0, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtForceReading1, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtForceReading2, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtForceReading3, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtForceReading4, Pt_ARG_TEXT_STRING, " ", 0);
		PtSetResource(ABW_TRedtForceReading5, Pt_ARG_TEXT_STRING, " ", 0);
		// Ustawienie przyciskow.
		SetButtonState(ABW_TRbtnPositionZero, false);
		SetButtonState(ABW_TRbtnStart, true);
		SetButtonState(ABW_TRbtnPause, false);
		SetButtonState(ABW_TRbtnStop, true);
		SetButtonState(ABW_TRbtnLoadTrajectory, false);
		SetButtonState(ABW_TRbtnSaveAll, false);
		SetButtonState(ABW_TRbtnDSSCalibrate, true);
		SetButtonState(ABW_TRbtnFSCalibrate, true);
		// Odswiezenie okna.
		PtDamageWidget(ABW_wndTrajectoryReproduce);
	}
	return (Pt_CONTINUE);
}

int TRbtnLoadTrajectory(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRbtnLoadTrajectory\n");
#endif
	// Komenda wysylana z okna FileDialog po wcisnieciu accept.
	FDCommand = lib::TR_LOAD_TRAJECTORY;
	// Stworzenie okna wyboru pliku.
	ApCreateModule(ABM_wndFileLocation, widget, cbinfo);
	return (Pt_CONTINUE);
}

int TRbtnSaveAll(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRbtnSaveAll\n");
#endif
	// Komenda wysylana z okna FileDialog po wcisnieciu accept.
	FDCommand = lib::TR_SAVE_READINGS;
	// Stworzenie okna wndFileLocation.
	ApCreateModule(ABM_wndFileLocation, widget, cbinfo);
	return (Pt_CONTINUE);
}

int TRConnect(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRConnect: Create connection.\n");
#endif
	// Nazwa polacznia.
	std::string
			tmp_name =
					interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "ecp_third_chan_attach_point", lib::irp6ot_m::ECP_SECTION);

#ifdef TRDEBUG
	printf("TRConnect: %s\n", tmp_name.c_str());
#endif
	// Otworzenie polaczenia.
	if ((ECPfd = name_open(tmp_name.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == -1) {
		perror("TRConnect: Connect to ECP failed");
		return EXIT_FAILURE;
	}
	return (Pt_CONTINUE);
} // end: TRConnect

int TRRefreshWindow(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRRefreshWindow\n");
#endif
	// Bufor uzywany do przetwarzania polozanie/odczytow do stringow.
	char tmp_buffer[20];
	// Wypisanie pozycji robota w oknie.
	// Zerowa os.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.robot_position[0]);
	PtSetResource(ABW_TRedtArm0, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	// Pierwsza os.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.robot_position[1]);
	PtSetResource(ABW_TRedtArm1, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	// Druga os.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.robot_position[2]);
	PtSetResource(ABW_TRedtArm2, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	// Trzecia os.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.robot_position[3]);
	PtSetResource(ABW_TRedtArm3, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	// Czwarta os.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.robot_position[4]);
	PtSetResource(ABW_TRedtArm4, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	// Piata os.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.robot_position[5]);
	PtSetResource(ABW_TRedtArm5, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	// Wypisanie odczytow czujnika zlozonego z linialow w oknie.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.digital_scales_sensor_reading[0]);
	PtSetResource(ABW_TRedtScaleReading0, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.digital_scales_sensor_reading[1]);
	PtSetResource(ABW_TRedtScaleReading1, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.digital_scales_sensor_reading[2]);
	PtSetResource(ABW_TRedtScaleReading2, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.digital_scales_sensor_reading[3]);
	PtSetResource(ABW_TRedtScaleReading3, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.digital_scales_sensor_reading[4]);
	PtSetResource(ABW_TRedtScaleReading4, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.digital_scales_sensor_reading[5]);
	PtSetResource(ABW_TRedtScaleReading5, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	// Wypisanie odczytow czujnika sily w oknie.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[0]);
	PtSetResource(ABW_TRedtForceReading0, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[1]);
	PtSetResource(ABW_TRedtForceReading1, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[2]);
	PtSetResource(ABW_TRedtForceReading2, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[3]);
	PtSetResource(ABW_TRedtForceReading3, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[4]);
	PtSetResource(ABW_TRedtForceReading4, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[5]);
	PtSetResource(ABW_TRedtForceReading5, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	// Wykonano nastepny makrokrok.
	current_macrostep_number++;
	sprintf(tmp_buffer, "%i", current_macrostep_number);
	// Wypisanie numeru makrokroku.
	PtSetResource(ABW_TRedtMacrostepNumber, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	return (Pt_CONTINUE);
}

int TRbtnDSSCalibrate(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRbtnDSSCalibrate\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> Kalibracja czujnika zlozonego z linialow.
	ui_ecp_msg.command = lib::TR_CALIBRATE_DIGITAL_SCALES_SENSOR;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("TRbtnDSSCalibrate: Send to ECP failed");
	}
	return (Pt_CONTINUE);
}

int TRbtnFSCalibrate(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRbtnFSCalibrate\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> Kalibracja czujnika sily.
	ui_ecp_msg.command = lib::TR_CALIBRATE_FORCE_SENSOR;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("TRbtnFSCalibrate: Send to ECP failed");
	}
	return (Pt_CONTINUE);
}

int TRbtnTryAgain(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRbtnTryAgain\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP -> Kalibracja czujnika sily.
	ui_ecp_msg.command = lib::TR_TRY_MOVE_AGAIN;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("TRbtnTryAgain: Send to ECP failed");
	} else {
		// Ustawienie stanu przyciskow.
		SetButtonState(ABW_TRbtnPause, true);
		SetButtonState(ABW_TRbtnTryAgain, false);
		// Odswiezenie okna.
		PtDamageWidget(ABW_wndTrajectoryReproduce);
	}
	return (Pt_CONTINUE);
}

int TRDangerousForceDetected(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)
{
#ifdef TRDEBUG
	printf("TRDangerousForceDetected\n");
#endif
	// Bufor uzywany do przetwarzania polozanie/odczytow do stringow.
	char tmp_buffer[20];
	// Wypisanie odczytow czujnika sily w oknie.
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[0]);
	PtSetResource(ABW_TRedtForceReading0, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[1]);
	PtSetResource(ABW_TRedtForceReading1, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[2]);
	PtSetResource(ABW_TRedtForceReading2, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[3]);
	PtSetResource(ABW_TRedtForceReading3, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[4]);
	PtSetResource(ABW_TRedtForceReading4, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	sprintf(tmp_buffer, "%5.5f", interface.ui_ecp_obj->ecp_to_ui_msg.R2S.force_sensor_reading[5]);
	PtSetResource(ABW_TRedtForceReading5, Pt_ARG_TEXT_STRING, tmp_buffer, 0);
	// Ustawienie stanu przyciskow.
	SetButtonState(ABW_TRbtnPause, false);
	SetButtonState(ABW_TRbtnTryAgain, true);
	// // Odswiezenie okna.
	PtDamageWidget(ABW_wndTrajectoryReproduce);
	return (Pt_CONTINUE);
}
