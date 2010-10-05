// ------------------------------------------------------------------------
// Proces: 	UI
// Plik:			wndForceControlEvents.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		wndForceControlEvents - okno do tworzenia trajektorii
// 				kontrola ruchu robota przy pomocy czujnika sily
// Autor:		tkornuta
// Data:		23.02.2005
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/

/* Standard headers */
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cerrno>

/* MRROC++ headers */
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "ui/src/ui.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"

#include "ui/src/ui_class.h"
// #include "common/y_config.h"
// Konfigurator.
#include "base/lib/configurator.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

//#define FCDEBUG 1

// Wiadomosc wysylana do ECP.
lib::UI_ECP_message ui_ecp_msg;
extern ui::common::Interface interface;

uint64_t e; // kod bledu systemowego


// Numer makrokroku.
int macrostep_number;
// PID ECP
int ECPfd;
// Tworzenie polaczenia z ECP.
bool CREATE_CONNECTION;
// Pobranie polozenia robota.
bool GET_ROBOT_POSITION;
// Kontrola readera.
bool READER_ON;
// Rodzaj sterowania.
lib::POSE_SPECIFICATION ps;

// komenda wysylana z okna FileDialog po wcisnieciu accept
extern uint8_t FDCommand;

int FCwndForceControlRealised(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
#ifdef FCDEBUG
	printf("FCwndForceControlRealised\n");
#endif

	// Deaktywacja zmiany rodzaju sterowania!
	SetButtonState(ABW_FCbtnChangeControl, false);

	// Zerowy makrokrok.
	macrostep_number = 0;
	// Stworzenie polaczenia.
	CREATE_CONNECTION = true;
	// Pobranie polozenia robota.
	GET_ROBOT_POSITION = true;
	// Reader wylaczony.
	READER_ON = false;
	// Sterowanie przyrostami na walach silnikow.
	ps = lib::MOTOR;
	return (Pt_CONTINUE);
}
; // end: FCwndForceControlRealised

int FCCreateConnection(void) {
#ifdef FCDEBUG
	printf("FCCreateConnection\n");
#endif
	// Wiadomosc z polozeniem robota otrzymana z ECP.
	// 	lib::ECP_message from_ecp;

	// Stworzenie nazwy.
	std::string tmp_name = interface.config->return_attach_point_name(
			lib::configurator::CONFIG_SERVER, "ecp_sec_chan_attach_point",
			lib::irp6ot_m::ECP_SECTION);

#ifdef FCDEBUG
	printf("FCCreateConnection: %s\n",tmp_name.c_str());
#endif
	// Otwarcie polaczenia.
	if ((ECPfd = name_open(tmp_name.c_str(), NAME_FLAG_ATTACH_GLOBAL)) == -1) {
		e = errno;
		perror("FCCreateConnection: Connect to ECP failed");
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int FCRefreshPosition(void) {
	// Wiadomosc odbierana z ECP.
	lib::ECP_message from_ecp;
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP.
	ui_ecp_msg.command = lib::FC_GET_DATA;
	// Wyslanie polecenia i odebranie polozenia robota.
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), &from_ecp,
			sizeof(lib::ECP_message)) == -1) {
		perror("FCRefreshWindow: Send to ECP failed");
		return EXIT_FAILURE;
	} else {
		// Wypisanie pozycji robota w oknie.
		// Zerowa os.
		char tmp[20];
		sprintf(tmp, "%5.5f", from_ecp.RS.robot_position[0]);
		PtSetResource(ABW_FCedtArm0, Pt_ARG_TEXT_STRING, tmp, 0);
		// Pierwsza os.
		sprintf(tmp, "%5.5f", from_ecp.RS.robot_position[1]);
		PtSetResource(ABW_FCedtArm1, Pt_ARG_TEXT_STRING, tmp, 0);
		// Druga os.
		sprintf(tmp, "%5.5f", from_ecp.RS.robot_position[2]);
		PtSetResource(ABW_FCedtArm2, Pt_ARG_TEXT_STRING, tmp, 0);
		// Trzecia os.
		sprintf(tmp, "%5.5f", from_ecp.RS.robot_position[3]);
		PtSetResource(ABW_FCedtArm3, Pt_ARG_TEXT_STRING, tmp, 0);
		// Czwarta os.
		sprintf(tmp, "%5.5f", from_ecp.RS.robot_position[4]);
		PtSetResource(ABW_FCedtArm4, Pt_ARG_TEXT_STRING, tmp, 0);
		// Piata os.
		sprintf(tmp, "%5.5f", from_ecp.RS.robot_position[5]);
		PtSetResource(ABW_FCedtArm5, Pt_ARG_TEXT_STRING, tmp, 0);
		// Wypisanie odczytow czujnika w oknie.
		sprintf(tmp, "%5.5f", from_ecp.RS.sensor_reading[0]);
		PtSetResource(ABW_FCedtForceReading0, Pt_ARG_TEXT_STRING, tmp, 0);
		sprintf(tmp, "%5.5f", from_ecp.RS.sensor_reading[1]);
		PtSetResource(ABW_FCedtForceReading1, Pt_ARG_TEXT_STRING, tmp, 0);
		sprintf(tmp, "%5.5f", from_ecp.RS.sensor_reading[2]);
		PtSetResource(ABW_FCedtForceReading2, Pt_ARG_TEXT_STRING, tmp, 0);
		sprintf(tmp, "%5.5f", from_ecp.RS.sensor_reading[3]);
		PtSetResource(ABW_FCedtForceReading3, Pt_ARG_TEXT_STRING, tmp, 0);
		sprintf(tmp, "%5.5f", from_ecp.RS.sensor_reading[4]);
		PtSetResource(ABW_FCedtForceReading4, Pt_ARG_TEXT_STRING, tmp, 0);
		sprintf(tmp, "%5.5f", from_ecp.RS.sensor_reading[5]);
		PtSetResource(ABW_FCedtForceReading5, Pt_ARG_TEXT_STRING, tmp, 0);
	}
	return EXIT_SUCCESS;
}

int FCTimerTick(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo) {
#ifdef FCDEBUG
	printf("FCTimerTick\n");
#endif
	// Jezeli trzeba stworzyc polaczenie z ECP.
	if (CREATE_CONNECTION) {
		// Jezeli sie powiodlo stworzenie polaczenia.
		if (FCCreateConnection() != EXIT_FAILURE)
			CREATE_CONNECTION = false;
		return (Pt_CONTINUE);
	}
	// Jezeli trzeba odswiezyc pozycje robota.
	if (GET_ROBOT_POSITION) {
		FCRefreshPosition();
		return (Pt_CONTINUE);
	}
	return (Pt_CONTINUE);
}

int FCbtnCalibrateSensor(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
#ifdef FCDEBUG
	printf("FCbtnCalibrateSensor\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// polecenie dla ECP -> kalibracja czujnika
	ui_ecp_msg.command = lib::FC_CALIBRATE_SENSOR;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("btnCalibrateSensor: Send to ECP failed");
	}
	return (Pt_CONTINUE);
}

int FCbtnAddMacrostep(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
#ifdef FCDEBUG
	printf("FCbtnAddMacrostep\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP.
	ui_ecp_msg.command = lib::FC_ADD_MACROSTEP;

	// Zczytanie czasu wykonywana kroku.
	int ivalue;
	int *i = &ivalue;

	PtGetResource(ABW_FCPtMotionTime, Pt_ARG_NUMERIC_VALUE, &i, 0);
	ui_ecp_msg.motion_time = *i;
	// Wyslanie danych o pozycji oraz odblokowanie ruchu.
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("FCbtnAddMacrostep: Send to ECP failed");
		return (Pt_CONTINUE);
	} else {
		// Dodanie makrokroku.
		macrostep_number++;
		// Wyswietlenie numeru makrokroku.
		char tmp[5];
		sprintf(tmp, "%i", macrostep_number);
		PtSetResource(ABW_FCedtMacrostepNumber, Pt_ARG_TEXT_STRING, tmp, 0);
		// aktywacja przycisku SAVE
		SetButtonState(ABW_FCbtnSaveTrajectory, true);
		// aktywacja przycisku NEW
		SetButtonState(ABW_FCbtnNewTrajectory, true);
		// odswiezenie okna
		PtDamageWidget(ABW_wndForceControl);
	}
	return (Pt_CONTINUE);
}

int FCbtnNewTrajectory(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
#ifdef FCDEBUG
	printf("FCbtnNewTrajectory\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP.
	ui_ecp_msg.command = lib::FC_NEW_TRAJECTORY;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("FCbtnNewTrajectory: Send to ECP failed");
	} else {
		// Wyzerowanie liczby makrokrokow.
		macrostep_number = 0;
		// Wyswietlenie numeru makrokroku.
		char tmp[5];
		sprintf(tmp, "%i", macrostep_number);
		PtSetResource(ABW_FCedtMacrostepNumber, Pt_ARG_TEXT_STRING, tmp, 0);
		// Deaktywacja przycisku SAVE.
		SetButtonState(ABW_FCbtnSaveTrajectory, false);
		// Deaktywacja przycisku NEW TRAJECTORY.
		SetButtonState(ABW_FCbtnNewTrajectory, false);
		// Odswiezenie okna.
		PtDamageWidget(ABW_wndForceControl);
	}
	return (Pt_CONTINUE);
}

int FCbtnSaveTrajectory(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
#ifdef FCDEBUG
	printf("FCbtnSaveTrajectory\n");
#endif
	// Komenda wysylana z okna FileDialog po wcisnieciu accept.
	FDCommand = lib::FC_SAVE_TRAJECTORY;
	// Stworzenie okna wyboru pliku.
	ApCreateModule(ABM_wndFileLocation, widget, cbinfo);
	return (Pt_CONTINUE);
}

int FCbtnExit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo) {
#ifdef FCDEBUG
	printf("FCbtnExit\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP.
	ui_ecp_msg.command = lib::FC_EXIT;
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("FCbtnExit: Send to ECP failed");
	}
	// Zamkniecie polaczenia.
	name_close(ECPfd);
	// Zamkniecie okna.
	PtDestroyWidget(ABW_wndForceControl);
	return (Pt_CONTINUE);
}

int FCbtnOnOffReader(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	char pulse_code;
	long pulse_value = 0;
#ifdef FCDEBUG
	printf("FCbtnOnOffReader\n");
#endif
	// Wlaczenie/Wylaczenie readera.
	if (READER_ON)
		pulse_code = READER_STOP; // stop
	else
		pulse_code = READER_START;// start
	if (MsgSendPulse(interface.irp6ot_m->state.edp.reader_fd, sched_get_priority_min(
			SCHED_FIFO), pulse_code, pulse_value) == -1) {
		perror("FCbtnOnOffReader: Send pulse to Reader failed");
	} else {
		// Reader wylaczony.
		if (READER_ON) {
			// Wylaczenie.
			READER_ON = false;
			// Zmiana przycisku.
			PtSetResource(ABW_FCbtnOnOffReader, Pt_ARG_TEXT_STRING, "ON", 0);
			PtSetResource(ABW_FClblReader, Pt_ARG_TEXT_STRING, "Reader [OFF]",
					0);
		} else {
			// Wylaczenie.
			READER_ON = true;
			// Zmiana przycisku.
			PtSetResource(ABW_FCbtnOnOffReader, Pt_ARG_TEXT_STRING, "OFF", 0);
			PtSetResource(ABW_FClblReader, Pt_ARG_TEXT_STRING, "Reader [ON]", 0);
		}
		// Odswiezenie okna.
		PtDamageWidget(ABW_wndForceControl);
	}
	return (Pt_CONTINUE);
}

int FCbtnChangeExternalMotorControl(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
#ifdef FCDEBUG
	printf("FCbtnChangeExternalMotorControl\n");
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP.
	ui_ecp_msg.command = lib::FC_CHANGE_CONTROL;
	// Zmiana sterowania.
	/*
	 if (ps == lib::MOTOR)
	 ui_ecp_msg.ps = lib::XYZ_EULER_ZYZ;
	 if (ps == lib::XYZ_EULER_ZYZ)
	 ui_ecp_msg.ps = lib::MOTOR;

	 if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
	 perror("FCbtnChangeExternalMotorControl: Send to ECP failed");
	 }else{
	 // Sterowanie przyrostami na walach silnikow.

	 if (ps == lib::MOTOR){
	 ps = lib::XYZ_EULER_ZYZ;
	 // Wypisanie na etykiecie  - czym sterujemy.
	 PtSetResource( ABW_FClblControl, Pt_ARG_TEXT_STRING, "Control [EXTERNAL]", 0);
	 // Zmiana przycisku.
	 PtSetResource(ABW_FCbtnChangeControl, Pt_ARG_TEXT_STRING ,  "MOTOR", 0);
	 // Zmiana etykiet.
	 PtSetResource(ABW_lbRobotControl, Pt_ARG_TEXT_STRING ,  "IRp-6 EXTERNAL Position [m]", 0);
	 PtSetResource(ABW_lblRobot0, Pt_ARG_TEXT_STRING ,  "X", 0);
	 PtSetResource(ABW_lblRobot1, Pt_ARG_TEXT_STRING ,  "Y", 0);
	 PtSetResource(ABW_lblRobot2, Pt_ARG_TEXT_STRING ,  "Z", 0);
	 PtSetResource(ABW_lblRobot3, Pt_ARG_TEXT_STRING ,  "Alfa", 0);
	 PtSetResource(ABW_lblRobot4, Pt_ARG_TEXT_STRING ,  "Beta", 0);
	 PtSetResource(ABW_lblRobot5, Pt_ARG_TEXT_STRING ,  "Gamma", 0);
	 PtSetResource(ABW_lblRobotControl0, Pt_ARG_TEXT_STRING ,  "X", 0);
	 PtSetResource(ABW_lblRobotControl1, Pt_ARG_TEXT_STRING ,  "Y", 0);
	 PtSetResource(ABW_lblRobotControl2, Pt_ARG_TEXT_STRING ,  "Z", 0);
	 PtSetResource(ABW_lblRobotControl3, Pt_ARG_TEXT_STRING ,  "Alfa", 0);
	 PtSetResource(ABW_lblRobotControl4, Pt_ARG_TEXT_STRING ,  "Beta", 0);
	 PtSetResource(ABW_lblRobotControl5, Pt_ARG_TEXT_STRING ,  "Gamma", 0);
	 }else{
	 ps = lib::MOTOR;
	 // Wypisanie na etykiecie  - czym sterujemy.
	 PtSetResource( ABW_FClblControl, Pt_ARG_TEXT_STRING, "Control [MOTOR]", 0);
	 // Zmiana przycisku.
	 PtSetResource(ABW_FCbtnChangeControl, Pt_ARG_TEXT_STRING ,  "EXTERNAL", 0);
	 // Zmiana etykiet.
	 PtSetResource(ABW_lbRobotControl, Pt_ARG_TEXT_STRING ,  "IRp-6 MOTOR Position [-]", 0);
	 PtSetResource(ABW_lblRobot0, Pt_ARG_TEXT_STRING ,  "Arm[0]", 0);
	 PtSetResource(ABW_lblRobot1, Pt_ARG_TEXT_STRING ,  "Arm[1]", 0);
	 PtSetResource(ABW_lblRobot2, Pt_ARG_TEXT_STRING ,  "Arm[2]", 0);
	 PtSetResource(ABW_lblRobot3, Pt_ARG_TEXT_STRING ,  "Arm[3]", 0);
	 PtSetResource(ABW_lblRobot4, Pt_ARG_TEXT_STRING ,  "Arm[4]", 0);
	 PtSetResource(ABW_lblRobot5, Pt_ARG_TEXT_STRING ,  "Arm[5]", 0);
	 PtSetResource(ABW_lblRobotControl0, Pt_ARG_TEXT_STRING ,  "Arm[0]", 0);
	 PtSetResource(ABW_lblRobotControl1, Pt_ARG_TEXT_STRING ,  "Arm[1]", 0);
	 PtSetResource(ABW_lblRobotControl2, Pt_ARG_TEXT_STRING ,  "Arm[2]", 0);
	 PtSetResource(ABW_lblRobotControl3, Pt_ARG_TEXT_STRING ,  "Arm[3]", 0);
	 PtSetResource(ABW_lblRobotControl4, Pt_ARG_TEXT_STRING ,  "Arm[4]", 0);
	 PtSetResource(ABW_lblRobotControl5, Pt_ARG_TEXT_STRING ,  "Arm[5]", 0);
	 }
	 // Odswiezenie okna.
	 PtDamageWidget(ABW_wndForceControl);
	 }
	 */
	return (Pt_CONTINUE);
}

/**************************** MOVE COMMANDS *****************************/
void SendMoveCommand(int move_type) {
#ifdef FCDEBUG
	printf("SendMoveCommand: %i\n",move_type);
#endif
#if !defined(USE_MESSIP_SRR)
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
#endif
	// Polecenie dla ECP.
	ui_ecp_msg.command = lib::FC_MOVE_ROBOT;
	// Rodzaj ruchu -> (okreslenie osi 1..6) && (+/- lewo/prawo).
	ui_ecp_msg.move_type = move_type;
	// Wyslanie polecenia.
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), NULL, 0) == -1) {
		perror("SendMoveCommand: Send to ECP failed");
	} else {
		// Odczyt polozenia w nastepnym cyklu timera.
		GET_ROBOT_POSITION = true;
	}
}
; // end: SendMoveCommand

int FCbtnMove0Left(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(-1);
	return (Pt_CONTINUE);
}

int FCbtnMove1Left(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(-2);
	return (Pt_CONTINUE);
}

int FCbtnMove2Left(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(-3);
	return (Pt_CONTINUE);
}

int FCbtnMove3Left(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(-4);
	return (Pt_CONTINUE);
}

int FCbtnMove4Left(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(-5);
	return (Pt_CONTINUE);
}

int FCbtnMove5Left(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(-6);
	return (Pt_CONTINUE);
}

int FCbtnMove0Right(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(+1);
	return (Pt_CONTINUE);
}

int FCbtnMove1Right(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(+2);
	return (Pt_CONTINUE);
}

int FCbtnMove2Right(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(+3);
	return (Pt_CONTINUE);
}

int FCbtnMove3Right(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(+4);
	return (Pt_CONTINUE);
}

int FCbtnMove4Right(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(+5);
	return (Pt_CONTINUE);
}

int FCbtnMove5Right(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	// Wyslanie odpowiedniego polecenia do ECP.
	// (okreslenie osi 1..6) && (+/- lewo/prawo)
	SendMoveCommand(+6);
	return (Pt_CONTINUE);
}
