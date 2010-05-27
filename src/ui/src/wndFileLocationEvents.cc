// ------------------------------------------------------------------------
// Proces: 	UI
// Plik:			wndFileLocationDialogEvents.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		wndFileLocationDialog -> okno do zapisywania/odczytywania 
// 				lokacji plikow z trajektoriami
// Autor:		tkornuta
// Data:		01.03.2005
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

/* MRROC++ headers */
#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ui/ui.h"
#include "ui/ui_const.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

// #define FLDEBUG

// Wiadomosc wysylana do ECP.
extern lib::UI_ECP_message ui_ecp_msg;

// PID ECP.
extern int ECPfd;
// Komenda wysylana z okna FileDialog po wcisnieciu accept.
lib::UI_TO_ECP_COMMAND FDCommand;

int FLbtnAcceptFile(PtWidget_t *widget, ApInfo_t *apinfo,
		PtCallbackInfo_t *cbinfo) {
	PtFileSelItem_t *item;
	// Lokalizacja oraz nazwa pliku.
	char *filename;
	int macrosteps;
#ifdef FLDEBUG
	printf("FLbtnAcceptFile\n");
#endif
	// Ustawienie typu wiadomosci.
	ui_ecp_msg.hdr.type = 0x00;
	ui_ecp_msg.hdr.subtype = 0x00;
	// Przepisanie polecenia dla ECP.
	ui_ecp_msg.command = FDCommand;
	// Odczytanie sciezki.
	item = PtFSGetCurrent(ABW_FLflsTrajectoryFile);
	// Sprawdzenie, czy wybrano plik czy katalog.
	if ((item->type) == Pt_FS_FILE) {
		// Jesli wyrano konkretny plik.
#ifdef FLDEBUG
		printf("FLbtnAcceptFile: path: %s\n", item->fullpath);
#endif
		strcpy(ui_ecp_msg.filename, item->fullpath);
	} else if (((item->type) == Pt_FS_DIR_OP) || ((item->type) == Pt_FS_DIR_CL)) {
		// Jesli wybrano katalog.
		// Jesli load -> nalezy wybrac istniejacy plik.
		if (FDCommand == lib::TR_LOAD_TRAJECTORY)
			return (Pt_CONTINUE);
		// Save -> sprawdzenie czy wpisano nazwe.
		PtGetResource(ABW_FLedtFilename, Pt_ARG_TEXT_STRING, &filename, 0);
		// Jesli pusta nazwa -> Break.
		if (strlen(filename) == 0)
			return (Pt_CONTINUE);
		// Skopiowanie nazwy.
#ifdef DEBUG
		printf("FLbtnAcceptFile: path: %s\n", item->fullpath);
		printf("FLbtnAcceptFile: name: %s\n", filename);
#endif
		strcpy(ui_ecp_msg.filename, item->fullpath);
		strcat(ui_ecp_msg.filename, "/");
		strcat(ui_ecp_msg.filename, filename);
	}
#ifdef DEBUG
	printf("FLbtnAcceptFile: filename: %s\n", ui_ecp_msg.filename);
#endif
	// Wyslanie nazwy do ECP.
	if (MsgSend(ECPfd, &ui_ecp_msg, sizeof(lib::UI_ECP_message), &macrosteps,
			sizeof(int)) == -1) {
		perror("FLbtnAcceptFile: Send to ECP failed");
	} else {
		if (FDCommand == lib::TR_LOAD_TRAJECTORY) {
			// Wypisanie liczby makrokrokow.
			char tmp[5];
			sprintf(tmp, "%i", macrosteps);
			PtSetResource(ABW_TRedtNumberOfMacrosteps, Pt_ARG_TEXT_STRING, tmp,
					0);
			// Ustawienie TRbtnPositionZero na true.
			SetButtonState(ABW_TRbtnPositionZero, true);
			// odswiezenie okna
			PtDamageWidget(ABW_wndTrajectoryReproduce);
		};
	}; // end else
	// Zamkniecie okna.
	PtDestroyWidget(ABW_wndFileLocation);
	return (Pt_CONTINUE);
}
; // end: FLbtnAcceptFile

int FLbtnExit(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo) {
	// Zamkniecie okna.
	PtDestroyWidget(ABW_wndFileLocation);
	return (Pt_CONTINUE);
}
; // end: FLbtnExit

