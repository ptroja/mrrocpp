// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_CLASS_H
#define __UI_CLASS_H

#include "ui/ui.h"
#include "ui/ui_r_bird_hand.h"
#include "lib/configurator.h"

//
//
// KLASA ui
//
//


// super klasa agregujaca porozrzucane struktury


class Ui {
private:

public:
	boost::mutex process_creation_mtx;
	lib::configurator* config;
	lib::sr_ecp* all_ecp_msg; // Wskaznik na obiekt do komunikacji z SR z fukcja ECP dla wszystkich robotow
	lib::sr_ui* ui_msg; // Wskaznik na obiekt do komunikacji z SR

	mp_state_def mp;
	// bool is_any_edp_active;
	bool is_mp_and_ecps_active;
	bool is_sr_thread_loaded;
	UI_ALL_EDPS_STATE all_edps;
	std::string config_file_relativepath; // sciezka lokalana do konfiguracji wraz z plikiem konfiguracyjnym
	std::string binaries_network_path; // sieciowa sciezka binariow mrrocpp
	std::string binaries_local_path; // lokalna sciezka binariow mrrocpp
	std::string mrrocpp_local_path; // lokalna sciezka mrrocpp: np. "/home/yoyek/mrrocpp/". W niej katalogi bin, configs etc.

	std::string teach_filesel_fullpath; // sciezka domyslana dla fileselect dla generatora uczacego
	std::string config_file;// nazwa pliku konfiguracyjnego dla UI
	std::string session_name; // nazwa sesji
	std::string config_file_fullpath; // sciezka globalna do konfiguracji


	std::string ui_attach_point;
	std::string network_sr_attach_point;
	std::string sr_attach_point;
	std::string ui_node_name; // nazwa wezla na ktorym jest uruchamiany UI

	UiRobotBirdHand bird_hand;

	Ui();
	void init();
	int manage_interface(void);
	int reload_whole_configuration();

};

#endif

