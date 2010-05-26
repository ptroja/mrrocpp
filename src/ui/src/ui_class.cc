/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/ui_class.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

//
//
// KLASA ui
//
//


Ui::Ui() :
	config(NULL), all_ecp_msg(NULL), ui_msg(NULL),
			is_mp_and_ecps_active(false), all_edps(UI_ALL_EDPS_NONE_EDP_LOADED) {

	mp.state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	mp.last_state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	mp.pid = -1;

}

// funkcja odpowiedzialna za wyglad aplikacji na podstawie jej stanu

int Ui::manage_interface(void) {
	int pt_res;
	pt_res = PtEnter(0);

	check_edps_state_and_modify_mp_state();

	// na wstepie wylaczamy przyciski EDP z all robots menu. Sa one ewentualnie wlaczane dalej
	ApModifyItemState(&all_robots_menu, AB_ITEM_DIM,
			ABN_mm_all_robots_preset_positions,
			ABN_mm_all_robots_synchronisation, ABN_mm_all_robots_edp_unload,
			ABN_mm_all_robots_edp_load, NULL);

	// menu file
	// ApModifyItemState( &file_menu, AB_ITEM_DIM, NULL);

	// Dla robota IRP6 ON_TRACK
	manage_interface_irp6ot();

	// Dla robota IRP6 ON_TRACK
	manage_interface_irp6ot_tfg();

	manage_interface_irp6p_tfg();

	// Dla robota IRP6 POSTUMENT
	manage_interface_irp6p();

	// Dla robota CONVEYOR
	manage_interface_conveyor();

	manage_interface_spkm();
	manage_interface_smb();
	manage_interface_shead();

	bird_hand.manage_interface();

	// Dla robota SPEAKER
	manage_interface_speaker();

	// Dla robota IRP6 MECHATRONIKA
	manage_interface_irp6m();

	// zadanie
	// kolorowanie menu all robots


	// wlasciwosci menu  ABW_base_all_robots


	switch (all_edps) {
	case UI_ALL_EDPS_NONE_EDP_ACTIVATED:
		//				printf("UI_ALL_EDPS_NONE_EDP_ACTIVATED\n");
		block_widget(ABW_base_all_robots);
		PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_GRAY, 0);
		block_widget(ABW_base_robot);
		PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_GRAY, 0);
		break;
	case UI_ALL_EDPS_NONE_EDP_LOADED:
		//				printf("UI_ALL_EDPS_NONE_EDP_LOADED\n");
		ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
				ABN_mm_all_robots_edp_load, NULL);
		PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_BLACK, 0);
		PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_BLACK, 0);
		unblock_widget(ABW_base_all_robots);
		unblock_widget(ABW_base_robot);
		break;
	case UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED:
		//			printf("UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED\n");
		ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
				ABN_mm_all_robots_edp_unload, NULL);
		ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
				ABN_mm_all_robots_edp_load, NULL);
		PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_BLACK, 0);
		PtSetResource(ABW_base_robot, Pt_ARG_COLOR, Pg_BLACK, 0);
		unblock_widget(ABW_base_all_robots);
		unblock_widget(ABW_base_robot);
		break;
	case UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED:
		//			printf("UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED\n");
		ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
				ABN_mm_all_robots_edp_unload, NULL);
		PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_DBLUE, 0);
		unblock_widget(ABW_base_all_robots);
		unblock_widget(ABW_base_robot);
		break;
	case UI_ALL_EDPS_LOADED_AND_SYNCHRONISED:
		//				printf("UI_ALL_EDPS_LOADED_AND_SYNCHRONISED\n");
		PtSetResource(ABW_base_all_robots, Pt_ARG_COLOR, Pg_BLUE, 0);
		unblock_widget(ABW_base_all_robots);
		unblock_widget(ABW_base_robot);

		// w zaleznosci od stanu MP
		switch (mp.state) {
		case UI_MP_NOT_PERMITED_TO_RUN:
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_edp_unload, NULL);
			break;
		case UI_MP_PERMITED_TO_RUN:
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_edp_unload,
					ABN_mm_all_robots_preset_positions, NULL);
			break;
		case UI_MP_WAITING_FOR_START_PULSE:
			ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL,
					ABN_mm_all_robots_preset_positions, NULL);
			break;
		case UI_MP_TASK_RUNNING:
		case UI_MP_TASK_PAUSED:
			ApModifyItemState(&all_robots_menu, AB_ITEM_DIM,
					ABN_mm_all_robots_preset_positions, NULL);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	// wlasciwosci menu task_menu
	switch (mp.state) {

	case UI_MP_NOT_PERMITED_TO_RUN:
		ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_load,
				ABN_mm_mp_unload, NULL);
		PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLACK, 0);
		break;
	case UI_MP_PERMITED_TO_RUN:
		ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_unload, NULL);
		ApModifyItemState(&task_menu, AB_ITEM_NORMAL, ABN_mm_mp_load, NULL);
		PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLACK, 0);
		break;
	case UI_MP_WAITING_FOR_START_PULSE:
		ApModifyItemState(&task_menu, AB_ITEM_NORMAL, ABN_mm_mp_unload, NULL);
		ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_load, NULL);
		//	ApModifyItemState( &all_robots_menu, AB_ITEM_DIM, ABN_mm_all_robots_edp_unload, NULL);
		PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_DBLUE, 0);
		break;
	case UI_MP_TASK_RUNNING:
	case UI_MP_TASK_PAUSED:
		ApModifyItemState(&task_menu, AB_ITEM_DIM, ABN_mm_mp_unload,
				ABN_mm_mp_load, NULL);
		PtSetResource(ABW_base_task, Pt_ARG_COLOR, Pg_BLUE, 0);
		break;
	default:
		break;
	}

	//	PtFlush();

	if (pt_res >= 0)
		PtLeave(0);

	return 1;
}

