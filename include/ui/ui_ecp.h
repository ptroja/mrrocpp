// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
// 
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_H
#define _UI_ECP_H

#include "ui/ui_ecp_r_conveyor.h"
#include "ui/ui_ecp_r_speaker.h"
#include "ui/ui_ecp_r_irp6_mechatronika.h"
#include "ui/ui_ecp_r_irp6_common.h"

typedef struct{
	//ui_irp6_on_track_robot *irp6_on_track;
	ui_common_robot *irp6_on_track;
	ui_common_robot *irp6_postument;
	ui_conveyor_robot *conveyor;
	ui_speaker_robot *speaker;
	ui_irp6_mechatronika_robot *irp6_mechatronika;
	} ui_robot_def;


#endif
