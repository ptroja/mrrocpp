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
#include "ui/ui_ecp_r_irp6_common.h"

#include "lib/irp6m_const.h"
#include "lib/irp6ot_const.h"
#include "lib/irp6ot_m_const.h"
#include "lib/irp6ot_tfg_const.h"
#include "lib/irp6p_const.h"
#include "lib/irp6p_m_const.h"
#include "lib/irp6p_tfg_const.h"
#include "lib/polycrank_const.h"
#include "lib/smb_const.h"
#include "lib/spkm_const.h"
#include "lib/speaker_const.h"

typedef struct{
	//ui_irp6_on_track_robot *irp6_on_track;
	ui_common_robot *irp6_on_track;
	ui_common_robot *irp6_postument;
	ui_conveyor_robot *conveyor;
	ui_speaker_robot *speaker;
	ui_common_robot *irp6_mechatronika;
	} ui_robot_def;


#endif
