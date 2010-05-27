// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_H
#define _UI_ECP_H

#include "ui/ui_ecp_r_tfg_and_conv.h"

#include "lib/robot_consts/all_robots_const.h"

typedef struct {
	ui_tfg_and_conv_robot *smb;
	ui_tfg_and_conv_robot *shead;
} ui_robot_def;

#endif
