/*
 * ToolBar.cpp
 *
 *  Created on: 23-11-2011
 *      Author: mboryn
 */

#include "ToolBar.h"

namespace Ui {

ToolBar::ToolBar(MenuBar *menuBar, QWidget *parent)
	: QToolBar(parent)
{
	setFloatable(false);

	addAction(menuBar->actionall_EDP_Load);
	addAction(menuBar->actionall_EDP_Unload);
	addAction(menuBar->actionall_Synchro_Position);
	addAction(menuBar->actionall_Front_Position);
	addAction(menuBar->actionall_Position_0);
	addAction(menuBar->actionall_Position_1);
	addAction(menuBar->actionall_Position_2);
	addAction(menuBar->actionReload_Configuration);
	addAction(menuBar->actionMP_Load);
	addAction(menuBar->actionMP_Unload);
	addAction(menuBar->actionClear_Console);
}
ToolBar::~ToolBar()
{

}

}
