/*
 * ToolBar.cpp
 *
 *  Created on: 23-11-2011
 *      Author: mboryn
 */

#include "ToolBar.h"

namespace Ui {

ToolBar::ToolBar(MenuBar *menuBar, QWidget *parent) :
		QToolBar(parent)
{
	setFloatable(false);

	addAction(menuBar->actionall_EDP_Load);
	addAction(menuBar->actionall_EDP_Unload);
	addAction(menuBar->actionMP_Load);
	addAction(menuBar->actionMP_Unload);
	addSeparator();
	addAction(menuBar->actionall_Synchro_Position);
	addSeparator();
	addAction(menuBar->actionReload_Configuration);
	addAction(menuBar->actionClear_Console);
}
ToolBar::~ToolBar()
{

}

}
