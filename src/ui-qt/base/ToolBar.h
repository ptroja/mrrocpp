/*
 * ToolBar.h
 *
 *  Created on: 23-11-2011
 *      Author: mboryn
 */

#ifndef TOOLBAR_H_
#define TOOLBAR_H_

#include <QToolBar>
#include "menu_bar.h"

namespace Ui {

class MenuBar;

class ToolBar : public QToolBar
{
	Q_OBJECT
public:
	ToolBar(MenuBar *menuBar, QWidget *parent);
	~ToolBar();
};

}

#endif /* TOOLBAR_H_ */
