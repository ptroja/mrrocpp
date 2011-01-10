#ifndef __INTERFACE_H
#define __INTERFACE_H

#include <QMainWindow>
#include "mainwindow.h"

class Interface
{
private:

public:
	Interface();
	void init();
	~Interface();

	std::string sr_attach_point;
	bool is_sr_thread_loaded;

	MainWindow* mw;

};

#endif

