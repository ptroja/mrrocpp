#ifndef __GOD_H
#define __GOD_H

#include <QMainWindow>
#include "mainwindow.h"

class God
{
private:

public:
	God();
	void init();
	~God();

	std::string sr_attach_point;
	bool is_sr_thread_loaded;

	MainWindow* mw;

};

#endif

