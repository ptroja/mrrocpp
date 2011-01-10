#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <strings.h>

#include <QtGui/QApplication>
#include "mainwindow.h"

#include "god.h"

God::God()
{
	mw = new MainWindow();
	//	printf("sr_buffer\n");
	//	thread_id.interrupt();
	//	thread_id.join();
}

void God::init()
{

	is_sr_thread_loaded = false;

	mw->show();

}

God::~God()
{
	//	printf("sr_buffer\n");
	//	thread_id.interrupt();
	//	thread_id.join();
}
