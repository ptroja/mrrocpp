#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <strings.h>

#include <QtGui/QApplication>
#include "mainwindow.h"

#include "interface.h"

Interface::Interface()
{
	mw = new MainWindow();
	//	printf("sr_buffer\n");
	//	thread_id.interrupt();
	//	thread_id.join();
}

void Interface::init()
{

	is_sr_thread_loaded = false;

	mw->show();

}

Interface::~Interface()
{
	//	printf("sr_buffer\n");
	//	thread_id.interrupt();
	//	thread_id.join();
}
