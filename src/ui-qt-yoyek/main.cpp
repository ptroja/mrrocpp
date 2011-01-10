#include <QtGui/QApplication>

#include "god.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	God gd;
	gd.init();

	return a.exec();
}
