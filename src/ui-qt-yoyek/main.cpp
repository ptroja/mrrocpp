#include <QtGui/QApplication>

#include "interface.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	mrrocpp::ui::common::Interface interface;
	interface.init();

	return a.exec();
}
