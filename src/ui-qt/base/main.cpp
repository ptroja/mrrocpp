#include <cstdio>
#include <signal.h>
#include <sys/wait.h>
#include <QtGui/QApplication>

#include "interface.h"

mrrocpp::ui::common::Interface* interface;

/* Przechwycenie sygnalu */
void catch_signal(int sig)
{

	// print a message
	fprintf(stderr, "UI: %s\n", strsignal(sig));

	switch (sig)
	{
		case SIGINT:
			if (interface) {
				interface->UI_close();
			}
			break;
		case SIGALRM:
			break;
		case SIGSEGV:
			signal(SIGSEGV, SIG_DFL);
			break;
		case SIGCHLD:
			/*
			 if (interface) {
			 interface->wait_for_child_termiantion(-1);
			 }
			 */
			break;
		default:
			fprintf(stderr, "UI: unknown signal (%d)\n", sig);
	} // end: switch
}

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	interface = new mrrocpp::ui::common::Interface();
	interface->init();
	int r = a.exec();

	// interface->UI_close();

	return r;
}
