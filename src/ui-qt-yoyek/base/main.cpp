#include <cstdio>
#include <signal.h>
#include <sys/wait.h>
#include <QtGui/QApplication>

#include "interface.h"

mrrocpp::ui::common::Interface* interface;

/* Przechwycenie sygnalu */
void catch_signal(int sig)
{
	int status;
	pid_t child_pid;

	// print a message
	fprintf(stderr, "UI: %s\n", strsignal(sig));

	switch (sig)
	{
		case SIGINT:
			interface->UI_close();
			break;
		case SIGALRM:
			break;
		case SIGSEGV:
			signal(SIGSEGV, SIG_DFL);
			break;
		case SIGCHLD:
			child_pid = waitpid(-1, &status, 0);

			if (child_pid == -1) {
				//	int e = errno;
				perror("UI: waitpid()");
			} else if (child_pid == 0) {
				fprintf(stderr, "UI: no child exited\n");
			} else {
				//fprintf(stderr, "UI: child %d...\n", child_pid);
				if (WIFEXITED(status)) {
					fprintf(stderr, "UI: child %d exited normally with status %d\n", child_pid, WEXITSTATUS(status));
				}
				if (WIFSIGNALED(status)) {
#ifdef WCOREDUMP
					if (WCOREDUMP(status)) {
						fprintf(stderr, "UI: child %d terminated by signal %d (core dumped)\n", child_pid, WTERMSIG(status));
					} else
#endif /* WCOREDUMP */
					{
						fprintf(stderr, "UI: child %d terminated by signal %d\n", child_pid, WTERMSIG(status));
					}
				}
				if (WIFSTOPPED(status)) {
					fprintf(stderr, "UI: child %d stopped\n", child_pid);
				}
				if (WIFCONTINUED(status)) {
					fprintf(stderr, "UI: child %d resumed\n", child_pid);
				}
			}
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
