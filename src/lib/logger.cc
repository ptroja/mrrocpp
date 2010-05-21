#include "logger.h"

#include <cstdio>
#include <cstdarg>

using namespace std;

namespace logger {

bool logEnabled = true;
bool logDbgEnabled = false;

void log(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!logEnabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush(stdout);
	va_end(ap);
}

void logDbg(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!logDbgEnabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush(stdout);
	va_end(ap);
}

}//namespace logger
