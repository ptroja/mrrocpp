/**
 * \file logger.cc
 * \brief Logging utilities.
 * \bug Not multi-thread safe use of log_*_enabled flags
 *
 * \author Mateusz Boryń <mateusz.boryn@gmail.com>
 */

#include <cstdio>
#include <cstdarg>

#include "logger.h"

using namespace std;

namespace logger {

bool log_enabled = true;
bool log_dbg_enabled = false;

void log(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!log_enabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush(stdout);
	va_end(ap);
}

void log_dbg(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!log_dbg_enabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush(stdout);
	va_end(ap);
}

}//namespace logger
