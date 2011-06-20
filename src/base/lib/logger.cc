/**
 * \file logger.cc
 * \brief Logging utilities.
 * \bug Not multi-thread safe use of log_*_enabled flags
 *
 * \author Mateusz Boryń <mateusz.boryn@gmail.com>
 */

#include <cstdio>
#include <cstdarg>

#include "base/lib/mrmath/homog_matrix.h"

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
	fflush( stdout);
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
	fflush( stdout);
	va_end(ap);
}

void print_hm(const mrrocpp::lib::Homog_matrix& hm)
{
	printf("[");
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 4; ++j) {
			printf("%f\t", hm(i, j));
		}
		if (i < 2) {
			printf("\t;\n");
		} else {
			printf("\t;\n");
		}
	}
	printf("]\n");
	fflush(stdout);
}

void log(const mrrocpp::lib::Homog_matrix& hm)
{
	if (log_enabled) {
		print_hm(hm);
	}
}

void log_dbg(const mrrocpp::lib::Homog_matrix& hm)
{
	if (log_dbg_enabled) {
		print_hm(hm);
	}
}

}//namespace logger
