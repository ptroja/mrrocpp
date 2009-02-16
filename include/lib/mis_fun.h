// ------------------------------------------------------------------------
// Plik:				mis_fun.h
// System:		QNX/MRROC++   v. 6.3
// Opis:			miscelangeous functions
// Modyfikacja:
// Jej autor:
// Data:			2006
// ------------------------------------------------------------------------

#ifndef __MIS_FUN_H
#define __MIS_FUN_H

#include <pthread.h>
#include <string.h>

#include "common/impconst.h"

// setting of thread priority
void set_thread_priority(pthread_t thread, int sched_priority_l);

// by Y
inline void copy_frame(frame_tab destination_frame, const frame_tab source_frame)
{
	memcpy(destination_frame, source_frame, sizeof(frame_tab));
	/*
	for (int   column = 0; column < 4; column++)
		for (int row = 0; row < 3; row++)
			destination_frame[column][row] = source_frame[column][row];
	*/
}

#endif
