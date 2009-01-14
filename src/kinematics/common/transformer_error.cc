// ------------------------------------------------------------------------
//                                  transformer_error.cc
//
//
// Ostatnia modyfikacja: styczen 2009
// -------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>

#include "kinematics/common/transformer_error.h"


transformer_error::Fatal_error::Fatal_error(uint64_t err_no_0, uint64_t err_no_1) :
	error0(err_no_0), error1(err_no_1)
{
}

transformer_error::NonFatal_error_1::NonFatal_error_1(uint64_t err_no) :
	error(err_no)
{
}

transformer_error::NonFatal_error_2::NonFatal_error_2(uint64_t err_no) :
	error(err_no)
{
}

transformer_error::NonFatal_error_3::NonFatal_error_3(uint64_t err_no) :
	error(err_no)
{
}

transformer_error::NonFatal_error_4::NonFatal_error_4(uint64_t err_no) :
	error(err_no)
{
}
