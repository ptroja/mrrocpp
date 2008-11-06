/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

//extern
 PtWidget_t * praw;


int raw_init( PtWidget_t *widget )  
{

	praw = widget;
	return PtSuperClassInit( PtBasic, widget );
}

