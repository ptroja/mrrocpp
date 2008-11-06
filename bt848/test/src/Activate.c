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

int
Activate( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{
/*
	int fd;
	unsigned char buffer[60000];
	int size_read;

	PgColor_t ImagePalette[256];
	PhDim_t ImageSize;
	PhPoint_t pos;
	int ImageBPL;
*/
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
/*
	ImageSize.w = 768;
	ImageSize.h = 576;

	pos.x = 20;
	pos.y = 20;

	ImageBPL = 1;

	fd = open("/dev/bttvx",O_RDWR);
*/ 
	 /* Read the text */
/*    	size_read = read( fd, buffer,
                      sizeof( buffer ) );
*/
	 /* Test for error */
/*    	if( size_read == -1 ) {
    	    perror( "Error reading myfile.dat" );
    	    return EXIT_FAILURE;
    	}

	PgSetPalette( ImagePalette, 0, 0, 256, Pg_PALSET_SOFT, 0);
	PgDrawImage( buffer, Pg_IMAGE_PALETTE_BYTE, &pos, &ImageSize, ImageBPL, 0); 
*/
    	/* Close the file */
/*    	close( fd );	
*/ 
	return( Pt_CONTINUE );

	}


