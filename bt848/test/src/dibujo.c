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

#define XMAX 768
#define YMAX 480
#define YMAX2 574

PhImage_t    *pimage;
int ImageBPL = 1024;
int state = 0;
int fd;
unsigned char rrr,ggg,bbb;
unsigned char val[768][480];
 int i,j;

int xxx,yyy, aaa;
 
unsigned char buffer[5000000];
unsigned char buffer1[1000000];

void dibujo( PtWidget_t *widget, PhTile_t *damage )  {

   PhRect_t     raw_canvas; 
   PhPoint_t    c1 = { 80, 60 };
   PhPoint_t    c2 = { 30, 210 };
   PhPoint_t    r = { 72, 52 }; 

   int size_read;
   int error ; 
   int i,j; 

   PgColor_t ImagePalette[256];
   PhDim_t ImageSize;
   PhPoint_t pos;

   error = 0;

   //memset(buffer,0,sizeof(tmpbuf));
  
   PtSuperClassDraw( PtBasic, widget, damage );
   PtCalcCanvas(widget, &raw_canvas); 

   ImageSize.w = 768;
   ImageSize.h = 576;

   pos.x = 0;
   pos.y = 0;

   //ImageBPL = 1024 * 2; 
 ImageBPL = 768 * 2; 
   fd = open("/dev/bttvx",O_RDWR);
   /* Read the text */
   size_read = read( fd, buffer,
              sizeof( buffer ) );

 //  lseek(fd,0,SEEK_SET);

    /* Test for error */
   if( size_read == -1 ) \
   {
     perror( "Error reading myfile.dat" );
     return EXIT_FAILURE;
   }

   /* Set the clipping area to be the raw widget's 
      canvas. */

   PtClipAdd ( widget, &raw_canvas);

   /* Draw the ellipses. */
   c1.x += raw_canvas.ul.x;
   c1.y += raw_canvas.ul.y;

   c2.x += raw_canvas.ul.x;
   c2.y += raw_canvas.ul.y;
  /*
   xxx=0;
   yyy=0;
   aaa=0;
   for(i=2; i<480-2; i++)
		for(j=2; j<2*768-2; j+=2)
   		{ 		
			//Rx[j/2][i]=buffer[i*2* 1024+j+1]&248;
			//Gx[j/2][i]=((buffer[i*2* 1024+j+1]&7)<<5)|((buffer[i*2* 1024+j]&192)>>3);
			//Bx[j/2][i]=(buffer[i*2* 1024+j]&31)<<3;
			rrr=buffer[i*2* 1024+j+1]&248;
			ggg=((buffer[i*2* 1024+j+1]&7)<<5)|((buffer[i*2* 1024+j]&192)>>3);
			bbb=(buffer[i*2* 1024+j]&31)<<3;
			val[j/2][i]=(unsigned char)((rrr+ggg+bbb)/3);
			//if(val[j/2][i]<240) val[j/2][i]=0; else val[j/2][i]=1;
			if(val[j/2][i]<210) val[j/2][i]=0; else val[j/2][i]=1;
			xxx+=(j/2)*val[j/2][i];
			yyy+=i*val[j/2][i];
			aaa+=val[j/2][i];
			val[j/2][i]*=255;
			buffer1[i*2* 1024+j+1]=(val[j/2][i]>>5)+(val[j/2][i]&248);
			buffer1[i*2* 1024+j]=(val[j/2][i]>>3)+((val[j/2][i]&28)<<3);
			//buffer1[i*2* 1024+j]=val[j/2][i]>>3;
			//buffer1[i*2* 1024+j+1]=val[j/2][i]>>5;
			//buffer1[i*2* 1024+j+1]=(val[j/2][i]&248);
			//buffer1[i*2* 1024+j]=(val[j/2][i]&28)<<3;
			
		}
		xxx=(int)((float)xxx/aaa);
		yyy=(int)((float)yyy/aaa);
		printf("%d,%d, %d\n",xxx,yyy,size_read);
*/
   if ( buffer != NULL)   
  PgDrawImage( buffer, Pg_IMAGE_DIRECT_565, &pos, &ImageSize, ImageBPL, 0);   
//    PgDrawImage( buffer1, Pg_IMAGE_DIRECT_565, &pos, &ImageSize, ImageBPL, 0);   
   close( fd );
   PgSetFillColor( Pg_RED );
    PgDrawIRect( xxx-2, yyy-2, xxx+2, yyy+2, Pg_DRAW_FILL );
    
    PgSetStrokeColor( Pg_BLUE );
    PgDrawILine( XMAX/2-4,YMAX2/2, XMAX/2+4,YMAX2/2);
    PgDrawILine( XMAX/2,YMAX2/2-4, XMAX/2,YMAX2/2+4);
    
    
    
    PgSetStrokeColor( Pg_GREEN );
    PgDrawILine( 422-4,242, 422+4,242);
    PgDrawILine( 422,242-4, 422,242+4);
    
   // PgDrawIRect( xxx-6, yyy-6, xxx+6, yyy+6, Pg_DRAW_FILL );
  //  PgSetStrokeColor( Pg_WHITE );
   /*
   PgSetFillColor( Pg_BLACK );
    
   PgDrawIRect( 0, 0, 780, 590, Pg_DRAW_FILL );
     PgSetFillColor( Pg_WHITE );
     xxx=50;
     yyy=50;
   PgDrawIRect( xxx-25, yyy-25, xxx+25, yyy+25, Pg_DRAW_FILL );
   	xxx=106;
   	yyy=50;
   	 PgDrawIRect( xxx-25, yyy-25, xxx+25, yyy+25, Pg_DRAW_FILL );
   	xxx=162;
   	yyy=50;
   	 PgDrawIRect( xxx-25, yyy-25, xxx+25, yyy+25, Pg_DRAW_FILL );
   	
   	xxx=50;
   	yyy=106;
   	 PgDrawIRect( xxx-25, yyy-25, xxx+25, yyy+25, Pg_DRAW_FILL );
   	xxx=106;
   	yyy=106;
   	 PgDrawIRect( xxx-25, yyy-25, xxx+25, yyy+25, Pg_DRAW_FILL );
   	xxx=162;
   	yyy=106;
   	 PgDrawIRect( xxx-25, yyy-25, xxx+25, yyy+25, Pg_DRAW_FILL );
   	
   	xxx=50;
   	yyy=162;
   	 PgDrawIRect( xxx-25, yyy-25, xxx+25, yyy+25, Pg_DRAW_FILL );
   	xxx=106;
   	yyy=162;
   	 PgDrawIRect( xxx-25, yyy-25, xxx+25, yyy+25, Pg_DRAW_FILL );
   	xxx=162;
   	yyy=162;
   	 PgDrawIRect( xxx-25, yyy-25, xxx+25, yyy+25, Pg_DRAW_FILL );
   */	
    
   /* Reset the clipping area. */
   PgFlush();

   PtClipRemove ();

	}

