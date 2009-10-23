// -------------------------------------------------------------------------
//                            vsp_s.cc 		dla QNX6.2.1
//
//            Virtual Sensor Process (lib::VSP) - methods for Schunk force/torgue sensor
// Metody klasy VSP
//
// Ostatnia modyfikacja: styczen 2005
// Autor: Yoyek (Tomek Winiarski)
// na podstawie szablonu vsp Tomka Kornuty i programu obslugi czujnika Artura Zarzyckiego
// - wersja do komuniacji z EDP
// -------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <process.h>
#include <math.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/sched.h>
#include <string.h>
#include <fstream>
#include <iomanip>
#include <ctype.h>
#include <errno.h>
#include <stddef.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>


#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/sensor.h"

#include "lib/srlib.h"
#include "vsp/vsp_pawel.h"

// Konfigurator
#include "lib/configurator.h"


namespace mrrocpp {
namespace vsp {
namespace sensor {


// extern lib::sr_vsp *sr_msg;       // Wskaznik na lacze z SR
int RGB2H[0xffff];
float RGB2S[0xffff];
float RGB2V[0xffff];


int threshold(Thsv a){

	return bw(a.h,160,240) && a.s > 0.3 && a.v > 0.3;
}

bool countLUT(){

	RGB2H[0x0]=0;
	RGB2S[0x0]=0.0;
	RGB2V[0x0]=0.0;

	for(unsigned short int i=0x1; i<=0xfffe; i++){

		float rr,gg,bb,mx,mn,oh,os,ov;

		int maxrgb=0,minrgb=0;

		rr=((float)((i&R_MASK)>>11))/31.0;
		gg=((float)((i&G_MASK)>>5))/63.0;
		bb=((float)((i&B_MASK)))/31.0;

		mx=max3(rr,gg,bb);
		mn=min3(rr,gg,bb);

		if(mx==rr){
				if(gg>=bb){	oh=60*(gg-bb)/(mx-mn);
				}else{		oh=360+60*(gg-bb)/(mx-mn);
				}
		}
		else if(mx==gg)	oh=120+60*(bb-rr)/(mx-mn);
		else if(mx==bb)	oh=240+60*(rr-gg)/(mx-mn);

		os=(mx-mn)/mx;
		ov=mx;

		RGB2H[i]=(int)(oh);
		RGB2S[i]=(float)(os);
		RGB2V[i]=(float)(ov);

	}
/*
 *  TODO: Array index out of bounds
 *
	RGB2H[0xffff]=0;
	RGB2S[0xffff]=0.0;
	RGB2V[0xffff]=0.0;
*/
	return 0;
}

/**************************** metody vsp_pawel ****************************/

sensor* return_created_sensor (lib::configurator &_config){

	return new pawel(_config);
}

pawel::pawel(lib::configurator &_config) : sensor(_config)
{
	struct timespec time_start, time_end;
	printf("[vsp_pawel]\tconstructor\n");
	union_size = sizeof(image.sensor_union.ball);
 	ms_nr = 0; // numer odczytu z czujnika

	clock_gettime( CLOCK_REALTIME , &time_start);
	countLUT();
	clock_gettime( CLOCK_REALTIME , &time_end);
	printf( "\n############\n[vsp_pawel] czas countLUT:  %f ms\n############\n\n", ((float)(time_end.tv_nsec - time_start.tv_nsec))/1000000.0 );

	fd = open("/dev/bttvx",O_RDWR);

	is_sensor_configured=false;
	is_reading_ready=false;

}

pawel::~pawel(void)
{
	close(fd);
	printf("Destruktor VSP\n");
}

/**************************** inicjacja czujnika ****************************/
void pawel::configure_sensor (void)
{
	is_sensor_configured=true;

	//srand(time(NULL));

	sr_msg->message ("Sensor configured");
//   	printf("[vsp_pawel]\tconfigure sensor\n");
}

void pawel::wait_for_event()
{

}

/*************************** inicjacja odczytu ******************************/
void pawel::initiate_reading (void)
{
	struct timespec tmp_time, time_start, time_end;
	clock_gettime( CLOCK_REALTIME , &time_start);

	if(!is_sensor_configured)
		throw sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);

	size_read = read( fd, buffer, sizeof( buffer ) );
	lseek(fd,0,SEEK_SET);

     	//sr_msg->message ("Reading initiated");

     double x,y,z;
	int n,i,j,st,ko,mst,mko;
	ball.x=0;
	ball.y=0;
	ball.z=0;

	for(i=2; i<YMAX-2; i+=2)
		for(j=2; j<XMAX-2; j+=2){
			n=i*XMAX+j;
			hsv.h=RGB2H[buffer[n]];
			hsv.s=RGB2S[buffer[n]];
			hsv.v=RGB2V[buffer[n]];
			kol[j][i]=threshold(hsv);
			col[j]=row[i]=0;
		}

	for(i=2; i<YMAX-2; i+=2){
		for(j=2; j<XMAX-2; j+=2) {
			col[j]+=kol[j][i];
			row[i]+=kol[j][i];
		}
	}

	for(i=2;i<YMAX-2;i+=2) if(row[i]<MINSIZE){ row[i]=0; }
	for(j=2;j<XMAX-2;j+=2) if(col[j]<MINSIZE){ col[j]=0; }

	st=ko=mst=mko=0;

	for(i=2;i<YMAX-2;i+=2){
			if(row[i]>0 && st==0){	st=ko=i;	}
			if(row[i]>0 && st>0){
					ko=i;
					if(ko-st>mko-mst){
							mst=st;
							mko=ko;
					}
			}
			if(row[i]==0 && st>0 ){
					st=0;
					ko=0;
			}
	}

	ball.y=(mko+mst)/2;

	st=ko=mst=mko=0;

	for(i=2;i<XMAX-2;i+=2){
			if(col[i]>0 && st==0){	st=ko=i;	}
			if(col[i]>0 && st>0){
					ko=i;
					if(ko-st>mko-mst){
							mst=st;
							mko=ko;
					}
			}
			if(col[i]==0 && st>0 ){
					st=0;
					ko=0;
			}
	}

	ball.x=(mko+mst)/2;
	ball.z=(mko-mst)/2;

	clock_gettime( CLOCK_REALTIME , &time_end);

	float duration = ((float)(time_end.tv_nsec - time_start.tv_nsec))/1000000.0;
	if( duration < 0 ) duration = ((float)(1000000000 - time_start.tv_nsec + time_end.tv_nsec))/1000000.0;

	//printf ("[initiate reading duration] %f ms\n", duration);

	is_reading_ready=true;
}

/***************************** odczyt z czujnika *****************************/
void pawel::get_reading (void)
{
	if(!is_sensor_configured)
		throw sensor_error (lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);

	if(!is_reading_ready)
		throw sensor_error (lib::FATAL_ERROR, READING_NOT_READY);

	struct timespec tmp_time, time_start, time_end;

	clock_gettime( CLOCK_REALTIME , &time_start);

	from_vsp.comm_image.sensor_union.ball.x  = ball.x;
	from_vsp.comm_image.sensor_union.ball.y  = ball.y;
	from_vsp.comm_image.sensor_union.ball.z  = ball.z;
	from_vsp.comm_image.sensor_union.ball.nr = ++ms_nr;

	from_vsp.vsp_report= lib::VSP_REPLY_OK;

	clock_gettime( CLOCK_REALTIME , &time_end);

	from_vsp.comm_image.sensor_union.ball.ts = time_end;
/*
	float duration = ((float)(time_end.tv_nsec - time_start.tv_nsec))/1000000.0;
	if( duration < 0 ) duration = ((float)(1000000000 - time_start.tv_nsec + time_end.tv_nsec))/1000000.0;

	printf ("[x] %f\t[y] %f\t[z] %f\t[duration] %f ms\n", ball.x, ball.y, ball.z, duration);
*/
     is_reading_ready=false; // 7
}

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp


