 // -------------------------------------------------------------------------
//                            vsp_s.cc 		dla QNX6.2
// 
//            Virtual Sensor Process (VSP) - methods
// Metody klasy VSP
// 
// Ostatnia modyfikacja: 25.06.03
// Autor: tkornuta
// odrem - prywrocic pry podlaczeniu klasy kamera
// -------------------------------------------------------------------------

#include <sys/neutrino.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vsp_vis.h"
#include "vsp/cmvision.h"
#include "vsp/cube.h"

// Konfigurator
#include "lib/configurator.h"

#include "vsp/global.h"
#include "vsp/calib.h"
#include "vsp/macierze_nr.h"

#include "include.c"

#define XMAX 768
#define YMAX 576

#pragma off(check_stack);
int interatt=0;
int x=0;
int z=0;
int irq_no;
int id;  
int md;
struct timespec start[9], stop[9], res;
short tmp[9];
struct sigevent event;

/****7****/


// unsigned char buffer[1000000]; 

float timex;
float timex1;


clock_t prev_time, curr_time;
struct timespec crr_time, s_time, e_time;

int debug=0;

float pose_x_prev, pose_y_prev, pose_z_prev;

int ret=0;
CMVision vision;
RubiksCube k1,k2;

#pragma on(check_stack);

// extern pid_t UI_pid;           // identyfikator procesu UI


// extern configurator* config;

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
vsp_sensor* return_created_sensor (void)
{
	return new vsp_vis_sensor();
}// : return_created_sensor




// Rejstracja procesu VSP
vsp_vis_sensor::vsp_vis_sensor(void){
	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.camera);


	uint64_t e;			// kod bledu systemowego
	
	is_sensor_configured=false;	// czujnik niezainicjowany 
	is_reading_ready=false;				// nie ma zadnego gotowego odczytu
	irq_no = 0;
	ThreadCtl (_NTO_TCTL_IO, NULL);  // by YOYEK & 7 - nadanie odpowiednich uprawnien watkowi 
	
	printf("Konstruktor VSP_VIS!\n");

	prev=120;
	current=0;
	current_y=0;
	current_z=0;
	
	cc=vvector(2);
	fc=vvector(2);
	kc=vvector(5);
	
	
	
/*	
	fc[1]=751.860077541601300;
	fc[2]=757.240379484519850;

	cc[1]=368.297088758283450; 
	cc[2]=275.241113833860310;

	kc[1]= -0.353305987532453; 
	kc[2]= 0.224942921451107; 
	kc[3]= 0.002144573630332; 
	kc[4]= 0.000737975434375; 
	kc[5]= 0.000000000000000;
*/
		fc[1]=1624.23566; //751.860077541601300;
	fc[2]=1630.87379; //757.240379484519850;

	cc[1]=378.16536; //368.297088758283450; 
	cc[2]=266.82798; //275.241113833860310;

	kc[1]= 0.00481; //-0.353305987532453; 
	kc[2]= 0.47232; //0.224942921451107; 
	kc[3]= -0.00174; //0.002144573630332; 
	kc[4]= -0.00569; //0.000737975434375; 
	kc[5]= 0.000000000000000;


	x_kk=matrix(2,4);
	X_kk=matrix(3,4);
	omckk=vvector(3);
	Tckk=vvector(3);
	Rckk=matrix(3,3);

	mrrocpp_network_path = config.return_mrrocpp_network_path();
	
	   int size = 1 + strlen(master.mrrocpp_network_path) + strlen("data/color.txt");
	    char * path1 = new char[size];
	    // Stworzenie sciezki do pliku.
	    strcpy(path1, master.mrrocpp_network_path);
	    sprintf(path1, "%sdata/color.txt", master.mrrocpp_network_path);
	   
	
	const char * file_location = path1;

	//vision.loadColors("color.txt");
  	//printf("ret%d",ret);
  	
			if (vision.loadColors(file_location)){
				 vision.initialize(XMAX,YMAX);
				 vision.countLUT();
				 vision.initEstim();
}
else printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
	fd = open("/dev/bttvx",O_RDWR); // bezposrednio odczyt ze sterownika zamiast konstruktora 
		
	z=0;
	x=0;
	
	};

vsp_vis_sensor::~vsp_vis_sensor(void){
	close (fd);
	
	free_matrix(Rckk);
	free_matrix(x_kk);
	free_matrix(X_kk);
	free_vector(cc);
	free_vector(fc);
	free_vector(kc);
	free_vector(omckk);
	free_vector(Tckk);
	
	
	
	printf("Destruktor VSP\n");
	};

/**************************** inicjacja czujnika ****************************/
void vsp_vis_sensor::configure_sensor (void){
// printf("7 - config\n");
	is_sensor_configured=true;
 //   printf("Sensor initiated\n");
     sr_msg->message ("Sensor initiated"); // 7 
	};
	
void vsp_vis_sensor::wait_for_event(){
// printf("7 - wait_for_event\n");
/*
if(interatt==0){
	memset(&event, 0, sizeof(event));// by y&w
	event.sigev_notify = SIGEV_INTR;// by y&w
	if ( (id =InterruptAttach (irq_no, int_handler, (void *) &md , sizeof(md), 0)) == -1)
		  printf( "Unable to attach interrupt handler: \n");
	interatt=1;
	};
InterruptWait (NULL, NULL);
*/
};	

/*************************** inicjacja odczytu ******************************/
void vsp_vis_sensor::initiate_reading (void){
// printf("7 - initiate reading\n");

	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	     
clock_gettime( CLOCK_REALTIME , &s_time);
 
 /****IPL7************/
 /*
  start_time = clock();	
*/
 
	size_read = read( fd, buffer, sizeof( buffer ) ); // bezposredni odczyt zamiast przez klase
   
	lseek(fd,0,SEEK_SET);

//recog
  vision.findBlobs(buffer);
 
vision.filterBlobsReset();
vision.filterBlobs(BLOB_SIZE_BIGGER,200.0);
vision.filterBlobs(BLOB_SIZE_SMALLER,10000.0);
vision.findVerticesAll();
vision.filterBlobs(VERTICES_BIGGER,3.0);
vision.filterBlobs(VERTICES_SMALLER,7.0);
vision.filterBlobs(BLOB_CIRCULARITY_BIGGER,0.5);
vision.filterBlobs(BLOB_CIRCULARITY_SMALLER,6.0);

 vision.findCorners();

//printf("VSP WHOLE=%d \n", vision.whole_face);

if(vision.whole_face==1)
	vision.estimPose();
 //2unrem

 k1.clear();

if(k1.build(&vision))	vision.setRoi(k1.roi,40);
else
vision.setRoi(k1.roi,1000);

//2unrem
/*	
	X_kk[1][1]=0; X_kk[1][2]=37; X_kk[1][3]=0; X_kk[1][4]=37; 
	X_kk[2][1]=37; X_kk[2][2]=37; X_kk[2][3]=0; X_kk[2][4]=0; 
	X_kk[3][1]=0; X_kk[3][2]=0; X_kk[3][3]=0; X_kk[3][4]=0; 
	
	x_kk[1][1]=vision.x_kk[1][1]; x_kk[1][2]=vision.x_kk[1][2]; x_kk[1][3]=vision.x_kk[1][3]; x_kk[1][4]=vision.x_kk[1][4]; 
	x_kk[2][1]=vision.x_kk[2][1]; x_kk[2][2]=vision.x_kk[2][2]; x_kk[2][3]=vision.x_kk[2][3]; x_kk[2][4]=vision.x_kk[2][4]; 
*/
/*
 if(vision.whole_face==1)
 {
	compute_extrinsic_init(x_kk,X_kk,omckk,Tckk,Rckk);
	compute_extrinsic_refine(x_kk,X_kk,omckk,Tckk,Rckk);
 }	
*/ //2unrem
	
	/*		
		current=(float)(X1+X2+X3+X4)*2.5;
		current_y=(float)(Y1+Y2+Y3+Y4)*2.5;
		current_z=(float)(Z1+Z2+Z3+Z4)*2.5;
		current_alfa=(float)(-1*atan((Y3-Y4)/(X3-X4)));
		pose_x_prev=current;
		pose_y_prev=current_y;
		pose_z_prev=current_z;
	*/	
		// printf(" VSP pose z= %f %f %f %f %f\n", current_z, Z1, Z2, Z3, Z4);
/*
}
else
{
	current=pose_x_prev; 
	current_y=pose_y_prev;
	current_z=pose_z_prev;
	// printf(" VSP bad = %f %f %f\n", current_y, current, current_z);
	sr_msg->message ("Kostka zle wyeksponowana!");
}
*/



// end_time = clock();
// timex=1000*(float)(end_time - start_time) / CLOCKS_PER_SEC;

clock_gettime( CLOCK_REALTIME , &e_time);
//printf("VSP\n");
//printf( "VSP= %f %f\n",(double)(e_time.tv_nsec-s_time.tv_nsec)/1000000,Tckk[1]);

// printf( "FXX= %d, %f %f\n",debug,(float)(curr_time) / CLOCKS_PER_SEC, (double)(e_time.tv_nsec-s_time.tv_nsec));	  
// printf( "VSP= %f %f\n",(double)(crr_time.tv_nsec), (double)(e_time.tv_nsec-s_time.tv_nsec));
// printf(" VSP pose after filtr= %f\n", current);
/****KONIEC IPL7**************/
		
// koniec przepisywania
	is_reading_ready=true;							// odczyt jakikolwiek
// InterruptEnable();	
//    sr_msg->message ("VSP Reading initiate ok");   
	}; // wait_for_event
		
/***************************** odczyt z czujnika *****************************/
void vsp_vis_sensor::get_reading (void){
// printf("7 - get reading\n");
	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// jezeli chcemy jakikolwiek odczyt	-> is_reading_ready
	// printf("7 - still reading %d\n",is_reading_ready);
	if(!is_reading_ready)
	     throw sensor_error (FATAL_ERROR, READING_NOT_READY);   

	// ok
	from_vsp.vsp_report=VSP_REPLY_OK;
	// tutaj: czujnik skalibrowany, odczyt dokonany, zapisany w "image", przepisanie wszystkich pol
	// przepisanie do bufora komunikacyjnego
	/*
	for(int i=3; i<6; i++)
	// 	from_vsp.comm_image.sensor_union.force.rez[i] = vis->x; // image.sensor_union.force.rez[i]; // odrem
	from_vsp.comm_image.sensor_union.force.rez[i] = 0;
	from_vsp.comm_image.sensor_union.force.rez[0] =(int)current_y ;// (int)(timex1);
	from_vsp.comm_image.sensor_union.force.rez[1] = (int)current; // (int)(current);
	from_vsp.comm_image.sensor_union.force.rez[2] = (int)current_z;
	from_vsp.comm_image.sensor_union.force.rez[3] = (int)current_alfa;
	*/
	/*
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
	// from_vsp.comm_image.sensor_union.camera.frame[i][j] = i+0.1*((double)(j));
	from_vsp.comm_image.sensor_union.camera.frame[4*i+j] = 0.5;
	
	for(int j=0; j<6; j++)
	from_vsp.comm_image.sensor_union.ds.readings[j] = 0.5;
	*/
	
	// fill up frame
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			from_vsp.comm_image.sensor_union.camera.frame[4*i+j]=vision.Rckk[i+1][j+1];
	for(int i=0; i<3; i++)
			from_vsp.comm_image.sensor_union.camera.frame[4*i+3]=vision.Tckk[i+1]; //vision.cube_center[i+1];
	for(int j=0; j<3; j++)
			from_vsp.comm_image.sensor_union.camera.frame[12+j]=0;
	if (vision.whole_face)
			from_vsp.comm_image.sensor_union.camera.frame[15]=1;
	else
			from_vsp.comm_image.sensor_union.camera.frame[15]=0;
	// for(int i=0; i<16; i++)
	// 	from_vsp.comm_image.sensor_union.camera.frame[i] = 0.5;
     // sr_msg->message ("VSP Get reading ok");   
     is_reading_ready=false; // 7
	};
