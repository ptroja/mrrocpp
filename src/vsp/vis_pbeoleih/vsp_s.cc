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

//#include "include.c"

#define XMAX 768/2
#define YMAX 576/2

#define  SMAX(a, b)  ((a) ^ (((a)^(b)) & (((a) > (b)) - 1)))
#define  SMIN(a, b)  ((a) ^ (((a)^(b)) & (((a) < (b)) - 1)))

int ImageBPL = 1024;
int state = 0;
int fd;
unsigned short buffer[600000];
unsigned short buffer1[150000];
//double *cc, *fc, *kc; // globalnie widoczne parametry kamery
int alloc_m=0, alloc_v=0; // globalnie widoczne liczby zaalokowanych macierzy i wektorow
/*
double **x_kk, **X_kk;
double *omckk;
double *Tckk;
double **Rckk;
*/
int size_read;
clock_t start_time, end_time;

// #pragma off(check_stack);
int interatt=0;
int x=0;
int z=0;
int irq_no;
int id;  
int md;
struct timespec start[9], stop[9], res;
//short tmp[9];
struct sigevent event;

/****7****/
FILE *fp;
char name[20];
int nr;
unsigned char Rx[384][288];
unsigned char Gx[384][288];
unsigned char Bx[384][288];

//do antybalansu bieli 

int Rm;
int Gm;
int Bm;

int Rnew;
int Gnew;
int Bnew;

unsigned short Rnew2;
unsigned short Gnew2;
unsigned short Bnew2;


int Rmx;
int Gmx;
int Bmx;
int Vmx;
int Rwb; //wsp. korekcji do balansu bieli
int Gwb;
int Bwb;




//tablice do balansu bieli dla 4 obszarow:
int WBx[4], WBy[4], WBs[4];


// unsigned char buffer[1000000]; 

float timex;
float timex1;


clock_t prev_time, curr_time;
struct timespec crr_time, s_time, e_time;

int debug=0;

//float pose_x_prev, pose_y_prev, pose_z_prev;

int ret=0;
CMVision vision;
RubiksCube k1,k2;

// #pragma on(check_stack);

extern pid_t UI_pid;           // identyfikator procesu UI


extern configurator* config;

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
vsp_sensor* return_created_sensor (void)
{
	return new vsp_vis_sensor();
}// : return_created_sensor




// Rejstracja procesu VSP
vsp_vis_sensor::vsp_vis_sensor(void){
	// Wielkosc unii.
	union_size = sizeof(image.cube_face);

//	uint64_t e;			// kod bledu systemowego
	
	is_sensor_configured=false;	// czujnik niezainicjowany 
	is_reading_ready=false;				// nie ma zadnego gotowego odczytu
	irq_no = 0;
	ThreadCtl (_NTO_TCTL_IO, NULL);  // by YOYEK & 7 - nadanie odpowiednich uprawnien watkowi 
	
//	printf("Konstruktor VSP_VIS pbeoleih!\n");
	
	nr=0;
/*
	cc=vvector(2);
	fc=vvector(2);
	kc=vvector(5);

	*/
	
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
*/ //stare

/*
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
*/
	
	mrrocpp_network_path = config->return_mrrocpp_network_path();
		
		   int size = 1 + strlen(mrrocpp_network_path) + strlen("data/color_eih.txt");
		    char * path1 = new char[size];
		    // Stworzenie sciezki do pliku.
		    strcpy(path1, mrrocpp_network_path);
		    sprintf(path1, "%sdata/color_eih.txt", mrrocpp_network_path);
		   
		
		 char * file_location = path1;
		
		   int size2 = 1 + strlen(mrrocpp_network_path) + strlen("data/pattern.txt");
		    char * path2 = new char[size2];
		    // Stworzenie sciezki do pliku.
		    strcpy(path2, mrrocpp_network_path);
		    sprintf(path2, "%sdata/pattern.txt", mrrocpp_network_path);
		
		     char * file_location2 = path2;
	
	//vision.loadColors("color.txt");
  	//printf("ret%d",ret);
  	
			if (vision.loadColors(file_location)){
				 vision.initialize(XMAX,YMAX);
				 vision.countLUT();
				 vision.initEstim(file_location2);
				 vision.initGrid();
}
else 
	{
	printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
	fflush(stdout);
	}
	fd = open("/dev/bttvx",O_RDWR); // bezposrednio odczyt ze sterownika zamiast konstruktora 
	printf("vsp fs:%d\n",fd);
	fflush(stdout);	
	z=0;
	x=0;
	
	};

vsp_vis_sensor::~vsp_vis_sensor(void){
	close (fd);
	/*
	free_matrix(Rckk);
	free_matrix(x_kk);
	free_matrix(X_kk);
	free_vector(cc);
	free_vector(fc);
	free_vector(kc);
	free_vector(omckk);
	free_vector(Tckk);
	*/
	
	
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

	for(int i=0; i<YMAX; i++)
		for(int j=0; j<XMAX; j++)
			buffer1[i*XMAX+j]=buffer[4*i*XMAX+2*j];
			
	for(int i=0; i<YMAX; i++)
		for(int j=0; j<XMAX; j++)
   		{
			Rx[j][i]=(buffer1[i*XMAX+j]&0xf800)>>8;
			Gx[j][i]=(buffer1[i*XMAX+j]&0x07e0)>>3;
			Bx[j][i]=(buffer1[i*XMAX+j]&0x001f)<<3;
		}



	Rm=0;
	Gm=0;
	Bm=0;
/*
	WBx[0]=162; WBy[0]=28; WBs[0]=5;
	WBx[1]=208; WBy[1]=30; WBs[1]=5;
	WBx[2]=210; WBy[2]=223; WBs[2]=5;
	WBx[3]=162; WBy[3]=229; WBs[3]=5;
*/
	WBx[0]=175; WBy[0]=40; WBs[0]=5;
	WBx[1]=219; WBy[1]=38; WBs[1]=5;
	WBx[2]=212; WBy[2]=225; WBs[2]=5;
	WBx[3]=166; WBy[3]=229; WBs[3]=5;


	for(int k=0; k<4; k++)
	for(int i=WBy[k]-WBs[k]; i<=WBy[k]+WBs[k]; i++)
		for(int j=WBx[k]-WBs[k]; j<=WBx[k]+WBs[k]; j++)
   		{
			Rm+=Rx[j][i];
			Gm+=Gx[j][i];
			Bm+=Bx[j][i];
		}
		
	Rmx=(unsigned short)(Rm/((2*WBs[0]+1)*(2*WBs[0]+1)+(2*WBs[1]+1)*(2*WBs[1]+1)+(2*WBs[2]+1)*(2*WBs[2]+1)+(2*WBs[3]+1)*(2*WBs[3]+1)));
	Gmx=(unsigned short)(Gm/((2*WBs[0]+1)*(2*WBs[0]+1)+(2*WBs[1]+1)*(2*WBs[1]+1)+(2*WBs[2]+1)*(2*WBs[2]+1)+(2*WBs[3]+1)*(2*WBs[3]+1)));
	Bmx=(unsigned short)(Bm/((2*WBs[0]+1)*(2*WBs[0]+1)+(2*WBs[1]+1)*(2*WBs[1]+1)+(2*WBs[2]+1)*(2*WBs[2]+1)+(2*WBs[3]+1)*(2*WBs[3]+1)));

	Vmx=(unsigned short)((Rmx+Gmx+Bmx)/3);

	for(int i=0; i<YMAX; i++)
		for(int j=0; j<XMAX; j++)
		{
	
			Rnew=Rx[j][i]*Vmx/Rmx;
			Gnew=Gx[j][i]*Vmx/Gmx;
			Bnew=Bx[j][i]*Vmx/Bmx;
		 
			Rnew = SMIN(SMAX(Rnew,0),255);
			Gnew = SMIN(SMAX(Gnew,0),255);
			Bnew = SMIN(SMAX(Bnew,0),255);

			Rnew2=((unsigned short)Rnew&248)<<8;
			Gnew2=((unsigned short)Gnew&252)<<3;
			Bnew2=((unsigned short)Bnew&248)>>3;

  			buffer1[i*XMAX+j]=Rnew2 | Gnew2 | Bnew2;
  	
			}



	


	
	//czy dobrze wstawilismy w bufor
	
	for(int i=0; i<YMAX; i++)
		for(int j=0; j<XMAX; j++)
   		{
			Rx[j][i]=(buffer1[i*XMAX+j]&0xf800)>>8;
			Gx[j][i]=(buffer1[i*XMAX+j]&0x07e0)>>3;
			Bx[j][i]=(buffer1[i*XMAX+j]&0x001f)<<3;
		}
	
	
	vision.classifyFace(buffer1);
	
	sprintf(name,"kostka%d.bmp",nr);

	fp=fopen(name,"wb");

	nr++;
	
	 putc(0x42, fp);
	 putc(0x4D, fp);
	 putc(0x36, fp);
	 putc(0x10, fp);
	 putc(0x05, fp);
	 putc(0x00, fp);
	 putc(0x00, fp);
	 putc(0x00, fp);
	 putc(0x00, fp);
	 putc(0x00, fp);
	 putc(0x36, fp);
	  putc(0x00, fp);
	 putc(0x00, fp);
	 putc(0x00, fp);
	  putc(0x28, fp);
	  putc(0x00, fp);
	 putc(0x00, fp);
	 putc(0x00, fp);
	 putc(0x80, fp);
	 putc(0x01, fp);
	  putc(0x00, fp);
	 putc(0x00, fp);
	  putc(0x20, fp);
	 putc(0x01, fp);
	  putc(0x00, fp);
	 putc(0x00, fp);
	  putc(0x01, fp);
	 putc(0x00, fp);
	 putc(0x18, fp);
	 putc(0x00, fp);
	  putc(0x00, fp);
	 putc(0x00, fp);
	  putc(0x00, fp);
	 putc(0x00, fp);
	  putc(0x00, fp);
	 putc(0x10, fp);
	  putc(0x05, fp);
	 putc(0x00, fp);
	 
	  putc(0x00, fp);
	 putc(0x00, fp);
	  putc(0x00, fp);
	 putc(0x00, fp);
	 
	 putc(0x00, fp);
	 putc(0x00, fp);
	  putc(0x00, fp);
	 putc(0x00, fp);
	 
	 putc(0x00, fp);
	 putc(0x00, fp);
	  putc(0x00, fp);
	 putc(0x00, fp);
	 
	 putc(0x00, fp);
	 putc(0x00, fp);
	  putc(0x00, fp);
	 putc(0x00, fp);
	 
	 int found=0;

	for(int i=288-1; i>=0; i--)
		for(int j=0; j<384; j++)
			{
				for(int k=0; k<9; k++)
					if((j<(int)vision.static_middle_grid_xa[k]+5) && (j>(int)vision.static_middle_grid_xa[k]-5)
					&& (i<(int)vision.static_middle_grid_ya[k]+5) && (i>(int)vision.static_middle_grid_ya[k]-5))
				 {
				 	putc(0, fp);
				  putc(0, fp);
				   putc(255, fp);
				   found=1;
				   //break;
				 }
			 	for(int k=0; k<4; k++)
				 if((j<WBx[k]+WBs[k]) && (j>WBx[k]-WBs[k])
					&& (i<WBy[k]+WBs[k]) && (i>WBy[k]-WBs[k]) && found==0 )
				 {
				 	putc(0, fp);
				  putc(0, fp);
				   putc(255, fp);
				   found=1;
				   //break;
				 }
				 
				 if(found==0)
				 {
					putc(Bx[j][i], fp);
				  putc(Gx[j][i], fp);
				   putc(Rx[j][i], fp);
				  }
				  found=0;
			}
			
			
	fclose(fp);
	

//recog
/*
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
	
	// fill up frame
	/*
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			from_vsp.comm_image.camera.frame[4*i+j]=vision.Rckk[i+1][j+1];
	for(int i=0; i<3; i++)
			from_vsp.comm_image.camera.frame[4*i+3]=vision.Tckk[i+1]; //vision.cube_center[i+1];
	for(int j=0; j<3; j++)
			from_vsp.comm_image.camera.frame[12+j]=0;
	if (vision.whole_face)
			from_vsp.comm_image.camera.frame[15]=1;
	else
			from_vsp.comm_image.camera.frame[15]=0;
	*/
	//fill up colors
	
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			from_vsp.comm_image.cube_face.colors[3*i+j]=vision.face_colors[3*i+j];
	
	// for(int i=0; i<16; i++)
	// 	from_vsp.comm_image.camera.frame[i] = 0.5;
     // sr_msg->message ("VSP Get reading ok");   
     is_reading_ready=false; // 7
	};
