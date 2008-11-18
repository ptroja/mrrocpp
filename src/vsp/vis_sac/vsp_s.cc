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

#include <strings.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
//#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vsp_vis_sac.h"
//#include "vsp/cmvision.h"
//#include "vsp/cube.h"

// Konfigurator
#include "lib/configurator.h"

//#include "vsp/global.h"
//#include "vsp/calib.h"
//#include "vsp/macierze_nr.h"

#define XMAX 768
#define YMAX 576

#define HOST "mieszko"
#define PORT 30000

int sockfd, portno, n;
struct sockaddr_in serv_addr;
struct hostent *server;

char buffer[256];


int ImageBPL = 1024;
int state = 0;
int fd;
//unsigned short buffer[600000];

int alloc_m=0, alloc_v=0; // globalnie widoczne liczby zaalokowanych macierzy i wektorow

int size_read;

// #pragma off(check_stack);
int interatt=0;
int x=0;
int y=0;
int z=0;
int id;  
int md;
struct timespec start[9], stop[9], res;
//short tmp[9];

clock_t prev_time, curr_time;
struct timespec crr_time, s_time, e_time;

int ret=0;
//CMVision vision;
//RubiksCube k1,k2;

// #pragma on(check_stack);

extern configurator* config;

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
vsp_sensor* return_created_sensor (void)
{
	return new vsp_vis_sac_sensor();
}// : return_created_sensor




// Rejstracja procesu VSP
vsp_vis_sac_sensor::vsp_vis_sac_sensor(void){
	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.vis_sac);

//	uint64_t e;			// kod bledu systemowego
	
	is_sensor_configured=false;	// czujnik niezainicjowany 
	is_reading_ready=false;				// nie ma zadnego gotowego odczytu
	
	
	// Obliczenie dlugosci sciezki do pliku INI.
/*	
	char* node_l = config->return_node();
	char* dir_l = config->return_dir();
	
	//int size = strlen("/net/") + strlen(node_l) + strlen(dir_l) + strlen("data/color.txt");
	int size = strlen("../data/color.txt");
	char * file_location = new char[size];
	// Stworzenie sciezki do pliku.
	//strcpy(file_location, "/net/");
	//strcat(file_location, node_l);
	//strcat(file_location, dir_l);
	//strcat(file_location, "data/color.txt");
	strcat(file_location, "../data/color.txt");

	//size = strlen("/net/") + strlen(node_l) + strlen(dir_l) + strlen("data/pattern.txt");
	size = strlen("../data/pattern.txt");
	char * file_location2 = new char[size];
	// Stworzenie sciezki do pliku.
	//strcpy(file_location2, "/net/");
	//strcat(file_location2, node_l);
	//strcat(file_location2, dir_l);
	//strcat(file_location2, "data/pattern.txt");
	strcat(file_location2, "../data/pattern.txt");
	
	//vision.loadColors("color.txt");
  	//printf("ret%d",ret);
  	
			if (vision.loadColors(file_location)){
				 vision.initialize(XMAX,YMAX);
				 vision.countLUT();
				 vision.initEstim(file_location2);

}
else printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
	fd = open("/dev/bttvx",O_RDWR); // bezposrednio odczyt ze sterownika zamiast konstruktora 
*/
		
	z=0;
	x=0;
		portno = PORT;
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
	{
	    printf("ERROR opening socket");
	    throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	}
	server = gethostbyname(HOST);
	if (server == NULL) {
	    printf("ERROR, no such host\n");	    
	    throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	}
	   bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, 
	     (char *)&serv_addr.sin_addr.s_addr,
	     server->h_length);
	serv_addr.sin_port = htons(portno);
	  if (connect(sockfd, (const struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
	  {
	    printf("ERROR connecting");
	    throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
		}
	sr_msg->message ("VSP VIS PB-ECL-SAC started");
	
	};

vsp_vis_sac_sensor::~vsp_vis_sac_sensor(void){
	close (fd);

	
	
	printf("Destruktor VSP\n");
	};

/**************************** inicjacja czujnika ****************************/
void vsp_vis_sac_sensor::configure_sensor (void){

	is_sensor_configured=true;

     sr_msg->message ("Sensor initiated"); // 7 
	};
	
void vsp_vis_sac_sensor::wait_for_event(){

};	

/*************************** inicjacja odczytu ******************************/
void vsp_vis_sac_sensor::initiate_reading (void){
// printf("7 - initiate reading\n");

	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	     
//clock_gettime( CLOCK_REALTIME , &s_time);
 

 /*
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


vision.estimPose3();
//vision.estimError();

 k1.clear();

if(k1.build(&vision))	vision.setRoi(k1.roi,40);
else
vision.setRoi(k1.roi,1000);

		*/
// koniec przepisywania


	n = write(sockfd,"x",strlen("x"));
    if (n < 0) 
         printf("ERROR writing to socket");
    bzero(buffer,256);
    n = read(sockfd,buffer,255);
    if (n < 0) 
         printf("ERROR reading from socket");
    x = atoi(buffer);
    n = write(sockfd,"y",strlen("y"));
    if (n < 0) 
         printf("ERROR writing to socket");
    bzero(buffer,256);
    n = read(sockfd,buffer,255);
    if (n < 0) 
         printf("ERROR reading from socket");
	y = atoi(buffer);
	
	is_reading_ready=true;							// odczyt jakikolwiek
	
   
	}; // wait_for_event
		
/***************************** odczyt z czujnika *****************************/
void vsp_vis_sac_sensor::get_reading (void){
// printf("7 - get reading\n");
	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// jezeli chcemy jakikolwiek odczyt	-> is_reading_ready
	if(!is_reading_ready)
	     throw sensor_error (FATAL_ERROR, READING_NOT_READY);   

	from_vsp.vsp_report=VSP_REPLY_OK;
	// tutaj: czujnik skalibrowany, odczyt dokonany, zapisany w "image", przepisanie wszystkich pol
	// przepisanie do bufora komunikacyjnego
	
	printf("QQQQQQQQQQQQQQQQQQQQQ get readX %d %d\n", x,y);
double aux=0;
	
	// fill up frame
	
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
	//		vision.E_Tx_G.get_value(j,i,aux);
			from_vsp.comm_image.sensor_union.camera.frame[4*i+j]=aux;
		}
	for(int i=0; i<3; i++)
	{
	//		vision.E_Tx_G.get_value(3,i,aux);
			from_vsp.comm_image.sensor_union.camera.frame[4*i+3]=aux; 
	}
	for(int j=0; j<3; j++)
			from_vsp.comm_image.sensor_union.camera.frame[12+j]=0;
	//if (vision.whole_face)
	//		from_vsp.comm_image.sensor_union.camera.frame[15]=1;
	//else
	//		from_vsp.comm_image.sensor_union.camera.frame[15]=0;
			
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
		//	vision.E_Tx_G.get_value(j,i,aux);
			from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[4*i+j]=aux;
		}
	for(int i=0; i<3; i++)
	{
	//		vision.E_Tx_G.get_value(3,i,aux);
			from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[4*i+3]=aux; 
	}
	for(int j=0; j<3; j++)
			from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[12+j]=0;
	//if (vision.whole_face)
	//		from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[15]=1;
	//else
	//		from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[15]=0;
	
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__f[0]=0;//vision.C_eps_EG[0];
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__f[1]=0;//vision.C_eps_EG[1];
	
	for(int i=2; i<6; i++)
		from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__f[i]=0;
	
	//cout << "from VSP" << endl;
	//for(int i=0; i<16; i++)
	//	cout << from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[i] << " ";
	//cout << endl;
	// for(int i=0; i<16; i++)
	// 	from_vsp.comm_image.sensor_union.camera.frame[i] = 0.5;
     // sr_msg->message ("VSP Get reading ok");   
     is_reading_ready=false; // 7
	};
