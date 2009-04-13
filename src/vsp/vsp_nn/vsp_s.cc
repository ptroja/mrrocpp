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

#include <stdio.h>
#include <sys/neutrino.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <strings.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>


#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"



#include "lib/srlib.h"
#include "vsp/vsp_nn.h"


// Konfigurator
#include "lib/configurator.h"
int alloc_m=0, alloc_v=0; // globalnie widoczne liczby zaalokowanych macierzy i wektorow
namespace mrrocpp {
namespace vsp {
namespace sensor {

//#define HOST "chrobry"
#define HOST "mieszko"
#define PORT 30000



int size_read;

// #pragma off(check_stack);
int interatt=0;

int irq_no;
int id;  
int md;
//short tmp[9];

int debug=0;

//float pose_x_prev, pose_y_prev, pose_z_prev;

int ret=0;

int x,y;
int sockfd, portno, n;
struct sockaddr_in serv_addr;
struct hostent *server;

char buffer[256];


// extern pid_t UI_pid;           // identyfikator procesu UI


// extern lib::configurator* config;

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
base* return_created_sensor (lib::configurator &_config)
{
	return new nn(_config);
}// : return_created_sensor




// Rejstracja procesu VSP
nn::nn(lib::configurator &_config) : base(_config){
	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.camera);

//	uint64_t e;			// kod bledu systemowego
	
	is_sensor_configured=false;	// czujnik niezainicjowany 
	is_reading_ready=false;				// nie ma zadnego gotowego odczytu
	irq_no = 0;
	ThreadCtl (_NTO_TCTL_IO, NULL);  // by YOYEK & 7 - nadanie odpowiednich uprawnien watkowi 
	
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
	};

nn::~nn(void){

	
	
	printf("Destruktor VSP\n");
	};

/**************************** inicjacja czujnika ****************************/
void nn::configure_sensor (void){

	is_sensor_configured=true;
 
     sr_msg->message ("Sensor initiated"); // 7 
	};
	
void nn::wait_for_event(){

};	

/*************************** inicjacja odczytu ******************************/
void nn::initiate_reading (void){


	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	     

//read from linux
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
 
    //printf("X: %d, Y: %d\n",x,y);
		
// koniec przepisywania
	is_reading_ready=true;							// odczyt jakikolwiek
   
	}; // wait_for_event
		
/***************************** odczyt z czujnika *****************************/
void nn::get_reading (void){

	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// jezeli chcemy jakikolwiek odczyt	-> is_reading_ready
	if(!is_reading_ready)
	     throw sensor_error (FATAL_ERROR, READING_NOT_READY);   

	// ok
	from_vsp.vsp_report=VSP_REPLY_OK;
	// tutaj: czujnik skalibrowany, odczyt dokonany, zapisany w "image", przepisanie wszystkich pol
	// przepisanie do bufora komunikacyjnego

	printf("get read\n");
	// fill up frame
	from_vsp.comm_image.sensor_union.camera.frame[4*1+0]=1; from_vsp.comm_image.sensor_union.camera.frame[4*1+1]= 0; from_vsp.comm_image.sensor_union.camera.frame[4*1+2]=0; from_vsp.comm_image.sensor_union.camera.frame[4*1+3]= 0.11*(y-288)/232+0.93+0.08;
	from_vsp.comm_image.sensor_union.camera.frame[4*0+0]=0; from_vsp.comm_image.sensor_union.camera.frame[4*0+1]= 1; from_vsp.comm_image.sensor_union.camera.frame[4*0+2]= 0; from_vsp.comm_image.sensor_union.camera.frame[4*0+3]= 0.185*(x-384)/290;
	from_vsp.comm_image.sensor_union.camera.frame[4*2+0]=0; from_vsp.comm_image.sensor_union.camera.frame[4*2+1]=0; from_vsp.comm_image.sensor_union.camera.frame[4*2+2]= 1; from_vsp.comm_image.sensor_union.camera.frame[4*2+3]= 982.285;
	from_vsp.comm_image.sensor_union.camera.frame[4*3+0]=0; from_vsp.comm_image.sensor_union.camera.frame[4*3+1]=0; from_vsp.comm_image.sensor_union.camera.frame[4*3+2]= 0; from_vsp.comm_image.sensor_union.camera.frame[4*3+3]= 1;
	
	
	
/*
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
*/   
     is_reading_ready=false; // 7
	};
} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

