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

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vsp_vis_pbeclsac.h"
#include "vsp/cmvision.h"
#include "vsp/cube.h"

// Konfigurator
#include "lib/configurator.h"

#include "vsp/global.h"
#include "vsp/calib.h"
#include "vsp/macierze_nr.h"
int alloc_m=0, alloc_v=0; // globalnie widoczne liczby zaalokowanych macierzy i wektorow
namespace mrrocpp {
namespace vsp {
namespace sensor {


#define XMAX 768
#define YMAX 576

int ImageBPL = 1024;
int state = 0;
int fd;
unsigned short buffer[600000];



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

/****7****/

float timex;
float timex1;


struct timespec crr_time, s_time, e_time;

int debug=0;



int ret=0;
CMVision vision;
RubiksCube k1,k2;

// #pragma on(check_stack);

// extern pid_t UI_pid;           // identyfikator procesu UI


// extern lib::configurator* config;

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
base* return_created_sensor (lib::configurator &_config)
{
	return new vis_pbeclsac(_config);
}// : return_created_sensor




// Rejstracja procesu VSP
vis_pbeclsac::vis_pbeclsac(lib::configurator &_config) : base(_config){
	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.camera);

//	uint64_t e;			// kod bledu systemowego
	
	is_sensor_configured=false;	// czujnik niezainicjowany 
	is_reading_ready=false;				// nie ma zadnego gotowego odczytu
	irq_no = 0;
	ThreadCtl (_NTO_TCTL_IO, NULL);  // by YOYEK & 7 - nadanie odpowiednich uprawnien watkowi 
	
	
	//mrrocpp_network_path = config->return_mrrocpp_network_path();
	
	   int size = 1 + strlen(mrrocpp_network_path) + strlen("data/color.txt");
	    char * path1 = new char[size];
	    // Stworzenie sciezki do pliku.
	    strcpy(path1, mrrocpp_network_path);
	    sprintf(path1, "%sdata/color.txt", mrrocpp_network_path);
	   
	
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
}
else printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
	fd = open("/dev/bttvx",O_RDWR); // bezposrednio odczyt ze sterownika zamiast konstruktora 
		
	z=0;
	x=0;
	
	sr_msg->message ("VSP VIS PB-ECL-SAC started");
	
	};

vis_pbeclsac::~vis_pbeclsac(void){
	close (fd);

	
	
	printf("Destruktor VSP\n");
	};

/**************************** inicjacja czujnika ****************************/
void vis_pbeclsac::configure_sensor (void){

	is_sensor_configured=true;

     sr_msg->message ("Sensor initiated"); // 7 
	};
	
void vis_pbeclsac::wait_for_event(){

};	

/*************************** inicjacja odczytu ******************************/
void vis_pbeclsac::initiate_reading (void){
// printf("7 - initiate reading\n");

	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	     
clock_gettime( CLOCK_REALTIME , &s_time);
 

 
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


 k1.clear();

if(k1.build(&vision))	vision.setRoi(k1.roi,40);
else
vision.setRoi(k1.roi,1000);

		
// koniec przepisywania
	is_reading_ready=true;							// odczyt jakikolwiek
	
   
	}; // wait_for_event
		
/***************************** odczyt z czujnika *****************************/
void vis_pbeclsac::get_reading (void){
// printf("7 - get reading\n");
	if(!is_sensor_configured)
	     throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// jezeli chcemy jakikolwiek odczyt	-> is_reading_ready
	if(!is_reading_ready)
	     throw sensor_error (FATAL_ERROR, READING_NOT_READY);   

	from_vsp.vsp_report= lib::VSP_REPLY_OK;
	// tutaj: czujnik skalibrowany, odczyt dokonany, zapisany w "image", przepisanie wszystkich pol
	// przepisanie do bufora komunikacyjnego
double aux=0;
	
	// fill up frame
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
			vision.E_Tx_G.get_value(j,i,aux);
			from_vsp.comm_image.sensor_union.camera.frame[4*i+j]=aux;
		}
	for(int i=0; i<3; i++)
	{
			vision.E_Tx_G.get_value(3,i,aux);
			from_vsp.comm_image.sensor_union.camera.frame[4*i+3]=aux; 
	}
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

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp
