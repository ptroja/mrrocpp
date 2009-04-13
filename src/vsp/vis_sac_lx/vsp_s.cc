#include <sys/neutrino.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
//#include <unistd.h>
#include <strings.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "vsp/vsp_vis_sac_lx.h"

#include "lib/configurator.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

//#include "vsp/cmvision.h"
//#include "vsp/cube.h"

//#include "vsp/global.h"
//#include "vsp/calib.h"
//#include "vsp/macierze_nr.h"


#define XMAX 768
#define YMAX 576

#define HOST "mieszko"
#define PORT 30000

#define HOST_EIH "chrobry" //"bona"
#define PORT_EIH 40000

#define BUFFER_SIZE 8*256 //8*

#define BUFFER_EIH_SIZE 22*256 //14

// extern lib::configurator* config;

int sockfd_sac, portno;
struct sockaddr_in serv_addr;
struct hostent *server;

int sockfd_eih, portno_eih;
struct sockaddr_in serv_addr_eih;
struct hostent *server_eih;

char buffer[BUFFER_SIZE];
char buffer_eih[BUFFER_SIZE];


int ImageBPL = 1024;
int state = 0;
int fd;

int size_read;
clock_t start_time, end_time;

// #pragma off(check_stack);
int interatt=0;
int x=0;
int y=0;
int z=0;

int a=0;
int b=0;
int g=0;

int x_sac=0;
int y_sac=0;
int z_sac=0;

int a_sac=0;
int b_sac=0;
int g_sac=0;

int x_jack_eih=0;
int y_jack_eih=0;
int z_jack_eih=0;

int a_jack_eih=0;
int b_jack_eih=0;
int g_jack_eih=0;

int f1x_eih=0;
int f2x_eih=0;
int f3x_eih=0;
int f4x_eih=0;

int f1y_eih=0;
int f2y_eih=0;
int f3y_eih=0;
int f4y_eih=0;

int C_T_G[16];



int irq_no;
int id;  
int md;
struct timespec start[9], stop[9], res;
//short tmp[9];
struct sigevent event;

/****7****/

float timex;
float timex1;


clock_t prev_time, curr_time;
struct timespec crr_time, s_time, e_time;

int debug=0;

char SAC_node_name[20]; 
char EIH_node_name[20]; 

int ret=0;
//CMVision vision;
//RubiksCube k1,k2;
// Rejstracja procesu VSP

vis_sac_lx::vis_sac_lx(lib::configurator &_config) : base(_config){

	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.vis_sac);

	is_sensor_configured=false; // czujnik niezainicjowany 
	is_reading_ready=false; // nie ma zadnego gotowego odczytu
	irq_no = 0;
	ThreadCtl(_NTO_TCTL_IO, NULL); // by YOYEK & 7 - nadanie odpowiednich uprawnien watkowi 

	z=0;
	x=0;
	
	strcpy(SAC_node_name, config.return_string_value("SAC_node_name"));
	strcpy(EIH_node_name, config.return_string_value("EIH_node_name"));

	//SAC
	if(strcmp( SAC_node_name, "NULL" )!=0)
	{
		portno = PORT;
		sockfd_sac = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd_sac < 0) {
			printf("ERROR opening socket");
			throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
		}
		//server = gethostbyname(HOST);
		server = gethostbyname(SAC_node_name);
		if (server == NULL) {
			printf("ERROR, no such host\n");
			throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
		}
		bzero((char *) &serv_addr, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
		serv_addr.sin_port = htons(portno);
		if (connect(sockfd_sac, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
			printf("ERROR connecting");
			throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
		}
	}
	//EIH
	if(strcmp( EIH_node_name, "NULL" )!=0)
	{	
		portno_eih = PORT_EIH;
		sockfd_eih = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd_eih < 0) {
			printf("ERROR opening socket");
			throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
		}
		//server_eih = gethostbyname(HOST_EIH);
		server_eih = gethostbyname(EIH_node_name);	
		if (server_eih == NULL) {
			printf("ERROR, no such host\n");
			throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
		}
		bzero((char *) &serv_addr_eih, sizeof(serv_addr_eih));
		serv_addr_eih.sin_family = AF_INET;
		bcopy((char *)server_eih->h_addr, (char *)&serv_addr_eih.sin_addr.s_addr, server_eih->h_length);
		serv_addr_eih.sin_port = htons(portno_eih);
		if (connect(sockfd_eih, (const struct sockaddr *) &serv_addr_eih, sizeof(serv_addr_eih)) < 0) {
			printf("ERROR connecting");
			throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
		}
	}
	sr_msg->message("VSP VIS PB-ECL-SAC LX started");
}

vis_sac_lx::~vis_sac_lx(void)
{
	close(sockfd_sac);
	close(sockfd_eih);
	printf("Destruktor VSP\n");
}

/**************************** inicjacja czujnika ****************************/
void vis_sac_lx::configure_sensor(void)
{
	is_sensor_configured=true;

	sr_msg->message("Sensor initiated"); // 7 
}
	
void vis_sac_lx::wait_for_event(){
	//delay(10);
}

/*************************** inicjacja odczytu ******************************/
void vis_sac_lx::initiate_reading(void)
{
	if (!is_sensor_configured)
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);

//	try
//	{
		int n;
		//SAC
	if(strcmp( SAC_node_name, "NULL" )!=0)
	{
		if (write(sockfd_sac,"x",1) != 1) {
			perror("write() to sockfd_sac error");
			exit(-1);
		}
		
		bzero(buffer,BUFFER_SIZE);
		
		n = read(sockfd_sac,buffer,BUFFER_SIZE);
		if (n < 0) {
			perror("read() from socket");
			exit(-1);
		} else if (n == 0) {
			printf("read() from socket returned no data");
			exit(-1);
		}
		if (sscanf(buffer,"%d %d %d %d %d %d", &x_sac,&y_sac,&z_sac, &a_sac, &b_sac, &g_sac) != 6) {
			fprintf(stderr, "sscanf(buffer) failed\n");
			exit(-1);
		}
		//printf("VSP_SAC - %d %d %d %d %d %d\n", x_sac,y_sac,z_sac, a_sac, b_sac, g_sac);
	}

		//EIH
	if(strcmp( EIH_node_name, "NULL" )!=0)
	{
	
		if (write(sockfd_eih,"x",1) != 1) {
			perror("write() to sockfd_eih error");
			exit(-1);
		}
		
		bzero(buffer_eih,BUFFER_SIZE);
		
		n = read(sockfd_eih,buffer_eih,BUFFER_EIH_SIZE);
		if (n < 0) {
			perror("read() from socket");
			exit(-1);
		} else if (n == 0) {
			printf("read() from socket returned no data");
			exit(-1);
		}
		
		if (sscanf(buffer_eih,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", 
						&x,&y,&z, &a, &b, &g, &x_jack_eih, &y_jack_eih, &z_jack_eih, &a_jack_eih, &b_jack_eih, &g_jack_eih,
						&f1x_eih, &f1y_eih, &f2x_eih, &f2y_eih, &f3x_eih, &f3y_eih, &f4x_eih, &f4y_eih) != 20) { //12
			fprintf(stderr, "sscanf(buffer) failed\n");
			exit(-1);
		}
	}
		//printf("VSP_EIH - %d %d %d %d %d %d %d %d\n", f1x_eih, f1y_eih, f2x_eih, f2y_eih, f3x_eih, f3y_eih, f4x_eih, f4y_eih);
		//printf("VSP_EIH - %d %d %d %d %d %d %d %d %d %d %d %d\n", x,y,z, a, b, g, x_jack_eih, y_jack_eih, z_jack_eih, a_jack_eih, b_jack_eih, g_jack_eih);
//	}
//	catch(...)
//	{
//		sr_msg->message ("Catched ERROR"); // 7 
//	}
	//for(int i=0; i<12; i++)
	/*
	 sscanf(buffer,"%d %d %d %d %d %d %d %d %d %d %d %d",
	 &(C_T_G[0]),&(C_T_G[1]),&(C_T_G[2]),&(C_T_G[3]),
	 &(C_T_G[4]),&(C_T_G[5]),&(C_T_G[6]),&(C_T_G[7]),
	 &(C_T_G[8]),&(C_T_G[9]),&(C_T_G[10]),&(C_T_G[11]));

	 printf("VSP - %d %d %d %d %d %d %d %d %d %d %d %d\n",
	 (C_T_G[0]),(C_T_G[1]),(C_T_G[2]),(C_T_G[3]),
	 (C_T_G[4]),(C_T_G[5]),(C_T_G[6]),(C_T_G[7]),
	 (C_T_G[8]),(C_T_G[9]),(C_T_G[10]),(C_T_G[11]));
	 */
	//printf("XXX=%d\n",x);
	is_reading_ready=true; // odczyt jakikolwiek
}
		
/***************************** odczyt z czujnika *****************************/
void vis_sac_lx::get_reading(void)
{
	// printf("7 - get reading\n");
	if (!is_sensor_configured)
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	// jezeli chcemy jakikolwiek odczyt	-> is_reading_ready
	if (!is_reading_ready)
		throw sensor_error (FATAL_ERROR, READING_NOT_READY);

	from_vsp.vsp_report=VSP_REPLY_OK;
	// tutaj: czujnik skalibrowany, odczyt dokonany, zapisany w "image", przepisanie wszystkich pol
	// przepisanie do bufora komunikacyjnego

	/*
	 po takim czyms mozemy pisac
	 struct {
	 double frame_O_T_G[16];
	 double frame_E_T_G[16];
	 double frame_E_r_G[6];
	 double frame_E_r_G__f[6];
	 } vis_sac;
	 */

	double aux=0;

	// fill up frame

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++) {
			//		vision.E_Tx_G.get_value(j,i,aux);
			from_vsp.comm_image.sensor_union.camera.frame[4*i+j]=aux;
		}
	for (int i=0; i<3; i++) {
		//		vision.E_Tx_G.get_value(3,i,aux);
		from_vsp.comm_image.sensor_union.camera.frame[4*i+3]=aux;
	}
	for (int j=0; j<3; j++)
		from_vsp.comm_image.sensor_union.camera.frame[12+j]=0;
	//if (vision.whole_face)
	//		from_vsp.comm_image.sensor_union.camera.frame[15]=1;
	//else
	//		from_vsp.comm_image.sensor_union.camera.frame[15]=0;

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++) {
			//	vision.E_Tx_G.get_value(j,i,aux);
			from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[4*i+j]=aux;
		}
	for (int i=0; i<3; i++) {
		//		vision.E_Tx_G.get_value(3,i,aux);
		from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[4*i+3]=aux;
	}

	/*
	 for(int j=0; j<12; j++)
	 from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[j]=(double)  C_T_G[j]/10000;
	 
	 for(int j=0; j<3; j++)
	 from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[12+j]=0;
	 
	 from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[15]=1;
	 */
	//if (vision.whole_face)
	//		from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[15]=1;
	//else
	//		from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[15]=0;

	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__CEIH[0]=(double) x_jack_eih/10000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__CEIH[1]=(double) y_jack_eih/10000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__CEIH[2]=(double) z_jack_eih/10000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__CEIH[3]=(double) a_jack_eih/100000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__CEIH[4]=(double) b_jack_eih/100000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__CEIH[5]=(double) g_jack_eih/100000;

	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__f[0]=(double) x/10000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__f[1]=(double) y/10000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__f[2]=(double) z/10000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__f[3]=(double) a/100000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__f[4]=(double) b/100000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__f[5]=(double) g/100000;

	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G[0]=(double) x_sac/10000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G[1]=(double) y_sac/10000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G[2]=(double) z_sac/10000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G[3]=(double) a_sac/100000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G[4]=(double) b_sac/100000;
	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G[5]=(double) g_sac/100000;
	
	from_vsp.comm_image.sensor_union.vis_sac.fEIH_G[0]=(double) f1x_eih/1000;
	from_vsp.comm_image.sensor_union.vis_sac.fEIH_G[1]=(double) f1y_eih/1000;
	from_vsp.comm_image.sensor_union.vis_sac.fEIH_G[2]=(double) f2x_eih/1000;
	from_vsp.comm_image.sensor_union.vis_sac.fEIH_G[3]=(double) f2y_eih/1000;
	from_vsp.comm_image.sensor_union.vis_sac.fEIH_G[4]=(double) f3x_eih/1000;
	from_vsp.comm_image.sensor_union.vis_sac.fEIH_G[5]=(double) f3y_eih/1000;
	from_vsp.comm_image.sensor_union.vis_sac.fEIH_G[6]=(double) f4x_eih/1000;
	from_vsp.comm_image.sensor_union.vis_sac.fEIH_G[7]=(double) f4y_eih/1000;
	

	//for(int i=2; i<6; i++)
	//	from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G__f[i]=0;

	//std::cout << "from VSP" << std::endl;
	//for(int i=0; i<6; i++)
	//std::cout << from_vsp.comm_image.sensor_union.vis_sac.frame_E_T_G[i] << " ";
	//	std::cout << from_vsp.comm_image.sensor_union.vis_sac.frame_E_r_G[i] << " ";
	//std::cout << std::endl;
	// for(int i=0; i<16; i++)
	// 	from_vsp.comm_image.sensor_union.camera.frame[i] = 0.5;
	// sr_msg->message ("VSP Get reading ok");   
	is_reading_ready=false; // 7
}

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
base* return_created_sensor(lib::configurator &_config)
{
	return new vis_sac_lx(_config);
}

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp
