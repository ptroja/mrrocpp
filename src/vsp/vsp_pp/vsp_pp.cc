// ------------------------------------------------------------------------
// Proces:		VIRTUAL SENSOR PROCESS (VSP)
// Plik:            vsp_dss.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		definicje metod klasy vsp_dsensor
// Autor:		tkornuta
// Data:		30.11.2006
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <devctl.h>         // devctl()
#include <string.h>
#include <signal.h>
#include <process.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/sched.h>
#include <fstream>            // open()
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <pthread.h>            // pthread_barrier_t
#include <sys/neutrino.h>
#include <termios.h>
#include <fcntl.h> // do flagi O_RDRW
#include <hw/inout.h>// do in out

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

// Konfigurator
#include "lib/configurator.h"
#include "vsp/vsp_pp.h"
//#include "vsp/moxaclass.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

/********************************* GLOBALS **********************************/
// Wskaznik na obiekt do komunikacji z SR.
// extern sr_vsp* sr_msg;

// Flaga uzywana do konczenia pracy watkow.
// extern short TERMINATE;

// Objekt uzywane do konfiguracji.
// extern common_config* common_c;
// extern vsp_config* vsp_c;
// extern config_directories_class* config_directories;
// extern ini_configs* ini_con;
// extern configurator* config;


// Czujnik wirtualny
// extern vsp_pp_sensor *vs;

// Zwrocenie stworzonego obiektu - czujnika. Funkcja implementowana w plikach klas dziedziczacych.
base* return_created_sensor (configurator &_config)
{
	return new pp(_config);
}// : return_created_sensor

/*****************************  KONSTRUKTOR *********************************/
pp::pp(configurator &_config) : base(_config){
	// Wielkosc unii.
	union_size = sizeof(image.sensor_union.pp);

    // Zerowe polozenia poczatkowe.
    for(int i=0; i<3; i++){
        position_lo_zero[i] = 0;
        position_hi_zero[i] = 0;
        position_lo_ext[i] = 0;
        position_hi_ext[i] = 0;
        axis_reading[i] = 0;
        image.sensor_union.pp.joy[i]=0.0;
	   from_vsp.comm_image.sensor_union.pp.joy[i] = 0.0;
	   joy_axis_img[i] = 0.0;
        };
	image.sensor_union.pp.active_motors = 0;
	from_vsp.comm_image.sensor_union.pp.active_motors = 0;
	Word_received = 0;
	Word_to_send = 0;
	Command_received = 0;

	// Ustawienie parametrow transmisji przez RS-232

    // Ustawienie flagi stanu procesu.
    readings_initiated = false;
    };// end: vsp_pp_sensor


pp::~pp(void){};


/************************** CONFIGURE SENSOR ******************************/
void pp::configure_sensor (void){
    // Przechowuje tryby odczytu/zapisu z portu RS-232.
    struct termios RS232_mode;
    // Dostep do sprzetu.
    ThreadCtl( _NTO_TCTL_IO, 0 );
    // Ustawienie dekryptora pliku portu RS-232
    RS_descriptor = open ("/dev/ser1", O_RDWR | O_NOCTTY | O_NONBLOCK);
    // Ustawienie parameterow transmisji.
    tcgetattr(RS_descriptor, &RS232_mode);
    cfsetispeed(&RS232_mode,2400);
    cfsetospeed(&RS232_mode,2400);
    RS232_mode.c_cflag = CREAD | CS8 | PARENB;
    tcsetattr(RS_descriptor, TCSANOW, &RS232_mode);
    // Usuniecie znakow z ukladu UART.
    tcflush(RS_descriptor, TCIOFLUSH);

	// Ustawienie dyskryptora pliku konfiguracji joysticka
	Joy_descriptor = open("../src/vsp/vsp_pp/joy.cfg", O_RDWR);
	if (Joy_descriptor == -1 && errno == ENOENT)
		Joy_descriptor = open("../src/vsp/vsp_pp/joy.cfg", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
	if (read (Joy_descriptor, &position_lo_zero[0], 1) == 0)
	{
		// Kalibracja joysticka - odczyt polozen zerowych i skrajnych
		Word_to_send = 1;
		write (RS_descriptor, &Word_to_send, 1);

     	while(read (RS_descriptor, &position_lo_zero[0], 1) <= 0);
     	while(read (RS_descriptor, &position_hi_zero[0], 1) <= 0);
     	while(read (RS_descriptor, &position_lo_ext[0], 1) <= 0);
     	while(read (RS_descriptor, &position_hi_ext[0], 1) <= 0);

     	while(read (RS_descriptor, &position_lo_zero[1], 1) <= 0);
     	while(read (RS_descriptor, &position_hi_zero[1], 1) <= 0);
     	while(read (RS_descriptor, &position_lo_ext[1], 1) <= 0);
     	while(read (RS_descriptor, &position_hi_ext[1], 1) <= 0);

     	while(read (RS_descriptor, &position_lo_zero[2], 1) <= 0);
     	while(read (RS_descriptor, &position_hi_zero[2], 1) <= 0);
     	while(read (RS_descriptor, &position_lo_ext[2], 1) <= 0);
     	while(read (RS_descriptor, &position_hi_ext[2], 1) <= 0);
     	
     	write (Joy_descriptor, &position_lo_zero[0], 1);
     	write (Joy_descriptor, &position_hi_zero[0], 1);
     	write (Joy_descriptor, &position_lo_ext[0], 1);
     	write (Joy_descriptor, &position_hi_ext[0], 1);

     	write (Joy_descriptor, &position_lo_zero[1], 1);
     	write (Joy_descriptor, &position_hi_zero[1], 1);
     	write (Joy_descriptor, &position_lo_ext[1], 1);
     	write (Joy_descriptor, &position_hi_ext[1], 1);

     	write (Joy_descriptor, &position_lo_zero[2], 1);
     	write (Joy_descriptor, &position_hi_zero[2], 1);
     	write (Joy_descriptor, &position_lo_ext[2], 1);
     	write (Joy_descriptor, &position_hi_ext[2], 1);
	}
	else
	{
		read (Joy_descriptor, &position_hi_zero[0], 1);
		read (Joy_descriptor, &position_lo_ext[0], 1);
		read (Joy_descriptor, &position_hi_ext[0], 1);
		
		read (Joy_descriptor, &position_lo_zero[1], 1);
		read (Joy_descriptor, &position_hi_zero[1], 1);
		read (Joy_descriptor, &position_lo_ext[1], 1);
		read (Joy_descriptor, &position_hi_ext[1], 1);

		read (Joy_descriptor, &position_lo_zero[2], 1);
		read (Joy_descriptor, &position_hi_zero[2], 1);
		read (Joy_descriptor, &position_lo_ext[2], 1);
		read (Joy_descriptor, &position_hi_ext[2], 1);
	}

/*
	printf("%d\n",position_lo_zero[0]);
	printf("%d\n",position_hi_zero[0]);
	printf("%d\n",position_lo_ext[0]);
	printf("%d\n",position_hi_ext[0]);
	printf("%d\n",position_lo_zero[1]);
	printf("%d\n",position_hi_zero[1]);
	printf("%d\n",position_lo_ext[1]);
	printf("%d\n",position_hi_ext[1]);
	printf("%d\n",position_lo_zero[2]);
	printf("%d\n",position_hi_zero[2]);
	printf("%d\n",position_lo_ext[2]);
	printf("%d\n",position_hi_ext[2]);
*/

	// Zamkniecie pliku konfiguracji joysticka
	close (Joy_descriptor);
    // Ustawienie flagi stanu procesu.
    is_sensor_configured = true;
    sr_msg->message ("Programming panel calibrated");
    };// end: configure_sensor

/**************************** INITIATE READING *******************************/
void pp::initiate_reading (void){
	// Czy czujnik skonfigurowany
	if(!is_sensor_configured)
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);

	// Sprawdz, czy panel programowania przeslal jakies polecenie
//	Word_received = 0;
//	if (Word_received == 1 || Word_received >= 4) 
//		read (RS_descriptor, &Word_received, 1);
//printf ("\n\n\nOdebrano: %d\n\n\n", Word_received);
	switch (Command_received)
	{
		case 2:
			Joy_descriptor = open("../src/vsp/vsp_pp/joy.cfg", O_RDWR | O_TRUNC);

	     	while(read (RS_descriptor, &position_lo_zero[0], 1) <= 0);
 		    	while(read (RS_descriptor, &position_hi_zero[0], 1) <= 0);
   		  	while(read (RS_descriptor, &position_lo_ext[0], 1) <= 0);
     		while(read (RS_descriptor, &position_hi_ext[0], 1) <= 0);

     		while(read (RS_descriptor, &position_lo_zero[1], 1) <= 0);
     		while(read (RS_descriptor, &position_hi_zero[1], 1) <= 0);
     		while(read (RS_descriptor, &position_lo_ext[1], 1) <= 0);
     		while(read (RS_descriptor, &position_hi_ext[1], 1) <= 0);

     		while(read (RS_descriptor, &position_lo_zero[2], 1) <= 0);
     		while(read (RS_descriptor, &position_hi_zero[2], 1) <= 0);
     		while(read (RS_descriptor, &position_lo_ext[2], 1) <= 0);
     		while(read (RS_descriptor, &position_hi_ext[2], 1) <= 0);
     	
     		write (Joy_descriptor, &position_lo_zero[0], 1);
     		write (Joy_descriptor, &position_hi_zero[0], 1);
     		write (Joy_descriptor, &position_lo_ext[0], 1);
     		write (Joy_descriptor, &position_hi_ext[0], 1);

     		write (Joy_descriptor, &position_lo_zero[1], 1);
     		write (Joy_descriptor, &position_hi_zero[1], 1);
     		write (Joy_descriptor, &position_lo_ext[1], 1);
     		write (Joy_descriptor, &position_hi_ext[1], 1);

     		write (Joy_descriptor, &position_lo_zero[2], 1);
     		write (Joy_descriptor, &position_hi_zero[2], 1);
     		write (Joy_descriptor, &position_lo_ext[2], 1);
     		write (Joy_descriptor, &position_hi_ext[2], 1);

			close (Joy_descriptor);
			break;
			
		case 3:
			from_vsp.comm_image.sensor_union.pp.active_motors += 1;
			from_vsp.comm_image.sensor_union.pp.active_motors %= 3;
			break;
			
		default:
			break;
	}

	// Zerowanie zmiennej
	Command_received = 0;
    // Przeslanie do panelu programowania polecenia wykonania odczytow osi
	Word_to_send = 2;
	write (RS_descriptor, &Word_to_send, 1);

    // Pobranie odczytow osi z bufora odbiorczego RS-232
// sprawdzic przedzial odczytu i wpisac do odpowiedniej komorki tablicy axis_reading[]
     while(read (RS_descriptor, &Word_received, 1) <= 0);
	if (Word_received >= 2 && Word_received <= 3)
	{
		Command_received = Word_received;
		while(read (RS_descriptor, &Word_received, 1) <= 0);
	}
	if (Word_received >= 4)
	{
		axis_reading[0] = Word_received;
     	while(read (RS_descriptor, &axis_reading[1], 1) <= 0);
     	while(read (RS_descriptor, &axis_reading[2], 1) <= 0);
/*
printf("Axis_X: %d\n", axis_reading[0]);
printf("Axis_Y: %d\n", axis_reading[1]);
printf("Axis_Z: %d\n", axis_reading[2]);
*/
    		// Agregacja odczytow i przepisanie do bufora komunikacyjnego.
    		for(int i=0; i<3; i++)
			// odczyt = obraz czujnika - polozenie zerowe
			if (axis_reading[i] < position_lo_zero[i])
				joy_axis_img[i] = 
					(double)(axis_reading[i] - position_lo_zero[i]) / (double)(position_lo_zero[i] - position_lo_ext[i]) / 1000.0;
			else if (axis_reading[i] > position_hi_zero[i])
				joy_axis_img[i] = 
					(double)(axis_reading[i] - position_hi_zero[i]) / (double)(position_hi_ext[i] - position_hi_zero[i]) / 1000.0;
			else joy_axis_img[i] = 0.0;
	}
	else
	{
		joy_axis_img[0] = 0.0;
		joy_axis_img[1] = 0.0;
		joy_axis_img[2] = 0.0;
	}

    // Ustawienie flagi stanu procesu.
	is_reading_ready=true;
    };// end: initiate_reading

/***************************** GET  READING *********************************/
void pp::get_reading (void){

	// Czy czujnik skonfigurowany
	if(!is_sensor_configured)
		throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);

    // Sprawdzenie, czy odczyty sa dostepne
    if (!is_reading_ready)
        throw sensor_error (NON_FATAL_ERROR, READING_NOT_READY);

	from_vsp.comm_image.sensor_union.pp.joy[0] = joy_axis_img[0];
	from_vsp.comm_image.sensor_union.pp.joy[1] = joy_axis_img[1];
	from_vsp.comm_image.sensor_union.pp.joy[2] = joy_axis_img[2];

    // Odczyty sa gotowe.
    from_vsp.vsp_report=VSP_REPLY_OK;

    // Ustawienie flagi stanu procesu.
	is_reading_ready=false;
    };// end: get_reading

/****************************** TERMINATE **********************************/
void pp::terminate(void){
    // Zamkniecie deskryptora urzadzenia.
    close(RS_descriptor);
    }; // end: terminate
} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

