// -------------------------------------------------------------------------
//                            edp_s.cc 		dla QNX6.3.0
// 
//            Virtual Sensor Process (VSP) - methods for Schunk force/torgue sensor
// Metody klasy VSP
// 
// Ostatnia modyfikacja: grudzie 2004
// Autor: Yoyek (Tomek Winiarski)
// na podstawie szablonu vsp Tomka Kornuty i programu obslugi czujnika Artura Zarzyckiego
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
#include <iomanip.h>
#include <ctype.h>
#include <errno.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <iostream>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <sys/dispatch.h>
#include <hw/pci.h>
#include <hw/pci_devices.h>
#include <stddef.h>
#include <sys/mman.h>


#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "edp/ati3084/edp_s.h"
#include "edp/common/edp.h"

// Konfigurator
#include "lib/configurator.h"

#define MDS_DATA_RANGE 20

int sint_id;
struct sigevent sevent;

extern configurator* config;

// aaa = config->return_double_value("x_axis");


struct mds_data {
	int intr_mode;
	int byte_counter;
	int is_received;
	short data[MDS_DATA_RANGE];
} mds;

uint64_t *int_timeout;// by Y
struct sigevent tim_event;
intrspin_t* spinlock;

// short schunk_ms[7];

extern edp_irp6s_postument_track_effector* master;   // Bufor polecen i odpowiedzi EDP_MASTER

struct pci_dev_info info;// do karty advantech 1751pci
uintptr_t base_io_adress; // do obslugi karty advantech pci1751

unsigned int ms_nr=0;// numer odczytu z czujnika
unsigned int int_attached=0;// informacja o tym, czy obsluga przerwanie jestjuz przypisana

struct timespec start[9];

extern sr_vsp *sr_msg;       // Wskaznik na lacze z SR
// extern pthread_mutex_t smutex;

// #pragma off(check_stack);

const struct sigevent * schunk_int_handler (void *arg, int sint_id) {

	struct timespec rqtp;
     rqtp.tv_sec=0;
     rqtp.tv_nsec=INTR_NS_DELAY;

	InterruptMask(info.Irq,sint_id);
	
	if (!check_intr()) {
		// przyczyna przerwania inna niz sygnal stb z karty advantech
		InterruptUnmask(info.Irq, sint_id);
		return NULL;
	}	else 
	{
		clear_intr();
		set_ibf(1);
		if (mds.intr_mode==0) {
			if (mds.is_received==1) {
				mds.byte_counter=-1;
				mds.is_received=0;
			}
			if ((mds.byte_counter) < (MDS_DATA_RANGE-1))
			{
				mds.data[++mds.byte_counter]=get_input();
			}
			nanospin(&rqtp);

			set_ibf(0);
			InterruptUnmask(info.Irq,sint_id);
	      	return (&sevent);
		} else {
			mds.data[mds.byte_counter]=get_input();
			nanospin(&rqtp);

			set_ibf(0);
	 		InterruptUnmask(info.Irq,sint_id);
			if (++mds.byte_counter>=7) {
				mds.byte_counter=0;
				return (&sevent);
		 	}
		 	return(NULL);
		}
	}
}


// #pragma on(check_stack);

    // Rejstracja procesu VSP
edp_ATI3084_force_sensor::edp_ATI3084_force_sensor(void) : edp_force_sensor()
{

// 	printf("Konstruktor VSP!\n");


	ThreadCtl (_NTO_TCTL_IO, NULL);  // nadanie odpowiednich uprawnien watkowi
// 	printf("KONTRUKTOR EDP_S POCATEK\n");

	// ZMIENNE POMOCNICZE
	int_timeout=new(uint64_t);
	*int_timeout=SCHUNK_INTR_TIMEOUT_HIGH;// by Y

	tim_event.sigev_notify = SIGEV_UNBLOCK;// by Y

// PODLACZENIE DO PCI, INICJACJA KARTY ADVANTECH I OBSLUGI PRZERWANIA

	phdl = pci_attach( 0 );
	if( phdl == -1 ) {
		fprintf( stderr, "Unable to initialize PCI\n" );

	// 	return EXIT_FAILURE;
	}
// 	printf("po pci_attach_device\n");
	delay(100);
	/* Initialize the pci_dev_info structure */
	memset( &info, 0, sizeof( info ) );
	pidx = 0x0;
	info.VendorId = 0x13fe;
	info.DeviceId = 0x1751;

	hdl = pci_attach_device( NULL, PCI_INIT_ALL, pidx, &info );
	if( hdl == NULL ) {
		fprintf( stderr, "Unable to locate Advantech 1751\n" );
	} else {
	// 	printf("connected to Advantech 1751\n");
		delay(100);
		// printf("Przerwanie numer: %d\n",info.Irq);
		base_io_adress =  mmap_device_io(info.BaseAddressSize[2],PCI_IO_ADDR(info.CpuBaseAddress[2]));
	// 	printf("base: %d\n",base_io_adress);

		initiate_registers();// konfiguracja karty

		memset(&sevent, 0, sizeof(sevent));// obsluga przerwania
		sevent.sigev_notify = SIGEV_INTR;

		mds.intr_mode=0; // obsluga przerwania ustawiona na odbior pojedynczych slow
		mds.byte_counter=0;
		mds.is_received=0;

		  if ( (sint_id =InterruptAttach (info.Irq, schunk_int_handler, (void *) &mds , sizeof(mds), 0)) == -1)
					printf( "Unable to attach interrupt handler: \n");
	}

// USTAWIENIE ZLACZA SZEREGOWEGO

	switch(COM_NR){
	case 1:
		LSREG=0x3FD;	 	/* Line Status Register */
		LCREG=0x3FB;	 	/* Line Control Register */
		IEREG=0x3F9;	 	/* Interrupt Enable Register */
		MCREG=0x3FC;	 	/* Modem Control Register */
		TxBUF=0x3F8;	 	/* Transmit Buffer */
		RxBUF=0x3F8;	 	/* Receive Buffer */
		DIVLSB=0x3F8;	 	/* Divisor Least Sign. Byte */
		DIVMSB=0x3F9;	 	/* Divisor Most Sign. Byte */
		FCREG=0x3FA;	 	/* FIFO Control Register */
		INT_NUM=4;
		NOT_IRQ=0xEF; 	 	/* IRQ4 */
		break;
	case 2:
		LSREG=0x2FD;
		LCREG=0x2FB;
		IEREG=0x2F9;
		MCREG=0x2FC;
		TxBUF=0x2F8;
		RxBUF=0x2F8;
		DIVLSB=0x2F8;
		DIVMSB=0x2F9;
		FCREG=0x2FA;	 	/* FIFO Control Register */
		INT_NUM=3;
		NOT_IRQ=0xF7; 		/* IRQ3 */
		break;
	}

	spinlock=new(intrspin_t);
	memset( spinlock, 0, sizeof( *spinlock ) );
	InterruptLock(spinlock );

	out8 ( LCREG, 0x80 );	/* DLAB=1 */
	delay( 1 );
	// out8 ( DIVLSB, 12 );	/* divisor=12, speed=9600 bytes/sec */
	// out8 ( DIVLSB, 6 );	/* divisor=6, speed=18200 bytes/sec */
	out8 ( DIVLSB, 3 );	/* divisor=3, speed=38400 bytes/sec */
	delay( 1 );
	out8 ( DIVMSB, 0 );
	delay( 1 );
	out8 ( LCREG, 3 );	/* DLAB=0, 8 bits, 1 stop bit, NoParity */
	delay( 1 );

	while( ( in8 ( LSREG )) & 0x01 )
		(void)in8 ( RxBUF );	/* initial reading from Receiver Buffer */

	delay( 1 );
	out8 ( IEREG, 0x01 );	/* enable Data Available Interrupt */
	delay( 1 );
	out8 ( MCREG, 0x08 );	/* enable interrupts */
	delay( 1 );
	out8 ( PICMASK, in8 ( PICMASK ) & NOT_IRQ );	/* interrupt number INT_NUM is unmasked */
	delay( 1 );
	out8(FCREG, 0x81); /*program fifo*/
	delay( 1 );
	InterruptUnlock(spinlock );	/* interrupts are enabled */

	do_init(); // komunikacja wstepna

	};

edp_ATI3084_force_sensor::~edp_ATI3084_force_sensor(void){
	pci_detach_device( hdl ); // odlacza driver od danego urzadzenia na PCI
	pci_detach( phdl );    // Disconnect from the PCI server
	if (gravity_transformation) delete gravity_transformation;
	printf("Destruktor edp_ATI3084_force_sensor\n");
};

/**************************** inicjacja czujnika ****************************/
void edp_ATI3084_force_sensor::configure_sensor (void){// by Y
	is_sensor_configured=true;
 //  printf("EDP Sensor configured\n");
     sr_msg->message ("EDP Sensor configured");

     mds.intr_mode=0;

    #ifdef SERIAL

	if (do_send_command(SB) == -1) printf("Blad wyslania polecenia SB\n");
	do_Wait("SB");
	#endif

	#ifdef PARALLEL

	parallel_do_send_command(SB);
	do_Wait("SB");
	do_Wait("SB");

	#endif
	mds.intr_mode=1; // przywrocenie do 7 bajtowego trybu odbiotu danych
	mds.byte_counter=0;
	// cout << "Przed konf" << endl;
	// jesli ma byc wykorzytstywana biblioteka transformacji sil
	if (master->force_tryb == 2) 
	{
		// synchronize gravity transformation
//		printf("master->force_tryb == 2\n");
		// polozenie kisci bez narzedzia wzgledem bazy
		Homog_matrix frame = master->return_current_frame(WITH_TRANSLATION);			// FORCE Transformation by Slawomir Bazant
		// Homog_matrix frame(master->force_current_end_effector_frame); // pobranie aktualnej ramki
		if (!gravity_transformation) // nie powolano jeszcze obiektu
		{
			// polozenie czujnika wzgledem kisci (bez narzedzia)
			//frame_tab sensor_rot = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, 0}};
			// polozenie czujnika wzgledem  koncowki lancucha kinematycznego
			// Homog_matrix sensor_frame = Homog_matrix(0, -1, 0,		1, 0, 0,	0, 0, 1,	0, 0, 0.09);
			
			Homog_matrix sensor_frame = Homog_matrix(0, 1, 0, 0,	-1, 0, 0, 0,	0, 0, 1, 0.09);
			
			double weight = config->return_double_value("weight");
			double point[3] ={	config->return_double_value("x_axis_arm"), 
									config->return_double_value("y_axis_arm"),
									config->return_double_value("z_axis_arm")};
			K_vector pointofgravity(point);
			gravity_transformation = new ForceTrans(FORCE_SENSOR_ATI3084, frame, sensor_frame, weight, pointofgravity);
		}
		else 
		{
			gravity_transformation->synchro(frame); 
		}
	}
	
};

void edp_ATI3084_force_sensor::wait_for_event(){

	int iw_ret;
	int iter_counter=0; // okresla ile razy pod rzad zostala uruchomiona ta metoda

	if (!(int_attached)) {
		int_attached++;
		mds.intr_mode=1; // obsluga przerwania ustawiona na odbior 7 slow
	}

	do  {
		iter_counter++;

		mds.byte_counter=0;// zabezpieczenie przed niektorymi bledami pomiarow - sprawdzone dziala ;)

		  #ifdef SERIAL

		if (do_send_command(SGET1) == -1)
				printf("blad w send_command(sget1)\n");
		#endif

		#ifdef PARALLEL

		parallel_do_send_command(SGET1);

		#endif

		mds.intr_mode=1; // przywrocenie do 7 bajtowego trybu odbiotu danych
		mds.byte_counter=0;
		*int_timeout=SCHUNK_INTR_TIMEOUT_LOW;// by Y
		TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_INTR ,  &tim_event, int_timeout, NULL );
		iw_ret=InterruptWait (NULL, NULL);
		// kiedy po uplynieciu okreslonego czasu nie zostanie zgloszone przerwanie
		if (iw_ret==-1)
		{
			if (iter_counter==1) {
				sr_msg->message (NON_FATAL_ERROR, "Force / Torque read error - check sensor controller");
			}
			if (iter_counter%10==0)  // raz na 10
			{	

				solve_transducer_controller_failure(); // na wypadek bledu kontrolera

			}
			usleep(10000);  // aby nadmiernie nie obciazac procesora
			mds.intr_mode=0; // obsluga przerwania ustawiona na odbior pojedynczych slow
			mds.byte_counter=0;
			mds.is_received=0;
			do_init(); // komunikacja wstepna
		} else {
			if (iter_counter>1) {
				sr_msg->message ("Force / Torque sensor connection reastablished");
			}
		}

	} while(iw_ret==-1);  // dopoki nie zostanie odebrana paczka pomiarow
};


/*************************** inicjacja odczytu ******************************/
void edp_ATI3084_force_sensor::initiate_reading (void){
	double kartez_force[6];
	short  measure_report;
	
	if(!is_sensor_configured) throw sensor_error (FATAL_ERROR, SENSOR_NOT_CONFIGURED);
	double ft_table[6];
	InterruptDisable();
	for (int i=0;i<6;i++) ft_table[i] = static_cast<double>(mds.data[i+1]);
	measure_report = mds.data[0];

	InterruptEnable();
	
	// jesli pomiar byl poprawny
	if (measure_report == COMMAND_OK) 
	{
		is_reading_ready=true;
		// jesli nie ma byc wykorzytstywana biblioteka transformacji sil
		if (master->force_tryb == 1) 
		{
			
				// NOWA orientacja    
				kartez_force[0]=ft_table[1];
				kartez_force[1]=ft_table[0];
				kartez_force[2]=-ft_table[2];
				kartez_force[3]=ft_table[3];
				kartez_force[4]=ft_table[4];
				kartez_force[5]=ft_table[5];
	
				master->force_msr_upload(kartez_force);// wpisanie sily do zmiennych globalnych dla calego procesu
	
			if ((((ms_nr++)%2000)==0)&& (1)){// by Y - debug
				if( clock_gettime( CLOCK_REALTIME , &start[0]) == -1 ) {
				    printf("blad pomiaru czasu");
			     }
		// 	     printf("%f, %f, %f, %f, %f, %f EDP pomiarow: %d,  czas: %d\n", ft_table[0], ft_table[1], ft_table[2], ft_table[3], ft_table[4],
		// 					 ft_table[5],  ms_nr, start[0].tv_sec%100);
			}
		}	
		
		// jesli ma byc wykorzytstywana biblioteka transformacji sil
		if (master->force_tryb == 2 && gravity_transformation)
		{
			for(int i=0;i<3;i++) ft_table[i]/=20;
//			for(int i=3;i<6;i++) ft_table[i]/=333;
			for(int i=3;i<6;i++) ft_table[i]/=1000; // by Y - korekta
			Homog_matrix frame = master->return_current_frame(WITH_TRANSLATION);
			// Homog_matrix frame(master->force_current_end_effector_frame);
			double* output = gravity_transformation->getForce (ft_table, frame);
			master->force_msr_upload(output);
/*			if (!((ms_nr++)%1000)) {
				cerr << "Output\t";
				for(int i=0;i<3;i++) output[i]*=20;
				for(int i=3;i<6;i++) output[i]*=333;
				for(int i=0;i<6;i++) cerr << ceil(output[i]) << "  ";
				cerr << endl;// << "Input\t";
				for(int i=6;i<12;i++) cerr << ceil(output[i]) << "  ";
				cerr << endl << endl;// << "Gravity\t";
				for(int i=12;i<18;i++) cerr << ceil(output[i]) << "  ";
				cerr << endl << "Bias\t";
				for(int i=18;i<24;i++) cerr << ceil(output[i]) << "  ";
				cerr << endl << endl;
				cerr << frame << endl;
			}
*/			delete[] output;
			
		}
	}


};

/***************************** odczyt z czujnika *****************************/
void edp_ATI3084_force_sensor::get_reading (void){};


/********************** zakonczenie dzialania czujnika *************************/
void edp_ATI3084_force_sensor::terminate(void){
//	printf("VSP terminate\n");
};

int edp_ATI3084_force_sensor::parallel_do_send_command(char* command){
char a;
short value=0;
struct timespec rqtp;

     rqtp.tv_sec=0;
     rqtp.tv_nsec=100000;

	while(( a = *command++ )!=0 ){
		value=short(a);
		set_output(value);
		while(!check_ack());
		set_obf(0);
		nanosleep(&rqtp,NULL);

		if (value!=23)	while(check_ack());	// jesli polcecenie rozne od RESET
			else delay(1);
		set_obf(1);
	}
	return 1;
}


void edp_ATI3084_force_sensor::set_char_output(char* znak){
	short value=0;
	value=*znak;
	set_output(value);
}


int edp_ATI3084_force_sensor::set_output(short value) {
	short output=0;
	unsigned short comp=0x0001;
	unsigned char lower,upper;
	// wersja z pajaczkiem
	// 	const unsigned char output_positions[16]={15,7,14,6,13,5,12,4,0,8,1,9,2,10,3,11};
	// wersja z nowa plytka
	const unsigned char output_positions[16]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

		for (int i=0;i<16;i++) {
			unsigned short mask=0x0001;
			mask<<=output_positions[i];
			if (value & comp) output|=mask;
			comp<<=1;
		}
	lower= (unsigned char) (output%256);
	upper	= (unsigned char) (output>>=8);

	out8(base_io_adress+LOWER_OUTPUT,lower);
	out8(base_io_adress+UPPER_OUTPUT,upper);

	return 0;
}


short get_input(void) {
	short input=0, temp_input;
	unsigned short comp=0x0001;
	unsigned char lower,upper;
	// wersja z pajaczkiem
	// 	const unsigned char input_positions[16]={8,10,12,14,7,5,3,1,9,11,13,15,6,4,2,0};
	// wersja z nowa plytka
	const unsigned char input_positions[16]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

	lower=in8(base_io_adress+LOWER_INPUT);
	upper=in8(base_io_adress+UPPER_INPUT);

	temp_input=lower+256*upper;

		for (int i=0;i<16;i++) {
			unsigned short mask=0x0001;
			mask<<=input_positions[i];
			if (temp_input & comp) input|=mask;
			comp<<=1;
		}
	return input;
}


void edp_ATI3084_force_sensor::set_obf(unsigned char state) {
unsigned char temp_register;
temp_register=in8(base_io_adress+CONTROL_OUTPUT);

if (state) temp_register|=0x10;// dla przejsciowki
	 else temp_register&=0xef;

out8(base_io_adress+CONTROL_OUTPUT,temp_register);
}


void set_ibf(unsigned char state) {
unsigned char temp_register;
temp_register=in8(base_io_adress+CONTROL_OUTPUT);

if (state) temp_register|=0x20;// dla przejsciowki
	 else temp_register&=0xdf;

out8(base_io_adress+CONTROL_OUTPUT,temp_register);
}


unsigned char edp_ATI3084_force_sensor::check_ack() {
	unsigned char temp_register;
	temp_register=in8(base_io_adress+ACK_PORT_INPUT);
	if (temp_register&0x01) return 1;
		else return 0;
}


unsigned char check_stb() {
	unsigned char temp_register;
	temp_register=in8(base_io_adress+STB_PORT_INPUT);
	if (temp_register&0x01) return 1;
		else return 0;
}


void edp_ATI3084_force_sensor::initiate_registers(void){
	out8(base_io_adress+PORT_0_CONFIG,0x03);
	out8(base_io_adress+PORT_1_CONFIG,0x03);
	out8(base_io_adress+INTER_CONFIG,0x10);// przerwanie od !stb, mozna dolaczyc przerwanie od !ack
	delay(100);
	set_obf(1);
	set_ibf(0);
}


unsigned char check_intr(void) {
	unsigned char temp_register;
	temp_register=in8(base_io_adress+INTER_CONFIG);
	if (temp_register&0x80) return 1;
		else return 0;
}


void edp_ATI3084_force_sensor::check_cs(void) {
printf("Input int: %d, char: %c,   ",get_input(), get_input());
	if (check_ack()) printf("ACK HIGH,  ");
		else printf("ACK LOW,   ");
	if (check_stb()) printf("STB HIGH\n");
		else printf("STB LOW\n");
}


short edp_ATI3084_force_sensor::do_Wait(char* command){
	int iw_ret;

	do  {
		TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_INTR ,  &tim_event, int_timeout, NULL );
		iw_ret=InterruptWait (NULL, NULL);
		InterruptMask(info.Irq,sint_id);
		if (iw_ret!=-1) {
			mds.is_received=1;
		}
		InterruptUnmask(info.Irq,sint_id);

	} while(iw_ret!=-1);

	return OK;
}


short edp_ATI3084_force_sensor::do_send_command(char* command){
char a;
unsigned int timeout;

	// ew. miejce na pzerwanie o pustej kolejce - obecnie while pod spodem
	// 	while ( ! ( in8 ( LSREG ) & 0x40 ));

	while(( a = *command++ )!=0 ){
		timeout = 65000;        /*time for sending 1 char*/
		while( ! ( in8 ( LSREG ) & 0x20 )){
/*                      delay(20);       */
			if( ! timeout-- )
				return ( -1 );
		}
		out8 ( TxBUF,a );
	}
	return OK;
}



// metoda na wypadek skasowanie pamiecia nvram
// uwaga sterownik czujnika wysyla komunikat po zlaczu szeregowym zaraz po jego wlaczeniu

void edp_ATI3084_force_sensor::solve_transducer_controller_failure(void)
{

	unsigned char char_buf[1000];
	int licznik=0, i;

	while	(( in8 ( LSREG )) & 0x01 )
	{
		char_buf[licznik++]=in8 ( RxBUF );
		usleep(10);  // aby nadmiernie nie obciazac procesora
	}

	usleep(10);  // aby nadmiernie nie obciazac procesora


	i = do_send_command ( YESCOMM );	/* command ^W to FT */
	if (i == -1)
	{
		ERROR_CODE = __ERROR_INIT_SEND;
		printf("Blad wyslania YESCOMM w solve_transducer_controller_failure\n");
	}

 	usleep(10);  // aby nadmiernie nie obciazac procesora
		
	licznik=0;

	while	( ( in8 ( LSREG )) & 0x01)
	{
		char_buf[licznik++]=in8 ( RxBUF );
		usleep(10);  // aby nadmiernie nie obciazac procesora
	}

	// 	printf("Input int: %s,   \n",aaac);
}



short edp_ATI3084_force_sensor::do_init(void){
	short i;

	*int_timeout=SCHUNK_INTR_TIMEOUT_HIGH; // by Y
		#ifdef SERIAL

		i = do_send_command ( RESET );	/* command ^W to FT */
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			delay(20);
		i=do_send_command(CL_0);
			if (i == -1) 	ERROR_CODE = __ERROR_INIT_SEND;
			delay(20);
		i=do_send_command(CD_B);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			delay(20);
		i=do_send_command(CD_B);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			delay(20);
		i=do_send_command(CD_R);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			delay(20);
		i=do_send_command(CV_6);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			delay(20);
		i=do_send_command(SA);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			delay(20);
		i=do_send_command(SM);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			delay(20);
		i=do_send_command(SB);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			delay(20);

		i=do_send_command(CP_P);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			i=do_Wait("CP_P");
		i=do_send_command(CL_0);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			i=do_Wait("CL_0");
		i=do_send_command(CD_B);
			if (i == -1)	ERROR_CODE = __ERROR_INIT_SEND;
			i=do_Wait("CD_B");
		i=do_send_command(CD_B);
			if (i == -1)	ERROR_CODE = __ERROR_INIT_SEND;
			i=do_Wait("CD_B");
		i=do_send_command(CD_R);
			if (i == -1)	ERROR_CODE = __ERROR_INIT_SEND;
			i=do_Wait("CD_R");
		i=do_send_command(CV_6);
			if (i == -1)	ERROR_CODE = __ERROR_INIT_SEND;
			i=do_Wait("CV_6");
		i=do_send_command(SA);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			i=do_Wait("SA");
		i=do_send_command(SM);
			if (i == -1) ERROR_CODE = __ERROR_INIT_SEND;
			i=do_Wait("SM");
		i=do_send_command(SZ);
			if (i == -1)	ERROR_CODE = __ERROR_INIT_SEND;
			i=do_Wait("SZ");
		i=do_send_command(SB);
			if (i == -1)	ERROR_CODE = __ERROR_INIT_SEND;
			i=do_Wait("SB");

		#endif

#ifdef PARALLEL

		parallel_do_send_command(RESET);
		delay(20);

		parallel_do_send_command(CD_B);
		delay(20);

		parallel_do_send_command(CD_B);
		delay(20);

		parallel_do_send_command(CD_B);
		delay(20);

		parallel_do_send_command(CD_R);
		delay(20);

		parallel_do_send_command(CV_6);
		delay(20);

		parallel_do_send_command(SA);
		delay(20);

		parallel_do_send_command(SM);
		delay(20);

		parallel_do_send_command(SB);
		delay(20);

		parallel_do_send_command(CP_P);
		i=do_Wait("CP_P");

		parallel_do_send_command(CL_0);
		i=do_Wait("CL_0");

		parallel_do_send_command(CD_B);
		i=do_Wait("CD_B");

		parallel_do_send_command(CD_B);
		i=do_Wait("CD_B");

		parallel_do_send_command(CD_R);
		i=do_Wait("CD_R");

		parallel_do_send_command(CV_6);
		i=do_Wait("CV_6");

		parallel_do_send_command(SA);
		i=do_Wait("SA");

		parallel_do_send_command(SM);
		i=do_Wait("SM");

		parallel_do_send_command(SZ);
		i=do_Wait("SZ");

		parallel_do_send_command(SB);
		i=do_Wait("SB");
		i=do_Wait("SB");// by Y bez tego nie dziala

#endif

	return OK;
}


void clear_intr(void) {
	unsigned char temp_register;
	temp_register=in8(base_io_adress+INTER_CONFIG);
	temp_register|=0x80;

	out8(base_io_adress+INTER_CONFIG,temp_register);
}


edp_force_sensor* return_created_edp_force_sensor (void)
{
	return new edp_ATI3084_force_sensor();
}// : return_created_sensor
