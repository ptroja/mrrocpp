// -------------------------------------------------------------------------
//                            edp_s.cc 		dla QNX6.3.0
//
//            Virtual Sensor Process (lib::VSP) - methods for Schunk force/torgue sensor
// Metody klasy VSP
//
// Ostatnia modyfikacja: grudzie 2004
// Autor: Yoyek (Tomek Winiarski)
// na podstawie szablonu vsp Tomka Kornuty i programu obslugi czujnika Artura Zarzyckiego
// -------------------------------------------------------------------------
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <process.h>
#include <cmath>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/sched.h>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <cctype>
#include <cerrno>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <iostream>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <sys/dispatch.h>
#include <hw/pci.h>
#include <hw/pci_devices.h>
#include <cstddef>
#include <sys/mman.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/edp/edp_e_manip.h"

#include "sensor/ati3084/edp_s.h"

// Konfigurator
#include "base/lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

//! PCI device base IO address
static uintptr_t base_io_address;

const struct sigevent * schunk_int_handler(void *arg, int sint_id)
{
	if (!check_intr()) {
		// przyczyna przerwania inna niz sygnal stb z karty advantech
		return NULL;
	}

	ATI3084_force::mds_data_t & mds = *(ATI3084_force::mds_data_t *) arg;

	struct timespec rqtp;
	rqtp.tv_sec = 0;
	rqtp.tv_nsec = INTR_NS_DELAY;

	clear_intr();
	set_ibf(1);

	InterruptLock(&mds.spinlock);

	if (mds.intr_mode == 0) {
		if (mds.is_received) {
			mds.byte_counter = -1;
			mds.is_received = false;
		}
		if ((mds.byte_counter) < (MDS_DATA_RANGE - 1)) {
			mds.data[++mds.byte_counter] = get_input();
		}

		InterruptUnlock(&mds.spinlock);

		nanospin(&rqtp);

		set_ibf(0);

		return (&mds.sevent);
	} else {
		bool return_sevent = false;

		mds.data[mds.byte_counter] = get_input();

		if (++mds.byte_counter >= 7) {
			mds.byte_counter = 0;
			return_sevent = true;
		}

		InterruptUnlock(&mds.spinlock);

		nanospin(&rqtp);

		set_ibf(0);

		return (return_sevent) ? (&mds.sevent) : NULL;
	}
}

ATI3084_force::ATI3084_force(common::manip_effector &_master) :
	force(_master), int_attached(false)
{
	memset(&mds, 0, sizeof(mds));
}

void ATI3084_force::connect_to_hardware(void)
{
	if (!(master.force_sensor_test_mode)) {
		// 	printf("Konstruktor VSP!\n");

		ThreadCtl(_NTO_TCTL_IO, NULL); // nadanie odpowiednich uprawnien watkowi
		// 	printf("KONTRUKTOR EDP_S POCATEK\n");

		// ZMIENNE POMOCNICZE
		int_timeout = SCHUNK_INTR_TIMEOUT_HIGH;// by Y

		tim_event.sigev_notify = SIGEV_UNBLOCK;// by Y

		// PODLACZENIE DO PCI, INICJACJA KARTY ADVANTECH I OBSLUGI PRZERWANIA

		phdl = pci_attach(0);
		if (phdl == -1) {
			fprintf(stderr, "Unable to initialize PCI\n");

			// 	return EXIT_FAILURE;
		}

		/* Initialize the pci_dev_info structure */
		memset(&info, 0, sizeof(info));
		pidx = 0x0;
		info.VendorId = 0x13fe;
		info.DeviceId = 0x1751;

		hdl = pci_attach_device(NULL, PCI_INIT_ALL, pidx, &info);
		if (hdl == NULL) {
			fprintf(stderr, "Unable to locate Advantech 1751\n");
		} else {
			// 	printf("connected to Advantech 1751\n");
			delay(100);
			// printf("Przerwanie numer: %d\n",info.Irq);
			base_io_address = mmap_device_io(info.BaseAddressSize[2], PCI_IO_ADDR(info.CpuBaseAddress[2]));
			// 	printf("base: %d\n",base_io_address);

			initiate_registers();// konfiguracja karty

			mds.sevent.sigev_notify = SIGEV_INTR;

			// spinlock is not required until interrupt attached
			mds.intr_mode = 0; // obsluga przerwania ustawiona na odbior pojedynczych slow
			mds.byte_counter = 0;
			mds.is_received = false;

			if ((sint_id = InterruptAttach(info.Irq, schunk_int_handler, (void *) &mds, sizeof(mds), 0)) == -1)
				printf("Unable to attach interrupt handler: \n");
		}

		// setup serial device
		uart = open("/dev/ser1", O_RDWR);
		if (uart == -1) {
			// TODO: throw
			perror("unable to open serial device for ATI3084 sensor");
		}

		// serial port settings: 38400, 8-N-1
		struct termios tattr;
		if (tcgetattr(uart, &tattr) == -1) {
			// TODO: throw
			perror("tcgetattr()");
		}

		// setup input speed
		if (cfsetispeed(&tattr, B38400) == -1) {
			perror("cfsetispeed()");
		}

		// setup output speed
		if (cfsetospeed(&tattr, B38400) == -1) {
			perror("cfsetospeed()");
		}

		// setup raw mode
		if (cfmakeraw(&tattr) == -1) {
			perror("cfmakeraw()");
		}

		// apply settings to serial port
		if (tcsetattr(uart, TCSANOW, &tattr) == -1) {
			perror("tcsetattr()");
		}

		do_init(); // komunikacja wstepna
	}

}

ATI3084_force::~ATI3084_force(void)
{
	if (!master.force_sensor_test_mode) {
		InterruptDetach(sint_id);
		pci_detach_device(hdl); // odlacza driver od danego urzadzenia na PCI
		pci_detach(phdl); // Disconnect from the PCI server
		close(uart);
	}
	if (gravity_transformation)
		delete gravity_transformation;
	printf("ATI3084_force::~ATI3084_force\n");
}

/**************************** inicjacja czujnika ****************************/
void ATI3084_force::configure_sensor(void)
{// by Y
	is_sensor_configured = true;
	//  printf("edp Sensor configured\n");
	sr_msg->message("edp Sensor configured");
	if (!master.force_sensor_test_mode) {
		InterruptLock(&mds.spinlock);
		mds.intr_mode = 0;
		InterruptUnlock(&mds.spinlock);

		do_send_command(SB);
		do_Wait();

#ifdef PARALLEL
		do_Wait();
#endif
	}
	InterruptLock(&mds.spinlock);
	mds.intr_mode = 1; // przywrocenie do 7 bajtowego trybu odbiotu danych
	mds.byte_counter = 0;
	InterruptUnlock(&mds.spinlock);
	// cout << "Przed konf" << endl;
	// jesli ma byc wykorzytstywana biblioteka transformacji sil

	// synchronize gravity transformation

	// polozenie kisci bez narzedzia wzgledem bazy
	lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION); // FORCE Transformation by Slawomir Bazant
	// lib::Homog_matrix frame(master.force_current_end_effector_frame); // pobranie aktualnej ramki
	if (!gravity_transformation) // nie powolano jeszcze obiektu
	{
		lib::Xyz_Angle_Axis_vector tab;
		lib::Homog_matrix sensor_frame;
		if (master.config.exists("sensor_in_wrist")) {
			char *tmp = strdup(master.config.value <std::string> ("sensor_in_wrist").c_str());
			char* toDel = tmp;
			for (int i = 0; i < 6; i++)
				tab[i] = strtod(tmp, &tmp);
			sensor_frame = lib::Homog_matrix(tab);
			free(toDel);
			// std::cout<<sensor_frame<<std::endl;
		} else
			sensor_frame = lib::Homog_matrix(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0.09);
		// lib::Homog_matrix sensor_frame = lib::Homog_matrix(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0.09);

		double weight = master.config.value <double> ("weight");

		double point[3];
		char *tmp = strdup(master.config.value <std::string> ("default_mass_center_in_wrist").c_str());
		char* toDel = tmp;
		for (int i = 0; i < 3; i++)
			point[i] = strtod(tmp, &tmp);
		free(toDel);
		// double point[3] = { master.config.value<double>("x_axis_arm"),
		//		master.config.value<double>("y_axis_arm"), master.config.return_double_value("z_axis_arm") };
		lib::K_vector pointofgravity(point);
		gravity_transformation
				= new lib::ForceTrans(edp::sensor::FORCE_SENSOR_ATI3084, frame, sensor_frame, weight, pointofgravity, is_right_turn_frame);
	} else {
		gravity_transformation->synchro(frame);
	}

}

void ATI3084_force::wait_for_event()
{
	int iw_ret;
	int iter_counter = 0; // okresla ile razy pod rzad zostala uruchomiona ta metoda

	if (!master.force_sensor_test_mode) {

		if (!int_attached) {
			int_attached = true;
			InterruptLock(&mds.spinlock);
			mds.intr_mode = 1; // obsluga przerwania ustawiona na odbior 7 slow
			InterruptUnlock(&mds.spinlock);
		}

		do {
			iter_counter++;

			InterruptLock(&mds.spinlock);

			mds.byte_counter = 0;// zabezpieczenie przed niektorymi bledami pomiarow - sprawdzone dziala ;)
			mds.intr_mode = 1; // przywrocenie do 7 bajtowego trybu odbiotu danych
			mds.byte_counter = 0;

			InterruptUnlock(&mds.spinlock);

			do_send_command(SGET1);

			int_timeout = SCHUNK_INTR_TIMEOUT_LOW;// by Y
			TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_INTR, &tim_event, &int_timeout, NULL);
			iw_ret = InterruptWait(0, NULL);
			// kiedy po uplynieciu okreslonego czasu nie zostanie zgloszone przerwanie
			if (iw_ret == -1) {
				if (iter_counter == 1) {
					sr_msg->message(lib::NON_FATAL_ERROR, "Force / Torque read error - check sensor controller");
				}
				if (iter_counter % 10 == 0) // raz na 10
				{

					solve_transducer_controller_failure(); // na wypadek bledu kontrolera

				}
				usleep(10000); // aby nadmiernie nie obciazac procesora
				InterruptLock(&mds.spinlock);
				mds.intr_mode = 0; // obsluga przerwania ustawiona na odbior pojedynczych slow
				mds.byte_counter = 0;
				mds.is_received = false;
				InterruptUnlock(&mds.spinlock);
				do_init(); // komunikacja wstepna
			} else {
				if (iter_counter > 1) {
					sr_msg->message("Force / Torque sensor connection reastablished");
				}
			}

		} while (iw_ret == -1); // dopoki nie zostanie odebrana paczka pomiarow
	} else {
		usleep(1000);
	}
}

/*************************** inicjacja odczytu ******************************/
void ATI3084_force::initiate_reading(void)
{
	if (!is_sensor_configured)
		throw lib::sensor::sensor_error(lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);
}

/***************************** odczyt z czujnika *****************************/
void ATI3084_force::get_reading(void)
{
	lib::Ft_vector kartez_force;

	if (master.force_sensor_test_mode) {
		for (int i = 0; i < 6; ++i) {
			kartez_force[i] = 0.0;
		}
		master.force_msr_upload(kartez_force);
	} else {
		lib::Ft_vector ft_table;

		InterruptLock(&mds.spinlock);

		for (int i = 0; i < 6; i++)
			ft_table[i] = static_cast <double> (mds.data[i + 1]);
		int16_t measure_report = mds.data[0];

		InterruptUnlock(&mds.spinlock);

		// jesli pomiar byl poprawny
		if (measure_report == COMMAND_OK) {
			is_reading_ready = true;

			// jesli ma byc wykorzytstywana biblioteka transformacji sil

			for (int i = 0; i < 3; i++)
				ft_table[i] /= 20;
			//			for(int i=3;i<6;i++) ft_table[i]/=333;
			for (int i = 3; i < 6; i++)
				ft_table[i] /= 1000; // by Y - korekta

			lib::Homog_matrix frame = master.return_current_frame(common::WITH_TRANSLATION);
			// lib::Homog_matrix frame(master.force_current_end_effector_frame);
			lib::Ft_vector output = gravity_transformation->getForce(ft_table, frame);
			master.force_msr_upload(output);
#if 0
			static int ms_nr = 0; // numer odczytu z czujnika
			if (!((ms_nr++)%1000)) {
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
#endif
		}

	}
}

void ATI3084_force::set_output(int16_t value)
{
	int16_t output = 0;
	uint16_t comp = 0x0001;
	uint8_t lower, upper;
	// wersja z pajaczkiem
	// 	const unsigned char output_positions[16]={15,7,14,6,13,5,12,4,0,8,1,9,2,10,3,11};
	// wersja z nowa plytka
	const unsigned char output_positions[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

	for (int i = 0; i < 16; i++) {
		uint16_t mask = 0x0001;
		mask <<= output_positions[i];
		if (value & comp)
			output |= mask;
		comp <<= 1;
	}
	lower = (unsigned char) (output % 256);
	upper = (unsigned char) (output >>= 8);

	out8(base_io_address + LOWER_OUTPUT, lower);
	out8(base_io_address + UPPER_OUTPUT, upper);
}

int16_t get_input(void)
{
	int16_t input = 0, temp_input;
	uint16_t comp = 0x0001;
	// wersja z pajaczkiem
	// 	const unsigned char input_positions[16]={8,10,12,14,7,5,3,1,9,11,13,15,6,4,2,0};
	// wersja z nowa plytka
	const unsigned char input_positions[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

	uint8_t lower = in8(base_io_address + LOWER_INPUT);
	uint8_t upper = in8(base_io_address + UPPER_INPUT);

	temp_input = lower + 256 * upper;

	for (int i = 0; i < 16; i++) {
		uint16_t mask = 0x0001;
		mask <<= input_positions[i];
		if (temp_input & comp)
			input |= mask;
		comp <<= 1;
	}
	return input;
}

void ATI3084_force::set_obf(unsigned char state)
{
	uint8_t temp_register = in8(base_io_address + CONTROL_OUTPUT);

	if (state)
		temp_register |= 0x10;// dla przejsciowki
	else
		temp_register &= 0xef;

	out8(base_io_address + CONTROL_OUTPUT, temp_register);
}

void set_ibf(unsigned char state)
{
	uint8_t temp_register = in8(base_io_address + CONTROL_OUTPUT);

	if (state)
		temp_register |= 0x20;// dla przejsciowki
	else
		temp_register &= 0xdf;

	out8(base_io_address + CONTROL_OUTPUT, temp_register);
}

bool ATI3084_force::check_ack()
{
	uint8_t temp_register = in8(base_io_address + ACK_PORT_INPUT);

	if (temp_register & 0x01)
		return true;
	else
		return false;
}

bool check_stb()
{
	uint8_t temp_register = in8(base_io_address + STB_PORT_INPUT);

	if (temp_register & 0x01)
		return true;
	else
		return false;
}

void ATI3084_force::initiate_registers(void)
{
	out8(base_io_address + PORT_0_CONFIG, 0x03);
	out8(base_io_address + PORT_1_CONFIG, 0x03);
	out8(base_io_address + INTER_CONFIG, 0x10);// przerwanie od !stb, mozna dolaczyc przerwanie od !ack
	delay(100);
	set_obf(1);
	set_ibf(0);
}

bool check_intr(void)
{
	uint8_t temp_register = in8(base_io_address + INTER_CONFIG);

	if (temp_register & 0x80)
		return 1;
	else
		return 0;
}

void ATI3084_force::check_cs(void)
{
	printf("Input int: %d, char: %c,   ", get_input(), get_input());
	if (check_ack())
		printf("ACK HIGH,  ");
	else
		printf("ACK LOW,   ");
	if (check_stb())
		printf("STB HIGH\n");
	else
		printf("STB LOW\n");
}

void ATI3084_force::do_Wait(void)
{
	int iw_ret;

	do {
		TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_INTR, &tim_event, &int_timeout, NULL);
		iw_ret = InterruptWait(0, NULL);
		InterruptLock(&mds.spinlock);
		if (iw_ret != -1) {
			mds.is_received = true;
		}
		InterruptUnlock(&mds.spinlock);
	} while (iw_ret != -1);
}

void ATI3084_force::do_send_command(const char* command)
{
#if SERIAL
	ssize_t data_written = write(uart, command, strlen(command));

	if (data_written != (ssize_t) strlen(command)) {
		perror("ATI3084 serial write to sensor failed");
	}
#endif
#if PARALLEL
	char a;
	struct timespec rqtp;

	rqtp.tv_sec = 0;
	rqtp.tv_nsec = 100000;

	while ((a = *command++) != 0) {
		int16_t value = int16_t(a);
		set_output(value);
		while (!check_ack());
		set_obf(0);
		nanosleep(&rqtp, NULL);

		if (value != 23)
		while (check_ack()); // jesli polcecenie rozne od RESET
		else
		delay(1);
		set_obf(1);
	}
#endif
}

// metoda na wypadek skasowanie pamiecia nvram
// uwaga sterownik czujnika wysyla komunikat po zlaczu szeregowym zaraz po jego wlaczeniu

void ATI3084_force::solve_transducer_controller_failure(void)
{
	tcflush(uart, TCIFLUSH);

	do_send_command(YESCOMM); /* command ^W to FT */

	tcflush(uart, TCIFLUSH);
}

void ATI3084_force::do_init(void)
{
	int_timeout = SCHUNK_INTR_TIMEOUT_HIGH; // by Y

	do_send_command(RESET); /* command ^W to FT */
	delay(20);

	do_send_command(CL_0);
	delay(20);

	do_send_command(CD_B);
	delay(20);

	do_send_command(CD_B);
	delay(20);

	do_send_command(CD_R);
	delay(20);

	do_send_command(CV_6);
	delay(20);

	do_send_command(SA);
	delay(20);

	do_send_command(SM);
	delay(20);

	do_send_command(SB);
	delay(20);

	do_send_command(CP_P);
	do_Wait();

	do_send_command(CL_0);
	do_Wait();

	do_send_command(CD_B);
	do_Wait();

	do_send_command(CD_B);
	do_Wait();

	do_send_command(CD_R);
	do_Wait();

	do_send_command(CV_6);
	do_Wait();

	do_send_command(SA);
	do_Wait();

	do_send_command(SM);
	do_Wait();

	do_send_command(SZ);
	do_Wait();

	do_send_command(SB);
	do_Wait();

#ifdef PARALLEL
	do_Wait();
#endif
}

void clear_intr(void)
{
	uint8_t temp_register = in8(base_io_address + INTER_CONFIG);

	temp_register |= 0x80;

	out8(base_io_address + INTER_CONFIG, temp_register);
}

force* return_created_edp_force_sensor(common::manip_effector &_master)
{
	return new ATI3084_force(_master);
}// : return_created_sensor

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
