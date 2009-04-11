// -------------------------------------------------------------------------
//                            vsp_sensor.h		dla QNX6.2
// 
// Definicje klasy vsp_vis_sensor i vsp_error
// 
// Ostatnia modyfikacja: 25.06.2003
// Autor: tkornuta
// -------------------------------------------------------------------------

#if !defined(_VSP_PAWEL_H)
#define _VSP_PAWEL_H

#include "vsp/vsp_sensor.h"
#include "common/sensor.h"

namespace mrrocpp {
namespace vsp {
namespace sensor {

#define VSP_MSG_SEND_TIMEOUT_HIGH 1000000
#define XMAX 768
#define YMAX 574
#define ZMAX 500

#define R_MASK 0xF800
#define G_MASK 0x07E0
#define B_MASK 0x001F
#define MINSIZE 40

#define max(x,y) ( x>y ? x : y )
#define min(x,y) ( x<y ? x : y )
#define max3(x,y,z) ( max(max(x,y),z) )
#define min3(x,y,z) ( min(min(x,y),z) )
#define bw(x,a,b) ( x>=a && x <=b )

typedef struct {
	float x,y,z;
} VECTOR;

typedef struct{
		unsigned short r,g,b;
} Tpix;

typedef struct{
		float h,s,v;
} Thsv;

/********** klasa czujnikow po stronie VSP **************/
class pawel: public base {

private:
	short zero;							// polozenie zerowe
	unsigned int ms_nr; // numer odczytu z czujnika
	SENSOR_IMAGE last;
	VECTOR a,v;	// przyspieszenie i predkosc
	VECTOR ball;
	struct timespec start[9];
	int fd;
	int size_read;
	unsigned short buffer[600000];
	Tpix pix;
	Thsv hsv;
	unsigned short kol[XMAX][YMAX];
	unsigned short col[XMAX];
	unsigned short row[YMAX];
	
public:
	short ERROR_CODE;
	
	pawel(configurator &_config);
	~pawel(void);

	void configure_sensor (void);	// konfiguracja czujnika
	void wait_for_event(void);		// oczekiwanie na zdarzenie
	void initiate_reading (void);		// zadanie odczytu od VSP
	void get_reading (void);			// odebranie odczytu od VSP		// zwraca blad
	void terminate (void);				// rozkaz zakonczenia procesu VSP
	
}; 

} // namespace sensor
} // namespace vsp
} // namespace mrrocpp

#endif
