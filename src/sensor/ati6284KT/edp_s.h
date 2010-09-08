// -------------------------------------------------------------------------
//
//
// Definicje klasy edp_ATI6284_force_sensor
//
// Ostatnia modyfikacja: styczen 2010
// Autor: labi (Kamil Tarkowski)
// Autor: Yoyek (Tomek Winiarski)
// -------------------------------------------------------------------------


#if !defined(_EDP_S_ATI6284_MS_H)
#define _EDP_S_ATI6284_MS_H


#include "sensor/ati6284KT/TimeUtil.h"
#include "sensor/ati6284KT/RawSocket.h"
#include "sensor/ati6284KT/ForceSensor.h"

#include "base/edp/edp_force_sensor.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

//250us
#define ETHERNET_FRAME_TIMEOUT 250000

static const uint8_t TARGET_ETHERNET_ADDRESS[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }; //board mac address


/* macierz konwersji napiecia z czujnika na sile i momenty sil */
double conversion_scale[6] =
		{ 4.5511972116989, 4.5511972116989, 1.41244051397552, 84.8843245576086, 84.8843245576086, 80.9472037525247 };

double
		conversion_matrix[6][6] =
				{ { -0.40709, -0.27318, 0.34868, -33.58156, -0.32609, 33.54162 }, { 0.35472, 38.22730, -0.41173, -19.49156, 0.49550, -19.15271 }, { 18.72635, -0.59676, 19.27843, -0.56931, 18.69352, -0.67633 }, { -0.40836, -0.95908, -33.37957, 1.38537, 32.52522, -0.51156 }, { 37.13715, -1.02875, -20.00474, -0.27959, -19.34135, 1.42577 }, { -0.15775, -18.16831, -0.00133, -18.78961, 0.31895, -18.38586 } };

/********** klasa czujnikow po stronie VSP **************/
class ATI6284_force : public force
{

private:
	RawSocket socket_;
	ForceSensor6284 sensor_;
	uint64_t frame_counter;
	unsigned char recvBuffer[512];
	int16_t adc_data[6];
	int16_t bias_data[6];

	mrrocpp::edp::sensor::TimeUtil timeUtil;
	bool measuring;

public:

	void connect_to_hardware(void);

	ATI6284_force(common::manip_effector &_master);
	virtual ~ATI6284_force();

	void configure_sensor(void); // konfiguracja czujnika
	void wait_for_event(void); // oczekiwanie na zdarzenie
	void initiate_reading(void); // zadanie odczytu od VSP
	void get_reading(void); // odebranie odczytu od VSP		// zwraca blad
	void startMeasurements();
	void stopMeasurements();
}; // end: class vsp_sensor

void send_request(uint64_t &counter);
void convert_data(int16_t result_raw[6], int16_t bias_raw[6], double force[6]);
int get_data_from_ethernet(unsigned char buffer[512], int16_t data_raw[6]);

} // namespace sensor
} // namespace edp
} // namespace mrrocpp


#endif
