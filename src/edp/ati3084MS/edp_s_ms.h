// -------------------------------------------------------------------------
// Definicje klasy edp_ATI3084_force_sensor
// z interfejsem Macieja Kuleszy
//
// Ostatnia modyfikacja: 2005
// Autor: yoyek
// -------------------------------------------------------------------------

#ifndef _EDP_S_ATI3084MS_H
#define _EDP_S_ATI3084MS_H

#include <boost/scoped_ptr.hpp>

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>

#include <kiper/clients/AsioUdpClient.hpp>
#include <kiper/clients/RawEthernetClient.hpp>

#include "edp/common/edp_irp6s_postument_track.h"
#include "ati3084.pb.h"

namespace mrrocpp {
namespace edp {
namespace sensor {


typedef struct {
int16_t ft[6];
} forceReadings;

#define SPEED B115200
#define PORT "/dev/ser1"

/********** klasa czujnikow po stronie VSP **************/
class ATI3084_force : public force{
typedef boost::scoped_ptr<google::protobuf::Closure> ClosurePtr;
private:
	kiper::clients::AsioUdpClient rpcClient_;
	//kiper::clients::RawEthernetClient rpcClient_;
	Ati3084_Stub sensor_;
	int uart, i,r;
	//int licz=0;
	forceReadings ftxyz;
	static const char* SENSOR_BOARD_HOST;
	static const unsigned int SENSOR_BOARD_PORT = 55555;

	static const char* SENSOR_DEVICE_NAME;
	static uint8_t SENSOR_BOARD_MAC[6];

	unsigned int ms_nr;// numer odczytu z czujnika
	struct timespec start[9];
	const bool FORCE_TEST_MODE;

	kiper::ClientRpcController sendBiasController;
	kiper::ClientRpcController getForceReadingController;

	ClosurePtr sendBiasClosure_;
	ClosurePtr getForceReadingClosure_;

	SendBiasReply sendBiasResponse;
	GenForceReading genForceReading;

	void solve_transducer_controller_failure(void);
	short do_init(void);
	int open_port(void);
	forceReadings getFT(int fd);
	void sendBias();
	bool sensorWorks();

	void handleSendBiasReply(kiper::ClientRpcController& controller);
	void handleGetGenForceReading(kiper::ClientRpcController& controller);

	void onReaderStarted();
	void onReaderStopped();
public:
	void connect_to_hardware (void);
	ATI3084_force(common::irp6s_postument_track_effector &_master);
	virtual ~ATI3084_force();
	void configure_sensor (void);	// konfiguracja czujnika
	void wait_for_event(void);		// oczekiwanie na zdarzenie
	void initiate_reading (void);		// zadanie odczytu od VSP
	void get_reading (void);			// odebranie odczytu od VSP		// zwraca blad
	void terminate (void);				// rozkaz zakonczenia procesu VSP

}; // end: class vsp_sensor


} // namespace sensor
} // namespace edp
} // namespace mrrocpp


#endif
