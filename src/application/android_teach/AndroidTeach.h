/*
 * AndroidTeach.h
 *
 *  Created on: Nov 20, 2011
 *      Author: hh7
 */

#ifndef ANDROIDTEACH_H_
#define ANDROIDTEACH_H_

#include "base/ecp_mp/ecp_mp_task.h"
//#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"
#include "application/android_teach/generator/EcpGAndroidJoint.h"
#include "application/android_teach/sensor/EcpMpAndroid.h"
#include "application/android_teach/generator/EcpSmoothGAndroid.h"
#include "application/android_teach/AndroidState.h"
#include "application/android_teach/Enums.h"
#include "application/android_teach/NetworkException.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {


class AndroidTeach: public common::task::task
{

protected:
//Generatory ruchu
	irp6ot_m::generator::EcpSmoothGAndroid* sg;
//	common::generator::newsmooth* sg;
	irp6ot_m::generator::EcpGAndroidJoint* jg;
    ecp_mp::sensor::android_teach::AndroidState androidState;

    common::generator::get_position* gg;


////Przyciski
//    ecp_mp::sensor::wiimote_t lastButtons;
//    ecp_mp::sensor::wiimote_t buttonsPressed;

//Plik z trajektoria
    char path[80];
    char filename[40];

    std::vector <double> coordinates;

    lib::Homog_matrix homog_matrix;

//Numeracja wezlow
int cnt;

//Numer aktualnego generatora
int gen;

//Czy wybrano plik trajektorii
bool has_filenames;

//Aktualny generator
common::generator::generator* g;

//Bufor na komunikaty konsolowe
char buffer[200];

////Struktura komunikacyjna z Wii-mote
//ecp_mp::sensor::wii_command_t message;

lib::ECP_POSE_SPECIFICATION pose_spec;
int axis_num;

std::string velocity;
std::string acceleration;
std::string type;
std::string mode;

    class n;
    class n
    {
        public:
            n* next;
            n* prev;
            int id;
	std::vector<double> position;

            n() : next(NULL), prev(NULL) {}

    };

    typedef n node;

    struct
    {
        node* head;
        node* tail;
        int count;
        node* current;
        int position;
    } trajectory;

    void updateButtonsPressed();

public:
	/**
	 * Tworzy obiekt zadania
	 * @param _config konfigurator
	 * @author OL
	 */

	AndroidTeach(lib::configurator &_config);
	virtual ~AndroidTeach();

/**
 * Realizuje zadanie
 * @author OL
 */
	void main_task_algorithm(void);

    void print_trajectory(void);

    //Odtwarzanie trajektorii
	void move_to_current(void);
	void move_to_next(void);
	void move_to_prev(void);
	void move_to_first(void);
	void move_to_last(void);

	void move_to_position(double position[NUMBER_OF_JOINTS]);

//	//Obsluga przyciskow
//	void handle12();
//	void handleA();
//	void handleB();
	void handlePlus();
	void handleMinus();
//	void handleHome();

    bool get_filenames(void);

    int load_trajectory(void);

    void save_trajectory(void);

};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ANDROIDTEACH_H_ */
