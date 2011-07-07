#ifndef MP_H_
#define MP_H_

#include "ui.h"



namespace mrrocpp {
namespace ui {
namespace common {
class UiRobot;
class Interface;


class Mp
{
public:
	Mp(Interface *iface);

	int MPup();
	int MPup_int();
	int MPslay();
	// MP pulse
	int pulse_start_mp();
	int pulse_stop_mp();
	int pulse_pause_mp();
	int pulse_resume_mp();
	int pulse_trigger_mp();

	int execute_mp_pulse(char pulse_code);
	void set_mp_state();
	void manage_interface();

	mp_state_def mp_state;
private:
	Interface *interface;
};

}
}
}

#endif /* MP_H_ */
