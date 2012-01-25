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
	Mp(Interface & iface);

	void MPup();
	int MPup_int();
	void MPslay();
	// MP pulse
	void pulse_start_mp();
	void pulse_stop_mp();
	void pulse_pause_mp();
	void pulse_resume_mp();
	void pulse_trigger_mp();

	void execute_mp_pulse(char pulse_code);
	void set_mp_state();
	void manage_interface();

	mp_state_def mp_state;
private:
	Interface & interface;
};

}
}
}

#endif /* MP_H_ */
