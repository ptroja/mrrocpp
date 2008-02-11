#include "mp/mp_generator.h"
#include "mp/mp_robot.h"

#include <map>

mp_generator::~mp_generator(void) { };

void mp_generator::re_run(void) // powrot do stanu wyjsciowego
{
	phase = BEFORE_FIRST_STEP;
	new_pulse_checked = true;
}

// kopiuje dane z robotow do generatora
void mp_generator::copy_data(std::map <ROBOT_ENUM, mp_robot*>& _robot_m) {
	for (std::map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = _robot_m.begin();
	        robot_m_iterator != _robot_m.end(); robot_m_iterator++) {
		robot_m_iterator->second->get_reply(); // odpowiedz z ECP
	}
}

// kopiuje polecenie stworzone w generatorze do robotow
void mp_generator::copy_generator_command (std::map <ROBOT_ENUM, mp_robot*>& _robot_m) {
	for (std::map <ROBOT_ENUM, mp_robot*>::iterator robot_m_iterator = _robot_m.begin();
	        robot_m_iterator != _robot_m.end(); robot_m_iterator++) {
		robot_m_iterator->second->create_command(); // rozkaz dla ECP
	}
}

mp_generator::MP_error::MP_error (uint64_t err0, uint64_t err1)
 : error_class(err0), mp_error(err1)
{}

mp_generator::mp_generator(mp_task& _mp_task) 
	: ecp_mp_generator (*_mp_task.sr_ecp_msg),
	wait_for_ECP_pulse(false),
	trigger(false),
	phase(BEFORE_FIRST_STEP),
	new_pulse_checked(true)
{
}