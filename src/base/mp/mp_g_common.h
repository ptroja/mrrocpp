#ifndef MP_GEN_COMMON_H_
#define MP_GEN_COMMON_H_

#include "base/mp/mp_generator.h"
#include "base/mp/mp_task.h"
#include "base/mp/mp.h"
#include "base/mp/MP_main_error.h"

// generator for setting the next ecps state

namespace mrrocpp {
namespace mp {
namespace generator {

class set_next_ecps_state : public generator
{
protected:
	lib::ecp_next_state_t ecp_next_state;

public:
	set_next_ecps_state(task::task& _mp_task);

	void
			configure(std::string l_mp_2_ecp_next_state, int l_mp_2_ecp_next_state_variant, const char* l_mp_2_ecp_next_state_string, int str_len);
	void configure(const lib::playerpos_goal_t &_goal);

	bool first_step();
	bool next_step();
};

// generator for sending end_motion mesage to ecps

class send_end_motion_to_ecps : public generator
{
public:

	// konstruktor
	send_end_motion_to_ecps(task::task& _mp_task);

	bool first_step();
	bool next_step();
};

// ####################################################################################################
// Rozszerzony Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ####################################################################################################

class extended_empty : public generator
{
	// Klasa dla generatorow trajektorii
	// Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
	// sprawdzania spelnienia warunku koncowego
protected:
	bool activate_trigger;

public:
	extended_empty(task::task& _mp_task);

	void configure(bool l_activate_trigger);

	bool first_step();
	bool next_step();
};

// ####################################################################################################
// Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ####################################################################################################

class empty : public generator
{
	// Klasa dla generatorow trajektorii
	// Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
	// sprawdzania spelnienia warunku koncowego
public:
	empty(task::task& _mp_task);

	virtual bool first_step();
	// generuje pierwszy krok ruchu -
	// pierwszy krok czesto rozni sie od pozostalych,
	// np. do jego generacji nie wykorzystuje sie czujnikow
	// (zadanie realizowane przez klase konkretna)
	virtual bool next_step();
	// generuje kazdy nastepny krok ruchu
	// (zadanie realizowane przez klase konkretna)
};

// ####################################################################################################
// KLASA BAZOWA dla generatorow o zadany przyrost polozenia/orientacji
// ####################################################################################################

class delta : public generator
{
public:
	delta(task::task& _mp_task);
	lib::trajectory_description irp6ot_td;
	lib::trajectory_description irp6p_td;
};

// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################

class tight_coop : public delta
{
public:
			tight_coop(task::task& _mp_task, lib::trajectory_description irp6ot_tr_des, lib::trajectory_description irp6p_tr_des);

	virtual bool first_step();

	virtual bool next_step();
};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
