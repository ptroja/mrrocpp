#ifndef MP_GENERATORS_H_
#define MP_GENERATORS_H_

#include "mp/mp_generator.h"
#include "mp/mp_task.h"

// generator for setting the next ecps state

namespace mrrocpp {
namespace mp {
namespace generator {

class set_next_ecps_state : public base
{
protected:
	ecp_next_state_t ecp_next_state;

public:

    set_next_ecps_state(task::base& _mp_task);

	void configure (int l_mp_2_ecp_next_state, int l_mp_2_ecp_next_state_variant, const char* l_mp_2_ecp_next_state_string);
	void configure (const playerpos_goal_t &_goal);

	bool first_step ();
	bool next_step ();

};

// generator for sending end_motion mesage to ecps

class send_end_motion_to_ecps : public base
{
public:

    // konstruktor
    send_end_motion_to_ecps(task::base& _mp_task);

	bool first_step ();
	bool next_step ();

};

// ####################################################################################################
// Rozszerzony Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ####################################################################################################

class extended_empty : public base {
	// Klasa dla generatorow trajektorii
	// Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
	// sprawdzania spelnienia warunku koncowego
 protected:
	bool activate_trigger;

 public:
	extended_empty(task::base& _mp_task);

	~extended_empty(){ };

	void configure (bool l_activate_trigger);

	bool first_step ();
	bool next_step ();

};

// ####################################################################################################
// Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ####################################################################################################

class empty : public base {
    // Klasa dla generatorow trajektorii
    // Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
    // sprawdzania spelnienia warunku koncowego
 public:
	empty(task::base& _mp_task);

  ~empty(){ };

  virtual bool first_step ();
      // generuje pierwszy krok ruchu -
      // pierwszy krok czesto rozni sie od pozostalych,
      // np. do jego generacji nie wykorzystuje sie czujnikow
      // (zadanie realizowane przez klase konkretna)
  virtual bool next_step ();
     // generuje kazdy nastepny krok ruchu
     // (zadanie realizowane przez klase konkretna)

};

// ####################################################################################################
// KLASA BAZOWA dla generatorow o zadany przyrost polozenia/orientacji
// ####################################################################################################

class delta : public base
{
protected:


public:
	delta(task::base& _mp_task);
	trajectory_description irp6ot_td;
	trajectory_description irp6p_td;
};

// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################

class tight_coop : public delta
{

public:
	tight_coop(task::base& _mp_task, trajectory_description irp6ot_tr_des, trajectory_description irp6p_tr_des);

  ~tight_coop();

  virtual bool first_step ();

  virtual bool next_step ();

};

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
