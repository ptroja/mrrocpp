#ifndef MP_GENERATORS_H_
#define MP_GENERATORS_H_

#include "mp/mp_generator.h"
#include "mp/mp_task.h"

// generator for setting the next ecps state

class mp_set_next_ecps_state_generator : public mp_generator
{
protected:
	int mp_2_ecp_next_state, mp_2_ecp_next_state_variant;
	char mp_2_ecp_next_state_string[MP_2_ECP_STRING_SIZE]; // skojarzone z NEXT_STATE

public:

    mp_set_next_ecps_state_generator(mp_task& _mp_task);

	void configure (int l_mp_2_ecp_next_state, int l_mp_2_ecp_next_state_variant, char* l_mp_2_ecp_next_state_string);

	bool first_step ();
	bool next_step ();

};

// generator for sending end_motion mesage to ecps

class mp_send_end_motion_to_ecps_generator : public mp_generator
{
public:

    // konstruktor
    mp_send_end_motion_to_ecps_generator(mp_task& _mp_task);

	bool first_step ();
	bool next_step ();

};

// ####################################################################################################
// Rozszerzony Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ####################################################################################################

class mp_extended_empty_generator : public mp_generator {
	// Klasa dla generatorow trajektorii
	// Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
	// sprawdzania spelnienia warunku koncowego
 protected:
	bool activate_trigger;

 public:
	mp_extended_empty_generator(mp_task& _mp_task);

	~mp_extended_empty_generator(){ };

	void configure (bool l_activate_trigger);

	bool first_step ();
	bool next_step ();

};

// ####################################################################################################
// Generator pusty. Faktyczna generacja trajektorii odbywa sie w ECP
// ####################################################################################################

class mp_empty_generator : public mp_generator {
    // Klasa dla generatorow trajektorii
    // Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
    // sprawdzania spelnienia warunku koncowego
 public:
	mp_empty_generator(mp_task& _mp_task);

  ~mp_empty_generator(){ };

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

class mp_delta_generator : public mp_generator
{
protected:


public:
	mp_delta_generator(mp_task& _mp_task);
	trajectory_description irp6ot_td;
	trajectory_description irp6p_td;
};

// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################

class mp_tight_coop_generator : public mp_delta_generator
{

public:
	mp_tight_coop_generator(mp_task& _mp_task, trajectory_description irp6ot_tr_des, trajectory_description irp6p_tr_des);

  ~mp_tight_coop_generator();

  virtual bool first_step ();

  virtual bool next_step ();

};

#endif /*MP_GENERATORS_H_*/
