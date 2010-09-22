#ifndef MP_GEN_COMMON_H_
#define MP_GEN_COMMON_H_

/*!
 * @file
 * @brief File contains mp common generators declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include "base/mp/generator/mp_generator.h"

// generator for setting the next ecps state

namespace mrrocpp {
namespace mp {
namespace generator {

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

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif /*MP_GENERATORS_H_*/
