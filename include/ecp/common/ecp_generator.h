#if !defined(_ECP_GENERATOR_H)
#define _ECP_GENERATOR_H

#include <map>

#include "lib/srlib.h"
#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_robot.h"

class ecp_generator {
    // Klasa bazowa dla generatorow trajektorii (klasa abstrakcyjna)
    // Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
    // sprawdzania spelnienia warunku 
  protected:
	bool is_robot_active;
	sr_ecp &sr_ecp_msg;    // by Y - obiekt do komunikacji z SR
	ecp_task& ecp_t;
    	
 public:
 
	ecp_robot* the_robot;
    	
	// mapa wszystkich transmiterow
	std::map <TRANSMITTER_ENUM, transmitter*> transmitter_m;
 
	// mapa wszystkich czujnikow
	std::map <SENSOR_ENUM, sensor*> sensor_m;

	ecp_generator(ecp_task& _ecp_task, bool _is_robot_active);
		
	virtual ~ecp_generator();
	
	// generuje pierwszy krok ruchu -
	// pierwszy krok czesto rozni sie od pozostalych,
	// np. do jego generacji nie wykorzystuje sie czujnikow
	// (zadanie realizowane przez klase konkretna)
	virtual bool first_step () = 0;

// generuje kazdy nastepny krok ruchu
// (zadanie realizowane przez klase konkretna)
	virtual bool next_step () = 0;
  
	bool is_EDP_error (ecp_robot& the_robot) const;

	class ECP_error {  // Klasa obslugi bledow generatora
	public:
       uint64_t error_class;
       uint64_t error_no;
       edp_error error;

       ECP_error ( uint64_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
   }; // end: class ECP_error

}; // end: class ecp_generator

#endif /* _ECP_GENERATOR_H */
