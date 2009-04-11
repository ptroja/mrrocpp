#if !defined(_ECP_TEACH_IN_GENERATOR_H)
#define  _ECP_TEACH_IN_GENERATOR_H

// ####################################################################################################
// ###############     KLASA glowna dla odtwarzania listy pozycji     #################################
// ####################################################################################################

#include <list>

#include "ecp/common/ecp_generator.h"
#include "ecp/common/ecp_taught_in_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class ecp_teach_in_generator : public ecp_generator {

protected:
    std::list<ecp_taught_in_pose> pose_list;
    std::list<ecp_taught_in_pose>::iterator pose_list_iterator;

public:
  // -------------------------------------------------------
  // konstruktor
  ecp_teach_in_generator (ecp_task& _ecp_task);
  
  // -------------------------------------------------------
  // destruktor
  virtual ~ecp_teach_in_generator (void);
  
  	// Uczenie robota
	void teach (POSE_SPECIFICATION ps, const char* msg);
	
	// --------------------------------------------------------------------------
	// Wczytanie trajektorii z pliku
	bool load_file_from_ui ();
		
	// --------------------------------------------------------------------------
	// Wczytanie trajektorii z pliku
	bool load_file_with_path (char* file_name);
	
	// --------------------------------------------------------------------------
	// Zapis trajektorii do pliku
	void save_file (POSE_SPECIFICATION ps);
	// --------------------------------------------------------------------------
  
  
  // -------------------------------------------------------
  void flush_pose_list ( void ); // end: flush_pose_list
  // -------------------------------------------------------
  void initiate_pose_list(void);
  // -------------------------------------------------------
  void next_pose_list_ptr (void);
  // -------------------------------------------------------
  void get_pose (ecp_taught_in_pose& tip);
  // -------------------------------------------------------
  // Pobierz nastepna pozycje z listy
  void get_next_pose (double next_pose[MAX_SERVOS_NR]);
  // -------------------------------------------------------
  void set_pose (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]);
  // -------------------------------------------------------
  bool is_pose_list_element ( void ) ;
  // -------------------------------------------------------
  bool is_last_list_element ( void );
  // -------------------------------------------------------
  
   void create_pose_list_head (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]);
  
  // by Y
  
   void create_pose_list_head (POSE_SPECIFICATION ps, double motion_time, int extra_info, double coordinates[MAX_SERVOS_NR]);

  void insert_pose_list_element (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]);
  
  // by Y
  
  void insert_pose_list_element (POSE_SPECIFICATION ps, double motion_time, int extra_info, double coordinates[MAX_SERVOS_NR]);
  
  // -------------------------------------------------------
  int pose_list_length(void);
  // -------------------------------------------------------
  virtual bool first_step ();
      // generuje pierwszy krok ruchu -
      // pierwszy krok czesto rozni sie od pozostalych,
      // np. do jego generacji nie wykorzystuje sie czujnikow
      // (zadanie realizowane przez klase konkretna)
  virtual bool next_step ();
     // generuje kazdy nastepny krok ruchu
     // (zadanie realizowane przez klase konkretna)
  
  ECP_TO_UI_COMMAND convert(POSE_SPECIFICATION ps) const;

};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TEACH_IN_GENERATOR_H */
