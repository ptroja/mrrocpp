#if !defined(_ECP_TEACH_GENERATOR_H)
#define  _ECP_TEACH_GENERATOR_H

// ####################################################################################################
// ###############     KLASA glowna dla odtwarzania listy pozycji     #################################
// ####################################################################################################

#include <list>

#include "base/ecp/ecp_generator.h"
#include "base/ecp/ecp_taught_in_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/** @addtogroup teach_in_dll
 *
 *  @{
 */

class teach_tmp : public common::generator::generator {

protected:
    std::list<ecp_taught_in_pose> pose_list;
    std::list<ecp_taught_in_pose>::iterator pose_list_iterator;

public:
  // -------------------------------------------------------
  // konstruktor
	teach_tmp (common::task::task& _ecp_task);

  // -------------------------------------------------------
  // destruktor
  virtual ~teach_tmp (void);

  	// Uczenie robota
	void teach (lib::ECP_POSE_SPECIFICATION ps, const char* msg);

	// --------------------------------------------------------------------------
	// Wczytanie trajektorii z pliku
	bool load_file_from_ui ();

	// --------------------------------------------------------------------------
	// Wczytanie trajektorii z pliku
	bool load_file_with_path (const char* file_name);

	// --------------------------------------------------------------------------
	// Zapis trajektorii do pliku
	void save_file (lib::ECP_POSE_SPECIFICATION ps);
	// --------------------------------------------------------------------------


  // -------------------------------------------------------
  void flush_pose_list ( void );
  // -------------------------------------------------------
  void initiate_pose_list(void);
  // -------------------------------------------------------
  void next_pose_list_ptr (void);
  // -------------------------------------------------------
  void get_pose (ecp_taught_in_pose& tip);
  // -------------------------------------------------------
  // Pobierz nastepna pozycje z listy
  void get_next_pose (double next_pose[lib::MAX_SERVOS_NR]);
  // -------------------------------------------------------
  void set_pose (lib::ECP_POSE_SPECIFICATION ps, double motion_time, double coordinates[lib::MAX_SERVOS_NR], int extra_info = 0);
  // -------------------------------------------------------
  bool is_pose_list_element ( void ) ;
  // -------------------------------------------------------
  bool is_last_list_element ( void );
  // -------------------------------------------------------

  void create_pose_list_head (lib::ECP_POSE_SPECIFICATION ps, double motion_time, const double coordinates[lib::MAX_SERVOS_NR], int extra_info = 0);

  void insert_pose_list_element (lib::ECP_POSE_SPECIFICATION ps, double motion_time, const double coordinates[lib::MAX_SERVOS_NR], int extra_info = 0);

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

  lib::ECP_TO_UI_COMMAND convert(lib::ECP_POSE_SPECIFICATION ps) const;

};

typedef teach_tmp* create_t(common::task::task& _ecp_task);
typedef void destroy_t(teach_tmp*);

/** @} */ // end of teach_in_dll

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_TEACH_IN_GENERATOR_H */
