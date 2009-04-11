#if !defined(_ECP_OPERATOR_REACTION_CONDITION_H)
#define  _ECP_OPERATOR_REACTION_CONDITION_H

#include <list>

#include "ecp/common/ecp_generator.h"
#include "ecp/common/ecp_taught_in_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {


class ecp_operator_reaction_condition: public ecp_generator {
   // Klasa konkretnych warunkow poczatkowych dla robota irp6_on_track
   // Spelnienie warunku poczatkowego konczy wykonanie instrukcji MOVE.
   // Stanowi ono warunek poczatkowy, ktorego spelnienie umozliwia
   // rozpoczecie wykonania nastepnej instrukcji MOVE.
protected:
    std::list<ecp_taught_in_pose> pose_list;
    std::list<ecp_taught_in_pose>::iterator pose_list_iterator;

public:
#if !defined(USE_MESSIP_SRR)
	int UI_fd;
#else
	messip_channel_t *UI_fd;
#endif
  // konstruktor
	ecp_operator_reaction_condition(ecp_task& _ecp_task);

  // destruktor
  virtual ~ecp_operator_reaction_condition (void);

  void flush_supplementary_list ( void ); // end: flush_supplementary_list

  void initiate_supplementary_list(void) ;

  void next_supplementary_list_ptr (void);

  void get_supplementary (ecp_taught_in_pose& tip);

  void set_supplementary (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]) ;

  void create_supplementary_list_head (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]);

  void insert_supplementary_list_element (POSE_SPECIFICATION ps, double motion_time, double coordinates[MAX_SERVOS_NR]);

  bool is_supplementary_list_element ( void );

  bool is_supplementary_list_head ( void ) ;

  int supplementary_list_length(void);

  virtual bool first_step ();
  virtual bool next_step ();
     // bada wartosc warunku poczatkowego
     // true - konczy czekanie (funkcja wait)
     // false - kontynuuje oczekiwanie
}; // end: class irp6ot_operator_reaction_condition

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_OPERATOR_REACTION_CONDITION_H */
