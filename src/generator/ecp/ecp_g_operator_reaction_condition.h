#if !defined(_ECP_OPERATOR_REACTION_CONDITION_H)
#define  _ECP_OPERATOR_REACTION_CONDITION_H

#include <list>

#include "base/ecp/ecp_generator.h"
#include "generator/ecp/teach_in/ecp_taught_in_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class operator_reaction_condition : public common::generator::generator
{
	// Klasa konkretnych warunkow poczatkowych dla robota irp6_on_track
	// Spelnienie warunku poczatkowego konczy wykonanie instrukcji MOVE.
	// Stanowi ono warunek poczatkowy, ktorego spelnienie umozliwia
	// rozpoczecie wykonania nastepnej instrukcji MOVE.
protected:
	std::list <ecp_taught_in_pose> pose_list;
	std::list <ecp_taught_in_pose>::iterator pose_list_iterator;

public:

	lib::fd_client_t UI_fd;

	// konstruktor
	operator_reaction_condition(task::task& _ecp_task);

	// destruktor
	virtual ~operator_reaction_condition(void);

	void flush_supplementary_list(void); // end: flush_supplementary_list

	void initiate_supplementary_list(void);

	void next_supplementary_list_ptr(void);

	void get_supplementary(ecp_taught_in_pose& tip);

	void
	set_supplementary(lib::ECP_POSE_SPECIFICATION ps, double motion_time, const double coordinates[lib::MAX_SERVOS_NR], int extra_info =
			0);

	void
	create_supplementary_list_head(lib::ECP_POSE_SPECIFICATION ps, double motion_time, const double coordinates[lib::MAX_SERVOS_NR], int extra_info =
			0);

	void
	insert_supplementary_list_element(lib::ECP_POSE_SPECIFICATION ps, double motion_time, const double coordinates[lib::MAX_SERVOS_NR], int extra_info =
			0);

	bool is_supplementary_list_element(void);

	bool is_supplementary_list_head(void);

	int supplementary_list_length(void);

	virtual bool first_step();
	virtual bool next_step();
	// bada wartosc warunku poczatkowego
	// true - konczy czekanie (funkcja wait)
	// false - kontynuuje oczekiwanie
};
// end: class irp6ot_operator_reaction_condition

}// namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_OPERATOR_REACTION_CONDITION_H */
