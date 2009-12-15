#if !defined(__MP_GEN_VIS_H)
#define __MP_GEN_VIS_H

namespace mrrocpp {
namespace mp {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class seven_eye : public generator
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)


  robot::robot *irp6ot, *irp6p;
    lib::sensor *vsp_force_irp6ot, *vsp_force_irp6p, *vsp_vis_sac;

    // do konfiguracji pracy generatora
    unsigned short irp6ot_con, irp6p_con;


     lib::trajectory_description td;

public:
       int step_no;
       double delta[6];

    // konstruktor
    seven_eye(task::task& _mp_task, int step=0);

	virtual bool first_step ();
	virtual bool next_step ();

}; // end : class nose_run_force
} // namespace common
} // namespace mp
} // namespace mrrocpp
#endif
