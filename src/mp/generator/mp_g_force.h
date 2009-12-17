// -------------------------------------------------------------------------
//
// Definicje struktur danych i metod dla procesow MP - generatory silowe
//
// -------------------------------------------------------------------------

#if !defined(__MP_GEN_FORCE_H)
#define __MP_GEN_FORCE_H

#include "lib/mathtr.h"

#include "mp/generator/mp_g_teach_in.h"

namespace mrrocpp {
namespace mp {
namespace generator {

// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class tff_single_robot_nose_run : public generator
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)


  robot::robot *irp6;
    lib::sensor *vsp_force;

     lib::trajectory_description td;

public:
       int step_no;
       double delta[6];

    // konstruktor
    tff_single_robot_nose_run(task::task& _mp_task, int step=0);

	virtual bool first_step ();
	virtual bool next_step ();

}; // end : class nose_run_force




// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class tff_nose_run : public generator
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)


    robot::robot *irp6ot, *irp6p;
    lib::sensor *vsp_force_irp6ot, *vsp_force_irp6p;

    // do konfiguracji pracy generatora
    unsigned short irp6ot_con, irp6p_con;


     lib::trajectory_description td;

public:
       int step_no;
       double delta[6];

    // konstruktor
    tff_nose_run(task::task& _mp_task, int step=0);

	void configure (unsigned short l_irp6ot_con , unsigned short l_irp6p_con );


	virtual bool first_step ();
	virtual bool next_step ();

}; // end : class nose_run_force




// --------------------------------------------------------------------------
// Generator do lapania kostki

class tff_rubik_grab : public generator
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)


    robot::robot *irp6ot, *irp6p;
    lib::sensor *vsp_force_irp6ot, *vsp_force_irp6p;

     lib::trajectory_description td;

    // do konfiguracji pracy generatora
    unsigned short irp6ot_con, irp6p_con;
    double goal_position, position_increment;
    int min_node_counter;
    bool irp6p_both_axes_running, irp6ot_both_axes_running;


public:
       int step_no;
       double delta[6];

    // konstruktor
    tff_rubik_grab(task::task& _mp_task, int step=0);

	void configure(unsigned short l_irp6ot_con, unsigned short l_irp6p_con, double l_goal_position,
		double l_position_increment, int l_min_node_counter, bool l_irp6p_both_axes_running = true, bool l_irp6ot_both_axes_running = true);

	virtual bool first_step ();
	virtual bool next_step ();

}; // end : class nose_run_force


// --------------------------------------------------------------------------
// Generator do obracania sciany kostki

class tff_rubik_face_rotate : public generator
{
protected:
	int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)

     lib::trajectory_description td;
    robot::robot *irp6ot, *irp6p;
    lib::sensor *vsp_force_irp6ot, *vsp_force_irp6p;
    // do konfiguracji pracy generatora
    double irp6ot_con, irp6p_con;
    double irp6ot_stored_gamma, irp6p_stored_gamma;
    bool irp6ot_range_change, irp6p_range_change;
public:
       int step_no;
       double delta[6];

    // konstruktor
    tff_rubik_face_rotate(task::task& _mp_task, int step=0);

	void configure(double l_irp6ot_con, double l_irp6p_con);

	virtual bool first_step ();
	virtual bool next_step ();

}; // end : class nose_run_force



// --------------------------------------------------------------------------
// Generator do nasuniecia chwytaka na kostke

class tff_gripper_approach : public generator
{
protected:
	int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)

     lib::trajectory_description td;
    robot::robot *irp6ot, *irp6p;
    lib::sensor *vsp_force_irp6ot, *vsp_force_irp6p;
    // do konfiguracji pracy generatora
    double irp6ot_speed, irp6p_speed;
	int motion_time;

public:
       int step_no;
       double delta[6];

    // konstruktor
    tff_gripper_approach(task::task& _mp_task, int step=0);

	void configure(double l_irp6ot_speed, double l_irp6p_speed, int l_motion_time);

	virtual bool first_step ();
	virtual bool next_step ();

}; // end : class nose_run_force





// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos

class nose_run_force : public generator
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)


      robot::robot *irp6ot, *irp6p, *conv;
    lib::sensor *vsp_force_irp6ot, *vsp_force_irp6p;

public:
     lib::trajectory_description td;
       int step_no;
       double delta[6];

    // konstruktor
    nose_run_force(task::task& _mp_task, int step=0);

   virtual bool first_step ();
   virtual bool next_step ();

}; // end : class nose_run_force

#define SILA_DOCISKUEDP_OPADANIE_EGG 15
#define PROG_ODLEGLOSCI_PODCZERWIEN_EGG 115
#define ODLEGLOSCI_PODCZERWIEN_MIN_VALUE 70
// #define POS_Z_AXIS_MACROSTEP_INC 0.0002
// #define POS_Z_AXIS_MACROSTEP_INC 0.0005
#define POS_Z_AXIS_MACROSTEP_INC 0.001
// #define POS_Z_AXIS_MACROSTEP_INC 0.0015
// #define POS_Z_AXIS_MACROSTEP_INC 0.002
// #define POS_Z_AXIS_MACROSTEP_INC 0.004
#define MOMENT_SILY_KONTAKTU_EGG -70
#define SILA_KONTAKTU_EGG 30
// #define MOMENT_SILY_KONTAKTU_EGG -200
#define INIT_ITER_NUMBER 20
#define IMPACT_ITERATIONS_NUMBER 20


    #define SILA_DOCISKU 0.00002 // by Y
    #define MIN_SILA_DOCISKUEDP 20
    #define SILA_DOCISKUEDP 20 // by Y
    #define SILA_DOCISKUEDP_OPADANIE 40

    #define YG_TEACH 0 // uczenie
    #define YG_MOVE 1 // ruch


    // po osiagnieciu powierzhcni zaczyna od MAX_SILA_DOCISKUEDP i osiaga SILA_DOCISKUEDP

class drawing_teach_in_force : public teach_in {

    private:
        int in_state_iteration; // numer interacji dla biezacego stanu generatora (powierzchni, uniesienia etc.)
        short gen_state, next_gen_state, prev_gen_state; // stan w ktorym znajduje sie generator
        double conv_summar_inc;

        robot::robot *irp6ot, *irp6p, *conv;
        lib::sensor *vsp_force_irp6ot, *vsp_force_irp6p;

    protected:

        lib::POSE_SPECIFICATION emptyps;

    public:
        double delta[6];
        lib::trajectory_description td;
        int step_no;

         // uczenie czy ruch - wybor trybu pracy generatora (definicje YG_TEACH, YG_MOVE)
        short teach_or_move;

    // konstruktor
    drawing_teach_in_force(task::task& _mp_task, int step=0);


        virtual bool first_step ();

        virtual bool next_step ();

}; // end: class drawing_teach_in_force
// --------------------------------------------------------------------------

} // namespace common
} // namespace mp
} // namespace mrrocpp

#endif
