// -------------------------------------------------------------------------
//                            generator/ecp_g_force.h dla QNX6
// Deklaracje generatorow dla procesow ECP z wykorzystaniem sily
//
// -------------------------------------------------------------------------


#if !defined(_ECP_GEN_FORCE_H)
#define _ECP_GEN_FORCE_H

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/common/generator/ecp_g_teach_in.h"
#include "lib/mrmath/mrmath.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// Generator do wykrywania zmiany wagi chwytaka wraz z obiektem chwytanym
// ciezary wyskalowane w newtonach
#define WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE 10
#define USLEEP_TIME 10000

class weight_meassure : public common::generator::generator
{
private:
    double weight_difference;  // roznica wagi do wykrycia
    double weight_in_cyclic_buffer[WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE];
    int current_buffer_pointer;
    double initial_weight; // pierwszy zmierzony ciezar
    bool initial_weight_counted; // czy wyznaczono juz poczatkowy ciezar
    int catch_lag; // ilosc potwierdzen zmiany masy do zwrocenia false;
    int initial_catch_lag; // ilosc potwierdzen zmiany masy do zwrocenia false;
    double catch_time; // czas przez ktory ma byc stwierdzona zmian ci???aru
    bool terminate_state_recognized; // wykryto warunek koncowy

    // wstawienie elementu do bufora cyklicznego
    void insert_in_buffer(const double fx);

    // wyznaczenie sredniej arytmetycznej ciezarow zapisanych w buforze
    double check_average_weight_in_buffer(void) const;

    // czyszczenie bufora cyklicznego
    void clear_buffer();

public:

    // ustawia nowa roznice wag
    void set_weight_difference(const double _weight_difference);

    // konstruktor
    weight_meassure(common::task::task& _ecp_task, double _weight_difference=0.0, double _catch_time = 1.0);

    bool first_step ();
    bool next_step ();

}; // end:




// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos
class y_nose_run_force : public common::generator::generator
{
private:
    lib::trajectory_description td;
    const int step_no;
    double delta[6];

public:
    // konstruktor
    y_nose_run_force(common::task::task& _ecp_task, int step=0);

    virtual bool first_step ();
    virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Generator do zadania z jajkiem


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



class y_egg_force : public common::generator::generator
{
private:
    short gen_state, next_gen_state, prev_gen_state; // stan w ktorym znajduje sie generator
    int in_state_iteration; // numer interacji dla biezacego stanu generatora (powierzchni, uniesienia etc.)

    lib::trajectory_description td;
    const int step_no;
    // TODO: this should be enum(erated)
    const int int_mode; // wewnetrzny tryb pracy
    double delta[6];

public:
    // konstruktor
    y_egg_force(common::task::task& _ecp_task, int step=0, int mode=0);

    virtual bool first_step ();
    virtual bool next_step ();
}; // end:
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Generator odtwarzajacy nauczone pozycje, ale nie sygnalizujacy zakonczenia
// zadania procesowi MP

#define SILA_DOCISKU 0.00002 // by Y
#define MIN_SILA_DOCISKUEDP 20
#define SILA_DOCISKUEDP 20 // by Y
#define SILA_DOCISKUEDP_OPADANIE 40

// po osiagnieciu powierzhcni zaczyna od MAX_SILA_DOCISKUEDP i osiaga SILA_DOCISKUEDP

class y_drawing_teach_in_force : public teach_in
{
protected:
    lib::POSE_SPECIFICATION emptyps;

    double delta[6];
    lib::trajectory_description td;
    const int step_no;

public:
    // uczenie czy ruch - wybor trybu pracy generatora (definicje YG_TEACH, YG_MOVE)
    enum Y_DRAWING_GEN_ENUM {
        YG_TEACH,
        YG_MOVE
    } teach_or_move;

    y_drawing_teach_in_force(common::task::task& _ecp_task, int step);

    virtual bool first_step ();
    virtual bool next_step ();
}; // end: class y_drawing_teach_in_force_generator
// --------------------------------------------------------------------------

// nowy silowy generator uczacy ( z odrywaniem )

class y_advanced_drawing_teach_in_force : public y_drawing_teach_in_force
{
private:
    int in_state_iteration; // numer interacji dla biezacego stanu generatora (powierzchni, uniesienia etc.)
    short gen_state, next_gen_state, prev_gen_state; // stan w ktorym znajduje sie generator
    // wykorzsytywany w next_step;

public:
    y_advanced_drawing_teach_in_force(common::task::task& _ecp_task, int step);

    virtual bool first_step ();
    virtual bool next_step ();
};


///
// --------------------------------------------------------------------------
class legobrick_attach_force : public teach_in
{

protected:

    lib::POSE_SPECIFICATION emptyps;
    lib::trajectory_description td;
    const int step_no;
    double delta[6];
    lib::Homog_matrix basic_rot_frame;
    lib::Homog_matrix tool_frame;
    lib::Homog_matrix ex_rot_frame;

public:

    // konstruktor
    legobrick_attach_force(common::task::task& _ecp_task, int step);

    virtual bool first_step ();

    virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------

//
class legobrick_detach_force : public teach_in
{

protected:

    lib::POSE_SPECIFICATION emptyps;
    lib::trajectory_description td;
    const int step_no;
    double delta[6];
    lib::Homog_matrix basic_rot_frame;
    lib::Homog_matrix tool_frame;
    lib::Homog_matrix ex_rot_frame;

    double start_position_w3;
    bool isStart;

public:

    // konstruktor
    legobrick_detach_force(common::task::task& _ecp_task, int step);

    virtual bool first_step ();

    virtual bool next_step ();

}; // end:
// --------------------------------------------------------------------------




// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class tff_nose_run : public common::generator::generator
{
protected:
    lib::trajectory_description td;
    // skladowe silowe i pozycyjne (zablokowane)
    bool selection_vector_l[6];
    // czy pulse_check ma byc aktywne
    bool pulse_check_activated;
	bool force_meassure;

	struct generator_edp_data_type
	{
		double next_inertia[6], next_reciprocal_damping[6];
		double next_velocity[MAX_SERVOS_NR], next_force_xyz_torque_xyz[6];
		lib::BEHAVIOUR_SPECIFICATION next_behaviour[6];
	} generator_edp_data;

public:
    const int step_no;

    // konstruktor
    tff_nose_run(common::task::task& _ecp_task, int step=0);
    void execute_motion (void);

	void configure_behaviour(lib::BEHAVIOUR_SPECIFICATION x, lib::BEHAVIOUR_SPECIFICATION y, lib::BEHAVIOUR_SPECIFICATION z,
		 lib::BEHAVIOUR_SPECIFICATION ax, lib::BEHAVIOUR_SPECIFICATION ay, lib::BEHAVIOUR_SPECIFICATION az);
	void configure_pulse_check(bool pulse_check_activated_l);
	void configure_velocity(double x, double y, double z,	 double ax, double ay, double az);
	void configure_force(double x, double y, double z,	 double ax, double ay, double az);
	void configure_reciprocal_damping(double x, double y, double z, double ax, double ay, double az);
	void configure_inertia(double x, double y, double z,	 double ax, double ay, double az);

    virtual bool first_step ();
    virtual bool next_step ();

    void set_force_meassure(bool fm);

}; // end : class ecp_tff_nose_run_generator

// --------------------------------------------------------------------------
// Generator trajektorii dla zadania kalibracji ukladu eih.
// Rozni sie od tff_nose_run tym ze zatrzymuje sie po chwili i trzeba go uzywac w petli

class eih_nose_run : public tff_nose_run
{
	int count;

public:
    // konstruktor
    eih_nose_run(common::task::task& _ecp_task, int step=0);

    virtual bool next_step ();

}; // end : class ecp_eih_nose_run_generator

// --------------------------------------------------------------------------
// Generator trajektorii dla zadania kalibracji ukladu eih.
// Rozni sie od tff_nose_run tym ze zatrzymuje sie po chwili i trzeba go uzywac w petli

class pcbird_nose_run : public tff_nose_run
{
	int count;

public:
    // konstruktor
	pcbird_nose_run(common::task::task& _ecp_task, int step=0);

    virtual bool first_step ();
    virtual bool next_step ();

}; // end : class ecp_eih_nose_run_generator

// --------------------------------------------------------------------------
// Generator trajektorii dla spots_recognition

class sr_nose_run : public common::generator::generator
{

	long iter_while_idle;
	short state; // 0 - first run, 1 - normal work, 2 - exit mode

protected:

    lib::trajectory_description td;
    // skladowesilowe i pozycyjne (zablokowane)
    bool selection_vector_l[6];
    // czy pulse_check ma byc aktywne
    bool pulse_check_activated;
	bool force_meassure;

	struct generator_edp_data_type
	{
		double next_inertia[6], next_reciprocal_damping[6];
		double next_velocity[MAX_SERVOS_NR], next_force_xyz_torque_xyz[6];
		lib::BEHAVIOUR_SPECIFICATION next_behaviour[6];
	} generator_edp_data;

public:
    const int step_no;

    // konstruktor
    sr_nose_run(common::task::task& _ecp_task, int step=0);
    void execute_motion (void);

	void configure_behaviour(lib::BEHAVIOUR_SPECIFICATION x, lib::BEHAVIOUR_SPECIFICATION y, lib::BEHAVIOUR_SPECIFICATION z,
		 lib::BEHAVIOUR_SPECIFICATION ax, lib::BEHAVIOUR_SPECIFICATION ay, lib::BEHAVIOUR_SPECIFICATION az);
	void configure_pulse_check(bool pulse_check_activated_l);
	void configure_velocity(double x, double y, double z,	 double ax, double ay, double az);
	void configure_force(double x, double y, double z,	 double ax, double ay, double az);
	void configure_reciprocal_damping(double x, double y, double z, double ax, double ay, double az);
	void configure_inertia(double x, double y, double z,	 double ax, double ay, double az);

    virtual bool first_step ();
    virtual bool next_step ();

    void set_force_meassure(bool fm);

    bool check_and_decide();
    short get_state();
    void next_state();

}; // end : class ecp_sr_nose_run_generator


class bias_edp_force : public common::generator::generator
{
public:
    // konstruktor
    bias_edp_force(common::task::task& _ecp_task);

    virtual bool first_step ();
    virtual bool next_step ();
};


// --------------------------------------------------------------------------
// Generator do lapania kostki

class tff_rubik_grab : public common::generator::generator
{
protected:
    lib::trajectory_description td;

    // do konfiguracji pracy generatora
    double goal_position, position_increment;
    unsigned int min_node_counter;
    bool both_axes_running;
    double desired_absolute_gripper_coordinate;

public:
    const int step_no;

    // konstruktor
    tff_rubik_grab(common::task::task& _ecp_task, int step=0);

    void configure(double l_goal_position, double l_position_increment, unsigned int l_min_node_counter,
                   bool l_both_axes_running = true);

    virtual bool first_step ();
    virtual bool next_step ();

}; // end : class ecp_tff_rubik_grab_generator


// --------------------------------------------------------------------------
// Generator do obracania sciany kostki

class tff_rubik_face_rotate : public common::generator::generator
{
protected:
    lib::trajectory_description td;

    // do konfiguracji pracy generatora

    double stored_gamma, turn_angle;
    bool range_change;

public:
    const int step_no;

    // konstruktor
    tff_rubik_face_rotate(common::task::task& _ecp_task, int step=0);

    void configure(double l_turn_angle);

    virtual bool first_step ();
    virtual bool next_step ();

}; // end : class ecp_tff_rubik_face_rotate_generator



// --------------------------------------------------------------------------
// Generator do nasuniecia chwytaka na kostke

class tff_gripper_approach : public common::generator::generator
{
protected:

    lib::trajectory_description td;

    // do konfiguracji pracy generatora
    double speed;
    unsigned int motion_time;

public:
    const int step_no;

    // konstruktor
    tff_gripper_approach(common::task::task& _ecp_task, int step=0);

    void configure(double l_speed, unsigned int l_motion_time);

    virtual bool first_step ();
    virtual bool next_step ();

}; // end : class ecp_tff_gripper_approach_generator

class force_tool_change : public common::generator::generator
{
protected:
	double tool_parameters[3]; // zobaczyc jeszcze co z tymi parametrami jak to bedzie w przypadku tego generatora
	double weight;
public:
	force_tool_change(common::task::task& _ecp_task);
	//ecp_force_tool_change_generator(common::task::task& _ecp_task, bool _is_synchronised, bool _debug);
	void set_tool_parameters(double x, double y, double z, double weight); // tez zobaczyc jakie tu mamy parametry

	virtual bool first_step();
	virtual bool next_step();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
