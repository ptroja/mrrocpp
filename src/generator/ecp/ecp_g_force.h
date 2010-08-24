// -------------------------------------------------------------------------
//                            generator/ecp_g_force.h dla QNX6
// Deklaracje generatorow dla procesow ECP z wykorzystaniem sily
//
// -------------------------------------------------------------------------


#if !defined(_ECP_GEN_FORCE_H)
#define _ECP_GEN_FORCE_H

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "generator/ecp/ecp_g_teach_in.h"
#include "base/lib/mrmath/mrmath.h"



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
