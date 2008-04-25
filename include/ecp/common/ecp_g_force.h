// -------------------------------------------------------------------------
//                            ecp_g_force.h dla QNX6
// Deklaracje generatorow dla procesow ECP z wykorzystaniem sily
//
// -------------------------------------------------------------------------


#if !defined(_ECP_GEN_FORCE_H)
#define _ECP_GEN_FORCE_H

#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/common/ecp_teach_in_generator.h"
#include "lib/mathtr.h"


// Generator do wykrywania zmiany wagi chwytaka wraz z obiektem chwytanym
// ciezary wyskalowane w newtonach
#define WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE 10
#define USLEEP_TIME 10000

class weight_meassure_generator : public ecp_generator
{
private:
    double weight_difference;  // roznica wagi do wykrycia
    double weight_in_cyclic_buffer[WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE];
    int current_buffer_pointer;
    double initial_weight; // pierwszy zmierzony ciezar
    bool initial_weight_counted; // czy wyznaczono juz poczatkowy ciezar
    int catch_lag; // ilosc potwierdzen zmiany masy do zwrocenia false;
    int initial_catch_lag; // ilosc potwierdzen zmiany masy do zwrocenia false;
    double catch_time; // czas przez ktory ma byc stwierdzona zmian ciê¿aru
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
    weight_meassure_generator(ecp_task& _ecp_task, double _weight_difference=0.0, double _catch_time = 1.0);

    bool first_step ();
    bool next_step ();

}
; // end:




// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos
class y_nose_run_force_generator : public ecp_generator
{


public:
    trajectory_description td;
    int step_no;
    double delta[6];

    // konstruktor
    y_nose_run_force_generator(ecp_task& _ecp_task, int step=0);

    virtual bool first_step ();
    virtual bool next_step ();

}
; // end:
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



class y_egg_force_generator : public ecp_generator
{

protected:

    short gen_state, next_gen_state, prev_gen_state; // stan w ktorym znajduje sie generator
    int in_state_iteration; // numer interacji dla biezacego stanu generatora (powierzchni, uniesienia etc.)

public:
    trajectory_description td;
    int step_no;
    int int_mode; // wewnetrzny tryb pracy
    double delta[6];

    // konstruktor
    y_egg_force_generator(ecp_task& _ecp_task, int step=0, int mode=0);

    virtual bool first_step ();

    virtual bool next_step ();

}
; // end:
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Generator odtwarzajacy nauczone pozycje, ale nie sygnalizujacy zakonczenia
// zadania procesowi MP

#define SILA_DOCISKU 0.00002 // by Y
	#define MIN_SILA_DOCISKUEDP 20
	#define SILA_DOCISKUEDP 20 // by Y
	#define SILA_DOCISKUEDP_OPADANIE 40


enum Y_DRAWING_GEN_ENUM {
    YG_TEACH,
    YG_MOVE
};


// po osiagnieciu powierzhcni zaczyna od MAX_SILA_DOCISKUEDP i osiaga SILA_DOCISKUEDP

class y_drawing_teach_in_force_generator : public ecp_teach_in_generator
{

protected:

    POSE_SPECIFICATION emptyps;

public:
    double delta[6];
    trajectory_description td;
    int step_no;

    // uczenie czy ruch - wybor trybu pracy generatora (definicje YG_TEACH, YG_MOVE)
    Y_DRAWING_GEN_ENUM teach_or_move;

    y_drawing_teach_in_force_generator(ecp_task& _ecp_task, int step);

    virtual bool first_step ();
    virtual bool next_step ();
}
; // end: class y_drawing_teach_in_force_generator
// --------------------------------------------------------------------------



class y_advanced_drawing_teach_in_force_generator : public y_drawing_teach_in_force_generator
{
    // nowy silowy generator uczacy ( z odrywaniem )

private:
    int in_state_iteration; // numer interacji dla biezacego stanu generatora (powierzchni, uniesienia etc.)
    short gen_state, next_gen_state, prev_gen_state; // stan w ktorym znajduje sie generator
    // wykorzsytywany w next_step;

public:
    y_advanced_drawing_teach_in_force_generator(ecp_task& _ecp_task, int step);

    virtual bool first_step ();
    virtual bool next_step ();
};


class y_edge_follow_force_generator : public ecp_teach_in_generator
{

protected:

    POSE_SPECIFICATION emptyps;
    trajectory_description td;
    int step_no;
    double delta[6];
    Homog_matrix basic_rot_frame;
    Homog_matrix tool_frame;
    Homog_matrix ex_rot_frame;

public:

    // konstruktor
    y_edge_follow_force_generator(ecp_task& _ecp_task, int step);

    virtual bool first_step ();

    virtual bool next_step ();

}
; // end:
///
// --------------------------------------------------------------------------
class legobrick_attach_force_generator : public ecp_teach_in_generator
{

protected:

    POSE_SPECIFICATION emptyps;
    trajectory_description td;
    int step_no;
    double delta[6];
    Homog_matrix basic_rot_frame;
    Homog_matrix tool_frame;
    Homog_matrix ex_rot_frame;

public:

    // konstruktor
    legobrick_attach_force_generator(ecp_task& _ecp_task, int step);

    virtual bool first_step ();

    virtual bool next_step ();

}
; // end:
// --------------------------------------------------------------------------

//
class legobrick_detach_force_generator : public ecp_teach_in_generator
{

protected:

    POSE_SPECIFICATION emptyps;
    trajectory_description td;
    int step_no;
    double delta[6];
    Homog_matrix basic_rot_frame;
    Homog_matrix tool_frame;
    Homog_matrix ex_rot_frame;

    double start_position_w3;

public:

    // konstruktor
    legobrick_detach_force_generator(ecp_task& _ecp_task, int step);

    virtual bool first_step ();

    virtual bool next_step ();

}
; // end:
// --------------------------------------------------------------------------




// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class ecp_tff_nose_run_generator : public ecp_generator
{
protected:


    trajectory_description td;
    // skladowesilowe i pozycyjne (zablokowane)
    bool selection_vector_l[6];
    // czy pulse_check ma byc aktywne
    bool pulse_check_activated;
	bool force_meassure;

public:
    int step_no;

    // konstruktor
    ecp_tff_nose_run_generator(ecp_task& _ecp_task, int step=0);

    // decyduje ktore osie maja byc podatne
    void configure(bool x, bool y, bool z, bool g, bool b, bool a, bool pulse_check_activated_l);
    virtual bool first_step ();
    virtual bool next_step ();
    
    void set_force_meassure(bool fm);

}
; // end : class ecp_tff_nose_run_generator






// --------------------------------------------------------------------------
// Generator do lapania kostki

class bias_edp_force_generator : public ecp_generator
{

public:

    // konstruktor
    bias_edp_force_generator(ecp_task& _ecp_task);

    virtual bool first_step ();
    virtual bool next_step ();
};


// --------------------------------------------------------------------------
// Generator do lapania kostki

class ecp_tff_rubik_grab_generator : public ecp_generator
{
protected:



    trajectory_description td;

    // do konfiguracji pracy generatora
    double goal_position, position_increment;
    int min_node_counter;
    bool both_axes_running;

    // czy generator ma sie zakonczyc
    bool finished;

public:
    int step_no;


    // konstruktor
    ecp_tff_rubik_grab_generator(ecp_task& _ecp_task, int step=0);

    void configure(double l_goal_position, double l_position_increment, int l_min_node_counter,
                   bool l_both_axes_running = true);

    virtual bool first_step ();
    virtual bool next_step ();

}
; // end : class ecp_tff_rubik_grab_generator


// --------------------------------------------------------------------------
// Generator do obracania sciany kostki

class ecp_tff_rubik_face_rotate_generator : public ecp_generator
{
protected:


    trajectory_description td;

    // do konfiguracji pracy generatora

    double stored_gamma, turn_angle;
    bool range_change;

    // czy generator ma sie zakonczyc
    bool finished;

public:
    int step_no;

    // konstruktor
    ecp_tff_rubik_face_rotate_generator(ecp_task& _ecp_task, int step=0);

    void configure(double l_turn_angle);

    virtual bool first_step ();
    virtual bool next_step ();

}
; // end : class ecp_tff_rubik_face_rotate_generator



// --------------------------------------------------------------------------
// Generator do nasuniecia chwytaka na kostke

class ecp_tff_gripper_approach_generator : public ecp_generator
{
protected:

    trajectory_description td;

    // do konfiguracji pracy generatora
    double speed;
    int motion_time;

    // czy generator ma sie zakonczyc
    bool finished;

public:
    int step_no;


    // konstruktor
    ecp_tff_gripper_approach_generator(ecp_task& _ecp_task, int step=0);

    void configure(double l_speed, int l_motion_time);

    virtual bool first_step ();
    virtual bool next_step ();

}
; // end : class ecp_tff_gripper_approach_generator

class ecp_force_tool_change_generator : public ecp_generator
{
protected:
	double tool_parameters[3]; // zobaczyc jeszcze co z tymi parametrami jak to bedzie w przypadku tego generatora
	double weight;
public:
	ecp_force_tool_change_generator(ecp_task& _ecp_task);
	//ecp_force_tool_change_generator(ecp_task& _ecp_task, bool _is_synchronised, bool _debug);
	void set_tool_parameters(double x, double y, double z, double weight); // tez zobaczyc jakie tu mamy parametry
	
	virtual bool first_step();
	virtual bool next_step();
};

#endif
