// -------------------------------------------------------------------------
//
// Definicje struktur danych i metod dla procesow MP - generatory silowe
// 
// -------------------------------------------------------------------------

#if !defined(__MP_GEN_FORCE_H)
#define __MP_GEN_FORCE_H

#include "lib/mathtr.h"

#include "mp/mp_g_teach_in.h"



// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class mp_tff_single_robot_nose_run_generator : public mp_generator
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)

  
    mp_robot *irp6;
    sensor *vsp_force;
    
     trajectory_description td;
 
public:
       int step_no;
       double delta[6];

    // konstruktor
    mp_tff_single_robot_nose_run_generator(mp_task& _mp_task, int step=0);
	
	virtual bool first_step ();    
	virtual bool next_step ();    

}; // end : class MP_nose_run_force_generator




// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class mp_tff_nose_run_generator : public mp_generator 
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)
   
  
    mp_robot *irp6ot, *irp6p;
    sensor *vsp_force_irp6ot, *vsp_force_irp6p;
    
    // do konfiguracji pracy generatora
    unsigned short irp6ot_con, irp6p_con;
 

     trajectory_description td;   
 
public:
       int step_no;
       double delta[6];

    // konstruktor
    mp_tff_nose_run_generator(mp_task& _mp_task, int step=0);
	
	void configure (unsigned short l_irp6ot_con , unsigned short l_irp6p_con );

	
	virtual bool first_step ();    
	virtual bool next_step ();    

}; // end : class MP_nose_run_force_generator



// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class mp_haptic_generator : public mp_generator 
{
protected:
   
  
    mp_robot *irp6ot, *irp6p;
    sensor *vsp_force_irp6ot, *vsp_force_irp6p;
    
    // do konfiguracji pracy generatora
    unsigned short irp6ot_con, irp6p_con;
	Homog_matrix global_base;

     trajectory_description td;   
 
public:
       int step_no;
  //     double delta[6];

    // konstruktor
    mp_haptic_generator(mp_task& _mp_task, int step=0);
	
	void configure (unsigned short l_irp6ot_con , unsigned short l_irp6p_con );

	
	virtual bool first_step ();    
	virtual bool next_step ();    

}; // end : class mp_haptic_generator


// --------------------------------------------------------------------------
// Generator do lapania kostki

class mp_tff_rubik_grab_generator : public mp_generator 
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)
   
  
    mp_robot *irp6ot, *irp6p;
    sensor *vsp_force_irp6ot, *vsp_force_irp6p;

     trajectory_description td;
    
    // do konfiguracji pracy generatora
    unsigned short irp6ot_con, irp6p_con;
    double goal_position, position_increment;
    int min_node_counter;
    bool irp6p_both_axes_running, irp6ot_both_axes_running;
    
 
public:
       int step_no;
       double delta[6];

    // konstruktor
    mp_tff_rubik_grab_generator(mp_task& _mp_task, int step=0);
	
	void configure(unsigned short l_irp6ot_con, unsigned short l_irp6p_con, double l_goal_position, 
		double l_position_increment, int l_min_node_counter, bool l_irp6p_both_axes_running = true, bool l_irp6ot_both_axes_running = true);
	
	virtual bool first_step ();    
	virtual bool next_step ();    

}; // end : class MP_nose_run_force_generator


// --------------------------------------------------------------------------
// Generator do obracania sciany kostki

class mp_tff_rubik_face_rotate_generator : public mp_generator 
{
protected:
	int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)
	
     trajectory_description td;
    mp_robot *irp6ot, *irp6p;
    sensor *vsp_force_irp6ot, *vsp_force_irp6p;
    // do konfiguracji pracy generatora
    double irp6ot_con, irp6p_con;
    double irp6ot_stored_gamma, irp6p_stored_gamma;
    bool irp6ot_range_change, irp6p_range_change;
public:
       int step_no;
       double delta[6];

    // konstruktor
    mp_tff_rubik_face_rotate_generator(mp_task& _mp_task, int step=0);  
	
	void configure(double l_irp6ot_con, double l_irp6p_con);
	
	virtual bool first_step ();    
	virtual bool next_step ();    

}; // end : class MP_nose_run_force_generator



// --------------------------------------------------------------------------
// Generator do nasuniecia chwytaka na kostke

class mp_tff_gripper_approach_generator : public mp_generator 
{
protected:
	int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)
	
     trajectory_description td;
    mp_robot *irp6ot, *irp6p;
    sensor *vsp_force_irp6ot, *vsp_force_irp6p;
    // do konfiguracji pracy generatora
    double irp6ot_speed, irp6p_speed;
	int motion_time;
	
public:
       int step_no;
       double delta[6];

    // konstruktor
    mp_tff_gripper_approach_generator(mp_task& _mp_task, int step=0);  
	
	void configure(double l_irp6ot_speed, double l_irp6p_speed, int l_motion_time);
	
	virtual bool first_step ();    
	virtual bool next_step ();    

}; // end : class MP_nose_run_force_generator





// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos

class mp_nose_run_force_generator : public mp_generator 
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)
   
  
      mp_robot *irp6ot, *irp6p, *conv;
    sensor *vsp_force_irp6ot, *vsp_force_irp6p;
 
public:
     trajectory_description td;
       int step_no;
       double delta[6];

    // konstruktor
    mp_nose_run_force_generator(mp_task& _mp_task, int step=0);  

   virtual bool first_step ();    
   virtual bool next_step ();    

}; // end : class MP_nose_run_force_generator

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

class mp_drawing_teach_in_force_generator : public mp_teach_in_generator {

    private:
        int in_state_iteration; // numer interacji dla biezacego stanu generatora (powierzchni, uniesienia etc.)
        short gen_state, next_gen_state, prev_gen_state; // stan w ktorym znajduje sie generator 
        double conv_summar_inc;
        
        mp_robot *irp6ot, *irp6p, *conv;
        sensor *vsp_force_irp6ot, *vsp_force_irp6p;
        
    protected:
        
        POSE_SPECIFICATION emptyps;
    
    public:
        double delta[6];
        trajectory_description td;
        int step_no;
        
         // uczenie czy ruch - wybor trybu pracy generatora (definicje YG_TEACH, YG_MOVE)
        short teach_or_move;
        
    // konstruktor
    mp_drawing_teach_in_force_generator(mp_task& _mp_task, int step=0);
        
        
        virtual bool first_step ();
            
        virtual bool next_step ();
            
}; // end: class mp_drawing_teach_in_force_generator
// --------------------------------------------------------------------------

#endif
