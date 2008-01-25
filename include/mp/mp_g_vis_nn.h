// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP - generatory silowe
// 
// Ostatnia modyfikacja: 2005
// -------------------------------------------------------------------------

#if !defined(__MP_G_VIS_NN_H)
#define __MP_G_VIS_NN_H

// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class mp_nn_eye_generator : public mp_generator 
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)
   int node_counter;               // biezacy wezel interpolacji
  
    mp_robot *irp6ot, *irp6p;
    sensor *vsp_force_irp6ot, *vsp_force_irp6p, *vsp_vis_sac;
    
    // do konfiguracji pracy generatora
    unsigned short irp6ot_con, irp6p_con;
 

     trajectory_description td;   
 
public:
       int step_no;
       double delta[6];

    // konstruktor
    mp_nn_eye_generator(mp_task& _mp_task, int step=0);  
	
	virtual bool first_step ();    
	virtual bool next_step ();    

}; // end : class MP_nose_run_force_generator

#endif
