// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
// 
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(__MP_GEN_TEST_H)
#define __MP_GEN_TEST_H

// ####################################################################################################
// KLASA BAZOWA dla generatorow o zadany przyrost polozenia/orientacji  
// ####################################################################################################

class MP_vf_generator : public mp_generator 
{
protected:
  int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)
   int node_counter;               // biezacy wezel interpolacji
  
public:	
   trajectory_description td;
   	int step_no;
   	
   	
   	double delta[6];
	double frame1[4][4];
	int valid_measure;
	bool the_first;
	bool the_second;
	double pose[6][5];
  	double pose_v[6][5];
  	double pose_a[6][5];
  	double pose_d[6][5];
  	double pose_d2[6][5];
	
	double measure[6][5];
	double measure_v[6][5];
	double measure_a[6][5];
	double measure_d[6][5];
	double measure_d2[6][5];

	double stearing[6][5];
	double stearing_v[6][5];
	double stearing_a[6][5];
	double stearing_d[6][5];
	double stearing_d2[6][5];
	
	double alfa;
	double beta;
	double gammax;


	// konstruktor
	MP_vf_generator(mp_task& _mp_task, int step=0);  

   virtual bool first_step ();	
   virtual bool next_step ();	

}; // end : class MP_delta_generator



// ########################################################################################################
// ####################################    KONIEC GENERATOROW   ###########################################
// ########################################################################################################


#endif
