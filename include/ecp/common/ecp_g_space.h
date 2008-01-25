
#if !defined ( _ECP_SPACE_H)
#define _ECP_SPACE_H

/********************************* INCLUDES *********************************/
#include <stdio.h>

#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/common/ecp_teach_in_generator.h"

#define NPOINTS 100         // Maximum number of controll poses

// ########################################################################################################
// ########################################################################################################
// ########################### Trajectory generators implemented by Krzysztof Bochnia  ####################
// ########################################################################################################
// ########################################################################################################


class irp6ot_hermite_spline_generator: public ecp_teach_in_generator {
protected:
  ecp_taught_in_pose starting_pose;   // stores coordinates of the starting pose of the robot
  double INTERVAL;								// time of interpolation of one macro step (in seconds)
  double T;										// time for which the values of spline functions are calculated (in seconds)
  double TS;									// time scale coefficient; time domain is multiplied by this coefficient
  double TSTEP;									// increment of T (in seconds)
  double yi[NPOINTS][MAX_SERVOS_NR];						// stores the values of coordinates of the control poses (no matter which system of the coordinates is used)
  double yiprim[NPOINTS][MAX_SERVOS_NR];					// stores the values of the first derivatives of the Hermite spline functions calculated for each control pose
  double time[NPOINTS];							// stores calculated times for each control pose
  double y[MAX_SERVOS_NR];									// stores values of the spline functions calculated for one value of T
  double a[MAX_SERVOS_NR];									// stores values of the second derivatives of the spline functions calculated for one value of T
  double v[MAX_SERVOS_NR];									// stores values of the first derivatives of the spline functions calculated for one value of T
  FILE *log_file;								// file that stores all calculated poses
  bool first_time;							// true - if the 'next_step' is called for the first time
  bool last_time;							// true - if the 'next_step' is called for the last time
  int npoints;									// number of control poses
  double v_def_motor[MAX_SERVOS_NR];						// 
  double v_def_joint[MAX_SERVOS_NR];						// arrays which store the values of the default velocities
  double v_def_xyz_euler[MAX_SERVOS_NR];					// 
  double v_def_xyz_angles[MAX_SERVOS_NR];					// 

public:	
   irp6ot_hermite_spline_generator (ecp_task& _ecp_task, double interval, double ts );		// constructor		 
   void fill_hermite_arrays (void);				// fills the arrays 'time', 'yi', 'yiprim'
   void calc_hermit();							// calculates values of the function for the current T
   virtual bool first_step ();
   virtual bool next_step ();
};// end: irp6ot_hermite_spline_generator


class irp6ot_natural_spline_generator: public ecp_teach_in_generator {
protected:
  ecp_taught_in_pose starting_pose;			// stores coordinates of the starting pose of the robot
  double INTERVAL;										// time of interpolation of one macro step (in seconds)
  double T;												// time for which the values of spline functions are calculated (in seconds)
  double TS;											// time scale coefficient; time domain is multiplied by this coefficient
  double TSTEP;											// increment of T (in seconds)two-dimensional array - stores the values of coordinates of the control poses (no matter which system of the coordinates is used)
  double yi[NPOINTS][MAX_SERVOS_NR];								// stores the values of coordinates of the control poses (no matter which system of the coordinates is used)
  double yibis[NPOINTS][MAX_SERVOS_NR];								// stores the values of the second derivatives of the natural spline functions calculated for each control pose
  double time[NPOINTS];									// stores calculated times for each control pose
  double y[MAX_SERVOS_NR];											// stores values of the spline functions calculated for one value of T
  double a[MAX_SERVOS_NR];											// stores values of the second derivatives of the spline functions calculated for one value of T
  double v[MAX_SERVOS_NR];											// stores values of the first derivatives of the spline functions calculated for one value of T
  FILE *log_file;										// file that stores all calculated poses
  bool first_time;									// true - if the 'next_step' is called for the first time
  bool last_time;									// true - if the 'next_step' is called for the last time
  int npoints;											// number of control poses
  double v_def_motor[MAX_SERVOS_NR];								// 
  double v_def_joint[MAX_SERVOS_NR];								// arrays which store the values of the default velocities
  double v_def_xyz_euler[MAX_SERVOS_NR];							// 
  double v_def_xyz_angles[MAX_SERVOS_NR];							// 

public:	
   irp6ot_natural_spline_generator(ecp_task& _ecp_task, double interval, double ts );		// constructor		 
   void fill_natural_arrays (void); 
   void calc_natural();
   virtual bool first_step ();
   virtual bool next_step ();
};// end: irp6ot_natural_spline_generator
// #################################################################################################

#endif
