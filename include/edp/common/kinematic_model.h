// *INDENT-OFF*
///////////////////////////////////////////////////////////////////////////////////////
/*! \file 		mrrocpp/include/edp/common/kinematic_model.h
 *\polish
 *	\brief 		$Header$
 *						Model kinematyki robota - klasa abstrakcyjna.
 *
 *						<szczegolowy opis>.
 *	\bug			Na razie nie stwierdzono.
 *	\warning	Na razie nie stwierdzono.			
 *\endpolish
 *
 *\english
 *	\brief 		$Header$
 *						Robot kinematic model - abstract class.
 *
 *						<details>.
 *	\bug			None yet revealed.
 *	\warning	None yet revealed.
 *\endenglish
 *
 *	\author 	\~polish	tkornuta.
 *						\~english	N/A.
 *	\version	QNX/MRROC++  v. 6.3
 */////////////////////////////////////////////////////////////////////////////////////
// *INDENT-ON*

#if !defined(__EDP_KIN_MODEL)  
#define __EDP_KIN_MODEL

// Klasa Homog_matrix.
#include "lib/mathtr.h"
// Klasa frame_tab.
#include "common/impconst.h"


class kinematic_model
{
protected:
  // Etykieta kinematyki.
  char* kinematic_model_label;

  // Ustawienie parametrow kinematycznych.
  virtual void set_kinematic_parameters(void) = 0;

  // Sprawdzenie ograniczen na polozenia katowe walow silnikow
  virtual void check_motor_position(const double motor_position[]) = 0;

  // Sprawdzenie ograniczen na wspolrzedne wewnetrzne
  virtual void check_joints(const double q[]) = 0;

  // Wspolczynnik kalibracji.
  double h;
  // Wektor korekcji - uzywany przy obliczeniach zwiazanych z korekcja lokalna.
  double V[6];
  // Macierz korekcji - uzywana przy obliczeniach zwiazanych z korekcja lokalna.
  double U[6][6];
  // Odwrocona macierz korekcji - uzywana przy obliczeniach zwiazanych z korekcja lokalna - zagadnienie odwrotne.
  double inv_U[6][6];

public:
  // Flaga - czy przeliczac do globalnego ukladu odniesienia.
  bool global_frame_computations;
  // Flaga - czy uzywc lokalnych korektorow.
  bool local_corrector_computations;
  // Flaga - czy wykonywac przeliczenia zwiazane z zamontowanym narzedziem.
  bool attached_tool_computations;

  // Macierz reprezentujaca narzedzie wzgledem koncowki manipulatora.
  Homog_matrix tool;

  // Macierz reprezentujaca pozycje bazy robota w globalnym ukladzie odniesienia.
  Homog_matrix global_base;

  // Konstruktor.
  kinematic_model(void);
  
  // Przeliczenie polozenia walow silnikow na wspolrzedne wewnetrzne.
  virtual void mp2i_transform(const double* local_current_motor_pos, double* local_current_joints) = 0; 

  // Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow.
  virtual void i2mp_transform(double* local_desired_motor_pos_new, double* local_desired_joints) = 0;

  // Przeliczenie polozenia ze wspolrzednych wewnetrznych na wspolrzedne zewnetrzne.
  virtual void i2e_transform(const double* local_current_joints, frame_tab* local_current_end_effector_frame);

  // Przeliczenie polozenia ze wspolrzednych zewnetrznych na wspolrzedne zewnetrzne.
   virtual void e2i_transform(double* local_desired_joints, double* local_current_joints, frame_tab* local_desired_end_effector_frame);

  // Przeliczenie bazy manipulatora w globalnym ukladzie odniesienia.
  virtual void global_frame_transform(Homog_matrix&);

  // Przeliczenie bazy manipulatora w globalnym ukladzie odniesienia - transformacja odwrotna.
  virtual void global_frame_inverse_transform(Homog_matrix&);

  // Poprawa polozenia koncowki przy uzyciu macierzy korekcji lokalnej.
  virtual void local_corrector_transform(Homog_matrix&);

  // Poprawa polozenia koncowki przy uzyciu macierzy korekcji lokalnej - transformacja odwrotna.
  virtual void local_corrector_inverse_transform(Homog_matrix&);

  // Przeliczenie polozenia koncowki zwiazane z dolaczonym narzedziem.
  virtual void attached_tool_transform(Homog_matrix&);

  // Przeliczenie polozenia koncowki zwiazane z dolaczonym narzedziem - transformacja odwrotna.
  virtual void attached_tool_inverse_transform(Homog_matrix&);

  // Rozwiazanie prostego zagadnienia kinematyki.
  virtual void direct_kinematics_transform(const double* local_current_joints, frame_tab* local_current_end_effector_frame) = 0;
  
  // Rozwiazanie odwrotnego zagadnienia kinematyki.
  virtual void inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, frame_tab* local_desired_end_effector_frame) = 0;

  // Zwraca etykiete modelu kinematycznego.
  virtual char* get_kinematic_model_label(void);
  // Ustawia pokazywana etykiete modelu kinematycznego.
  virtual void set_kinematic_model_label(const char*);
  
};//: kinematic_model

#endif					   
