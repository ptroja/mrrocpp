// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6ot_with_wrist.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na torze
//				- deklaracja klasy
//				- wykorzystanie nowego stopnia swobody  jako czynnego stopnia swobody
//				- tor jest biernym stopniem swobody.
//
// Autor:		tkornuta
// Data:		31.01.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6OT_KIN_MODEL_WITH_WRIST)
#define _IRP6OT_KIN_MODEL_WITH_WRIST

// Definicja klasy kinematic_model.
#include "kinematics/common/kinematic_model_with_tool.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6ot {

class model_with_wrist : public common::kinematic_model_with_tool
{
protected:
  // Dlugosci czlonow
  double d1;
  double a2;
  double a3;
  double d5;
  double d6;
  double d7;

  // Zmienne opisujace przekladnie dla wszystkich stopni swobody.
  double gear[8];
  double theta[8];

  // Zmienne uzywane przy obliczeniach zwiazanych z ramieniami dolnym i gornym.
  double sl123;
  double mi2;
  double ni2;
  double mi3;
  double ni3;

  // Zmienne zwiazane z obliczeniami zwarcia/rozwarcia chwytaka.
  double dir_a_7;
  double dir_b_7;
  double dir_c_7;
  double inv_a_7;
  double inv_b_7;
  double inv_c_7;
  double inv_d_7;

  // Zakresy ruchu walow silnikow w radianach.
  double lower_limit_axis[8];
  double upper_limit_axis[8];
  // Zakresy ruchu poszczegolnych stopni swobody (w radianach lub milimetrach).
  double lower_limit_joint[8];
  double upper_limit_joint[8];

  // Polozenia synchronizacji (polozenia walow silnikow).
  double synchro_motor_position[8];
  // Polozenia synchronizacji (polozenia we wspolrzednych wewnetrznych).
  double synchro_joint_position[8];

  // Ustawienie parametrow kinematycznych.
  virtual void set_kinematic_parameters(void);


  // Sprawdzenie ograniczen na polozenia katowe walow silnikow
  virtual void check_motor_position(const std::vector<double> & motor_position);
  // Sprawdzenie ograniczen na wspolrzedne wewnetrzne
  virtual void check_joints(const std::vector<double> & q);

public:
  // Konstruktor.
  model_with_wrist ( void );

  // Przeliczenie polozenia walow silnikow na wspolrzedne wewnetrzne.
  virtual void mp2i_transform(const std::vector<double> & local_current_motor_pos, std::vector<double> & local_current_joints);

  // Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow.
  virtual void i2mp_transform(std::vector<double> & local_desired_motor_pos_new, std::vector<double> & local_desired_joints);

  // Rozwiazanie prostego zagadnienia kinematyki.
  virtual void direct_kinematics_transform(const std::vector<double> & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame);

  // Rozwiazanie odwrotnego zagadnienia kinematyki.
  virtual void inverse_kinematics_transform(std::vector<double> & local_desired_joints, std::vector<double> & local_current_joints, lib::Homog_matrix& local_desired_end_effector_frame);

};//: kinematic_model_irp6ot_with_wrist;

} // namespace irp6ot
} // namespace kinematic
} // namespace mrrocpp

#endif
