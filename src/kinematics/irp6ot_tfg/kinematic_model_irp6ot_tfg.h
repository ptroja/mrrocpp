// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_conveyor.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki tasmociogu
//				- deklaracja klasy
//
// Autor:		tkornuta
// Data:		31.01.2007
// ------------------------------------------------------------------------

#if !defined(_IRP6OT_TFG_KIN_MODEL)
#define _IRP6OT_TFG_KIN_MODEL

#include "kinematics/common/kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6ot {

class model : public common::kinematic_model
{
protected:

  // Zmienne opisujace przekladnie dla wszystkich stopni swobody.
  double gear;
  double theta;

  // Zakresy ruchu walow silnikow w radianach.
  double lower_limit_axis;
  double upper_limit_axis;
  // Zakresy ruchu poszczegolnych stopni swobody (w radianach lub milimetrach).
  double lower_limit_joint;
  double upper_limit_joint;

  // Polozenia synchronizacji (polozenia walow silnikow).
  double synchro_motor_position;
  // Polozenia synchronizacji (polozenia we wspolrzednych wewnetrznych).
  double synchro_joint_position;

  // Zmienne zwiazane z obliczeniami zwarcia/rozwarcia chwytaka.
  double dir_a_7;
  double dir_b_7;
  double dir_c_7;
  double inv_a_7;
  double inv_b_7;
  double inv_c_7;
  double inv_d_7;

  // Ustawienie parametrow kinematycznych.
  virtual void set_kinematic_parameters(void);

  // Sprawdzenie ograniczen na polozenia katowe walow silnikow
  virtual void check_motor_position(const lib::MotorArray & motor_position);
  // Sprawdzenie ograniczen na wspolrzedne wewnetrzne
  virtual void check_joints(const lib::JointArray & q);

public:
  // Konstruktor.
  model (void);

  // Przeliczenie polozenia walow silnikow na wspolrzedne wewnetrzne.
  virtual void mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints);

  // Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow.
  virtual void i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, lib::JointArray & local_desired_joints);


};//: kinematic_model_conveyor;

} // namespace irp6ot
} // namespace kinematic
} // namespace mrrocpp

#endif
