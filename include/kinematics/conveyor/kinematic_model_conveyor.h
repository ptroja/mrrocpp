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

#if !defined(_CONVEYOR_KIN_MODEL)
#define _CONVEYOR_KIN_MODEL

// Definicja klasy kinematic_model.
#include "kinematics/common/kinematic_model.h"

namespace mrrocpp {
namespace kinematic {
namespace conveyor {

class kinematic_model_conveyor : public common::kinematic_model
{
protected:
  // Polozenie synchronizacji.
  double synchro_motor_position;
  // Stosunek polozenia walu silnika do polozenia we wsp. wewn (zewn) w metrach.
  double motor_to_intext_ratio;

  // Ustawienie parametrow kinematycznych.
  virtual void set_kinematic_parameters(void);

  // Sprawdzenie ograniczen na polozenia katowe walow silnikow
  virtual void check_motor_position(const double motor_position[]);
  // Sprawdzenie ograniczen na wspolrzedne wewnetrzne
  virtual void check_joints(const double q[]);

public:
  // Konstruktor.
  kinematic_model_conveyor (void);

  // Przeliczenie polozenia walow silnikow na wspolrzedne wewnetrzne.
  virtual void mp2i_transform(const double* local_current_motor_pos, double* local_current_joints);

  // Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow.
  virtual void i2mp_transform(double* local_desired_motor_pos_new, double* local_desired_joints);

  // Rozwiazanie prostego zagadnienia kinematyki.
  virtual void direct_kinematics_transform(const double* local_current_joints, frame_tab* local_current_end_effector_frame);

  // Rozwiazanie odwrotnego zagadnienia kinematyki.
  virtual void inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, frame_tab* local_desired_end_effector_frame);


};//: kinematic_model_conveyor;

} // namespace conveyor
} // namespace kinematic
} // namespace mrrocpp

#endif
