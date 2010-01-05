// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_irp6m_with_wrist.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na postumencie
//				- deklaracja klasy
//				- wykorzystanie nowego stopnia swobody  jako czynnego stopnia swobody
//
// Autor:		tkornuta
// Data:		31.01.2007
// ------------------------------------------------------------------------

#if !defined(_SPKM_KIN_model)
#define _SPKM_KIN_model

#include "kinematics/common/simple_kinematic_model.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

class model: public common::simple_kinematic_model
{
	protected:
		// Ustawienie parametrow kinematycznych.
		virtual void set_kinematic_parameters(void);

		// Sprawdzenie ograniczen na polozenia katowe walow silnikow
		virtual void check_motor_position(const double motor_position[]);

		// Sprawdzenie ograniczen na wspolrzedne wewnetrzne
		virtual void check_joints(const double q[]);

	public:
		// Konstruktor.
		model(void);

		// Przeliczenie polozenia walow silnikow na wspolrzedne wewnetrzne.
		virtual void mp2i_transform(const double* local_current_motor_pos, double* local_current_joints);

		// Przeliczenie wspolrzednych wewnetrznych na polozenia walow silnikow.
		virtual void i2mp_transform(double* local_desired_motor_pos_new, double* local_desired_joints);

		// Rozwiazanie prostego zagadnienia kinematyki.
		virtual void direct_kinematics_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame);

		// Rozwiazanie odwrotnego zagadnienia kinematyki.
		virtual void inverse_kinematics_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame);

};

} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp


#endif

