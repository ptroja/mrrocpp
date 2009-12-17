// *INDENT-OFF*
///////////////////////////////////////////////////////////////////////////////////////
/*! \file 		mrrocpp/include/kinematics/common/kinematic_model.h
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

#include <string>

// Klasa lib::Homog_matrix.
#include "lib/mathtr/mrmath.h"
// Klasa lib::frame_tab.
#include "lib/impconst.h"

#include "simple_model.h"

namespace mrrocpp {
namespace kinematic {
namespace common {

class model : public mrrocpp::kinematic::common::simple_model
{
protected:

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
  lib::Homog_matrix tool;

  // Macierz reprezentujaca pozycje bazy robota w globalnym ukladzie odniesienia.
  lib::Homog_matrix global_base;

	//! Class constructor - empty.
	model();

	//! Class virtual destructor - empty.
	virtual ~model();

	//! Computes external coordinates on the base of internal coordinates (i2e - internal to external).
	virtual void i2e_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame);

	//! Computes external coordinates on the base of internal coordinates, without the computations related with the attached tool.
	virtual void i2e_wo_tool_transform(const double* local_current_joints, lib::frame_tab* local_current_end_effector_frame);

  // Przeliczenie polozenia ze wspolrzednych zewnetrznych na wspolrzedne wewnetrzne.
  virtual void e2i_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame);

  // Przeliczenie polozenia ze wspolrzednych zewnetrznych na wspolrzedne wewnetrzne - bez obliczen zwiazanych z narzedziem.
  virtual void e2i_wo_tool_transform(double* local_desired_joints, double* local_current_joints, lib::frame_tab* local_desired_end_effector_frame);

  // Przeliczenie bazy manipulatora w globalnym ukladzie odniesienia.
  virtual void global_frame_transform(lib::Homog_matrix&);

  // Przeliczenie bazy manipulatora w globalnym ukladzie odniesienia - transformacja odwrotna.
  virtual void global_frame_inverse_transform(lib::Homog_matrix&);

  // Poprawa polozenia koncowki przy uzyciu macierzy korekcji lokalnej.
  virtual void local_corrector_transform(lib::Homog_matrix&);

  // Poprawa polozenia koncowki przy uzyciu macierzy korekcji lokalnej - transformacja odwrotna.
  virtual void local_corrector_inverse_transform(lib::Homog_matrix&);

  // Przeliczenie polozenia koncowki zwiazane z dolaczonym narzedziem.
  virtual void attached_tool_transform(lib::Homog_matrix&);

  // Przeliczenie polozenia koncowki zwiazane z dolaczonym narzedziem - transformacja odwrotna.
  virtual void attached_tool_inverse_transform(lib::Homog_matrix&);

};//: kinematic_model

} // namespace common
} // namespace kinematic
} // namespace mrrocpp

#endif
