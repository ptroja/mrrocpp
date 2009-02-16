// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_calibrated_correction_matrix_irp6p_with_wrist.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na postumencie
//				- definicja metod klasy
//				- wykorzystanie nowego stopnia swobody  jako czynnego stopnia swobody
//				- tor jest biernym stopniem swobody
//				- parametry obliczone zostaly podczas kalibracji
//
// Autor:		tkornuta
// Data:		17.03.2007
// ------------------------------------------------------------------------

// Klasa kinematic_model_calibrated_irp6ot_with_wrist.
#include "kinematics/irp6_postument/kinematic_model_calibrated_correction_matrix_irp6p_with_wrist.h"

/* -----------------------------------------------------------------------
  Konstruktor.
 ------------------------------------------------------------------------- */
kinematic_model_calibrated_correction_matrix_irp6p_with_wrist::kinematic_model_calibrated_correction_matrix_irp6p_with_wrist (void)
{
  // Ustawienie etykiety modelu kinematycznego.
  set_kinematic_model_label("Switching to calibrated and locally corrected kinematic model with active wrist");

  // Ustawienie parametrow kinematycznych.
  set_kinematic_parameters();

}; //: kinematic_model_calibrated_correction_matrix_irp6p_with_wrist

/* -----------------------------------------------------------------------
  Ustawienia wszystkie parametry modelu kinematycznego danego modelu.
 ------------------------------------------------------------------------- */
void kinematic_model_calibrated_correction_matrix_irp6p_with_wrist::set_kinematic_parameters(void)
{
	// Uzycie przeliczen zwiazanych z lokalnym korektorem.
	local_corrector_computations = true;

	// Wspolczynniki.
	h = 300.0;

	// Wektor korekcji V.
	V[0] =4.186245467019;
	V[1] =2.094569693552;
	V[2] =-0.161588754272;
	V[3] =77.951897120918;
	V[4] =-52.826191387489;
	V[5] =173.454098910559;

	// Macierz U.
	// U - wiersz pierwszy.
	U[0][0] =1.00102463485959;
	U[0][1] =0.00161056648449;
	U[0][2] =0.00169054802793;
	U[0][3] =0.00469995608682;
	U[0][3] =0.00097850153470;
	U[0][4] =0.00253527366760;

	U[1][0] =-0.00042951885466;
	U[1][1] =0.99952371920133;
	U[1][2] =-0.00264345780279;
	U[1][3] =0.00206828641808;
	U[1][4] =-0.00448852239811;
	U[1][5] =-0.00151520496490;

	U[2][0] =-0.00043441612583;
	U[2][1] =-0.00093513330445;
	U[2][2] =1.00133131155755;
	U[2][3] =0.00182284913467;
	U[2][4] =-0.00695132215441;
	U[2][5] =-0.00451529673725;

	U[3][0] =-0.02242975313157;
	U[3][1] =0.02790112089215;
	U[3][2] =-0.01190614370180;
	U[3][3] =1.04006492574638;
	U[3][4] =-0.03067763227170;
	U[3][5] =0.03303363783823;

	U[4][0] =0.00275483494508;
	U[4][1] =-0.00569922454321;
	U[4][2] =-0.01229349281362;
	U[4][3] =0.00495336143024;
	U[4][4] =1.04792249625837;
	U[4][5] =-0.04384123837724;

	U[5][0] =-0.04105860298285;
	U[5][1] =0.07350997499060;
	U[5][2] =0.01551023500714;
	U[5][3] =-0.05178010035371;
	U[5][4] =-0.19822234780872;
	U[5][5] =1.07788018287476;

	// Odwrocona macierz korekcji U.
	// inv_U - wiersz pierwszy.
	inv_U[0][0] =0.99895769229565;
	inv_U[0][1] =-0.00159353281977;
	inv_U[0][2] =-0.00173090753504;
	inv_U[0][3] =-0.00092590176691;
	inv_U[0][4] =-0.00247771494379;
	inv_U[0][5] =-0.00008189247676;

	inv_U[1][0] =0.00044271916106;
	inv_U[1][1] =1.00043599983882;
	inv_U[1][2] =0.00264758770625;
	inv_U[1][3] =-0.00193350095465;
	inv_U[1][4] =0.00455941934814;
	inv_U[1][5] =0.00166213409191;

	inv_U[2][0] =0.00056422860193;
	inv_U[2][1] =0.00068592299831;
	inv_U[2][2] =0.99867382333885;
	inv_U[2][3] =-0.00156185543253;
	inv_U[2][4] =0.00743828878575;
	inv_U[2][5] =0.00453486909415;

	inv_U[3][0] =0.02027373993958;
	inv_U[3][1] =-0.02459428546424;
	inv_U[3][2] =0.01203423546810;
	inv_U[3][3] =0.95996131197021;
	inv_U[3][4] =0.02264023941202;
	inv_U[3][5] =-0.02848309664881;

	inv_U[4][0] =-0.00109019404926;
	inv_U[4][1] =0.00268329143917;
	inv_U[4][2] =0.01117646079544;
	inv_U[4][3] =-0.00265009474478;
	inv_U[4][4] =0.96170373502841;
	inv_U[4][5] =0.03924773496285;

	inv_U[5][0] =0.03878741609764;
	inv_U[5][1] =-0.06898698000770;
	inv_U[5][2] =-0.01198352139018;
	inv_U[5][3] =0.04574713031121;
	inv_U[5][4] =0.17743272882975;
	inv_U[5][5] =0.93341454258182;

}; // end: set_kinematic_parameters
