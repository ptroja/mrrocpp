// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			kinematic_model_calibrated_correction_matrix_irp6ot_with_wrist.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Model kinematyki robota IRp-6 na torze
//				- definicja metod klasy
//				- wykorzystanie nowego stopnia swobody  jako czynnego stopnia swobody
//				- tor jest biernym stopniem swobody
//				- parametry obliczone zostaly podczas kalibracji
//
// Autor:		tkornuta
// Data:		24.02.2007
// ------------------------------------------------------------------------

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mathtr.h"

// Klasy bledow, itp.
#include "kinematics/common/transformer_error.h"

// Klasa kinematic_model_calibrated_irp6ot_with_wrist.
#include "kinematics/irp6_on_track/kinematic_model_calibrated_correction_matrix_irp6ot_with_wrist.h"


/* -----------------------------------------------------------------------
  Konstruktor.
 ------------------------------------------------------------------------- */
kinematic_model_calibrated_correction_matrix_irp6ot_with_wrist::kinematic_model_calibrated_correction_matrix_irp6ot_with_wrist (void)
{
  // Ustawienie etykiety modelu kinematycznego.
  set_kinematic_model_label("Switching to calibrated and locally corrected kinematic model with active wrist");

  // Ustawienie parametrow kinematycznych.
  set_kinematic_parameters();

}; //: kinematic_model_calibrated_correction_matrix_irp6ot_with_wrist

/* -----------------------------------------------------------------------
  Ustawienia wszystkie parametry modelu kinematycznego danego modelu.
 ------------------------------------------------------------------------- */
void kinematic_model_calibrated_correction_matrix_irp6ot_with_wrist::set_kinematic_parameters(void)
{
	// Uzycie przeliczen zwiazanych z lokalnym korektorem.
	local_corrector_computations = true;

	// Wspolczynnik.
	h = 300.0;

	// Wektor korekcji V.
	V[0] =83.824567500917;;
	V[1] =11.417550314916;;
	V[2] =11.012091765384;
	V[3] =-16.908942178416;
	V[4] =23.003338261798;
	V[5] =-104.994144497905;
	
	// Macierz U.
	// U - wiersz pierwszy.
	U[0][0] =0.98772609866074;
	U[0][1] =-0.04418110123766;
	U[0][2] =0.00259317623899;
	U[0][3] =-0.00236506539411;
	U[0][4] =-0.01731106183985;
	U[0][5] =0.03862431295963;
	// U - wiersz drugi.
	U[1][0] =-0.02358646932205;
	U[1][1] =0.98595223027733;
	U[1][2] =-0.00432826058203;
	U[1][3] =0.00242742096276;
	U[1][4] =0.02460642183860;
	U[1][5] =0.01222252511252;
	// U - wiersz trzeci.
	U[2][0] =-0.00403441234531;
	U[2][1] =-0.00106429052282;
	U[2][2] =1.00122435299502;
	U[2][3] =0.00545655318476;
	U[2][4] =-0.00532471346011;
	U[2][5] =0.01369920615647;
	// U - wiersz czwarty.
	U[3][0] =0.01521873738518;
	U[3][1] =0.00340414102429;
	U[3][2] =-0.01394458919130;
	U[3][3] =0.97739184665380;
	U[3][4] =0.03617982001668;
	U[3][5] =-0.01471864736754;
	// U - wiersz piaty.
	U[4][0] =-0.00880942548026;
	U[4][1] =0.00058072172737;
	U[4][2] =-0.00637015531500;
	U[4][3] =0.01118236461340;
	U[4][4] =0.97524468477232;
	U[4][5] =0.02271706370894;
	// U - wiersz szosty.
	U[5][0] =0.02564137561644;
	U[5][1] =0.06621702930948;
	U[5][2] =0.00221550345528;
	U[5][3] =0.00283554308545;
	U[5][4] =0.02369229639424;
	U[5][5] =0.96776559310888;
	
	// Odwrocona macierz korekcji U.
	// inv_U - wiersz pierwszy.
	inv_U[0][0] =1.01476867442650;
	inv_U[0][1] =0.04823622950840;
	inv_U[0][2] =-0.00218377913600;
	inv_U[0][3] =0.00226559826816;
	inv_U[0][4] =0.01770681932610;
	inv_U[0][5] =-0.04145972540771;
	// inv_U - wiersz drugi.
	inv_U[1][0] =0.02444315765362;
	inv_U[1][1] =1.01626444805704;
	inv_U[1][2] =0.00417161633155;
	inv_U[1][3] =-0.00216594299111;
	inv_U[1][4] =-0.02478077195007;
	inv_U[1][5] =-0.01332089010826;
	// inv_U - wiersz trzeci.
	inv_U[2][0] =0.00465258695959;
	inv_U[2][1] =0.00228154073292;
	inv_U[2][2] =0.99876769316795;
	inv_U[2][3] =-0.00559710457356;
	inv_U[2][4] =0.00603999778458;
	inv_U[2][5] =-0.01457946698898;
	// inv_U - wiersz czwarty.
	inv_U[3][0] =-0.01662444061761;
	inv_U[3][1] =-0.00538250439346;
	inv_U[3][2] =0.01399113014209;
	inv_U[3][3] =1.02341686736348;
	inv_U[3][4] =-0.03846284219804;
	inv_U[3][5] =0.01700133064313;
	// inv_U - wiersz piaty.
	inv_U[4][0] =0.01004299216942;
	inv_U[4][1] =0.00155734302831;
	inv_U[4][2] =0.00640433723291;
	inv_U[4][3] =-0.01168869109039;
	inv_U[4][4] =1.02659343268679;
	inv_U[4][5] =-0.02478689362342;
	// inv_U - wiersz szosty.
	inv_U[5][0] =-0.02876701577508;
	inv_U[5][1] =-0.07084106521390;
	inv_U[5][2] =-0.00271183036677;
	inv_U[5][3] =-0.00261145988982;
	inv_U[5][4] =-0.02380720366601;
	inv_U[5][5] =1.03590839825862;

}; // end: set_kinematic_parameters


