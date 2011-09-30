// ------------------------------------------------------------------------
// Proces:		-
// Plik:			mathtr.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Klasy K_vector, Homog_matrix, Ft_v_vector
//				- deklaracja metod klas
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <cmath>
#include <cstdio>

#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace lib {

// ******************************************************************************************
//                                           definicje skladowych klasy Jacobian_matrix
//
//Sibi
// ******************************************************************************************

/* ------------------------------------------------------------------------
 Konstruktor klasy - zainicjowane wartosciami zerowymi
 ------------------------------------------------------------------------ */

Jacobian_matrix::Jacobian_matrix()
{
	for (int i = 0; i <= 5; ++i)
		for (int j = 0; j <= 5; ++j)
			matrix[i][j] = 0;
}// end Jacobian_matrix::Jacobian_matrix()


/* ------------------------------------------------------------------------
 Wyznaczenie transpozycji jakobianu
 ------------------------------------------------------------------------ */

void Jacobian_matrix::transpose()
{
	double tmp[6][6]; //Tymczasowa macierz z wynikami

	for (int a = 0; a <= 5; a++) {
		for (int b = 0; b <= 5; b++) {
			tmp[a][b] = matrix[a][b];
		}
	}

	for (int a = 0; a <= 5; a++) {
		for (int b = 0; b <= 5; b++) {
			matrix[a][b] = tmp[b][a];
		}
	}
}

/* ------------------------------------------------------------------------
 Przedefiniowanie operatora mnozenia dla macierz * wektor
 ------------------------------------------------------------------------ */

Xyz_Angle_Axis_vector Jacobian_matrix::operator*(const Xyz_Angle_Axis_vector & w) const
{
	Xyz_Angle_Axis_vector zwracany;

	for (int j = 0; j < 6; j++)
		for (int i = 0; i < 6; i++)
			zwracany[j] += matrix[j][i] * w[i];

	return zwracany;
}

void Jacobian_matrix::print()
{
	for (int i = 0; i < 6; i++)
		printf("% 7.3f, % 7.3f, % 7.3f, % 7.3f, % 7.3f, % 7.3f\n", matrix[i][0], matrix[i][1], matrix[i][2], matrix[i][3], matrix[i][4], matrix[i][5]);
}

void Jacobian_matrix::to_table(double array[6][6]) const
{
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
			array[i][j] = matrix[i][j];
}

Xyz_Angle_Axis_vector Jacobian_matrix::jacobian_inverse_gauss(const Xyz_Angle_Axis_vector & dist)
{
	//zmienne pomocnicze w eliminacji Gaussa
	double L, a;
	int p[6], k, d, i, j, s, tmp;
	const double eps = 1e-10;
	double w[6];

	Xyz_Angle_Axis_vector q;

	//Metoda Eliminacji Gaussa - rozklad LU jakobianu
	//Wektor permutacji - czesciowy wybor elementu podstawowego
	for (int i = 0; i < 6; i++) {
		p[i] = i;
		w[i] = dist[i];
	}

	//V=Jq -> J=PLU -> V=PLUq

	for (k = 0; k < 6; k++) {
		a = matrix[p[k]][k]; //wybor elementu podstawowego
		s = k;
		for (d = k + 1; d < 6; d++) {
			if (fabs(a) < fabs(matrix[p[d]][k])) {
				a = matrix[p[d]][k]; //wybierany max element kolumny
				s = d;
			}
		}
		if (s != k) { //modyfikacja wektora permutacji
			tmp = p[k];
			p[k] = p[s];
			p[s] = tmp;
		}
		for (i = k + 1; i < 6; i++) { //eliminacja Gaussa
			if (fabs(matrix[p[i]][k]) > eps) {
				L = matrix[p[i]][k] / a;
				for (j = k + 1; j < 6; j++) {
					matrix[p[i]][j] = matrix[p[i]][j] - L * matrix[p[k]][j];
				}
				w[p[i]] = w[p[i]] - L * w[p[k]];
				matrix[p[i]][k] = 0;
			}
		}
	}

	//Rozwiazanie ukladu r�wnan z macierza gorna trojkatna

	q[5] = (w[p[5]]) / matrix[p[5]][5];
	q[4] = (w[p[4]] - (q[5] * matrix[p[4]][5])) / matrix[p[4]][4];
	q[3] = (w[p[3]] - (q[5] * matrix[p[3]][5] + q[4] * matrix[p[3]][4])) / matrix[p[3]][3];
	q[2] = (w[p[2]] - (q[5] * matrix[p[2]][5] + q[4] * matrix[p[2]][4] + q[3] * matrix[p[2]][3])) / matrix[p[2]][2];
	q[1] = (w[p[1]] - (q[5] * matrix[p[1]][5] + q[4] * matrix[p[1]][4] + q[3] * matrix[p[1]][3] + q[2]
			* matrix[p[1]][2])) / matrix[p[1]][1];
	q[0] = (w[p[0]] - (q[5] * matrix[p[0]][5] + q[4] * matrix[p[0]][4] + q[3] * matrix[p[0]][3] + q[2]
			* matrix[p[0]][2] + q[1] * matrix[p[0]][1])) / matrix[p[0]][0];

	return q;
}

void Jacobian_matrix::irp6_6dof_equations(const Xyz_Angle_Axis_vector & w)
{
	const double s1 = sin(w[0]);
	const double c1 = cos(w[0]);
	const double s2 = sin(w[1]);
	const double c2 = cos(w[1]);
	const double s3 = sin(w[2]);
	const double c3 = cos(w[2]);
	const double s4 = sin(w[3]);
	const double c4 = cos(w[3]);
	const double s5 = sin(w[4]);
	const double c5 = cos(w[4]);
	//const double s6 = sin(w[5]);
	//const double c6 = cos(w[5]);

	const double a2 = 0.455;
	const double a3 = 0.67;
	const double d5 = 0.19;
	//const double d6 = 0.095;
	//const double d7 = 0.20;

	//Wyznaczenie wzoru dla jakobianu

	matrix[0][0] = 0;
	matrix[1][0] = 0;
	matrix[2][0] = 1;
	matrix[3][0] = -s1 * (c4 * d5 + c3 * a3 + c2 * a2);
	matrix[4][0] = c1 * (c4 * d5 + c3 * a3 + c2 * a2);
	matrix[5][0] = 0;

	matrix[0][1] = 0;
	matrix[1][1] = 0;
	matrix[2][1] = 0;
	matrix[3][1] = -c1 * a2 * s2;
	matrix[4][1] = -s1 * a2 * s2;
	matrix[5][1] = -a2 * c2;

	matrix[0][2] = 0;
	matrix[1][2] = 0;
	matrix[2][2] = 0;
	matrix[3][2] = -c1 * a3 * s3;
	matrix[4][2] = -s1 * a3 * s3;
	matrix[5][2] = -a3 * c3;

	matrix[0][3] = -s1;
	matrix[1][3] = c1;
	matrix[2][3] = 0;
	matrix[3][3] = -c1 * s4 * d5;
	matrix[4][3] = -s1 * s4 * d5;
	matrix[5][3] = -d5 * c4;

	matrix[0][4] = c1 * c4;
	matrix[1][4] = s1 * c4;
	matrix[2][4] = -s4;
	matrix[3][4] = 0;
	matrix[4][4] = 0;
	matrix[5][4] = 0;

	matrix[0][5] = c1 * s4 * s5 - s1 * c5;
	matrix[1][5] = s1 * s4 * s5 + c1 * c5;
	matrix[2][5] = c4 * s5;
	matrix[3][5] = 0;
	matrix[4][5] = 0;
	matrix[5][5] = 0;

}

double Jacobian_matrix::irp6_6dof_determinant(const Xyz_Angle_Axis_vector & w)
{
	//const double s1 = sin(w[0]);
	//const double c1 = cos(w[0]);
	const double s2 = sin(w[1]);
	const double c2 = cos(w[1]);
	const double s3 = sin(w[2]);
	const double c3 = cos(w[2]);
	//const double s4 = sin(w[3]);
	const double c4 = cos(w[3]);
	const double s5 = sin(w[4]);
	//const double c5 = cos(w[4]);
	//const double s6 = sin(w[5]);
	//const double c6 = cos(w[5]);

	const double a2 = 0.455;
	const double a3 = 0.67;
	const double d5 = 0.19;
	//const double d6 = 0.095;
	//const double d7 = 0.20;

	const double det = s5 * a2 * a3 * (a2 * c2 + d5 * c4 + a3 * c3) * (s2 * c3 - s3 * c2);

	return det;

}

void Jacobian_matrix::irp6_6dof_inverse_equations(const Xyz_Angle_Axis_vector & w)
{
	const double s1 = sin(w[0]);
	const double c1 = cos(w[0]);
	const double s2 = sin(w[1]);
	const double c2 = cos(w[1]);
	const double s3 = sin(w[2]);
	const double c3 = cos(w[2]);
	const double s4 = sin(w[3]);
	const double c4 = cos(w[3]);
	const double s5 = sin(w[4]);
	const double c5 = cos(w[4]);
	//const double s6 = sin(w[5]);
	//const double c6 = cos(w[5]);

	const double a2 = 0.455;
	const double a3 = 0.67;
	const double d5 = 0.19;
	//const double d6 = 0.095;
	//const double d7 = 0.20;

	//Wyznaczenie wzoru na odwrotnosc jakobianu

	matrix[0][0] = 0.0;
	matrix[0][1] = 0.0;
	matrix[0][2] = 0.0;
	matrix[0][3] = -s1 / (a2 * c2 + d5 * c4 + a3 * c3);
	matrix[0][4] = c1 / (a2 * c2 + d5 * c4 + a3 * c3);
	matrix[0][5] = 0.0;

	matrix[1][0] = (c1 * s3 * s4 * c5 * c4 - c1 * c3 * c5 + c1 * c3 * c5 * c4 * c4 + s3 * s1 * s5 * c4 - c3 * s4 * s1
			* s5) * d5 / (s3 * c2 - s2 * c3) / a2 / s5;
	matrix[1][1] = -(c1 * s3 * s5 * c4 - c1 * c3 * s4 * s5 - s1 * s3 * s4 * c5 * c4 + c3 * s1 * c5 - c3 * s1 * c5 * c4
			* c4) * d5 / (s3 * c2 - s2 * c3) / a2 / s5;
	matrix[1][2] = (s3 * c4 - c3 * s4) * c4 * d5 * c5 / (s3 * c2 - s2 * c3) / a2 / s5;
	matrix[1][3] = (c1 * c4 * d5 * c3 * s5 + c1 * c3 * c3 * a3 * s5 + c1 * c2 * a2 * c3 * s5 - c3 * s1 * c4 * d5 * s4
			* c5 + s1 * s3 * c4 * c4 * d5 * c5) / (a2 * c2 + d5 * c4 + a3 * c3) / (s3 * c2 - s2 * c3) / a2 / s5;
	matrix[1][4] = -(c1 * s3 * c4 * c4 * d5 * c5 - c1 * c3 * c4 * d5 * s4 * c5 - c2 * a2 * s1 * c3 * s5 - c4 * d5 * s1
			* c3 * s5 - c3 * c3 * a3 * s1 * s5) / (a2 * c2 + d5 * c4 + a3 * c3) / (s3 * c2 - s2 * c3) / a2 / s5;
	matrix[1][5] = -s3 / a2 / (s3 * c2 - s2 * c3);

	matrix[2][0] = (c1 * c2 * c5 - c1 * c2 * c5 * c4 * c4 - c1 * s2 * c4 * s4 * c5 + c2 * s4 * s1 * s5 - s2 * c4 * s1
			* s5) * d5 / (s3 * c2 - s2 * c3) / a3 / s5;
	matrix[2][1] = -(c1 * c2 * s4 * s5 - c1 * s2 * c4 * s5 - c2 * s1 * c5 + c2 * s1 * c5 * c4 * c4 + s1 * s2 * c4 * s4
			* c5) * d5 / (s3 * c2 - s2 * c3) / a3 / s5;
	matrix[2][2] = (c2 * s4 - s2 * c4) * c4 * d5 * c5 / (s3 * c2 - s2 * c3) / a3 / s5;
	matrix[2][3] = -(c1 * c4 * d5 * c2 * s5 + c1 * c3 * a3 * c2 * s5 + c1 * c2 * c2 * a2 * s5 - c2 * s1 * c4 * d5 * s4
			* c5 + s1 * s2 * c4 * c4 * d5 * c5) / (a2 * c2 + d5 * c4 + a3 * c3) / (s3 * c2 - s2 * c3) / a3 / s5;
	matrix[2][4] = -(-c1 * s2 * c4 * c4 * d5 * c5 + c1 * c2 * s4 * d5 * c4 * c5 + s1 * c2 * c2 * a2 * s5 + c2 * s1 * s5
			* c4 * d5 + s1 * c3 * a3 * c2 * s5) / (a2 * c2 + d5 * c4 + a3 * c3) / (s3 * c2 - s2 * c3) / a3 / s5;
	matrix[2][5] = 1 / (s3 * c2 - s2 * c3) / a3 * s2;

	matrix[3][0] = -(s1 * s5 + s4 * c1 * c5) / s5;
	matrix[3][1] = (c1 * s5 - s4 * s1 * c5) / s5;
	matrix[3][2] = -c4 * c5 / s5;
	matrix[3][3] = -s1 * c4 * c5 / (a2 * c2 + d5 * c4 + a3 * c3) / s5;
	matrix[3][4] = c1 * c4 * c5 / (a2 * c2 + d5 * c4 + a3 * c3) / s5;
	matrix[3][5] = 0.0;

	matrix[4][0] = c1 * c4;
	matrix[4][1] = s1 * c4;
	matrix[4][2] = -s4;
	matrix[4][3] = -s4 * s1 / (a2 * c2 + d5 * c4 + a3 * c3);
	matrix[4][4] = c1 * s4 / (a2 * c2 + d5 * c4 + a3 * c3);
	matrix[4][5] = 0.0;

	matrix[5][0] = c1 * s4 / s5;
	matrix[5][1] = s4 * s1 / s5;
	matrix[5][2] = c4 / s5;
	matrix[5][3] = c4 * s1 / (a2 * c2 + d5 * c4 + a3 * c3) / s5;
	matrix[5][4] = -c4 * c1 / (a2 * c2 + d5 * c4 + a3 * c3) / s5;
	matrix[5][5] = 0.0;
}


} // namespace lib
} // namespace mrrocpp

