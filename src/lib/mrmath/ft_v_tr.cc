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

#include <math.h>
#include <stdio.h>
#include <iostream>

#include "lib/mis_fun.h"
#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace lib {


// ******************************************************************************************
//                                           definicje skladowych klasy Ft_v_tr
// ******************************************************************************************

Ft_v_tr::Ft_v_tr()
{
	// konstruktor domniemany
	// tworzy macierz jednostkowa

	// i - i-ta kolumna
	// j - j-ty wiersz


	for(int i=0; i<6; i++)
		for(int j=0; j<6; j++)
		{
			if(i == j)
				matrix_m[i][j] = 1;
			else
				matrix_m[i][j] = 0;
		}

	variant = NOT_SET;


}// end Ft_v_tr()

/*
Ft_v_tr::Ft_v_tr(VARIANT variant_l)
{
	// konstruktor domniemany
	// tworzy macierz jednostkowa

	// i - i-ta kolumna
	// j - j-ty wiersz


	for(int i=0; i<6; i++)
		for(int j=0; j<6; j++)
		{
			if(i == j)
				matrix_m[i][j] = 1;
			else
				matrix_m[i][j] = 0;
		}

	variant = variant_l;


}// end Ft_v_tr()
*/

Ft_v_tr::Ft_v_tr(const Homog_matrix & p, VARIANT variant_l)
{
	// macierz tworzona jest zgodnie ze wzorem 5.105 ze strony 196 (5.72 str 154 - wydanie angielskie)
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	// i - i-ta kolumna
	// j - j-ty wiersz

	variant = variant_l;

	base_frame = p;

	set_from_frame (base_frame);

}// end Ft_v_tr::Ft_v_tr(const Homog_matrix & p)

// konstruktor kopiujacy
// jest on uzywany podczas inicjalizacji obiektu w momencie jego tworzenia (np. Homog_matrix B = A;)
Ft_v_tr::Ft_v_tr(const Ft_v_tr &wzor)
{
	variant = wzor.variant;

	base_frame = wzor.base_frame;

	set_from_frame(wzor.base_frame);
}

void Ft_v_tr::set_from_frame(const Homog_matrix & p)
{
	// macierz tworzona jest zgodnie ze wzorem 5.105 ze strony 196 (5.70, 5.72 str 154 - wydanie angielskie)
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	// i - i-ta kolumna
	// j - j-ty wiersz

	if (variant == NOT_SET) printf("BLAD Ft_v_tr::set_from_frame - FT_V_VARIANT_NOT_SET\n");

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
			matrix_m[i][j] = p.matrix_m[i][j];
			matrix_m[i+3][j+3] = p.matrix_m[i][j];

			switch (variant)
			{
				case FT:
					matrix_m[j][i+3] = 0;
				break;
				case V:
					matrix_m[j+3][i] = 0;
				break;
				default:
				break;
			}
		}

	double Porg[3][3];

	Porg[0][0] = 0;
	Porg[0][1] = -p.matrix_m[2][3];
	Porg[0][2] = p.matrix_m[1][3];

	Porg[1][0] = p.matrix_m[2][3];
	Porg[1][1] = 0;
	Porg[1][2] = -p.matrix_m[0][3];

	Porg[2][0] = -p.matrix_m[1][3];
	Porg[2][1] = p.matrix_m[0][3];
	Porg[2][2] = 0;

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
			switch (variant)
			{
				case FT:
					matrix_m[j+3][i] = 0;
					for(int a=0; a<3; a++)
					{
						matrix_m[j+3][i] += Porg[j][a] * p.matrix_m[a][i];
					}
				break;
				case V:
					matrix_m[j][i+3] = 0;
					for(int a=0; a<3; a++)
					{
						matrix_m[j][i+3] += Porg[j][a] * p.matrix_m[a][i];
					}
				break;
				default:
				break;
			}
		}

}// end Ft_v_tr::Ft_v_tr(const Homog_matrix & p)



Ft_v_tr Ft_v_tr::operator* (const Ft_v_tr & m) const
{
// mnozenie macierzy

//	Ft_v_tr zwracana;

	// REMOVED BECAUSE IT IS PROBABLY NOT CORRECT TO MULTIPLY IN SUCH A WAY
	/*
	int i, j;			// i - i-ta kolumna
					// j - j-ty wiersz
	// macierz rotacji
	for(i=0; i<6; i++)
	{
		for(j=0; j<6; j++)
		{
			for(int a=0; a<6; a++)
			{
				if(a == 0)
					zwracana.matrix[i][j] = matrix[a][j] * m.matrix[i][a];
				else
					zwracana.matrix[i][j] += matrix[a][j] * m.matrix[i][a];

			}
		}
	}
*/

Ft_v_tr zwracana(base_frame * m.base_frame, variant) ;

return zwracana;
}// end Homog_matrix::operator* (const Homog_matrix & m) const


Ft_v_vector Ft_v_tr::operator*(const Ft_v_vector & w) const
{
	Ft_v_vector zwracany;
	int i;
	int j;
	// i - i-ta kolumna
	// j - j-ty wiersz

	for(j=0;j<6;j++)
		for(i=0;i<6;i++)
			zwracany.w[j] += matrix_m[j][i] * w.w[i];
//	std::cout << "zwracany " << zwracany <<std::endl;
//	std::cout << "w " << zwracany <<std::endl;
return zwracany;
}// Ft_v_tr::operator*(const Ft_v_vector & w) const


Ft_v_tr & Ft_v_tr::operator=(const Ft_v_tr & wzor)
{
	// operator przypisania
	// parametry macierzy przyjmuja wartosc jak parametry macierzy podanej jako argumet

	if(this == &wzor) return *this;

	// do zamiany na mem_cpy

	memcpy(matrix_m, wzor.matrix_m, sizeof(matrix_m));
	/*
	for(int i=0; i<6; i++)
		for(int j=0; j<6; j++)
			matrix[i][j] = wzor.matrix[i][j];
*/

	variant = wzor.variant;

	base_frame = wzor.base_frame;

return *this;
}// end Ft_v_tr::operator=(const Ft_v_tr & wzor)


Ft_v_tr Ft_v_tr::operator!() const
{
	// przeksztalcenie odwrotne

	Ft_v_tr zwracana;

	zwracana.variant = variant;
	zwracana.base_frame = !base_frame;

	zwracana.set_from_frame(zwracana.base_frame);

return zwracana;

}// end Ft_v_tr::operator!()


std::ostream&  operator<<(std::ostream & strumien, Ft_v_tr & m)
{
	// operator wypisania
	// przedstawia macierz w przyjaznej dla czlowieka formie

	for(int j=0; j<6; j++)
	{
		for(int i=0; i<6; i++)
		{
			strumien << m.matrix_m[j][i] << "\t\t";
		}
		strumien << std::endl;
	}

return strumien;
}// end operator<<(std::ostream & strumien, Ft_v_tr & m)

// ******************************************************************************************
//                                           definicje skladowych klasy Jacobian_matrix
//
//Sibi
// ******************************************************************************************


} // namespace lib
} // namespace mrrocpp

