#include "lib/matrix.h"

// --------------------------------------------------------
// Konstruktor bezparametrowy klasy matrix
// --------------------------------------------------------
matrix::matrix()
{
	rows = 0;
	columns = 0;

	ptr = new double *[rows];
	for (int i = 0; i < rows; i++)
		ptr[i] = new double[columns];
}
// --------------------------------------------------------
// Konstruktor bezparametrowy klasy matrix
// --------------------------------------------------------
matrix::matrix(int r, int c)
{
	ptr = new double *[rows];
	for (int i = 0; i < rows; i++)
		ptr[i] = new double[columns];
}
// --------------------------------------------------------
// konstruktor kopiujacy klasy matrix
// --------------------------------------------------------
matrix::matrix(const matrix & srcMatrix)
{
	int i, j;
	ptr = new double *[rows];
	for (i = 0; i < rows; i++)
		ptr[i] = new double[columns];
	for (i = 0; i < rows; i++)
		for (j = 0; j < columns; j++)
			ptr[i][j] = srcMatrix.ptr[i][j];
}
// --------------------------------------------------------
// destruktor klasy matrix
// --------------------------------------------------------
matrix::~matrix()
{
	for (int i = 0; i < rows; i++)
		delete ptr[i];
}
// --------------------------------------------------------
// operator dostepu do elementow macierzy
// --------------------------------------------------------
double& matrix::operator()(int col, int row)
{
	return ptr[col][row];
}
// --------------------------------------------------------
