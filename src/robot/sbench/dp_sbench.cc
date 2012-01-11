/*!
 * @file
 * @brief File contains dp_sbench class definition for SwarmItFix bench
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sbench
 */

#include <cmath>
#include <cstring>
#include <ostream>

#include "dp_sbench.h"

namespace mrrocpp {
namespace lib {
namespace sbench {

// translation table of bench docks to io card ports
static int voltage_translation_table[8][8] =
		{ { 0, 1, 2, 3, 4, 5, 6, 7 },
		//
		{ 8, 9, 10, 11, 12, 13, 14, 15 },
		//
		{ 16, 17, 18, 19, 20, 21, 22, 23 }, { 24, 25, 26, 27, 28, 29, 30, 31 }, { 32, 33, 34, 35, 36, 37, 38, 39 }, { 40, 41, 42, 43, 44, 45, 46, 47 }, { 48, 49, 50, 51, 52, 53, 54, 55 }, { 56, 57, 58, 59, 60, 61, 62, 63 } };

static int preasure_translation_table[8][8] = {
// bench raw 1
{ 55, 54, 53, 52, 51, 50, 49, 61 },
//  bench raw  2
{ 48, 47, 46, 45, 44, 43, 42, 41 },
//  bench raw  3
{ 40, 39, 38, 37, 36, 35, 34, 62 },
//  bench raw  4
{ 33, 32, 31, 30, 29, 28, 27, 26 },
//  bench raw  5
{ 25, 24, 23, 22, 21, 20, 19, 63 },
//  bench raw  6
{ 18, 17, 16, 15, 14, 13, 12, 11 },
//  bench raw  7
{ 10, 9, 8, 7, 6, 5, 4, 3 },
// not used
{ 2, 1, 0, 56, 57, 58, 59, 60 } };

pins_buffer::pins_buffer()
{

}

void pins_buffer::set_zeros()
{
	for (int i = 0; i < lib::sbench::NUM_OF_PINS; i++) {

		pins_state[i] = 0;
	}
}

void pins_buffer::set_value(int row, int column, bool value)
{
	pins_state[translation_table[row][column]] = value;
}

bool pins_buffer::get_value(int row, int column)
{
	return pins_state[translation_table[row][column]];
}

bool pins_buffer::is_any_doubled_value()
{
// doubled value check
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {

			for (int k = 0; k < 8; k++) {
				for (int g = 0; g < 8; g++) {
					if ((!((k == i) && (j == g))) && (translation_table[k][g] == translation_table[i][j])) {
						std::cout << "  pins_buffer wrong value:   " << translation_table[k][g] << " \n\n\n";
						return true;
					}
				}
			}

		}
	}
	return false;
}

voltage_buffer::voltage_buffer() :
		pins_buffer()
{
// doubled value check
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			translation_table[i][j] = voltage_translation_table[i][j];
		}
	}

	if (is_any_doubled_value()) {
		std::cout << "multiply value at voltage_translation_table dp_sbench\n\n\n";
	}
}

preasure_buffer::preasure_buffer() :
		pins_buffer()
{
// doubled value check
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			translation_table[i][j] = preasure_translation_table[i][j];
		}
	}
	if (is_any_doubled_value()) {
		std::cout << "multiply value at preasure_translation_table dp_sbench\n\n\n";
	}

}

} // namespace robot
} // namespace mp
} // namespace mrrocpp

