#include "vsp/rcs/Kociemba/KociembaMoveTable.h"

int KociembaMoveTable::GetCoordSize(int coordNum)
    throw (KociembaException)
{
    switch (coordNum)
    {
        case 0: return KociembaCube::CORNER_ORIENTATION_NUMBER;
        case 1: return KociembaCube::EDGE_SLICE_COMBINATION_NUMBER;
        case 2: return KociembaCube::EDGE_ORIENTATION_NUMBER;
        case 3: return KociembaCube::CORNER_PERMUTATION_NUMBER;
        case 4: return KociembaCube::EDGE_SLICE_PERMUTATION_NUMBER;
        case 5: return KociembaCube::EDGE_TWO_SLICES_PERMUTATION_NUMBER;
    }
    
    // invalid argument
    throw KociembaException(KociembaException::ERR_MOVETABLE_COORD_INVALID);
}
