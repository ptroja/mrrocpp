/////////////////////////////////////////////////////////
// Name: KociembaCube.cpp
// Implements : KociembaCube
/////////////////////////////////////////////////////////

#include "vsp/rcs/Kociemba/KociembaCube.h"

#include "vsp/rcs/Combinatorials/Permutation.h"
#include "vsp/rcs/Combinatorials/Variation.h"
#include "vsp/rcs/Combinatorials/Combination.h"


const int KociembaCube::CORNER_PERMUTATION_NUMBER = Permutation::NUMBER(CubieCube::CORNER_NUMBER);
const int KociembaCube::CORNER_ORIENTATION_NUMBER =   Variation::NUMBER(CubieCube::CORNER_NUMBER - 1, CubieCube::CORNER_TWIST_NUMBER);
const int KociembaCube::EDGE_SLICE_COMBINATION_NUMBER      = Combination::NUMBER(EDGE_SLICE_NUMBER, CubieCube::EDGE_NUMBER);  
const int KociembaCube::EDGE_SLICE_PERMUTATION_NUMBER      = Permutation::NUMBER(EDGE_SLICE_NUMBER);
const int KociembaCube::EDGE_TWO_SLICES_PERMUTATION_NUMBER = Permutation::NUMBER(2*EDGE_SLICE_NUMBER);
const int KociembaCube::EDGE_ORIENTATION_NUMBER            =   Variation::NUMBER(CubieCube::EDGE_NUMBER - 1, CubieCube::EDGE_FLIP_NUMBER);

const char *KociembaCube::TYPE = "KociembaCube";


////////////////////////////////////////////////////////////////////////////////
// TRANSFORM STATIC PROTECTED METHODS
////////////////////////////////////////////////////////////////////////////////

void KociembaCube::InitCoords(int coords[])
{
    coords[0] = 0;
    coords[1] = EDGE_SLICE_COMBINATION_NUMBER - 1;
    coords[2] = 0;
    coords[3] = 0;
    coords[4] = 0;
    coords[5] = 0;
}

void KociembaCube::CubieCubeToCoords(CubieCube& cube, int coords[])
{
    int i, sm, snm, off;
    int state[2*CubieCube::CORNER_NUMBER + 2*CubieCube::EDGE_NUMBER];
    int eTab[16];
     
    // get cubie cube's state
    cube.ToTable(state);
     
     // split edge parameters into middle and non-middle slices
    off = 2*CubieCube::CORNER_NUMBER;
    for (i=0, sm=0, snm=0; i<CubieCube::EDGE_NUMBER; i++)
    {
        // non-middle slice 
        if (state[i+off] < 2*EDGE_SLICE_NUMBER)
        {
            eTab[2*EDGE_SLICE_NUMBER+snm] = state[i+off];
            snm++;
        }
        // middle slice
        else 
        {
            eTab[sm] = i;
            eTab[EDGE_SLICE_NUMBER+sm] = state[i+off] - 2*EDGE_SLICE_NUMBER;
            sm++;
        }
    } 

    // create combinatorials 
    Permutation cornerPermutation(CubieCube::CORNER_NUMBER);
    Variation cornerOrientation(CubieCube::CORNER_NUMBER - 1, CubieCube::CORNER_TWIST_NUMBER);
    Combination edgeMidSliceCombination(EDGE_SLICE_NUMBER, CubieCube::EDGE_NUMBER);
    Permutation edgeMidSlicePermutation(EDGE_SLICE_NUMBER);
    Permutation edgeNonMidSlicePermutation(2*EDGE_SLICE_NUMBER);
    Variation edgeOrientation(CubieCube::EDGE_NUMBER - 1, CubieCube::EDGE_FLIP_NUMBER);

    // set combinatorials values from tables
    cornerPermutation.FromTable(state);
    cornerOrientation.FromTable(&state[CubieCube::CORNER_NUMBER]);
    edgeMidSliceCombination.FromTable(eTab);
    edgeMidSlicePermutation.FromTable(&eTab[EDGE_SLICE_NUMBER]);
    edgeNonMidSlicePermutation.FromTable(&eTab[2*EDGE_SLICE_NUMBER]);
    edgeOrientation.FromTable(&state[2*CubieCube::CORNER_NUMBER + CubieCube::EDGE_NUMBER]);

    // set coordinals from combinatorials
    coords[3] = cornerPermutation.ToOrdinal();
    coords[0] = cornerOrientation.ToOrdinal();
    coords[1] = edgeMidSliceCombination.ToOrdinal();
    coords[4] = edgeMidSlicePermutation.ToOrdinal();
    coords[5] = edgeNonMidSlicePermutation.ToOrdinal();
    coords[2] = edgeOrientation.ToOrdinal();
}

void KociembaCube::CoordsToCubieCube(CubieCube& cube, int coords[])
     throw (CubeException)
{
    int i, sm, snm, ori;
    int state[2*CubieCube::CORNER_NUMBER + 2*CubieCube::EDGE_NUMBER];
    int eTab[16];

    // create combinatorials 
    Permutation cornerPermutation(CubieCube::CORNER_NUMBER);
    Variation cornerOrientation(CubieCube::CORNER_NUMBER - 1, CubieCube::CORNER_TWIST_NUMBER);
    Combination edgeMidSliceCombination(EDGE_SLICE_NUMBER, CubieCube::EDGE_NUMBER);
    Permutation edgeMidSlicePermutation(EDGE_SLICE_NUMBER);
    Permutation edgeNonMidSlicePermutation(2*EDGE_SLICE_NUMBER);
    Variation edgeOrientation(CubieCube::EDGE_NUMBER - 1, CubieCube::EDGE_FLIP_NUMBER);
    
    // set combinatorials values from coords
    cornerPermutation.FromOrdinal(coords[3]);
    cornerOrientation.FromOrdinal(coords[0]);
    edgeMidSliceCombination.FromOrdinal(coords[1]);
    edgeMidSlicePermutation.FromOrdinal(coords[4]);
    edgeNonMidSlicePermutation.FromOrdinal(coords[5]);
    edgeOrientation.FromOrdinal(coords[2]);

    // get edge orientation and edge-slices combination and permutations in dedicated table
    edgeMidSliceCombination.ToTable(eTab);
    edgeMidSlicePermutation.ToTable(&eTab[EDGE_SLICE_NUMBER]);
    edgeNonMidSlicePermutation.ToTable(&eTab[2*EDGE_SLICE_NUMBER]);
     
    // copy corner perm and orie and edge orie to state table
    cornerPermutation.ToTable(state);
    cornerOrientation.ToTable(&state[CubieCube::CORNER_NUMBER]);
    edgeOrientation.ToTable(&state[2*CubieCube::CORNER_NUMBER + CubieCube::EDGE_NUMBER]);

    // set orientation of last corner
    for (i=0, ori=0; i<CubieCube::CORNER_NUMBER - 1; i++)
        ori += state[CubieCube::CORNER_NUMBER + i];
    state[2*CubieCube::CORNER_NUMBER - 1] = (CubieCube::TWIST_NUMBER - ori % CubieCube::TWIST_NUMBER) % CubieCube::TWIST_NUMBER;

    // set orientation of last edge
    for (i=0, ori=0; i<CubieCube::EDGE_NUMBER - 1; i++)
        ori += state[2*CubieCube::CORNER_NUMBER + CubieCube::EDGE_NUMBER + i];
    state[2*CubieCube::CORNER_NUMBER + 2 * CubieCube::EDGE_NUMBER - 1] = ori % CubieCube::FLIP_NUMBER;

    // join edge-slices into edge permutation
    for (i=0, sm=0, snm=0; i<CubieCube::EDGE_NUMBER; i++)
    {
        // edge from middle-slice
        if (eTab[sm] == i)
        {
            state[2*CubieCube::CORNER_NUMBER+i] = eTab[EDGE_SLICE_NUMBER+sm] + 2*EDGE_SLICE_NUMBER;;
            sm++;
        }
        else
        {
            state[2*CubieCube::CORNER_NUMBER+i] = eTab[2*EDGE_SLICE_NUMBER+snm];
            snm++;
        }
    }
    
    try
    {
        // set cubie cube's state
        cube.FromTable(state);
    }
    catch (CubeException& exp)
    {
        throw;
    }
}
