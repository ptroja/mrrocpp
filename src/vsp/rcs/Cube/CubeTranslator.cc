/////////////////////////////////////////////////////////
// Name: CubeTranslator.h
// Implements: CubeTranslator
/////////////////////////////////////////////////////////

#include "vsp/rcs/Cube/CubeTranslator.h"
#include "vsp/rcs/Cube/FaceletCube.h"
#include "vsp/rcs/Cube/CubieCube.h"
#include "vsp/rcs/Combinatorials/Combination.h"

#include <string>


const int CubeTranslator::CORNER_MAP[CubieCube::CORNER_NUMBER][CubieCube::TWIST_NUMBER] = {  { 26,  9,  2 }, { 24,  0, 38 }, { 18, 36, 29 }, { 20, 27, 11 }, { 47,  8, 15 }, { 45, 44,  6 }, { 51, 35, 42 }, { 53, 17, 33 } };
const int CubeTranslator::EDGE_MAP[CubieCube::EDGE_NUMBER][CubieCube::FLIP_NUMBER] = { { 23, 10 }, { 25,  1 }, { 21, 37 }, { 19, 28 }, { 50, 16 }, { 46,  7 }, { 48, 43 }, { 52, 34 }, {  5, 12 }, {  3, 41 }, { 32, 39 }, { 30, 14 } };
  

void CubeTranslator::FaceletToCubie(FaceletCube& fCube, CubieCube& cCube)
{
    int i,j,k;
    bool found;
    int cState[2*CubieCube::CORNER_NUMBER + 2*CubieCube::EDGE_NUMBER];
    int fState[6*9];

    // Get state of FaceletCube
    fCube.ToTable(fState);

    // Set corners
    for (i=0; i<CubieCube::CORNER_NUMBER; i++)
    {
        // - CORNER_MAP[i] - facelets of i-th corner position
        // - fState[CORNER_MAP[i][x]] - facelets on i-th corner position
        // scan all CORNER_MAP -> find corner (j) and twist (k)
        // - cState[i] = j - corner permutation
        // - cState[CubieCube::CORNER_NUMBER+i] = k - corner orientation
        found = false;
        for (j=0; j<CubieCube::CORNER_NUMBER; j++)
            for (k=0; k<CubieCube::TWIST_NUMBER; k++)
            {
                if (fState[CORNER_MAP[i][0]] == CORNER_MAP[j][k] / FaceletCube::FACELET_IN_FACE_NUMBER
                 && fState[CORNER_MAP[i][1]] == CORNER_MAP[j][(k+1)%CubieCube::TWIST_NUMBER] / FaceletCube::FACELET_IN_FACE_NUMBER
                 && fState[CORNER_MAP[i][2]] == CORNER_MAP[j][(k+2)%CubieCube::TWIST_NUMBER] / FaceletCube::FACELET_IN_FACE_NUMBER)
                {
                    cState[i] = j;
                    cState[CubieCube::CORNER_NUMBER+i] = k;
                    found = true;
                    
                    // break loop
                    k = CubieCube::TWIST_NUMBER;
                    j = CubieCube::CORNER_NUMBER;
                }
            }
        
        if (!found)
            throw CubeException(NULL, NULL, i, 
                  CubeException::ERR_FCUBE_INVALID_CORNER_MARKINGS);
    }
        
    // Set edge facelets in fState
    for (i=0; i<CubieCube::EDGE_NUMBER; i++)
    {
        // - EDGE_MAP[i] - facelets of i-th edge position
        // - fState[EDGE_MAP[i][x]] - facelets on i-th edge position
        // scan all EDGE_MAP -> find edge (j) and flip (k)
        // - cState[2*CubieCube::CORNER_NUMBER+j] = i - edge permutation
        // - cState[2*CubieCube::CORNER_NUMBER+CubieCube::EDGE_NUMBER+j] = k - edge orientation
        found = false;
        for (j=0; j<CubieCube::EDGE_NUMBER; j++)
            for (k=0; k<CubieCube::FLIP_NUMBER; k++)
            {
                if (fState[EDGE_MAP[i][0]] == EDGE_MAP[j][k] / FaceletCube::FACELET_IN_FACE_NUMBER
                 && fState[EDGE_MAP[i][1]] == EDGE_MAP[j][(k+1)%CubieCube::FLIP_NUMBER] / FaceletCube::FACELET_IN_FACE_NUMBER)
                {
                    cState[2*CubieCube::CORNER_NUMBER+i] = j;
                    cState[2*CubieCube::CORNER_NUMBER+CubieCube::EDGE_NUMBER+i] = k;
                    found = true;
                    
                    // break loop
                    k = CubieCube::FLIP_NUMBER;
                    j = CubieCube::EDGE_NUMBER;
                }
            }
        
        if (!found)
            throw CubeException(NULL, NULL, i, 
                  CubeException::ERR_FCUBE_INVALID_EDGE_MARKINGS);
    }

    cCube.FromTable(cState);
}

void CubeTranslator::CubieToFacelet(FaceletCube& fCube, CubieCube& cCube)
{
    int i;
    int cState[2*CubieCube::CORNER_NUMBER + 2*CubieCube::EDGE_NUMBER];
    int fState[6*9];
   
    // Get state of CubieCube
    cCube.ToTable(cState);
    
    // Set center facelets in fState
    for (i=0; i<FaceletCube::FACE_NUMBER; i++)
        fState[i*FaceletCube::FACELET_IN_FACE_NUMBER + 4] = i;
    
    // Set corners
    for (i=0; i<CubieCube::CORNER_NUMBER; i++)
    {
        // - cState[i] - corner permutation
        // - cState[CubieCube::CORNER_NUMBER+i] - corner orientation
        // - CORNER_MAP[i] - facelets of i-th corner position
        // - CORNER_MAP[cState[i]] - facelets placed in i-th corner position
        fState[CORNER_MAP[i][0]] = CORNER_MAP[cState[i]][(cState[CubieCube::CORNER_NUMBER+i]+0)%CubieCube::TWIST_NUMBER] / FaceletCube::FACELET_IN_FACE_NUMBER;
        fState[CORNER_MAP[i][1]] = CORNER_MAP[cState[i]][(cState[CubieCube::CORNER_NUMBER+i]+1)%CubieCube::TWIST_NUMBER] / FaceletCube::FACELET_IN_FACE_NUMBER;
        fState[CORNER_MAP[i][2]] = CORNER_MAP[cState[i]][(cState[CubieCube::CORNER_NUMBER+i]+2)%CubieCube::TWIST_NUMBER] / FaceletCube::FACELET_IN_FACE_NUMBER;
    }
        
    // Set edge facelets in fState
    for (i=0; i<CubieCube::EDGE_NUMBER; i++)
    {
        // - cState[2*CubieCube::EDGE_MAP+i] - edge permutation
        // - cState[2*CubieCube::EDGE_MAP+CubieCube::EDGE_NUMBER+i] - edge orientation
        // - EDGE_MAP[i] - facelets of i-th edge position
        // - EDGE_MAP[cState[2*CubieCube::CORNER_NUMBER+i]] - facelets placed in i-th edge position
        fState[EDGE_MAP[i][0]] = EDGE_MAP[cState[2*CubieCube::CORNER_NUMBER+i]][(cState[2*CubieCube::CORNER_NUMBER+CubieCube::EDGE_NUMBER+i]+0)%CubieCube::FLIP_NUMBER] / FaceletCube::FACELET_IN_FACE_NUMBER;
        fState[EDGE_MAP[i][1]] = EDGE_MAP[cState[2*CubieCube::CORNER_NUMBER+i]][(cState[2*CubieCube::CORNER_NUMBER+CubieCube::EDGE_NUMBER+i]+1)%CubieCube::FLIP_NUMBER] / FaceletCube::FACELET_IN_FACE_NUMBER;
    }
    
    fCube.FromTable(fState);
}
