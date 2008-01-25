/////////////////////////////////////////////////////////
// Name: KociembaPhase2Cube.cpp
// Implements : KociembaPhase2Cube
/////////////////////////////////////////////////////////

#include "vsp/rcs/Kociemba/KociembaPhase2Cube.h"
#include "vsp/rcs/Kociemba/KociembaPhase2MoveTable.h"

#include <string>


const char *KociembaPhase2Cube::TYPE = "KociembaPhase2Cube";


////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT, CLONE, COMPARE
////////////////////////////////////////////////////////////////////////////////

KociembaPhase2Cube::KociembaPhase2Cube()
    : cornerPermutationCoord(0),
      edgeMidSlicePermutationCoord(0),
      edgeNonMidSlicePermutationCoord(0)
{
    SInit();
}
    
KociembaPhase2Cube::KociembaPhase2Cube(const KociembaPhase2Cube& cube)
    : cornerPermutationCoord(cube.cornerPermutationCoord),
      edgeMidSlicePermutationCoord(cube.edgeMidSlicePermutationCoord),
      edgeNonMidSlicePermutationCoord(cube.edgeNonMidSlicePermutationCoord)
{
    SInit();
}

KociembaPhase2Cube::~KociembaPhase2Cube()
{
}

int KociembaPhase2Cube::operator==(const KociembaPhase2Cube& cube)
{
    return (this->cornerPermutationCoord == cube.cornerPermutationCoord)
        && (this->edgeMidSlicePermutationCoord == cube.edgeMidSlicePermutationCoord)
        && (this->edgeNonMidSlicePermutationCoord == cube.edgeNonMidSlicePermutationCoord);
}


////////////////////////////////////////////////////////////////////////////////
// GET/SET STATE PUBLIC METHODS
////////////////////////////////////////////////////////////////////////////////

void KociembaPhase2Cube::SetInitState()
{
    int coords[6];
    InitCoords(coords);
    cornerPermutationCoord = coords[3];
    edgeMidSlicePermutationCoord = coords[4];
    edgeNonMidSlicePermutationCoord = coords[5];
}

void KociembaPhase2Cube::FromCubieCube(CubieCube& cube)
     throw (KociembaException)
{
    int coords[6], coords_init[6];
    KociembaCube::CubieCubeToCoords(cube, coords);
    
    // check if it is second phase cube
    InitCoords(coords_init);
    if ( coords[0] != coords_init[0]
      || coords[1] != coords_init[1]
      || coords[2] != coords_init[2])
        throw KociembaException(KociembaException::ERR_PHASE2_CUBE_INVALID);
    
    FromCoords(&coords[3]);
}

void KociembaPhase2Cube::ToCubieCube(CubieCube& cube)
{
    int coords[6];
    InitCoords(coords);
    ToCoords(&coords[3]);
    KociembaCube::CoordsToCubieCube(cube, coords);
}

void KociembaPhase2Cube::FromCoords(int coords[])
{
    cornerPermutationCoord = coords[0];
    edgeMidSlicePermutationCoord = coords[1];
    edgeNonMidSlicePermutationCoord = coords[2];
}

void KociembaPhase2Cube::ToCoords(int coords[])
{
    coords[0] = cornerPermutationCoord;
    coords[1] = edgeMidSlicePermutationCoord;
    coords[2] = edgeNonMidSlicePermutationCoord;
}

char* KociembaPhase2Cube::ToString()
{
    char *sCube = new char[18];
    int i, mult;
    
    strncpy(sCube, "0 000, 00, 00 000", 18);

    for (i=0, mult=1; i<6; i++, mult*=10)
    {
        if (i == 3)
            i++;
        sCube[16-i] = '0' + (edgeNonMidSlicePermutationCoord / mult) % 10;
        if (i < 5) 
        {
            sCube[4-i] = '0' + (cornerPermutationCoord / mult) % 10;
            if (i < 2)
                sCube[8-i] = '0' + (edgeMidSlicePermutationCoord / mult) % 10;
        }
    }

    return sCube;
}


////////////////////////////////////////////////////////////////////////////////
// MOVE METHOD
////////////////////////////////////////////////////////////////////////////////

bool KociembaPhase2Cube::IsAllowed(CubeMove::eMove move, CubeMove::eTurn turn)
{
     if (move != CubeMove::U && move != CubeMove::D && turn != CubeMove::HALF)
         return false;
     return true;
}

void KociembaPhase2Cube::Move(CubeMove::eMove move, CubeMove::eTurn turn)
{
    if (!IsAllowed(move, turn))
        throw KociembaException(KociembaException::ERR_PHASE2_MOVE_NOTALLOWED);

    // perform a quater move turn times
    if (move != CubeMove::U && move != CubeMove::D)
        MoveCoords(move);
    else
    {
        for (int i=0; i<=turn; i++)
            MoveCoords(move);
    }
}

void KociembaPhase2Cube::MoveCoords(CubeMove::eMove move)
{
    // moves in phase 2 affect only permutation of corners and edges
    // other coordinates remain untouched
    cornerPermutationCoord = mtCP->Get(cornerPermutationCoord, move);
    edgeMidSlicePermutationCoord = mtEMP->Get(edgeMidSlicePermutationCoord, move);
    edgeNonMidSlicePermutationCoord = mtENP->Get(edgeNonMidSlicePermutationCoord, move);
}


////////////////////////////////////////////////////////////////////////////////
// DECLARATION AND INITIALIZATION OF FIELDS FOR MOVES
////////////////////////////////////////////////////////////////////////////////

bool KociembaPhase2Cube::bInitialized = false;
KociembaPhase2MoveTable* KociembaPhase2Cube::mtCP = NULL;
KociembaPhase2MoveTable* KociembaPhase2Cube::mtEMP = NULL;
KociembaPhase2MoveTable* KociembaPhase2Cube::mtENP = NULL;

void KociembaPhase2Cube::SInit()
{
    if (bInitialized)
        return;
    
    // create move tables
    mtCP = new KociembaPhase2MoveTable(0);
    mtEMP = new KociembaPhase2MoveTable(1);
    mtENP = new KociembaPhase2MoveTable(2);

    bInitialized = true;

    // initizalize move tables for phase 2 (generate and save to files or read from file)
    mtCP->Init();
    mtEMP->Init();
    mtENP->Init();
}

void KociembaPhase2Cube::SClear()
{
    if (!bInitialized)
        return;
    
    bInitialized = false;

    // delete move tables
    delete mtCP;
    delete mtEMP;
    delete mtENP;
}
