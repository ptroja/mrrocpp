/////////////////////////////////////////////////////////
// Name: KociembaCube.cpp
// Implements : KociembaCube
/////////////////////////////////////////////////////////

#include "vsp/rcs/Kociemba/KociembaPhase1Cube.h"
#include "vsp/rcs/Kociemba/KociembaPhase1MoveTable.h"

#include <string>

const char *KociembaPhase1Cube::TYPE = "KociembaPhase1Cube";


////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT, CLONE, COMPARE
////////////////////////////////////////////////////////////////////////////////

KociembaPhase1Cube::KociembaPhase1Cube()
    : cornerOrientationCoord(0),
      edgeMidSliceCombinationCoord(EDGE_SLICE_COMBINATION_NUMBER - 1),
      edgeOrientationCoord(0)
{
    SInit();
}
    
KociembaPhase1Cube::KociembaPhase1Cube(const KociembaPhase1Cube& cube)
    : cornerOrientationCoord(cube.cornerOrientationCoord),
      edgeMidSliceCombinationCoord(cube.edgeMidSliceCombinationCoord),
      edgeOrientationCoord(cube.edgeOrientationCoord)
{
    SInit();
}

KociembaPhase1Cube::~KociembaPhase1Cube()
{
}

int KociembaPhase1Cube::operator==(const KociembaPhase1Cube& cube)
{
    return (this->cornerOrientationCoord == cube.cornerOrientationCoord)
        && (this->edgeMidSliceCombinationCoord == cube.edgeMidSliceCombinationCoord)
        && (this->edgeOrientationCoord == cube.edgeOrientationCoord);
}


////////////////////////////////////////////////////////////////////////////////
// GET/SET STATE PUBLIC METHODS
////////////////////////////////////////////////////////////////////////////////

void KociembaPhase1Cube::SetInitState()
{
    int coords[6];
    InitCoords(coords);
    cornerOrientationCoord = coords[0];
    edgeMidSliceCombinationCoord = coords[1];
    edgeOrientationCoord = coords[2];
}

void KociembaPhase1Cube::FromCubieCube(CubieCube& cube)
     throw (KociembaException)
{
    int coords[6];
    KociembaCube::CubieCubeToCoords(cube, coords);
    FromCoords(coords);
}

void KociembaPhase1Cube::ToCubieCube(CubieCube& cube)
{
    int coords[6];
    InitCoords(coords);
    ToCoords(coords);

    bool valid = false;
    while (!valid)
    {
        valid = true;
        try
        {
            KociembaCube::CoordsToCubieCube(cube, coords);
        }
        catch (CubeException& exp)
        {
            valid = false;
            coords[4]++;
        }
    }
}

void KociembaPhase1Cube::FromCoords(int coords[])
{
    cornerOrientationCoord = coords[0];
    edgeMidSliceCombinationCoord = coords[1];
    edgeOrientationCoord = coords[2];
}

void KociembaPhase1Cube::ToCoords(int coords[])
{
    coords[0] = cornerOrientationCoord;
    coords[1] = edgeMidSliceCombinationCoord;
    coords[2] = edgeOrientationCoord;
}

char* KociembaPhase1Cube::ToString()
{
    char *sCube = new char[19];
    int i, mult;
    
    strncpy(sCube, "00 000, 000, 0 000", 19);

    for (i=0, mult=1; i<6; i++, mult*=10)
    {
        if (i == 3)
            i++;
        sCube[5-i] = '0' + (cornerOrientationCoord / mult) % 10;
        if (i < 5) 
        {
            sCube[17-i] = '0' + (edgeOrientationCoord / mult) % 10;
            if (i < 3)
                sCube[10-i] = '0' + (edgeMidSliceCombinationCoord / mult) % 10;
        }
    }

    return sCube;
}


////////////////////////////////////////////////////////////////////////////////
// MOVE METHOD
////////////////////////////////////////////////////////////////////////////////

void KociembaPhase1Cube::Move(CubeMove::eMove move, CubeMove::eTurn turn)
{
    // perform a quater move turn times
    for (int i=0; i<=turn; i++)
        MoveCoords(move);  
}

void KociembaPhase1Cube::MoveCoords(CubeMove::eMove move)
{
    // moves in phase 1 affect only orientation and combination of edges for middle slice
    // other coordinates are neglected and are not proper
    cornerOrientationCoord = mtCO->Get(cornerOrientationCoord, move);
    edgeMidSliceCombinationCoord = mtEMC->Get(edgeMidSliceCombinationCoord, move);
    edgeOrientationCoord = mtEO->Get(edgeOrientationCoord, move);
}


////////////////////////////////////////////////////////////////////////////////
// DECLARATION AND INITIALIZATION OF FIELDS FOR MOVES
////////////////////////////////////////////////////////////////////////////////

bool KociembaPhase1Cube::bInitialized = false;
KociembaPhase1MoveTable* KociembaPhase1Cube::mtCO = NULL;
KociembaPhase1MoveTable* KociembaPhase1Cube::mtEMC = NULL;
KociembaPhase1MoveTable* KociembaPhase1Cube::mtEO = NULL;

void KociembaPhase1Cube::SInit()
{
    if (bInitialized)
        return;
    
    // create move tables
    mtCO = new KociembaPhase1MoveTable(0);
    mtEMC = new KociembaPhase1MoveTable(1);
    mtEO = new KociembaPhase1MoveTable(2);

    bInitialized = true;

    // initizalize move tables for phase 1 (generate and save to files or read from file)
    mtCO->Init();
    mtEMC->Init();
    mtEO->Init();
}

void KociembaPhase1Cube::SClear()
{
    if (!bInitialized)
        return;
    
    bInitialized = false;

    // delete move tables
    delete mtCO;
    delete mtEMC;
    delete mtEO;
}
