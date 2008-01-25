/////////////////////////////////////////////////////////
// Name: CubeMoveTable.cpp
// Implements : CubeMoveTable
/////////////////////////////////////////////////////////

#include "vsp/rcs/Cube/CubeMoveTable.h"
#include <fstream>


////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT
////////////////////////////////////////////////////////////////////////////////

CubeMoveTable::CubeMoveTable(int coordSize) 
    : DataTable(coordSize * CubeMove::MOVE_NUMBER),
      iCoordSize(coordSize),
      pCube(NULL)
{}

CubeMoveTable::CubeMoveTable(const CubeMoveTable& table)
    : DataTable(*((DataTable*) &table)),
      iCoordSize(table.iCoordSize),
      pCube(NULL)
{
}

CubeMoveTable::~CubeMoveTable()
{
    if (pCube == NULL)
        delete pCube;
}


////////////////////////////////////////////////////////////////////////////////
// DATA ACCESS INLINE METHODS
////////////////////////////////////////////////////////////////////////////////
    
/*inline */int CubeMoveTable::Get(int ordinal, CubeMove::eMove move) 
{
    return GetData(GetDataCoord(ordinal, move));
}

inline int CubeMoveTable::GetDataCoord(int coord, CubeMove::eMove move)
{
    return coord*CubeMove::MOVE_NUMBER + move;
}    

////////////////////////////////////////////////////////////////////////////////
// HELPER METHODS FOR TABLE GENERATION
////////////////////////////////////////////////////////////////////////////////

void CubeMoveTable::PreGenerate()
{
    pCube = new CubieCube();
    Cube::SetValidation(false);
}

void CubeMoveTable::PostGenerate()
{
    Cube::SetValidation(true);
    delete pCube;
    pCube = NULL;
}

CubeMove::eTurn CubeMoveTable::GetTurn(CubeMove::eMove move)
{
    return CubeMove::QUARTER;
}


////////////////////////////////////////////////////////////////////////////////
// CORE METHODS - GENERATION (private)
////////////////////////////////////////////////////////////////////////////////

void CubeMoveTable::Generate() 
{
    int coord, mov;
    CubeMove::eMove move;
    
    // perform operations before generation
    PreGenerate();
      
    // fill al fields of the table
    for (coord=0; coord<iCoordSize; coord++) 
    {
        // set state identified by coord number
        SetCoord(coord);
            
        // fill a transformation of a state with all 6 moves
        for (mov=0; mov<CubeMove::MOVE_NUMBER; mov++) 
        {
            move = (CubeMove::eMove) mov;
            
            // perform a move
            pCube->Move(move, GetTurn(move));

            // count and save resulting coord repesenting state
            SetData(GetDataCoord(coord, move), GetCoord());

            // perform a reverse move
            pCube->Move(move, CubeMove::REVERSE_TURN(GetTurn(move)));
        }
    }

    // perform operations after table generation
    PostGenerate();    
}
