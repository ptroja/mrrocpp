/////////////////////////////////////////////////////////
// Name: CubieCube.h
// Implements: CubieCube
/////////////////////////////////////////////////////////

#include "vsp/rcs/Cube/CubieCube.h"
#include "vsp/rcs/Cube/CubeMove.h"

#include <string>


const char *CubieCube::TYPE = "CubieCube";


////////////////////////////////////////////////////////////////////////////////
// INITIALIZATION OF CONSTANT VALUES
////////////////////////////////////////////////////////////////////////////////

const int CubieCube::MOVE_CORNERS[CubeMove::MOVE_NUMBER][CORNER_IN_FACE_NUMBER] = 
  { {UFL, URF, DFR, DLF}, // F
    {URF, UBR, DRB, DFR}, // R
    {ULB, UBR, URF, UFL}, // U
    {UBR, ULB, DBL, DRB}, // B
    {ULB, UFL, DLF, DBL}, // L
    {DLF, DFR, DRB, DBL}  // D
  };
  
const int CubieCube::MOVE_EDGES[CubeMove::MOVE_NUMBER][EDGE_IN_FACE_NUMBER] = 
  { {UF, FR, DF, FL}, // F
    {UR, BR, DR, FR}, // R
    {UB, UR, UF, UL}, // U
    {UB, BL, DB, BR}, // B
    {UL, FL, DL, BL}, // L
    {DF, DR, DB, DL}  // D
  };

const bool CubieCube::MOVE_TWIST[CubeMove::MOVE_NUMBER] = 
  { true,   // F
    true,   // R
    false,  // U
    true,   // B
    true,   // L
    false   // D
  };
  
const bool CubieCube::MOVE_FLIP[CubeMove::MOVE_NUMBER] = 
  { true,   // F
    false,  // R
    false,  // U
    true,   // B
    false,  // L
    false   // D
  };

  
////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT, COMPARE
////////////////////////////////////////////////////////////////////////////////

CubieCube::CubieCube() 
    : cornerPermutation(CubieCube::CORNER_NUMBER),
      cornerOrientation(CubieCube::CORNER_NUMBER, CubieCube::CORNER_TWIST_NUMBER),
      edgePermutation(CubieCube::EDGE_NUMBER),
      edgeOrientation(CubieCube::EDGE_NUMBER, CubieCube::EDGE_FLIP_NUMBER)
{
    SetInitState();
}

CubieCube::CubieCube(const CubieCube& cube)
    : cornerPermutation(cube.cornerPermutation),
      cornerOrientation(cube.cornerOrientation),
      edgePermutation(cube.edgePermutation),
      edgeOrientation(cube.edgeOrientation)
{
}

CubieCube::~CubieCube()
{
}

int CubieCube::operator==(const CubieCube& cube)
{
    return (this->cornerPermutation == cube.cornerPermutation)
        && (this->cornerOrientation == cube.cornerOrientation)
        && (this->edgePermutation == cube.edgePermutation)
        && (this->edgeOrientation == cube.edgeOrientation);
}


////////////////////////////////////////////////////////////////////////////////
// GET/SET STATE PUBLIC METHODS
////////////////////////////////////////////////////////////////////////////////

void CubieCube::SetInitState()
{
    cornerPermutation.SetInitState();
    cornerOrientation.SetInitState();
    edgePermutation.SetInitState();
    edgeOrientation.SetInitState();
}

void CubieCube::FromTable(const int cube[]) 
     throw (CubeException)
{
    int backup[2*CubieCube::CORNER_NUMBER + 2*CubieCube::EDGE_NUMBER];

    // Create backup
    cornerPermutation.ToTable(backup);     
    cornerOrientation.ToTable(&backup[CubieCube::CORNER_NUMBER]);
    edgePermutation.ToTable(&backup[2*CubieCube::CORNER_NUMBER]);
    edgeOrientation.ToTable(&backup[2*CubieCube::CORNER_NUMBER + CubieCube::EDGE_NUMBER]);
    
    // Try filling and validating cube state
    try
    {
        // Fill new state
        cornerPermutation.FromTable(cube);     
        cornerOrientation.FromTable(&cube[CubieCube::CORNER_NUMBER]);
        edgePermutation.FromTable(&cube[2*CubieCube::CORNER_NUMBER]);
        edgeOrientation.FromTable(&cube[2*CubieCube::CORNER_NUMBER + CubieCube::EDGE_NUMBER]);
    
        Validate();
    }
    catch (CubeException& exp)
    {
        // Revert state
        cornerPermutation.FromTable(backup);     
        cornerOrientation.FromTable(&backup[CubieCube::CORNER_NUMBER]);
        edgePermutation.FromTable(&backup[2*CubieCube::CORNER_NUMBER]);
        edgeOrientation.FromTable(&backup[2*CubieCube::CORNER_NUMBER + CubieCube::EDGE_NUMBER]);
        throw;
    }  
    catch (Exception& exp)
    {
        // Revert state
        cornerPermutation.FromTable(backup);     
        cornerOrientation.FromTable(&backup[CubieCube::CORNER_NUMBER]);
        edgePermutation.FromTable(&backup[2*CubieCube::CORNER_NUMBER]);
        edgeOrientation.FromTable(&backup[2*CubieCube::CORNER_NUMBER + CubieCube::EDGE_NUMBER]);
        throw CubeException(&exp, CubeException::ERR_CCUBE_INVALID_TABLE);
    }  
}

void CubieCube::ToTable(int cube[])
{
    // Fill tables with state
    cornerPermutation.ToTable(cube);     
    cornerOrientation.ToTable(&cube[CubieCube::CORNER_NUMBER]);
    edgePermutation.ToTable(&cube[2*CubieCube::CORNER_NUMBER]);
    edgeOrientation.ToTable(&cube[2*CubieCube::CORNER_NUMBER + CubieCube::EDGE_NUMBER]);
}

char* CubieCube::ToString()
{
    char *sCube = new char[200];
    char *str;
    int pos=0;

    str = cornerPermutation.ToString();   
    strncpy(&sCube[pos], str, 10+3*8);
    pos += 10+3*8+1;
    sCube[pos-1] = ' ';
    delete[] str;

    str = cornerOrientation.ToString();   
    strncpy(&sCube[pos], str, 10+3*8);
    pos += 10+3*8+1;
    sCube[pos-1] = ' ';
    delete[] str;

    str = edgePermutation.ToString();   
    strncpy(&sCube[pos], str, 10+3*12);
    pos += 10+3*12+1;
    sCube[pos-1] = ' ';
    delete[] str;

    str = edgeOrientation.ToString();   
    strncpy(&sCube[pos], str, 10+3*12);
    pos += 10+3*12+1;
    sCube[pos-1] = '\0';
    delete[] str;

    return sCube;
}


////////////////////////////////////////////////////////////////////////////////
// MOVE PUBLIC METHOD
////////////////////////////////////////////////////////////////////////////////

void CubieCube::Move(CubeMove::eMove move, CubeMove::eTurn turn) 
{
    // switch between moving face move clockwise, double clockwise or counter-clockwise
    switch (turn) 
    {
        case CubeMove::QUARTER:
             MoveCubies(move, true);  
             break;
        case CubeMove::HALF:
             MoveCubies(move, true);  
             MoveCubies(move, true);  
             break;
        case CubeMove::ANTI_QUARTER:   
             MoveCubies(move, false);  
             break;
    }
}

////////////////////////////////////////////////////////////////////////////////
// MOVE PRIVATE METHOD
////////////////////////////////////////////////////////////////////////////////

void CubieCube::MoveCubies(CubeMove::eMove move, bool clockwise) 
{
    int i;
    
    Permutation moveCorners(CubieCube::CORNER_NUMBER), 
        moveEdges(CubieCube::EDGE_NUMBER);

    // Get permutation of move on corners and edges
    moveCorners.FromCycle(CubieCube::MOVE_CORNERS[move], 4);
    moveEdges.FromCycle(CubieCube::MOVE_EDGES[move], 4);
    
    // Inverse permutation if move is counter-clockwise
    if (clockwise)
    {
        moveCorners.Inverse();
        moveEdges.Inverse();
    }

    // Perform composition of corner and edge permutations with appropriate move perms
    cornerPermutation.Composite(moveCorners);
    cornerOrientation.Composite(moveCorners);
    edgePermutation.Composite(moveEdges);
    edgeOrientation.Composite(moveEdges);

    // twist corners if move does it
    clockwise = true;
    if (CubieCube::MOVE_TWIST[move])
        for (i=0; i<CubieCube::CORNER_IN_FACE_NUMBER; i++) 
        {
            cornerOrientation.ChooseNext(CubieCube::MOVE_CORNERS[move][i], clockwise);
            clockwise = !clockwise;
        }
    
    // flip edges if move does it
    if (CubieCube::MOVE_FLIP[move])
        for (i=0; i<CubieCube::EDGE_IN_FACE_NUMBER; i++) 
            edgeOrientation.ChooseNext(CubieCube::MOVE_EDGES[move][i]);

}


////////////////////////////////////////////////////////////////////////////////
// VALIDATE PRIVATE METHOD
////////////////////////////////////////////////////////////////////////////////

void CubieCube::Validate() 
     throw (CubeException)
{
    int par;
    
    // Validate only in validation mode
    if (!bValidation)
        return;
        
    // Checks if corners parity is even
    if ((par = cornerOrientation.Parity()) != 0)
        throw CubeException(this, NULL, par, 
              CubeException::ERR_CCUBE_INVALID_CORNER_PARITY);     

    // Checks if edges parity is even
    if ((par = edgeOrientation.Parity()) != 0)
        throw CubeException(this, NULL, par, 
              CubeException::ERR_CCUBE_INVALID_EDGE_PARITY);

    // Check if corner and edges permutation parity are equal
    if ((par = cornerPermutation.Parity() - edgePermutation.Parity()) != 0)
        throw CubeException(this, NULL, par, 
              CubeException::ERR_CCUBE_UNEQUAL_PERM_PARITY);
}
