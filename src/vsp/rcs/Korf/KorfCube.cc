/////////////////////////////////////////////////////////
// Name: KorfCube.cpp
// Implements : KorfCube
/////////////////////////////////////////////////////////

#include "vsp/rcs/Korf/KorfCube.h"
#include "vsp/rcs/Korf/KorfMoveTable.h"
#include "vsp/rcs/Korf/KorfMoveTable_CP.h"
#include "vsp/rcs/Korf/KorfMoveTable_CO.h"
#include "vsp/rcs/Korf/KorfMoveTable_ECP.h"
#include "vsp/rcs/Korf/KorfMoveTable_ECO.h"

#include "vsp/rcs/Combinatorials/Permutation.h"
#include "vsp/rcs/Combinatorials/Variation.h"
#include "vsp/rcs/Combinatorials/Combination.h"

#include <string>


const int KorfCube::CORNER_PERMUTATION_NUMBER = Permutation::NUMBER(CubieCube::CORNER_NUMBER);
const int KorfCube::CORNER_ORIENTATION_NUMBER =   Variation::NUMBER(CubieCube::CORNER_NUMBER - 1, CubieCube::CORNER_TWIST_NUMBER);
const int KorfCube::EDGE_HALF_COMBINATION_NUMBER = Combination::NUMBER(EDGE_HALF_NUMBER, CubieCube::EDGE_NUMBER);  
const int KorfCube::EDGE_HALF_PERMUTATION_NUMBER = Permutation::NUMBER(EDGE_HALF_NUMBER);
const int KorfCube::EDGE_HALF_ORIENTATION_NUMBER =   Variation::NUMBER(EDGE_HALF_NUMBER, CubieCube::EDGE_FLIP_NUMBER);
const int KorfCube::CORNER_COORD_NUMBER    = KorfCube::CORNER_PERMUTATION_NUMBER * KorfCube::CORNER_ORIENTATION_NUMBER;
const int KorfCube::EDGE_HALF_COORD_NUMBER = KorfCube::EDGE_HALF_COMBINATION_NUMBER * KorfCube::EDGE_HALF_PERMUTATION_NUMBER * KorfCube::EDGE_HALF_ORIENTATION_NUMBER;

const char *KorfCube::TYPE = "KorfCube";


////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT, CLONE, COMPARE
////////////////////////////////////////////////////////////////////////////////

KorfCube::KorfCube()
    : cornerCoord(CORNER(0,0)),
      edgeFirstHalfCoord(EDGE(0,0,0)),
      edgeSecondHalfCoord(EDGE(EDGE_HALF_COMBINATION_NUMBER - 1,0,0))
{
    SInit();
}
    
KorfCube::KorfCube(const KorfCube& cube)
    : cornerCoord(cube.cornerCoord),
      edgeFirstHalfCoord(cube.edgeFirstHalfCoord),
      edgeSecondHalfCoord(cube.edgeSecondHalfCoord)
{
    SInit();
}

KorfCube::~KorfCube()
{
}

int KorfCube::operator==(const KorfCube& cube)
{
    return (this->cornerCoord == cube.cornerCoord)
        && (this->edgeFirstHalfCoord == cube.edgeFirstHalfCoord)
        && (this->edgeSecondHalfCoord == cube.edgeSecondHalfCoord);
}
    

////////////////////////////////////////////////////////////////////////////////
// GET/SET STATE PUBLIC METHODS
////////////////////////////////////////////////////////////////////////////////

inline void KorfCube::SetInitState()
{
    this->cornerCoord = CORNER(0,0);
    this->edgeFirstHalfCoord = EDGE(0,0,0);
    this->edgeSecondHalfCoord = EDGE(EDGE_HALF_COMBINATION_NUMBER - 1,0,0);
}

void KorfCube::FromCubieCube(CubieCube& cube)
{
     int i, h1, h2, off;
     int state[2*CubieCube::CORNER_NUMBER + 2*CubieCube::EDGE_NUMBER];
     int cTab[16], eTab1[18], eTab2[18];
     
     // get cubie cube's state
     cube.ToTable(state);
     
     // copy corner permutation and orientation to dedicated table
     memcpy(cTab, state, 2*CubieCube::CORNER_NUMBER*sizeof(int));
     
     // split edge parameters into first and second half
     off = 2*CubieCube::CORNER_NUMBER;
     for (i=0, h1=0, h2=0; i<CubieCube::EDGE_NUMBER; i++)
     {
         if (state[i+off] < EDGE_HALF_NUMBER)
         {
             eTab1[h1] = i;
             eTab1[h1+EDGE_HALF_NUMBER] = state[i+off];
             eTab1[h1+2*EDGE_HALF_NUMBER] = state[i+CubieCube::EDGE_NUMBER+off];
             h1++;
         }
         else
         {
             eTab2[h2] = i;
             eTab2[h2+EDGE_HALF_NUMBER] = state[i+off] - EDGE_HALF_NUMBER;
             eTab2[h2+2*EDGE_HALF_NUMBER] = state[i+CubieCube::EDGE_NUMBER+off];
             h2++;
         }
     } 

    // create combinatorials 
    Permutation cornerPermutation(CubieCube::CORNER_NUMBER);
    Variation cornerOrientation(CubieCube::CORNER_NUMBER, CubieCube::TWIST_NUMBER);
    Combination edgeFirstHalfCombination(EDGE_HALF_NUMBER, CubieCube::EDGE_NUMBER);
    Permutation edgeFirstHalfPermutation(EDGE_HALF_NUMBER);
    Variation edgeFirstHalfOrientation(EDGE_HALF_NUMBER, CubieCube::FLIP_NUMBER);
    Combination edgeSecondHalfCombination(EDGE_HALF_NUMBER, CubieCube::EDGE_NUMBER);
    Permutation edgeSecondHalfPermutation(EDGE_HALF_NUMBER);
    Variation edgeSecondHalfOrientation(EDGE_HALF_NUMBER, CubieCube::FLIP_NUMBER);
    
    // set combinatorials values from tables
    cornerPermutation.FromTable(cTab);
    cornerOrientation.FromTable(&cTab[CubieCube::CORNER_NUMBER]);
    edgeFirstHalfCombination.FromTable(eTab1);
    edgeFirstHalfPermutation.FromTable(&eTab1[EDGE_HALF_NUMBER]);
    edgeFirstHalfOrientation.FromTable(&eTab1[2*EDGE_HALF_NUMBER]);
    edgeSecondHalfCombination.FromTable(eTab2);
    edgeSecondHalfPermutation.FromTable(&eTab2[EDGE_HALF_NUMBER]);
    edgeSecondHalfOrientation.FromTable(&eTab2[2*EDGE_HALF_NUMBER]);

    // set coordinals from combinatorials
    cornerCoord = CORNER(cornerPermutation.ToOrdinal(), cornerOrientation.ToOrdinal() / 3);
    edgeFirstHalfCoord = EDGE(edgeFirstHalfCombination.ToOrdinal(),
        edgeFirstHalfPermutation.ToOrdinal(), edgeFirstHalfOrientation.ToOrdinal());
    edgeSecondHalfCoord = EDGE(edgeSecondHalfCombination.ToOrdinal(),
        edgeSecondHalfPermutation.ToOrdinal(), edgeSecondHalfOrientation.ToOrdinal());
}

void KorfCube::ToCubieCube(CubieCube& cube)
{
    int i, corn;
    int state[2*CubieCube::CORNER_NUMBER + 2*CubieCube::EDGE_NUMBER];
    int cTab[16], eTab1[18], eTab2[18];

    // create combinatorials 
    Permutation cornerPermutation(CubieCube::CORNER_NUMBER);
    Variation cornerOrientation(CubieCube::CORNER_NUMBER, CubieCube::TWIST_NUMBER);
    Combination edgeFirstHalfCombination(EDGE_HALF_NUMBER, CubieCube::EDGE_NUMBER);
    Permutation edgeFirstHalfPermutation(EDGE_HALF_NUMBER);
    Variation edgeFirstHalfOrientation(EDGE_HALF_NUMBER, CubieCube::FLIP_NUMBER);
    Combination edgeSecondHalfCombination(EDGE_HALF_NUMBER, CubieCube::EDGE_NUMBER);
    Permutation edgeSecondHalfPermutation(EDGE_HALF_NUMBER);
    Variation edgeSecondHalfOrientation(EDGE_HALF_NUMBER, CubieCube::FLIP_NUMBER);
    
    // set combinatorials values from coords
    cornerPermutation.FromOrdinal(CORNER_PERMUTATION(cornerCoord));
    cornerOrientation.FromOrdinal(CORNER_ORIENTATION(cornerCoord) * 3);
    edgeFirstHalfCombination.FromOrdinal(EDGE_COMBINATION(edgeFirstHalfCoord));
    edgeFirstHalfPermutation.FromOrdinal(EDGE_PERMUTATION(edgeFirstHalfCoord));
    edgeFirstHalfOrientation.FromOrdinal(EDGE_ORIENTATION(edgeFirstHalfCoord));
    edgeSecondHalfCombination.FromOrdinal(EDGE_COMBINATION(edgeSecondHalfCoord));
    edgeSecondHalfPermutation.FromOrdinal(EDGE_PERMUTATION(edgeSecondHalfCoord));
    edgeSecondHalfOrientation.FromOrdinal(EDGE_ORIENTATION(edgeSecondHalfCoord));

    // set corner permutation and orientation in dedicated table
    cornerPermutation.ToTable(cTab);
    cornerOrientation.ToTable(&cTab[CubieCube::CORNER_NUMBER]);
    
    // set orientation of last corner
    for (i=0, corn=0; i<CubieCube::CORNER_NUMBER - 1; i++)
        corn += cTab[CubieCube::CORNER_NUMBER + i];
    cTab[2*CubieCube::CORNER_NUMBER - 1] = (3 - corn % CubieCube::TWIST_NUMBER) % CubieCube::TWIST_NUMBER;
     
    // set edge-halves permutation, orientation and combination in dedicated table
    edgeFirstHalfCombination.ToTable(eTab1);
    edgeFirstHalfPermutation.ToTable(&eTab1[EDGE_HALF_NUMBER]);
    edgeFirstHalfOrientation.ToTable(&eTab1[2*EDGE_HALF_NUMBER]);
    edgeSecondHalfCombination.ToTable(eTab2);
    edgeSecondHalfPermutation.ToTable(&eTab2[EDGE_HALF_NUMBER]);
    edgeSecondHalfOrientation.ToTable(&eTab2[2*EDGE_HALF_NUMBER]);

    // copy corner parameters to state table
     memcpy(state, cTab, 2*CubieCube::CORNER_NUMBER*sizeof(int));

    // join edge-halves into edge parameters
    for (i=0; i<EDGE_HALF_NUMBER; i++)
    {
        state[2*CubieCube::CORNER_NUMBER + eTab1[i]] = eTab1[i+EDGE_HALF_NUMBER];
        state[2*CubieCube::CORNER_NUMBER + eTab1[i]+CubieCube::EDGE_NUMBER] = eTab1[i+2*EDGE_HALF_NUMBER];
    }
    for (i=0; i<EDGE_HALF_NUMBER; i++)
    {
        state[2*CubieCube::CORNER_NUMBER + eTab2[i]] = eTab2[i+EDGE_HALF_NUMBER] + EDGE_HALF_NUMBER;
        state[2*CubieCube::CORNER_NUMBER + eTab2[i]+CubieCube::EDGE_NUMBER] = eTab2[i+2*EDGE_HALF_NUMBER];
    }

    // set cubie cube's state
    cube.FromTable(state);
}

void KorfCube::FromCoords(int coords[])
{
    cornerCoord = coords[0];
    edgeFirstHalfCoord = coords[1];
    edgeSecondHalfCoord = coords[2];
}

void KorfCube::ToCoords(int coords[])
{
    coords[0] = cornerCoord;
    coords[1] = edgeFirstHalfCoord;
    coords[2] = edgeSecondHalfCoord;
}

char* KorfCube::ToString()
{
    char *sCube = new char[36];
    int i, mult;
    
    strncpy(sCube, "000 000 000, 00 000 000, 00 000 000", 36);

    for (i=0, mult=1; i<10; i++, mult*=10)
    {
        if (i == 3 || i == 7)
            i++;
        sCube[10-i] = '0' + (cornerCoord / mult) % 10;
        sCube[22-i] = '0' + (edgeFirstHalfCoord / mult) % 10;
        sCube[34-i] = '0' + (edgeSecondHalfCoord / mult) % 10;
    }
    sCube[0] = '0' + (cornerCoord / mult) % 10;

    return sCube;
}


////////////////////////////////////////////////////////////////////////////////
// MOVE METHOD
////////////////////////////////////////////////////////////////////////////////

void KorfCube::Move(CubeMove::eMove move, CubeMove::eTurn turn)
{
    // switch between moving face move clockwise, double clockwise or counter-clockwise
    switch (turn) 
    {
        case CubeMove::QUARTER:
             MoveCoords(move);  
             break;
        case CubeMove::HALF:
             MoveCoords(move);  
             MoveCoords(move);  
             break;
        case CubeMove::ANTI_QUARTER:   
             MoveCoords(move);  
             MoveCoords(move);  
             MoveCoords(move);  
             break;
    }
}

void KorfCube::MoveCoords(CubeMove::eMove move)
{
    int cp, co, e1c, e1p, e1o, e2c, e2p, e2o; 
    int e1cp, e1co, e2cp, e2co;

    // Split current coordinates
    cp = CORNER_PERMUTATION(cornerCoord);
    co = CORNER_ORIENTATION(cornerCoord);
    e1c = EDGE_COMBINATION(edgeFirstHalfCoord);
    e1p = EDGE_PERMUTATION(edgeFirstHalfCoord);
    e1o = EDGE_ORIENTATION(edgeFirstHalfCoord);
    e2c = EDGE_COMBINATION(edgeSecondHalfCoord);
    e2p = EDGE_PERMUTATION(edgeSecondHalfCoord);
    e2o = EDGE_ORIENTATION(edgeSecondHalfCoord);
    
    // Create move table cordinates
    e1cp = e1c + e1p * EDGE_HALF_COMBINATION_NUMBER;
    e1co = e1c + e1o * EDGE_HALF_COMBINATION_NUMBER;
    e2cp = e2c + e2p * EDGE_HALF_COMBINATION_NUMBER;
    e2co = e2c + e2o * EDGE_HALF_COMBINATION_NUMBER;
    
    // Get coordinates after move
    cp = mtCP->Get(cp, move);
    co = mtCO->Get(co, move);
    e1cp = mtE1CP->Get(e1cp, move);
    e1co = mtE1CO->Get(e1co, move);
    e2cp = mtE2CP->Get(e2cp, move);
    e2co = mtE2CO->Get(e2co, move);
    
   // Split move coordinates
    e1c = e1cp % EDGE_HALF_COMBINATION_NUMBER;
    e1p = e1cp / EDGE_HALF_COMBINATION_NUMBER;
    e1o = e1co / EDGE_HALF_COMBINATION_NUMBER;
    e2c = e2cp % EDGE_HALF_COMBINATION_NUMBER;
    e2p = e2cp / EDGE_HALF_COMBINATION_NUMBER;
    e2o = e2co / EDGE_HALF_COMBINATION_NUMBER;
    
    // Check consistency
    if (e1c != e1co % EDGE_HALF_COMBINATION_NUMBER || e2c != e2co % EDGE_HALF_COMBINATION_NUMBER)
        // throw exception
        ;
    
    // Set cube's coordinates
    cornerCoord = CORNER(cp, co);
    edgeFirstHalfCoord = EDGE(e1c, e1p, e1o);
    edgeSecondHalfCoord = EDGE(e2c, e2p, e2o);
}

int KorfCube::MoveCoord(int coordNum, int coordValue, CubeMove::eMove move)
{
    // move corner
    if (coordNum == 0) 
    {
        int cp, co;

        // Split current coordinates
        cp = CORNER_PERMUTATION(coordValue);
        co = CORNER_ORIENTATION(coordValue);
        
        // Get coordinates after move
        cp = mtCP->Get(cp, move);
        co = mtCO->Get(co, move);
        
        // Set cube's coordinates
        return CORNER(cp, co);
    } 
    
    // edges from first half
    else if (coordNum == 1)
    {
        int e1c, e1p, e1o;
        int e1cp, e1co;
    
        // Split current coordinates
        e1c = EDGE_COMBINATION(coordValue);
        e1p = EDGE_PERMUTATION(coordValue);
        e1o = EDGE_ORIENTATION(coordValue);
        
        // Create move table cordinates
        e1cp = e1c + e1p * EDGE_HALF_COMBINATION_NUMBER;
        e1co = e1c + e1o * EDGE_HALF_COMBINATION_NUMBER;
        
        // Get coordinates after move
        e1cp = mtE1CP->Get(e1cp, move);
        e1co = mtE1CO->Get(e1co, move);
        
       // Split move coordinates
        e1c = e1cp % EDGE_HALF_COMBINATION_NUMBER;
        e1p = e1cp / EDGE_HALF_COMBINATION_NUMBER;
        e1o = e1co / EDGE_HALF_COMBINATION_NUMBER;
        
        // Check consistency
        if (e1c != e1co % EDGE_HALF_COMBINATION_NUMBER)
            // throw exception
            ;
        
        // Set cube's coordinates
        return EDGE(e1c, e1p, e1o);
    } 
    
    // move edge from second half
    else if (coordNum == 2)
    {
        int e2c, e2p, e2o; 
        int e2cp, e2co;
    
        // Split current coordinates
        e2c = EDGE_COMBINATION(coordValue);
        e2p = EDGE_PERMUTATION(coordValue);
        e2o = EDGE_ORIENTATION(coordValue);
        
        // Create move table cordinates
        e2cp = e2c + e2p * EDGE_HALF_COMBINATION_NUMBER;
        e2co = e2c + e2o * EDGE_HALF_COMBINATION_NUMBER;
        
        // Get coordinates after move
        e2cp = mtE2CP->Get(e2cp, move);
        e2co = mtE2CO->Get(e2co, move);
        
       // Split move coordinates
        e2c = e2cp % EDGE_HALF_COMBINATION_NUMBER;
        e2p = e2cp / EDGE_HALF_COMBINATION_NUMBER;
        e2o = e2co / EDGE_HALF_COMBINATION_NUMBER;
        
        // Check consistency
        if (e2c != e2co % EDGE_HALF_COMBINATION_NUMBER)
            // throw exception
            ;
        
        // Set cube's coordinates
        return EDGE(e2c, e2p, e2o);
    }
    
    return -1;
}


////////////////////////////////////////////////////////////////////////////////
// DECLARATION AND INITIALIZATION OF FIELDS FOR MOVES
////////////////////////////////////////////////////////////////////////////////

bool KorfCube::bInitialized = false;
KorfMoveTable* KorfCube::mtCP = NULL;
KorfMoveTable* KorfCube::mtCO = NULL;
KorfMoveTable* KorfCube::mtE1CP = NULL;
KorfMoveTable* KorfCube::mtE1CO = NULL;
KorfMoveTable* KorfCube::mtE2CP = NULL;
KorfMoveTable* KorfCube::mtE2CO = NULL;

void KorfCube::SInit()
{
    if (bInitialized)
        return;
    
    // create move tables
    mtCP = new KorfMoveTable_CP();
    mtCO = new KorfMoveTable_CO();
    mtE1CP = new KorfMoveTable_ECP(true);
    mtE1CO = new KorfMoveTable_ECO(true);
    mtE2CP = new KorfMoveTable_ECP(false);
    mtE2CO = new KorfMoveTable_ECO(false);

    bInitialized = true;

    // initizalize tables (generate and save to files or read from file)
    mtCP->Init();
    mtCO->Init();
    mtE1CP->Init();
    mtE1CO->Init();
    mtE2CP->Init();
    mtE2CO->Init();
}

void KorfCube::SClear()
{
    if (!bInitialized)
        return;
    
    bInitialized = false;

    // delete move tables
    delete mtCP;
    delete mtCO;
    delete mtE1CP;
    delete mtE1CO;
    delete mtE2CP;
    delete mtE2CO;
}
