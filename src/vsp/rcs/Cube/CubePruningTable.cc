/////////////////////////////////////////////////////////
// Name: CubePruningTable.cpp
// Implements : CubePruningTable
/////////////////////////////////////////////////////////

#include "vsp/rcs/Cube/CubePruningTable.h"
#include "vsp/rcs/Cube/Cube.h"

const int CubePruningTable::MAX_DEPTH=0x0f;

    
////////////////////////////////////////////////////////////////////////////////
// BASIC METHODS - CONSTRUCT, DESTRUCT
////////////////////////////////////////////////////////////////////////////////

CubePruningTable::CubePruningTable(int coordSize) 
    : DataTable(GetDataCoord(coordSize)), iCoordSize(coordSize)
{
}
    
CubePruningTable::~CubePruningTable()
{
}


////////////////////////////////////////////////////////////////////////////////
// DATA ACCESS INLINE METHODS
////////////////////////////////////////////////////////////////////////////////

inline int CubePruningTable::Get(int coord) 
{
    // Gets data from table and chooses only part of data relevant for coord
    return ( ((char*) iaTable)[coord/2] >> (4*(coord%2))) & 0x0f;
}
    
inline void CubePruningTable::Set(int coord, int value) 
{
    // Gets data from table and updates only part of data relevant for coord
    ((char*) iaTable)[coord/2] = ( ((char*) iaTable)[coord/2] & ~(0x0f << ((4*(coord%2))))) 
             | ( (value & 0x0f) << ((4*(coord%2))) );
    
}

inline int CubePruningTable::GetDataCoord(int coord)
{
    return coord / (sizeof(int) / sizeof(char) * 2);
}    

inline int CubePruningTable::GetInDataPos(int coord)
{
    return 4 * coord % (sizeof(int) / sizeof(char) * 2);
}    


////////////////////////////////////////////////////////////////////////////////
// HELPER METHODS FOR TABLE GENERATION
////////////////////////////////////////////////////////////////////////////////

void CubePruningTable::PreGenerate()
{
    Cube::SetValidation(false);
}

void CubePruningTable::PostGenerate()
{
    Cube::SetValidation(true);
}


////////////////////////////////////////////////////////////////////////////////
// CORE METHODS - GENERATION (private)
////////////////////////////////////////////////////////////////////////////////

void CubePruningTable::Generate() 
{
    int depth = 0;      // current depth of tree
    int nodesNum;       // number of created nodes of tree
    int coord, coord2;  // indices of table
    int move, turn;
    
    // Clears table
    for (coord=0; coord<=GetDataCoord(iCoordSize-1); coord++)
        SetData(coord, 0xffffffff);
    
    // Prepare data for generation
    PreGenerate();
    
    // Prepares root
    Set(GetInitCoord(), depth);
    nodesNum = 1; 
    if (bLog) printf("Generates %d nodes for pruning table.\n", iCoordSize);

    // For first part of fields
    while (nodesNum < iCoordSize/2) 
    {
        // For all fields of the table
        for (coord=0; coord<iCoordSize; coord++) 
        {
            // Expand nodes on current depth
            if (Get(coord) == depth) 
            {
                // For all possible moves
                for (move=0; move<CubeMove::MOVE_NUMBER; move++) 
                    // For all different powers of move
                    for (turn=0; turn<CubeMove::TURN_NUMBER; turn++) 
                        // Perform move only if is allowed
                        if (IsAllowed((CubeMove::eMove) move, (CubeMove::eTurn) turn))
                        {
                            // Get coord number identifying coord after move
                            coord2 = GetCoord(coord, (CubeMove::eMove) move, (CubeMove::eTurn) turn);

                            // If child node not yet reached, fill it in table
                            if (Get(coord2) == MAX_DEPTH) 
                            {
                                Set(coord2, depth+1);
                                nodesNum++;
                            }
                        }
            }
        }

        if (bLog) printf("Depth %d completed, %d nodes to go.\n", depth, 
           (iCoordSize - nodesNum));
        depth++;
    }

    // While there are not filled fields in table
    while (nodesNum < iCoordSize) 
    {
        // For all fields of the table
        for (coord=0; coord<iCoordSize; coord++) 
        {
            // Choose not yet expanded nodes
            if (Get(coord) == MAX_DEPTH) 
            {
                // For all possible moves
                for (move=0; move<CubeMove::MOVE_NUMBER; move++) 
                    // For all different powers of move
                    for (turn=0; turn<CubeMove::TURN_NUMBER; turn++) 
                        // Perform move only if is allowed
                        if (IsAllowed((CubeMove::eMove) move, (CubeMove::eTurn) turn))
                        {
                            // Get coord number identifying coord after move
                            coord2 = GetCoord(coord, (CubeMove::eMove) move, (CubeMove::eTurn) turn);

                            // If dest node has been reached, fill coord as it's child
                            if (Get(coord2) == depth) 
                            {
                                Set(coord, depth+1);
                                nodesNum++;
                                turn = CubeMove::TURN_NUMBER;
                                move = CubeMove::MOVE_NUMBER;
                            }
                        }
            }
        }

        if (bLog) printf("Depth %d completed, %d nodes to go.\n", depth, 
           (iCoordSize - nodesNum));
        depth++;
    }

    // Clears data used during generation
    PostGenerate();
}
