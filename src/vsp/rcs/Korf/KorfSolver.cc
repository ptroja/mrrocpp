/////////////////////////////////////////////////////////
// Nazwa: KorfSolver.cpp
// Implementacja : KorfSolver
/////////////////////////////////////////////////////////


#include "vsp/rcs/Korf/KorfSolver.h"
#include "vsp/rcs/Korf/KorfPruningTable_Corner.h"
#include "vsp/rcs/Korf/KorfPruningTable_Edge.h"
#include "vsp/rcs/Korf/KorfException.h"

#include <stdio.h>


#define RET_SUCCESS   0
#define RET_FAILURE   1
#define WARN_NONE           0
#define WARN_ALREADY_SOLVED 1
#define ERR_NONE      0     

#define HUGE_INT 10000
#define SEARCH_LIMIT 10000000


KorfPruningTable *KorfSolver::ptCorner = NULL;
KorfPruningTable *KorfSolver::ptEdgeFirstHalf = NULL;
KorfPruningTable *KorfSolver::ptEdgeSecondHalf = NULL;


bool KorfSolver::bInitialized = false;


void KorfSolver::Init() {
    SInit();
}

void KorfSolver::SInit() {
    
    // check if already initiated
    if (bInitialized)
        return; 

    bInitialized = true;
        
    // create pruning tables
    ptCorner         = new KorfPruningTable_Corner();
    ptEdgeFirstHalf  = new KorfPruningTable_Edge(true);
    ptEdgeSecondHalf = new KorfPruningTable_Edge(false);

    // initialize pruning tables
    ptCorner->Init();
    ptEdgeFirstHalf->Init();
    ptEdgeSecondHalf->Init();
}    

void KorfSolver::SClear() {
     
    // check if initiated
    if (!bInitialized)
        return; 

    bInitialized = false;
            
    // delete pruning tables
    delete ptCorner;
    delete ptEdgeFirstHalf;
    delete ptEdgeSecondHalf;
}    
   
int KorfSolver::Heuristic(Cube* cube) 
{
    int cost, cost2;
    int coords[3];
    
    // get coordinates of the cube
    ((KorfCube*) cube)->ToCoords(coords);

    // joins three heuristics by choosing maximum value
    cost = ptCorner->Get(coords[0]);
    cost2 = ptEdgeFirstHalf->Get(coords[1]);
    if (cost2 > cost) cost = cost2;
    cost2 = ptEdgeSecondHalf->Get(coords[2]);
    if (cost2 > cost) cost = cost2;

    return cost;
}

CubeSolution* KorfSolver::FindSolution(Cube* scrambledCube)
    throw (Exception)
{
    if (strcmp(scrambledCube->GetType(), KorfCube::TYPE) == 0)
        return CubeSolver::FindSolution(scrambledCube);
    else if (strcmp(scrambledCube->GetType(), CubieCube::TYPE) == 0)
    {
        KorfCube *pKCube = new KorfCube();
        pKCube->FromCubieCube(*((CubieCube*) scrambledCube));
        CubeSolution *ret_sol = CubeSolver::FindSolution(pKCube);
        delete pKCube;
        return ret_sol;
    }
    else
        throw new KorfException(KorfException::ERR_SOLVER_INPUT_INVALID);
}
  
