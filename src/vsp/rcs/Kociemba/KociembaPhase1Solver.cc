/////////////////////////////////////////////////////////
// Nazwa: KociembaPhase1Solver.cpp
// Implementacja : KociembaPhase1Solver
/////////////////////////////////////////////////////////

#include "vsp/rcs/Kociemba/KociembaPhase1Solver.h"
#include "vsp/rcs/Kociemba/KociembaException.h"


KociembaPhase1PruningTable *KociembaPhase1Solver::ptCOnEMC = NULL;
KociembaPhase1PruningTable *KociembaPhase1Solver::ptCOnEO = NULL;
KociembaPhase1PruningTable *KociembaPhase1Solver::ptEMCnEO = NULL;

bool KociembaPhase1Solver::bInitialized = false;


void KociembaPhase1Solver::Init() 
{
    SInit();
}

void KociembaPhase1Solver::SInit() 
{
    // check if already initiated
    if (bInitialized) 
        return;

    bInitialized = true;
    
    KociembaPhase1Cube::SInit();
        
    // create pruning tables
    ptCOnEMC = new KociembaPhase1PruningTable(0, 1);
    ptCOnEO  = new KociembaPhase1PruningTable(0, 2);
    ptEMCnEO = new KociembaPhase1PruningTable(1, 2);

    // initialize pruning tables
    ptCOnEMC->Init();
    ptCOnEO->Init();
    ptEMCnEO->Init();
}    

void KociembaPhase1Solver::SClear() 
{
    // check if initiated
    if (!bInitialized)
        return;

    bInitialized = false;
        
    // delete pruning tables
    delete ptCOnEMC;
    delete ptCOnEO;
    delete ptEMCnEO;
}    

CubeSolution* KociembaPhase1Solver::FindSolution(Cube* scrambledCube)
    throw (Exception)
{
    if (strcmp(scrambledCube->GetType(), KociembaPhase1Cube::TYPE) == 0)
        return CubeSolver::FindSolution(scrambledCube);
    else if (strcmp(scrambledCube->GetType(), CubieCube::TYPE) == 0)
    {
        KociembaPhase1Cube *pKCube = new KociembaPhase1Cube();
        pKCube->FromCubieCube(*((CubieCube*) scrambledCube));
        CubeSolution *ret_sol = CubeSolver::FindSolution(pKCube);
        delete pKCube;
        return ret_sol;
    }
    else
        throw new KociembaException(KociembaException::ERR_PHASE1_SOLVER_INPUT_INVALID);
}

CubeSolution* KociembaPhase1Solver::FindNextSolution(Cube* scrambledCube, const CubeSolution& solution)
    throw (Exception)
{
    if (strcmp(scrambledCube->GetType(), KociembaPhase1Cube::TYPE) == 0)
        return CubeSolver::FindNextSolution(scrambledCube, solution);
    else if (strcmp(scrambledCube->GetType(), CubieCube::TYPE) == 0)
    {
        KociembaPhase1Cube *pKCube = new KociembaPhase1Cube();
        pKCube->FromCubieCube(*((CubieCube*) scrambledCube));
        CubeSolution *ret_sol = CubeSolver::FindNextSolution(pKCube, solution);
        delete pKCube;
        return ret_sol;
    }
    else
        throw new KociembaException(KociembaException::ERR_PHASE1_SOLVER_INPUT_INVALID);
}
   
int KociembaPhase1Solver::Heuristic(Cube* cube) 
{
    int cost, cost2;
    int coords[3];
    
    // get coordinates of the cube
    ((KociembaCube*) cube)->ToCoords(coords);

    // joins three heuristics by choosing maximum value
    cost = ptCOnEMC->Get(coords[0], coords[1]);
    cost2 = ptCOnEO->Get(coords[0], coords[2]);
    if (cost2 > cost) cost = cost2;
    cost2 = ptEMCnEO->Get(coords[1], coords[2]);
    if (cost2 > cost) cost = cost2;

    return cost;
}
    
