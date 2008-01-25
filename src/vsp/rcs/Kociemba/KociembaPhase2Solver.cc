/////////////////////////////////////////////////////////
// Nazwa: KociembaPhase2Solver.cpp
// Implementacja : KociembaPhase2Solver
/////////////////////////////////////////////////////////

#include "vsp/rcs/Kociemba/KociembaPhase2Solver.h"


#define RET_SUCCESS   0
#define RET_FAILURE   1
#define WARN_NONE           0
#define WARN_ALREADY_SOLVED 1
#define ERR_NONE      0     

#define HUGE_INT 10000
#define SEARCH_LIMIT 10000000


KociembaPhase2PruningTable *KociembaPhase2Solver::ptCPnEMP = NULL;
KociembaPhase2PruningTable *KociembaPhase2Solver::ptEMPnENP = NULL;


bool KociembaPhase2Solver::bInitialized = false;


void KociembaPhase2Solver::Init() {
    SInit();
}

void KociembaPhase2Solver::SInit() {
    // check if already initiated
    if (bInitialized) 
        return;

    bInitialized = true;
    
    // create pruning tables
    ptCPnEMP  = new KociembaPhase2PruningTable(0, 1);
    ptEMPnENP = new KociembaPhase2PruningTable(1, 2);

    KociembaPhase2Cube::SInit();
        
    // initialize pruning tables
    ptCPnEMP->Init();
    ptEMPnENP->Init();
}    
   
void KociembaPhase2Solver::SClear() {
    // check if initiated
    if (!bInitialized) 
        return;

    bInitialized = false;
    
    // delete pruning tables
    delete ptCPnEMP;
    delete ptEMPnENP;
}    
   
CubeSolution* KociembaPhase2Solver::FindSolution(Cube* scrambledCube)
    throw (Exception)
{
    if (strcmp(scrambledCube->GetType(), KociembaPhase2Cube::TYPE) == 0)
        return CubeSolver::FindSolution(scrambledCube);
    else if (strcmp(scrambledCube->GetType(), CubieCube::TYPE) == 0)
    {
        KociembaPhase2Cube *pKCube = new KociembaPhase2Cube();
        CubeSolution *ret_sol = NULL;
        try 
        {
            pKCube->FromCubieCube(*((CubieCube*) scrambledCube));
            ret_sol = CubeSolver::FindSolution(pKCube);
        }
        catch (KociembaException& exp)
        {
            delete pKCube;
            throw exp;
        }
        delete pKCube;
        return ret_sol;
    }
    else
        throw new KociembaException(KociembaException::ERR_PHASE2_SOLVER_INPUT_INVALID);
}
   
int KociembaPhase2Solver::Heuristic(Cube* cube) 
{
    int cost, cost2;
    int coords[3];
    
    // get coordinates of the cube
    ((KociembaCube*) cube)->ToCoords(coords);

    // joins three heuristics by choosing maximum value
    cost = ptCPnEMP->Get(coords[0], coords[1]);
    cost2 = ptEMPnENP->Get(coords[1], coords[2]);
    if (cost2 > cost) cost = cost2;

    return cost;
}
    
