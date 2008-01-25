/////////////////////////////////////////////////////////
// Nazwa: KociembaSolver.cpp
// Implementacja : KociembaSolver
/////////////////////////////////////////////////////////


#include "vsp/rcs/Kociemba/KociembaSolver.h"
#include "vsp/rcs/Kociemba/KociembaPhase1Solver.h"
#include "vsp/rcs/Kociemba/KociembaPhase2Solver.h"


CubeSolution* KociembaSolver::FindSolution(Cube* scrambledCube)
              throw (Exception)
{
    Cube* cube = NULL;
    
    // check type of scrambledCube -> accept only Cubie or Kociemba
    if (strcmp(scrambledCube->GetType(), CubieCube::TYPE) != 0
     && strcmp(scrambledCube->GetType(), KociembaPhase1Cube::TYPE) != 0
     && strcmp(scrambledCube->GetType(), KociembaPhase2Cube::TYPE) != 0)
        throw new KociembaException(KociembaException::ERR_SOLVER_INPUT_INVALID);

    // solve phase 1
    CubeSolution *ph1Solution;
    if (strcmp(scrambledCube->GetType(), KociembaPhase2Cube::TYPE) == 0)
    {
        ph1Solution = new CubeSolution();
        cube = scrambledCube->Clone();
    }
    else 
    {
        KociembaPhase1Solver *ph1Solver = new KociembaPhase1Solver();
        if (strcmp(scrambledCube->GetType(), CubieCube::TYPE) == 0)
            cube = scrambledCube->Clone();
        else // if (strcmp(scrambledCube->GetType(), KociembaPhase1Cube::TYPE) == 0)
        {
            cube = new CubieCube();  
            ((KociembaPhase1Cube*) scrambledCube)->ToCubieCube(*((CubieCube*) cube));
        }
        ph1Solution = ph1Solver->FindSolution(cube);
        CubeSolver::Solve(cube, *ph1Solution);
        delete ph1Solver;
    }
    
    // solve phase 2
    KociembaPhase2Solver *ph2Solver = new KociembaPhase2Solver();
    CubeSolution *ph2Solution = ph2Solver->FindSolution(cube);
    delete ph2Solver;
    delete cube;

    // join solutions
    CubeSolution *ret_sol = new KociembaSolution(*ph1Solution, *ph2Solution);

    delete ph1Solution;
    delete ph2Solution;
    return ret_sol;
}

CubeSolution* KociembaSolver::FindNextSolution(Cube* scrambledCube, const KociembaSolution& solution)
              throw (Exception)
{
    Cube* cube = NULL;
    
    if (solution.GetLength() == 0)
        return FindSolution(scrambledCube);

    // check type of scrambledCube -> accept only Cubie or Kociemba
    if (strcmp(scrambledCube->GetType(), CubieCube::TYPE) != 0
     && strcmp(scrambledCube->GetType(), KociembaPhase1Cube::TYPE) != 0
     && strcmp(scrambledCube->GetType(), KociembaPhase2Cube::TYPE) != 0)
        throw KociembaException(KociembaException::ERR_SOLVER_INPUT_INVALID);

    // if scrambledCube is of phase 2, only one solution may be found
    if (strcmp(scrambledCube->GetType(), KociembaPhase2Cube::TYPE) == 0
     || solution.GetInfo(CubeSolution::INFO_OPTIMAL))
        throw KociembaException(KociembaException::ERR_SOLVER_NEXT_IMPOSSIBLE);

    // if previous Phase 1 solution is empty, next solution cannot be found
    CubeSolution *ph1PrevSolution = GetPhase1Solution(solution);
    if (ph1PrevSolution->GetLength() == 0)
    {
        delete ph1PrevSolution;
        throw KociembaException(KociembaException::ERR_SOLVER_NEXT_IMPOSSIBLE);
    }

    // solve phase 1
    KociembaPhase1Solver *ph1Solver = new KociembaPhase1Solver();
    if (strcmp(scrambledCube->GetType(), CubieCube::TYPE) == 0)
        cube = scrambledCube->Clone();
    else // if (strcmp(scrambledCube->GetType(), KociembaPhase1Cube::TYPE) == 0)
    {
        cube = new CubieCube();  
        ((KociembaPhase1Cube*) scrambledCube)->ToCubieCube(*((CubieCube*) cube));
    }
    CubeSolution *ph1Solution = ph1Solver->FindNextSolution(cube, *ph1PrevSolution);
    CubeSolver::Solve(cube, *ph1Solution);
    delete ph1PrevSolution;
    delete ph1Solver;

    // solve phase 2
    KociembaPhase2Solver *ph2Solver = new KociembaPhase2Solver();
    CubeSolution *ph2Solution = ph2Solver->FindSolution(cube);
    delete ph2Solver;
    delete cube;

    // join solutions
    CubeSolution *ret_sol = new KociembaSolution(*ph1Solution, *ph2Solution);

    delete ph1Solution;
    delete ph2Solution;
    return ret_sol;
}

CubeSolution* KociembaSolver::FindOptimalSolution(Cube* scrambledCube)
    throw (Exception)
{
    CubieCube* cube = NULL;
    
    // check type of scrambledCube -> accept only Cubie or Kociemba
    if (strcmp(scrambledCube->GetType(), CubieCube::TYPE) != 0
     && strcmp(scrambledCube->GetType(), KociembaPhase1Cube::TYPE) != 0
     && strcmp(scrambledCube->GetType(), KociembaPhase2Cube::TYPE) != 0)
        throw new KociembaException(KociembaException::ERR_SOLVER_INPUT_INVALID);

    // solve phase 1
    CubeSolution *ph1Solution;
    if (strcmp(scrambledCube->GetType(), KociembaPhase2Cube::TYPE) == 0)
    {
        ph1Solution = new CubeSolution();
        cube = (CubieCube*) scrambledCube->Clone();
    }
    else 
    {
        KociembaPhase1Solver *ph1Solver = new KociembaPhase1Solver();
        if (strcmp(scrambledCube->GetType(), CubieCube::TYPE) == 0)
            cube = (CubieCube*) scrambledCube->Clone();
        else // if (strcmp(scrambledCube->GetType(), KociembaPhase1Cube::TYPE) == 0)
        {
            cube = new CubieCube();  
            ((KociembaPhase1Cube*) scrambledCube)->ToCubieCube(*((CubieCube*) cube));
        }
        
        // find first phase 1solution
        ph1Solution = ph1Solver->FindSolution(cube);

        if (!ph1Solution->GetInfo(CubeSolution::INFO_ALREADY_SOLVED))
        {        
            // check if it full solution
            CubeSolution *csol;
            while (!IsFullSolution(cube, *ph1Solution))
            {
                csol = ph1Solver->FindNextSolution(cube, *ph1Solution);
                delete ph1Solution;
                ph1Solution = csol;
            }
        }

        // prepare cube for finding phase 2 solution
        CubeSolver::Solve(cube, *ph1Solution);
        delete ph1Solver;
    }
    
    // solve phase 2
    KociembaPhase2Solver *ph2Solver = new KociembaPhase2Solver();
    CubeSolution *ph2Solution = ph2Solver->FindSolution(cube);
    delete ph2Solver;
    delete cube;

    // join solutions
    CubeSolution *ret_sol = new KociembaSolution(*ph1Solution, *ph2Solution);

    delete ph1Solution;
    delete ph2Solution;
    return ret_sol;
}

CubeSolution* KociembaSolver::GetPhase1Solution(const KociembaSolution& solution)
{
    CubeSolution *ret_sol = new CubeSolution();
    CubeMove::eMove move;
    CubeMove::eTurn turn;
    for (int i=0; i<solution.GetPhase1Length(); i++)
    {
        solution.GetMove(i, move, turn);
        ret_sol->SetMove(i, move, turn);
    }   

    return ret_sol;
}

bool KociembaSolver::IsFullSolution(CubieCube* scrambledCube, const CubeSolution& solution)
    throw (Exception)
{
    CubieCube cube(*scrambledCube), cube2;
    CubeSolver::Solve(&cube, solution);
    return (cube==cube2);
}

void KociembaSolver::SInit()
{
    KociembaPhase1Solver::SInit();
    KociembaPhase2Solver::SInit();
}
