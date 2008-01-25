/////////////////////////////////////////////////////////
// Name: KociembaTester.cpp
// Implements : KociembaTester
/////////////////////////////////////////////////////////

#include "vsp/rcs/Kociemba/KociembaTester.h"
#include "vsp/rcs/Kociemba/KociembaCube.h"
#include "vsp/rcs/Kociemba/KociembaPhase1MoveTable.h"
#include "vsp/rcs/Kociemba/KociembaPhase2MoveTable.h"
#include "vsp/rcs/Kociemba/KociembaSolver.h"
#include "vsp/rcs/Kociemba/KociembaPhase1Solver.h"
#include "vsp/rcs/Kociemba/KociembaPhase2Solver.h"

#include <string.h>
#include <stdlib.h> 
#include <stdio.h>
#include <time.h>


////////////////////////////////////////////////////////////////////////////////
// MAIN TEST METHOD
////////////////////////////////////////////////////////////////////////////////

bool KociembaTester::Test()
{
    printf("TEST KOCIEMBA\n\n");
    bool result = true;

    // initialize random seed
    srand (time(NULL));
    
    if (!TestKociemba1Cube()) result = false;
    if (!TestKociemba2Cube()) result = false;
    if (!TestKociembaSolver()) result = false;

    // Free static memory
    KociembaPhase1Cube::SClear();
    KociembaPhase2Cube::SClear();
    KociembaPhase1Solver::SClear();
    KociembaPhase2Solver::SClear();
    
    printf("TEST KOCIEMBA : %s\n\n\n", ResultText(result));
    if (bLog) printf("\n");
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE Kociemba1 CUBE TESTS
////////////////////////////////////////////////////////////////////////////////

bool KociembaTester::TestKociemba1Cube()
{
    printf("TEST KOCIEMBA1_CUBE\n");
    if (bLog) printf("\n");
    bool result = true, subres, subres2;


    // Test cube creation
    if (bLog) printf("TEST Create\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociemba1CubeCreate();
        printf("TEST Create (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Create - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube to/from cubies
    if (bLog) printf("TEST To&From Cubies\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociemba1CubeCubie();
        printf("TEST To&From Cubies (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST To&From Cubies - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube move tables
    if (bLog) printf("TEST MoveTables\n");
    subres = true;
    KociembaPhase1MoveTable* mt;
    
    mt = new KociembaPhase1MoveTable(0);
    subres2 = TestKociemba1CubeMoveTable(mt);
    printf("TEST MoveTables (CO) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    mt = new KociembaPhase1MoveTable(1);
    subres2 = TestKociemba1CubeMoveTable(mt);
    printf("TEST MoveTables (EMC) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    mt = new KociembaPhase1MoveTable(2);
    subres2 = TestKociemba1CubeMoveTable(mt);
    printf("TEST MoveTables (EO) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    printf("TEST MoveTables - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube move
    if (bLog) printf("TEST Move\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociemba1CubeMove();
        printf("TEST Move (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Move - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST KOCIEMBA1_CUBE : %s\n\n", ResultText(subres));
    if (bLog) printf("\n");
    return result;
}

bool KociembaTester::TestKociemba1CubeCreate()
{
    bool result = true;
    
    // Create Kociemba cube and apply moves
    KociembaPhase1Cube cube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &cube, iRep);
    Print(moves, turns, iRep);
    Move(&cube, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    Print("K1", &cube);

    // Create cube from cube
    KociembaPhase1Cube cube2(cube);
    Print("K2", &cube2);
    if (!(cube==cube2)) result = false;
    
    // Clone cube from cube
    KociembaPhase1Cube *cube3 = (KociembaPhase1Cube*) cube.Clone();
    Print("K3", cube3);
    if (!(cube==*cube3)) result = false;
    delete cube3;
         
    return result;
}

bool KociembaTester::TestKociemba1CubeCubie()
{
    bool result = true;
int iRep = 1;
    // Apply moves on Kociemba
    KociembaPhase1Cube kcube1;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &kcube1, iRep);
    Print(moves, turns, iRep);
    Move(&kcube1, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    
    // Do conversions
    CubieCube ccube1;
    KociembaPhase1Cube kcube2;
    CubieCube ccube2;
    kcube1.ToCubieCube(ccube1);
    kcube2.FromCubieCube(ccube1);
    kcube2.ToCubieCube(ccube2);

    // Print cubes
    Print("K1", &kcube1);
    Print("K2", &kcube2);
    Print("C1", &ccube1);
    Print("C2", &ccube2);

    // Compare
    if (! (kcube1==kcube2 && ccube1==ccube2)) result = false;
         
    return result;
}

bool KociembaTester::TestKociemba1CubeMoveTable(KociembaMoveTable* mt)
{
    bool result = true;
    if (bLog) printf("\tparams: %s\n", mt->GetName());

    // prepare table
    mt->Init();
    
    // create empty usage table
    int size = mt->GetSize();
    int *usage = new int[size];
    for (int i=0; i<size; i++)
        usage[i] = 0;

    // count usage of each value in table
    int val;
    for (int i=0; i<size; i++)
        for (int j=0; j<CubeMove::MOVE_NUMBER; j++)
        {
            val = mt->Get(i, (CubeMove::eMove) j);
            if (val < 0 || val >= size)
            {
                result = false;
                if (bLog) printf("MT contains invalid value on [%d,%d] = %d\n", i, j, val);
            }
            else
                usage[val]++;
        }
    
    // print counters on screen
    if (bLog) 
        for (int i=0; i<size; i++)
            if (usage[i] != 6)
            {
                result = false;
                printf("\tMT contains invalid number (%d) of %d\n", usage[i], i);
            }

    delete[] usage;
    return result;
}

bool KociembaTester::TestKociemba1CubeMove()
{
    bool result = true;
    
    // Apply moves on Kociemba
    KociembaPhase1Cube kcube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &kcube, iRep);
    Print(moves, turns, iRep);
    Move(&kcube, moves, turns, iRep, true);
    Print("KFOR", &kcube);

    // Apply moves on Cubie and compare
    CubieCube ccube;
    Move(&ccube, moves, turns, iRep, true);
    KociembaPhase1Cube kcube2;
    kcube2.FromCubieCube(ccube);
    Print("CFOR", &kcube2);
    if (!(kcube==kcube2)) result = false;
    
    // Apply moves backwards
    Move(&kcube, moves, turns, iRep, false);
    Print("KBAC", &kcube);
    delete[] moves;
    delete[] turns;
        
    // Compare with initialized cube
    KociembaPhase1Cube kcube3;
    if (!(kcube==kcube3)) result = false;
         
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE Kociemba CUBE TESTS
////////////////////////////////////////////////////////////////////////////////

bool KociembaTester::TestKociemba2Cube()
{
    printf("TEST KOCIEMBA2_CUBE\n");
    if (bLog) printf("\n");
    bool result = true, subres, subres2;


    // Test from cubie exceptions
    if (bLog) printf("TEST CubieException\n");
    subres = true;
    const CubeMove::eMove BAD_MOVES[4] = { CubeMove::F, CubeMove::R, CubeMove::B, CubeMove::L };
    CubieCube *ccube = new CubieCube();
    for (int i=0; i<4; i++)
    {
        ccube->Move(BAD_MOVES[i], CubeMove::QUARTER);
        subres2 = TestKociemba2CubeCubieException(*ccube);
        printf("TEST CubieException (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
        ccube->Move(BAD_MOVES[i], CubeMove::ANTI_QUARTER);
    }
    for (int i=0; i<4; i++)
    {
        ccube->Move(BAD_MOVES[i], CubeMove::ANTI_QUARTER);
        subres2 = TestKociemba2CubeCubieException(*ccube);
        printf("TEST CubieException (%d) %s\n", 4+i, ResultText(subres2));
        if (!subres2) subres = false;
        ccube->Move(BAD_MOVES[i], CubeMove::QUARTER);
    }
    delete ccube;
    printf("TEST CubieException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube creation
    if (bLog) printf("TEST Create\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociemba2CubeCreate();
        printf("TEST Create (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Create - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube to/from cubies
    if (bLog) printf("TEST To&From Cubies\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociemba2CubeCubie();
        printf("TEST To&From Cubies (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST To&From Cubies - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube move tables
    if (bLog) printf("TEST MoveTables\n");
    subres = true;
    KociembaMoveTable* mt;
    
    mt = new KociembaPhase2MoveTable(0);
    subres2 = TestKociemba2CubeMoveTable(mt);
    printf("TEST MoveTables (CP) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    mt = new KociembaPhase2MoveTable(1);
    subres2 = TestKociemba2CubeMoveTable(mt);
    printf("TEST MoveTables (EMP) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    mt = new KociembaPhase2MoveTable(2);
    subres2 = TestKociemba2CubeMoveTable(mt);
    printf("TEST MoveTables (ENP) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    printf("TEST MoveTables - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");

    // Test cube move exceptions
    if (bLog) printf("TEST MoveException\n");
    subres = true;
    for (int i=0; i<4; i++)
    {
        subres2 = TestKociemba2CubeMoveException(BAD_MOVES[i], CubeMove::QUARTER);
        printf("TEST MoveException (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    for (int i=0; i<4; i++)
    {
        subres2 = TestKociemba2CubeMoveException(BAD_MOVES[i], CubeMove::ANTI_QUARTER);
        printf("TEST MoveException (%d) %s\n", i+4, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST MoveException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");

    
    // Test cube move
    if (bLog) printf("TEST Move\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociemba2CubeMove();
        printf("TEST Move (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Move - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST KOCIEMBA2_CUBE : %s\n\n", ResultText(result));
    if (bLog) printf("\n");
    return result;
}

bool KociembaTester::TestKociemba2CubeCubieException(CubieCube& ccube)
{
    Print("params: ", &ccube);
    try 
    {
        KociembaPhase2Cube cube;
        cube.FromCubieCube(ccube);
    } 
    catch (KociembaException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool KociembaTester::TestKociemba2CubeCreate()
{
    bool result = true;
    
    // Create Kociemba cube and apply moves
    KociembaPhase2Cube cube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &cube, iRep);
    Print(moves, turns, iRep);
    
    try {
    Move(&cube, moves, turns, iRep, true);
    } catch (Exception& exp) {
    char* str = exp.ToString();
    printf("EXP = %s\n", str);
    delete[] str;
    }
    
    delete[] moves;
    delete[] turns;
    Print("K1", &cube);

    // Create cube from cube
    KociembaPhase2Cube cube2(cube);
    Print("K2", &cube2);
    if (!(cube==cube2)) result = false;
    
    // Clone cube from cube
    KociembaPhase2Cube *cube3 = (KociembaPhase2Cube*) cube.Clone();
    Print("K3", cube3);
    if (!(cube==*cube3)) result = false;
    delete cube3;

    return result;
}

bool KociembaTester::TestKociemba2CubeCubie()
{
    bool result = true;

    // Apply moves on Kociemba
    KociembaPhase2Cube kcube1;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &kcube1, iRep);
    Print(moves, turns, iRep);
    Move(&kcube1, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    
    // Do conversions
    CubieCube ccube1;
    KociembaPhase2Cube kcube2;
    CubieCube ccube2;
    kcube1.ToCubieCube(ccube1);
    kcube2.FromCubieCube(ccube1);
    kcube2.ToCubieCube(ccube2);

    // Print cubes
    Print("K1", &kcube1);
    Print("K2", &kcube2);
    Print("C1", &ccube1);
    Print("C2", &ccube2);

    // Compare
    if (! (kcube1==kcube2 && ccube1==ccube2)) result = false;
         
    return result;
}

bool KociembaTester::TestKociemba2CubeMoveTable(KociembaMoveTable* mt)
{
    bool result = true;
    if (bLog) printf("\tparams: %s\n", mt->GetName());

    // prepare table
    mt->Init();
    
    // create empty usage table
    int size = mt->GetSize();
    int *usage = new int[size];
    for (int i=0; i<size; i++)
        usage[i] = 0;

    // count usage of each value in table
    int val;
    for (int i=0; i<size; i++)
        for (int j=0; j<CubeMove::MOVE_NUMBER; j++)
        {
            val = mt->Get(i, (CubeMove::eMove) j);
            if (val < 0 || val >= size)
            {
                result = false;
                if (bLog) printf("MT contains invalid value on [%d,%d] = %d\n", i, j, val);
            }
            else
                usage[val]++;
        }
    
    // print counters on screen
    if (bLog) 
        for (int i=0; i<size; i++)
            if (usage[i] != 6)
            {
                result = false;
                printf("\tMT contains invalid number (%d) of %d\n", usage[i], i);
            }

    delete[] usage;
    return result;
}

bool KociembaTester::TestKociemba2CubeMoveException(CubeMove::eMove move, CubeMove::eTurn turn)
{
    if (bLog) printf("\tparams: %c%d", CubeMove::MOVE_NAMES[move], turn+1);
    try 
    {
        KociembaPhase2Cube cube;
        cube.Move(move, turn);
    } 
    catch (KociembaException& exp)
    {
        Print("MEXP", &exp);
        return true;
    }
         
    return false;
}

bool KociembaTester::TestKociemba2CubeMove()
{
    bool result = true;
    
    // Apply moves on Kociemba
    KociembaPhase2Cube kcube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &kcube, iRep);
    Print(moves, turns, iRep);
    Move(&kcube, moves, turns, iRep, true);
    Print("KFOR", &kcube);

    // Apply moves on Cubie and compare
    CubieCube ccube;
    Move(&ccube, moves, turns, iRep, true);
    KociembaPhase2Cube kcube2;
    kcube2.FromCubieCube(ccube);
    Print("CFOR", &kcube2);
    if (!(kcube==kcube2)) result = false;
    
    // Apply moves backwards
    Move(&kcube, moves, turns, iRep, false);
    Print("KBAC", &kcube);
    delete[] moves;
    delete[] turns;
        
    // Compare with initialized cube
    KociembaPhase2Cube kcube3;
    if (!(kcube==kcube3)) result = false;
         
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// TEST SOLVING Kociemba CUBE
////////////////////////////////////////////////////////////////////////////////

bool KociembaTester::TestKociembaSolver()
{
    printf("TEST KOCIEMBA_SOLVER\n");
    if (bLog) printf("\n");
    bool result = true, subres, subres2;
    
    // repeated parameters
    CubeMove::eMove *moves = NULL;
    CubeMove::eTurn *turns = NULL;

    
    // Test solve Phase1
    if (bLog) printf("TEST SolvePhase1\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociembaSolverSolvePhase1();
        printf("TEST SolvePhase1 (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST SolvePhase1 - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test solve Phase1 next
    if (bLog) printf("TEST SolvePhase1Next\n");
    subres = true;
    KociembaPhase1Cube *kcube = new KociembaPhase1Cube();
    CubeSolution *csol = new CubeSolution();
    moves = new CubeMove::eMove[iRep];
    turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, kcube, iRep);
    Print(moves, turns, iRep);
    Move(kcube, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    Print("CUBE", kcube);
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociembaSolverSolvePhase1Next(*kcube, csol);
        printf("TEST SolvePhase1Next (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    delete kcube;
    delete csol;
    printf("TEST SolvePhase1Next - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test from cubie exceptions
    if (bLog) printf("TEST SolvePhase2Exception\n");
    subres = true;
    const CubeMove::eMove BAD_MOVES[4] = { CubeMove::F, CubeMove::R, CubeMove::B, CubeMove::L };
    CubieCube *ccube = new CubieCube();
    for (int i=0; i<4; i++)
    {
        ccube->Move(BAD_MOVES[i], CubeMove::QUARTER);
        subres2 = TestKociembaSolverSolvePhase2Exception(*ccube);
        printf("TEST SolvePhase2Exception (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
        ccube->Move(BAD_MOVES[i], CubeMove::ANTI_QUARTER);
    }
    for (int i=0; i<4; i++)
    {
        ccube->Move(BAD_MOVES[i], CubeMove::ANTI_QUARTER);
        subres2 = TestKociembaSolverSolvePhase2Exception(*ccube);
        printf("TEST SolvePhase2Exception (%d) %s\n", 4+i, ResultText(subres2));
        if (!subres2) subres = false;
        ccube->Move(BAD_MOVES[i], CubeMove::QUARTER);
    }
    delete ccube;
    printf("TEST SolvePhase2Exception - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test solve Phase2
    if (bLog) printf("TEST SolvePhase2\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociembaSolverSolvePhase2();
        printf("TEST SolvePhase2 (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST SolvePhase2 - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test solve
    if (bLog) printf("TEST Solve\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociembaSolverSolve();
        printf("TEST Solve (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Solve - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test solve next
    if (bLog) printf("TEST SolveNext\n");
    subres = true;
    ccube = new CubieCube();
    KociembaSolution *ksol = new KociembaSolution();
    moves = new CubeMove::eMove[iRep];
    turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, ccube, iRep);
    Print(moves, turns, iRep);
    Move(ccube, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    Print("CUBE", ccube);
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociembaSolverSolveNext(*ccube, ksol);
        printf("TEST SolveNext (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    delete ccube;
    delete ksol;
    printf("TEST SolveNext - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test solve
    if (bLog) printf("TEST SolveOptimal\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKociembaSolverSolveOptimal();
        printf("TEST SolveOptimal (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST SolveOptimal - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST KOCIEMBA_SOLVER : %s\n\n", ResultText(result));
    if (bLog) printf("\n");
    return result;
}

bool KociembaTester::TestKociembaSolverSolvePhase1()
{
    bool result = true;
    
    // Create cube and apply moves on it
    KociembaPhase1Cube kcube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &kcube, iRep);
    Print(moves, turns, iRep);
    Move(&kcube, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    Print("CUBE", &kcube);

    // Find solution and solve cube
    KociembaPhase1Solver kSolver;
    CubeSolution *pSol = kSolver.FindSolution(&kcube);
    Print("CSOL", pSol);
    CubeSolver::Solve(&kcube, *pSol);
    delete pSol;
        
    // Compare with initialized cube
    KociembaPhase1Cube kcube2;
    if (!(kcube==kcube2)) result = false;
         
    return result;
}

bool KociembaTester::TestKociembaSolverSolvePhase1Next(KociembaPhase1Cube& cube, CubeSolution *sol)
{
    bool result = true;
    
    // Copy cube
    KociembaPhase1Cube kcube(cube);
   
    // Find solution and solve cube
    KociembaPhase1Solver kSolver;
    CubeSolution *csol = kSolver.FindNextSolution(&kcube, *sol);
    Print("CSOL", csol);
    CubeSolver::Solve(&kcube, *csol);
        
    // Compare with initialized cube
    KociembaPhase1Cube kcube2;
    if (!(kcube==kcube2)) result = false;
    
    // compare with previous solution
    if (csol==sol) result = false;
    
    // Replace solution
    *sol = *csol;
    delete csol;
    
    return result;
}

bool KociembaTester::TestKociembaSolverSolvePhase2Exception(CubieCube& ccube)
{
    Print("params: ", &ccube);
    try 
    {
        KociembaPhase2Solver kSolver;
        CubeSolution *pSol = kSolver.FindSolution(&ccube);
        delete pSol;
    } 
    catch (KociembaException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool KociembaTester::TestKociembaSolverSolvePhase2()
{
    bool result = true;
    
    // Create cube and apply moves on it
    KociembaPhase2Cube kcube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &kcube, iRep);
    Print(moves, turns, iRep);
    Move(&kcube, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    Print("CUBE", &kcube);

    // Find solution and solve cube
    KociembaPhase2Solver kSolver;
    CubeSolution *pSol = kSolver.FindSolution(&kcube);
    Print("CSOL", pSol);
    CubeSolver::Solve(&kcube, *pSol);
    delete pSol;
        
    // Compare with initialized cube
    KociembaPhase2Cube kcube2;
    if (!(kcube==kcube2)) result = false;
         
    return result;
}

bool KociembaTester::TestKociembaSolverSolve()
{
    bool result = true;

    // Create cube and apply moves on it
    CubieCube ccube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &ccube, iRep);
    Print(moves, turns, iRep);
    Move(&ccube, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    Print("CUBE", &ccube);

    // Find solution and solve cube
    KociembaSolver kSolver;
    CubeSolution *pSol = kSolver.FindSolution(&ccube);
    Print("CSOL", pSol);
    CubeSolver::Solve(&ccube, *pSol);
    delete pSol;
        
    // Compare with initialized cube
    CubieCube ccube2;
    if (!(ccube==ccube2)) result = false;

    return result;
}

bool KociembaTester::TestKociembaSolverSolveNext(CubieCube& cube, KociembaSolution *sol)
{
    bool result = true;

    try
    {
        // Copy cube
        CubieCube ccube(cube);
       
        // Find solution and solve cube
        KociembaSolver kSolver;
        KociembaSolution *ksol = (KociembaSolution*) kSolver.FindNextSolution(&ccube, *sol);
        Print("KSOL", ksol);
        CubeSolver::Solve(&ccube, *ksol);
            
        // Compare with initialized cube
        CubieCube ccube2;
        if (!(ccube==ccube2)) result = false;
        
        // compare with previous solution
        if (ksol==sol) result = false;
        
        // Replace solution
        *sol = *ksol;
        delete ksol;
    }
    catch(KociembaException &exp)
    {
        Print("SEXP", &exp);
        return true;
    }
    
    return result;
}

bool KociembaTester::TestKociembaSolverSolveOptimal()
{
    bool result = true;

    // Create cube and apply moves on it
    CubieCube ccube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &ccube, iRep);
    Print(moves, turns, iRep);
    Move(&ccube, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    Print("CUBE", &ccube);

    // Find solution and solve cube
    KociembaSolver kSolver;
    CubeSolution *pSol = kSolver.FindOptimalSolution(&ccube);
    Print("CSOL", pSol);
    CubeSolver::Solve(&ccube, *pSol);
    delete pSol;
        
    // Compare with initialized cube
    CubieCube ccube2;
    if (!(ccube==ccube2)) result = false;

    return result;
}


////////////////////////////////////////////////////////////////////////////////
// MOVE METHODS
////////////////////////////////////////////////////////////////////////////////

void KociembaTester::GetMoves(CubeMove::eMove *moves, CubeMove::eTurn *turns, Cube *ref, int number)
{
    for (int i=0; i<number; i++)
    {
        do 
        {
            moves[i] = (CubeMove::eMove) (rand()%CubeMove::MOVE_NUMBER);
            turns[i] = (CubeMove::eTurn) (rand()%CubeMove::TURN_NUMBER);
        } 
        while (!ref->IsAllowed(moves[i], turns[i]));
    }
}

void KociembaTester::Move(Cube *cube, CubeMove::eMove *moves, CubeMove::eTurn *turns, int number, bool forward)
{
    if (forward)
        for (int i=0; i<number; i++)
            cube->Move(moves[i], turns[i]); 
    else
        for (int i=number-1; i>=0; i--)
            cube->Move(moves[i], CubeMove::REVERSE_TURN(turns[i])); 
}

void KociembaTester::Print(CubeMove::eMove *moves, CubeMove::eTurn *turns, int number)
{
    if (!bLog) return;
    printf("\tMOVE=");
    for (int i=0; i<number; i++)
        printf("%c%d%c", CubeMove::MOVE_NAMES[moves[i]], (turns[i]+1), (i==number?'.':','));
    printf("\n");
}


////////////////////////////////////////////////////////////////////////////////
// PRINT OBJECT
////////////////////////////////////////////////////////////////////////////////

void KociembaTester::Print(const char *desc, Cube *cube)
{
    if (!bLog) return;
    char *str = cube->ToString();
    printf("\t%s CUBE: %s\n", (desc!=NULL ? desc : ""), str);
    delete[] str;
}

void KociembaTester::Print(const char *desc, CubeSolution *sol)
{
    if (!bLog) return;
    char *str = sol->ToString();
    printf("\t%s SOL: %s\n", (desc!=NULL ? desc : ""), str);
    delete[] str;
}

void KociembaTester::Print(const char *desc, KociembaException *exp)
{
    if (!bLog) return;
    char *str = exp->ToString();
    printf("\t%s EXC: %s\n", (desc!=NULL ? desc : ""), str);
    delete[] str;
}
