/////////////////////////////////////////////////////////
// Name: KorfTester.cpp
// Implements : KorfTester
/////////////////////////////////////////////////////////

#include "vsp/rcs/Korf/KorfTester.h"
#include "vsp/rcs/Korf/KorfCube.h"
#include "vsp/rcs/Korf/KorfSolver.h"

#include <string.h>
#include <stdlib.h> 
#include <stdio.h>
#include <time.h>


////////////////////////////////////////////////////////////////////////////////
// MAIN TEST METHOD
////////////////////////////////////////////////////////////////////////////////

bool KorfTester::Test()
{
    printf("TEST KORF\n\n");
    bool result = true;

    // initialize random seed
    srand (time(NULL));
    
    if (!TestKorfCube()) result = false;
    if (!TestKorfSolver()) result = false;
    
    // clear static memory
    KorfCube::SClear();
    KorfSolver::SClear();

    printf("TEST KORF : %s\n\n\n", ResultText(result));
    if (bLog) printf("\n");
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE KORF CUBE TESTS
////////////////////////////////////////////////////////////////////////////////

bool KorfTester::TestKorfCube()
{
    printf("TEST KORF_CUBE\n");
    if (bLog) printf("\n");
    bool result = true, subres, subres2;


    // Test cube creation
    if (bLog) printf("TEST Create\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKorfCubeCreate();
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
        subres2 = TestKorfCubeCubie();
        printf("TEST To&From Cubies (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST To&From Cubies - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube move tables
    if (bLog) printf("TEST MoveTables\n");
    subres = true;
    KorfMoveTable* mt;
    
    mt = new KorfMoveTable_CP();
    subres2 = TestKorfCubeMoveTable(mt);
    printf("TEST MoveTables (CP) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    mt = new KorfMoveTable_CO();
    subres2 = TestKorfCubeMoveTable(mt);
    printf("TEST MoveTables (CO) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    mt = new KorfMoveTable_ECP(true);
    subres2 = TestKorfCubeMoveTable(mt);
    printf("TEST MoveTables (E1CP) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    mt = new KorfMoveTable_ECO(true);
    subres2 = TestKorfCubeMoveTable(mt);
    printf("TEST MoveTables (E1CO) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    mt = new KorfMoveTable_ECP(false);
    subres2 = TestKorfCubeMoveTable(mt);
    printf("TEST MoveTables (E2CP) - %s\n", ResultText(subres2));
    if (!subres2) subres = false;
    delete mt;
    
    mt = new KorfMoveTable_ECO(false);    
    subres2 = TestKorfCubeMoveTable(mt);
    printf("TEST MoveTables (E2CO) - %s\n", ResultText(subres2));
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
        subres2 = TestKorfCubeMove();
        printf("TEST Move (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Move - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST KORF_CUBE : %s\n\n", ResultText(subres));
    if (bLog) printf("\n");
    return result;
}

bool KorfTester::TestKorfCubeCreate()
{
    bool result = true;
    
    // Create Korf cube and apply moves
    KorfCube cube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &cube, iRep);
    Print(moves, turns, iRep);
    Move(&cube, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    Print("K1", &cube);

    // Create cube from cube
    KorfCube cube2(cube);
    Print("K2", &cube2);
    if (!(cube==cube2)) result = false;
    
    // Clone cube from cube
    KorfCube *cube3 = (KorfCube*) cube.Clone();
    Print("K3", cube3);
    if (!(cube==*cube3)) result = false;
    delete cube3;
         
    return result;
}

bool KorfTester::TestKorfCubeCubie()
{
    bool result = true;

    // Apply moves on Korf
    KorfCube kcube1;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &kcube1, iRep);
    Print(moves, turns, iRep);
    Move(&kcube1, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    
    // Do conversions
    CubieCube ccube1;
    KorfCube kcube2;
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

bool KorfTester::TestKorfCubeMoveTable(KorfMoveTable* mt)
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
                if (bLog) printf("MT contains invalid value on [%d,%d]] = %d\n", 
                    i, j, val);
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

bool KorfTester::TestKorfCubeMove()
{
    bool result = true;
    
    // Apply moves on Korf
    KorfCube kcube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &kcube, iRep);
    Print(moves, turns, iRep);
    Move(&kcube, moves, turns, iRep, true);
    Print("KFOR", &kcube);

    // Apply moves on Cubie and compare
    CubieCube ccube;
    Move(&ccube, moves, turns, iRep, true);
    KorfCube kcube2;
    kcube2.FromCubieCube(ccube);
    Print("CFOR", &kcube2);
    if (!(kcube==kcube2)) result = false;
    
    // Apply moves backwards
    Move(&kcube, moves, turns, iRep, false);
    Print("KBAC", &kcube);
    delete[] moves;
    delete[] turns;
        
    // Compare with initialized cube
    KorfCube kcube3;
    if (!(kcube==kcube3)) result = false;
         
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// TEST SOLVING KORF CUBE
////////////////////////////////////////////////////////////////////////////////

bool KorfTester::TestKorfSolver()
{
    printf("TEST KORF_SOLVER\n");
    if (bLog) printf("\n");
    bool result = true, subres, subres2;


    // Test solve
    if (bLog) printf("TEST Solve\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestKorfSolverSolve();
        printf("TEST Solve (%d) %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Solve - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST KORF_SOLVER : %s\n\n", ResultText(subres));
    if (bLog) printf("\n");
    return result;
}

bool KorfTester::TestKorfSolverSolve()
{
    bool result = true;
    
    // Create cube and apply moves on it
    KorfCube kcube;
    CubeMove::eMove *moves = new CubeMove::eMove[iRep];
    CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
    GetMoves(moves, turns, &kcube, iRep);
    Print(moves, turns, iRep);
    Move(&kcube, moves, turns, iRep, true);
    delete[] moves;
    delete[] turns;
    Print("CUBE", &kcube);

    // Find solution and solve cube
    KorfSolver kSolver;
    CubeSolution *pSol = kSolver.FindSolution(&kcube);
    Print("CSOL", pSol);
    CubeSolver::Solve(&kcube, *pSol);
    delete pSol;
        
    // Compare with initialized cube
    KorfCube kcube2;
    if (!(kcube==kcube2)) result = false;
         
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// MOVE METHODS
////////////////////////////////////////////////////////////////////////////////

void KorfTester::GetMoves(CubeMove::eMove *moves, CubeMove::eTurn *turns, Cube *ref, int number)
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

void KorfTester::Move(Cube *cube, CubeMove::eMove *moves, CubeMove::eTurn *turns, int number, bool forward)
{
    if (forward)
        for (int i=0; i<number; i++)
            cube->Move(moves[i], turns[i]); 
    else
        for (int i=number-1; i>=0; i--)
            cube->Move(moves[i], CubeMove::REVERSE_TURN(turns[i])); 
}

void KorfTester::Print(CubeMove::eMove *moves, CubeMove::eTurn *turns, int number)
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

void KorfTester::Print(const char *desc, Cube *cube)
{
    if (!bLog) return;
    char *str = cube->ToString();
    printf("\t%s CUBE: %s\n", (desc!=NULL ? desc : ""), str);
    delete[] str;
}

void KorfTester::Print(const char *desc, CubeSolution *sol)
{
    if (!bLog) return;
    char *str = sol->ToString();
    printf("\t%s SOL: %s\n", (desc!=NULL ? desc : ""), str);
    delete[] str;
}
