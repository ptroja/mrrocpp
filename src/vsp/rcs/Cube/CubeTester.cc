/////////////////////////////////////////////////////////
// Name: CubeTester.cpp
// Implements : CubeTester
/////////////////////////////////////////////////////////

#include "vsp/rcs/Cube/CubeTester.h"
#include "vsp/rcs/Cube/FaceletCube.h"
#include "vsp/rcs/Cube/CubieCube.h"
#include "vsp/rcs/Cube/CubeTranslator.h"

#include <string.h>
#include <stdlib.h> 
#include <stdio.h>
#include <time.h>


////////////////////////////////////////////////////////////////////////////////
// MAIN TEST METHOD
////////////////////////////////////////////////////////////////////////////////

bool CubeTester::Test()
{
    printf("TEST CUBE\n\n");
    bool result = true;

    // initialize random seed
    srand (time(NULL));
    
    if (!TestFaceletCube()) result = false;
    if (!TestCubieCube()) result = false;
    if (!TestTranslator()) result = false;

    printf("TEST CUBE : %s\n\n\n", ResultText(result));
    if (bLog) printf("\n");
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE FACELET_CUBE TESTS
////////////////////////////////////////////////////////////////////////////////

bool CubeTester::TestFaceletCube()
{
    printf("TEST FACELET_CUBE\n");
    if (bLog) printf("\n");
    bool result = true, subres, subres2;


    // Test from string exceptions
    if (bLog) printf("TEST StringException\n");
    subres = TestFaceletCubeStringException(NULL);
    printf("TEST StringException (NULL) - %s\n", ResultText(subres));
    if (!subres) result = false;

    subres = true;
    const char BAD_CUBES[6][78] = {
        "F:bbbbbbbbb,R:rrrrrrrrr,U:wwwwwwwww,B:ggggggggg,L:ooooooooo,D:yyyyyyyyy",
        "F:bbbbbbbbb,R:rrrrrrrrr,U:wwwwwwwww,B:ggggggggg,L:ooooooooo,D:yyyyyyyyx,",
        "F:bbbbbbbbb,R:rrrrrrrrr,U:wwwwwwwww,B:ggggggggg,L:ooooooooo,X:yyyyyyyyy,",
        "F:bbbbbbbbb,R:rrrrrrrrr,U:wwwwwwwww,B:ggggggggg,L:ooooooooo,D:yyyyyyyyy.",
        "F:bbbbbbbbb,R:rrrrrrrrr,U:wwwwwwwww,B:ggggggggg,L:ooooooooo,L:yyyyyyyyy,",
        "F:bbbbbbbbb,R:rrrrrrrrr,U:wwwwwwwww,B:ggggggggg,L:ooooooooo,D:yyyyyyyyo,",
    };
    for (int i=0; i<6; i++)
    {
        subres2 = TestFaceletCubeStringException(BAD_CUBES[i]);
        printf("TEST StringException (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST StringException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test from table exceptions
    if (bLog) printf("TEST TableException\n");
    subres = true;
    const int BAD_ICUBES[2][54] = {
        { 0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1, 2,2,2,2,2,2,2,2,2, 3,3,3,3,3,3,3,3,3, 4,4,4,4,4,4,4,4,4, 5,5,5,5,5,5,5,5,6 },
        { 0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1, 2,2,2,2,2,2,2,2,2, 3,3,3,3,3,3,3,3,3, 4,4,4,4,4,4,4,4,4, 5,5,5,5,5,5,5,5,4 }
    };
    for (int i=0; i<2; i++)
    {
        subres2 = TestFaceletCubeTableException(BAD_ICUBES[i]);
        printf("TEST TableException (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST TableException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test move exceptions
    if (bLog) printf("TEST MoveException\n");
    subres = TestFaceletCubeMoveException();
    printf("TEST MoveException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube creation
    if (bLog) printf("TEST Create\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestFaceletCubeCreate();
        printf("TEST Create (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Create - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube string
    if (bLog) printf("TEST To&From String\n");
    subres = true;
    const char CUBES[5][78] = {
        "F:bbwyrrygg,R:rrryggoow,U:bwgrwgrwg,B:ygwoowobb,L:owwobbyrr,D:byybyyoog,",
        "F:ooybbybbw,R:ooboogrrg,U:wyrwygwwg,B:yroyggygg,L:bbgrrorro,D:yybwwbwwr,",
        "F:broogwoyg,R:yoygrgryr,U:orroygrbb,B:gwwrbrygb,L:gbwyowowg,D:ybwbwowyb,",
        "F:bbbbbbbbb,R:rrrrrrrrr,U:wwwwwwwww,B:ggggggggg,L:ooooooooo,D:yyyyyyyyy,",
        "F:bbybbybby,R:rrrrrrrrr,U:wwbwwbwwb,B:wggwggwgg,L:ooooooooo,D:yygyygyyg,",
    };
    for (int i=0; i<5; i++)
    {
        subres2 = TestFaceletCubeString(CUBES[i]);
        printf("TEST To&From String (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST To&From String - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube tables
    if (bLog) printf("TEST To&From Tables\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestFaceletCubeTables();
        printf("TEST To&From Tables (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST To&From Tables - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST FACELET_CUBE : %s\n\n", ResultText(subres));
    if (bLog) printf("\n");
    return result;
}
    
bool CubeTester::TestFaceletCubeStringException(const char* scube)
{
    if (bLog) printf("\tparams: %s\n", ( scube ? scube : "NULL" ));
    try 
    {
        FaceletCube cube;
        cube.FromString(scube);
    } 
    catch (CubeException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CubeTester::TestFaceletCubeTableException(const int icube[])
{
    if (bLog) 
    {
        printf("\tparams: [");
        for (int i=0; i<54; i++)
            printf("%d%c", icube[i], ( i==53 ? '.' : ',' ));
        printf("]\n");
    }
    try 
    {
        FaceletCube cube;
        cube.FromTable(icube);
    } 
    catch (CubeException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CubeTester::TestFaceletCubeMoveException()
{
    try 
    {
        FaceletCube cube;
        cube.Move(CubeMove::F, CubeMove::QUARTER);
    } 
    catch (CubeException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CubeTester::TestFaceletCubeCreate()
{
    bool result = true;
    
    try 
    {
        // Create cubie cube and apply moves
        CubieCube ccube;
        CubeMove::eMove *moves = new CubeMove::eMove[iRep];
        CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
        GetMoves(moves, turns, &ccube, iRep);
        Move(&ccube, moves, turns, iRep, true);
        Print(moves, turns, iRep);
        delete[] moves;
        delete[] turns;

        // Convert to facelet cube
        FaceletCube cube;
        CubeTranslator::CubieToFacelet(cube, ccube);
        Print("F1", &cube);
        
        // Create cube from cube
        FaceletCube cube2(cube);
        Print("F2", &cube2);
        if (!(cube==cube2)) result = false;
        
        // Clone cube from cube
        FaceletCube *cube3 = (FaceletCube*) cube.Clone();
        Print("F3", cube3);
        if (!(cube==*cube3)) result = false;
        delete cube3;
    } 
    catch (CubeException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}

bool CubeTester::TestFaceletCubeString(const char* scube)
{
    if (bLog) printf("\tparams: %s\n", scube);
    bool result = true;
    
    try 
    {
        // Create cube and get table
        FaceletCube cube;
        cube.FromString(scube);
        Print("C1", &cube);
        
        // Create cube and get state from table
        FaceletCube cube2(cube);
        char* scube2 = cube2.ToString();
        Print("C2", &cube2);
        if (strcmp(scube,scube2) != 0) result = false;
        delete[] scube2;
    } 
    catch (CubeException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}

bool CubeTester::TestFaceletCubeTables()
{
    bool result = true;
    
    try 
    {
        // Create cubie cube and apply moves
        CubieCube ccube;
        CubeMove::eMove *moves = new CubeMove::eMove[iRep];
        CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
        GetMoves(moves, turns, &ccube, iRep);
        Move(&ccube, moves, turns, iRep, true);
        Print(moves, turns, iRep);
        delete[] moves;
        delete[] turns;

        // Convert to facelet cube
        FaceletCube cube;
        CubeTranslator::CubieToFacelet(cube, ccube);
        Print("F1", &cube);
        
        // Set state to table
        int ctable[54];
        cube.ToTable(ctable);
        
        // Create cube and get state from table
        FaceletCube cube2;
        cube2.FromTable(ctable);
        Print("F2", &cube);
        if (!(cube==cube2)) result = false;
    } 
    catch (CubeException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE CUBIE_CUBE TESTS
////////////////////////////////////////////////////////////////////////////////

bool CubeTester::TestCubieCube()
{
    printf("TEST CUBIE_CUBE\n");
    bool result = true, subres, subres2;
    if (bLog) printf("\n");


    // Test from table exceptions
    if (bLog) printf("TEST TableException\n");
    subres = true;
    const int BAD_ICUBES[8][40] = {
        { 0,1,2,3,4,5,6,8, 0,0,0,0,0,0,0,0, 0,1,2,3,4,5,6,7,8,9,10,11, 0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,1,2,3,4,5,6,6, 0,0,0,0,0,0,0,0, 0,1,2,3,4,5,6,7,8,9,10,11, 0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,1,2,3,4,5,6,7, 0,0,0,0,0,0,0,3, 0,1,2,3,4,5,6,7,8,9,10,11, 0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,1,2,3,4,5,6,7, 0,0,0,0,0,0,0,1, 0,1,2,3,4,5,6,7,8,9,10,11, 0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,1,2,3,4,5,6,7, 0,0,0,0,0,0,0,0, 0,1,2,3,4,5,6,7,8,9,10,12, 0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,1,2,3,4,5,6,7, 0,0,0,0,0,0,0,0, 0,1,2,3,4,5,6,7,8,9,10,10, 0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,1,2,3,4,5,6,7, 0,0,0,0,0,0,0,0, 0,1,2,3,4,5,6,7,8,9,10,11, 0,0,0,0,0,0,0,0,0,0,0,2 },
        { 0,1,2,3,4,5,6,7, 0,0,0,0,0,0,0,0, 0,1,2,3,4,5,6,7,8,9,10,11, 0,0,0,0,0,0,0,0,0,0,0,1 },
    };
    for (int i=0; i<8; i++)
    {
        subres2 = TestCubieCubeTableException(BAD_ICUBES[i]);
        printf("TEST TableException (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST TableException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube creation
    if (bLog) printf("TEST Create\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestCubieCubeCreate();
        printf("TEST Create (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Create - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube tables
    if (bLog) printf("TEST To&From Tables\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestCubieCubeTables();
        printf("TEST To&From Tables (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST To&From Tables - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test cube moves
    if (bLog) printf("TEST Moves\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestCubieCubeMoves();
        printf("TEST Moves (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Moves - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST CUBIE_CUBE : %s\n\n", ResultText(subres));
    if (bLog) printf("\n");
    return result;
}
    
bool CubeTester::TestCubieCubeTableException(const int icube[])
{
    if (bLog) 
    {
        printf("\tparams: [");
        for (int i=0; i<40; i++)
            printf("%d%c", icube[i], ( i==39 ? '.' : ',' ));
        printf("]\n");
    }
    try 
    {
        CubieCube cube;
        cube.FromTable(icube);
    } 
    catch (CubeException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CubeTester::TestCubieCubeCreate()
{
    bool result = true;
    
    try 
    {
        // Create cube and apply moves
        CubieCube cube;
        CubeMove::eMove *moves = new CubeMove::eMove[iRep];
        CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
        GetMoves(moves, turns, &cube, iRep);
        Move(&cube, moves, turns, iRep, true);
        Print(moves, turns, iRep);
        delete[] moves;
        delete[] turns;
        Print("C1", &cube);
        
        // Create cube from cube
        CubieCube cube2(cube);
        Print("C2", &cube2);
        if (!(cube==cube2)) result = false;
        
        // Clone cube from cube
        CubieCube *cube3 = (CubieCube*) cube.Clone();
        Print("C3", cube3);
        if (!(cube==*cube3)) result = false;
        delete cube3;
    } 
    catch (CubeException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}

bool CubeTester::TestCubieCubeTables()
{
    bool result = true;
    
    try 
    {
        // Create cube and get table
        CubieCube cube;
        CubeMove::eMove *moves = new CubeMove::eMove[iRep];
        CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
        GetMoves(moves, turns, &cube, iRep);
        Move(&cube, moves, turns, iRep, true);
        Print(moves, turns, iRep);
        delete[] moves;
        delete[] turns;
        Print("C1", &cube);
        int ctable[40];
        cube.ToTable(ctable);
        
        // Create cube and get state from table
        CubieCube cube2;
        cube2.FromTable(ctable);
        Print("C2", &cube);
        if (!(cube==cube2)) result = false;
    } 
    catch (CubeException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}

bool CubeTester::TestCubieCubeMoves()
{
    bool result = true;
    
    try 
    {
        // Create cube and apply moves forward and back
        CubieCube cube;
        CubeMove::eMove *moves = new CubeMove::eMove[iRep];
        CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
        GetMoves(moves, turns, &cube, iRep);
        Move(&cube, moves, turns, iRep, true);
        Print(moves, turns, iRep);
        Print("CFOR", &cube);
        Move(&cube, moves, turns, iRep, false);
        Print("CBAC", &cube);
        delete[] moves;
        delete[] turns;
        
        // Compare with initialized cube
        CubieCube cube2;
        if (!(cube==cube2)) result = false;
    } 
    catch (CubeException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// PRIVATE TRANSLATOR TESTS
////////////////////////////////////////////////////////////////////////////////

bool CubeTester::TestTranslator()
{
    printf("TEST CUBE_TRANSLATOR\n");
    if (bLog) printf("\n");
    bool result = true, subres, subres2;


    // Test translator constants
    if (bLog) printf("TEST Const\n");
    subres = TestTranslatorConst();
    printf("TEST Const - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test translate exceptions
    if (bLog) printf("TEST TranslateException\n");
    subres = true;
    const char BAD_CUBES[6][78] = {
        "F:bbbbbbbbb,R:rrrrrrrrr,U:wwwwwwwww,B:ggggggggg,L:ooooooooy,D:yyyyyyyyo,",
        "F:bbbbbbbbb,R:rrrrrrrrr,U:wwwwwwwww,B:ggggggggg,L:oooooooyo,D:yyyyyyyoy,",
        "F:bbwbbbbbb,R:brrrrrrrr,U:wwwwwwwwr,B:ggggggggg,L:ooooooooo,D:yyyyyyyyy,",
        "F:bbrbbbbbb,R:wrrrrrrrr,U:wwwwwwwwb,B:ggggggggg,L:ooooooooo,D:yyyyyyyyy,",
        "F:bbbbbrbbb,R:rrrbrrrrr,U:wwwwwwwww,B:ggggggggg,L:ooooooooo,D:yyyyyyyyy,",
        "F:bbbbbybbb,R:rrrrrrrrr,U:wwwwwbwww,B:gggwggggg,L:ooooooooo,D:yyyyygyyy,",
    };
    for (int i=0; i<6; i++)
    {
        subres2 = TestTranslatorTranslateException(BAD_CUBES[i]);
        printf("TEST TranslateException (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST TranslateException - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    // Test translation
    if (bLog) printf("TEST Translate\n");
    subres = true;
    for (int i=0; i<iRep; i++)
    {
        subres2 = TestTranslatorTranslate();
        printf("TEST Translate (%d) - %s\n", i, ResultText(subres2));
        if (!subres2) subres = false;
    }
    printf("TEST Translate - %s\n", ResultText(subres));
    if (!subres) result = false;
    if (bLog) printf("\n");


    printf("TEST CUBE_TRANSLATOR : %s\n\n", ResultText(subres));
    if (bLog) printf("\n");
    return result;
}
    
bool CubeTester::TestTranslatorConst()
{
    int i,j, fac[FaceletCube::FACELET_NUMBER];
    bool result = true;
    
    // clear
    for (i=0; i<FaceletCube::FACELET_NUMBER; i++)
        fac[i] = 0;
    
    // mark corners
    for (i=0; i<CubieCube::CORNER_NUMBER;i++)
        for (j=0; j<CubieCube::TWIST_NUMBER; j++)
            fac[CubeTranslator::CORNER_MAP[i][j]]++;
            
    // mark edges
    for (i=0; i<CubieCube::EDGE_NUMBER;i++)
        for (j=0; j<CubieCube::FLIP_NUMBER; j++)
            fac[CubeTranslator::EDGE_MAP[i][j]]++;
    
    // check if middles not used and others used once
    if (bLog) printf("\tUSAGE: ");
    for (i=0; i<FaceletCube::FACELET_NUMBER; i++)
    {
        if (bLog)
        {
            printf("%d", fac[i]);
            if ((i+1)%9 == 0) printf(" ");
        }
        if ( (i%9 != 4 && fac[i] != 1) || (i%9 == 4 && fac[i] != 0))
            result = false;
    }
    if (bLog) printf("\n");

    return result;
}

bool CubeTester::TestTranslatorTranslateException(const char* scube)
{
    if (bLog) printf("\tparams: %s\n", ( scube ? scube : "NULL" ));
    try 
    {
        FaceletCube fcube;
        fcube.FromString(scube);
        CubieCube ccube;
        CubeTranslator::FaceletToCubie(fcube, ccube);
    } 
    catch (CubeException& exp)
    {
        Print(NULL, &exp);
        return true;
    }
         
    return false;
}

bool CubeTester::TestTranslatorTranslate()
{
    bool result = true;
    
    try 
    {
        // Create cubie cube and apply moves
        CubieCube ccube;
        CubeMove::eMove *moves = new CubeMove::eMove[iRep];
        CubeMove::eTurn *turns = new CubeMove::eTurn[iRep];
        GetMoves(moves, turns, &ccube, iRep);
        Move(&ccube, moves, turns, iRep, true);
        Print(moves, turns, iRep);
        delete[] moves;
        delete[] turns;
        Print("C1", &ccube);
        
        // Convert to facelet cube
        FaceletCube fcube;
        CubeTranslator::CubieToFacelet(fcube, ccube);
        Print("F1", &fcube);

        // Convert back to cubie
        CubieCube ccube2;
        CubeTranslator::FaceletToCubie(fcube, ccube2);
        Print("C2", &ccube2);
        if (!(ccube==ccube2)) result = false;
        
        // Convert again to facelet
        FaceletCube fcube2;
        CubeTranslator::CubieToFacelet(fcube2, ccube2);
        Print("F2", &fcube2);
        if (!(fcube==fcube2)) result = false;
    } 
    catch (CubeException& exp)
    {
        Print("ERR", &exp);
        return false;
    }
         
    return result;
}


////////////////////////////////////////////////////////////////////////////////
// MOVE METHODS
////////////////////////////////////////////////////////////////////////////////

void CubeTester::GetMoves(CubeMove::eMove *moves, CubeMove::eTurn *turns, Cube *ref, int number)
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

void CubeTester::Move(Cube *cube, CubeMove::eMove *moves, CubeMove::eTurn *turns, int number, bool forward)
{
    if (forward)
        for (int i=0; i<number; i++)
            cube->Move(moves[i], turns[i]); 
    else
        for (int i=number-1; i>=0; i--)
            cube->Move(moves[i], CubeMove::REVERSE_TURN(turns[i])); 
}

void CubeTester::Print(CubeMove::eMove *moves, CubeMove::eTurn *turns, int number)
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

void CubeTester::Print(const char *desc, Cube *cube)
{
    if (!bLog) return;
    char *str = cube->ToString();
    printf("\t%s CUBE: %s\n", (desc!=NULL ? desc : ""), str);
    delete[] str;
}

void CubeTester::Print(const char *desc, CubeException *exp)
{
    if (!bLog) return;
    char *str = exp->ToString();
    printf("\t%s EXC: %s\n", (desc!=NULL ? desc : ""), str);
    delete[] str;
}
