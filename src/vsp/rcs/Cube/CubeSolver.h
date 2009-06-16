/////////////////////////////////////////////////////////
// Nazwa: CubeSolver.h
// Opis: Klasa abstrakcyjna CubeSolver definiuje metody, ktore musza byc 
//      zaimplementowane przez klasy rozwiazujace kostke.
/////////////////////////////////////////////////////////

#ifndef CubeSolver_h
#define CubeSolver_h

#include "Cube.h"
#include "../General/Exception.h"
#include "CubeSolution.h"


// Klasa abstrakcyjna definiujaca metody klas znajdujacych rozwiazanie kostki.
class CubeSolver 
{
      
  protected:
            
    // Klasa kontekstu rozwiazywanego problemu.
    class CubeSolverContext
    {
      public:
             
        bool bNext;
        int iMinSolLength;
    
        // Liczba rozwinietych wezlow w aktualnej iteracji
        int iNodesExpanded;
        
        // Liczba sprawdzonych wezlow w aktualnej iteracji
        int iNodesChecked;
        
        // Liczba wszystkich rozwinietych wezlow
        int iNodesExpandedAll;
        
        // Liczba wszystkich sprawdzonych wezlow
        int iNodesCheckedAll;
        
        // Aktualna glebokosc
        int iDepth;
        
        // Aktualna iteracja szukania rozwiazania
        int iIteration;
        
        // Aktualna wartosc progowa funkcji heurystycznej
        int iThreshold;
        
        // Nowa wartosc progowa funkcji heurystycznej
        int iThresholdNext;
        
        // Znalezione rozwiazanie
        CubeSolution solution;
        
        // Konstruktor domyslny. Inicjalizuje pola.
        CubeSolverContext() : bNext(false), iMinSolLength(0), 
                              iNodesExpanded(0), iNodesChecked(0),
                              iNodesExpandedAll(0), iNodesCheckedAll(0),
                              iDepth(0), iIteration(0), 
                              iThreshold(0), iThresholdNext(0), solution() {}

        // Konstruktor domyslny. Inicjalizuje pola.
        CubeSolverContext(CubeSolution sol) : bNext(true), iMinSolLength(sol.GetLength()), 
                              iNodesExpanded(0), iNodesChecked(0),
                              iNodesExpandedAll(0), iNodesCheckedAll(0),
                              iDepth(0), iIteration(0), 
                              iThreshold(0), iThresholdNext(0), solution(sol) {}
    };

    // Znacznik czy wyswietlac komentarze
    static bool bLog;

    // Znacznik czy wyswietlac wartosci dla testow
    static bool bTest;


  protected:
            
    // Konstruktor.
    CubeSolver() {} 

    // Inicjalizuje klase.
    virtual void Init() = 0;

    CubeSolution* FindNextSolution(Cube* scrambledCube, const CubeSolution& solution)
        throw (Exception);


  public:
    
    // Destruktor.
    virtual ~CubeSolver() {}

    // Znajduje rozwiazanie kostki. Zwraca kod sukcesu lub porazki. Ustawia 
    // kod bledu i kod ostrzezenia.
    CubeSolution* FindSolution(Cube* scrambledCube)
        throw (Exception);

    static void Solve(Cube* scrambledCube, const CubeSolution& solution)
        throw (Exception);

    // Ustawia znacznik wyswietlania komentarzy.
    static void SetLog(bool log)
        { bLog = log; }

    // Ustawia znacznik wyswietlania wartosci dla testow.
    static void SetTest(bool test)
        { bTest = test; }


  protected: 
    
    // Rekurencyjne przeszukiwanie drzewa wedlug strategi IDA*
    int Search(CubeSolverContext& context, Cube *cube, int parent_totalcost);
           
    // Rekurencyjne przeszukiwanie drzewa wedlug strategi IDA*
    int SearchNext(CubeSolverContext& context, Cube *cube, int parent_totalcost);

    // Funkcja heurystyczna
    virtual int Heuristic(Cube* cube) = 0;
    
};


#endif // CubeSolver_h

