/////////////////////////////////////////////////////////
// Nazwa: KociembaSolution.h
// Opis: Klasa KociembaSolution opisuje rozwiazanie kostki znalezione
//       algorytmem Kociemby i wyrozniajace czesci rozwiazania 
//       znalezione w dwoch fazach algorytmu.
/////////////////////////////////////////////////////////

#ifndef KociembaSolution_h
#define KociembaSolution_h

#include "../Cube/CubeSolution.h"
#include "KociembaException.h"


// Klasa opisujaca rozwiazanie kostki.
class KociembaSolution : public CubeSolution
{

  private:
          
    // dlugosc rozwiazania znalezionego w pierwszej fazie
    int iPhase1Length;


  public:
             
    // Konstruktor. Tworzy puste rozwiazanie.
    KociembaSolution();

    // Konstruktor kopiujacy.
    KociembaSolution(const KociembaSolution& sol);

    // Konstruktor. Tworzy rozwiazanie jako sume dwoch innych rozwiazan.
    KociembaSolution(const CubeSolution& sol1, const CubeSolution& sol2);

    // Operator przypisania.
    KociembaSolution& operator=(const KociembaSolution& sol);

    // Kopiuje stan kostki. Zwraca nowy stan, dla ktore alokuje pamiec.
    virtual KociembaSolution* Clone() { return new KociembaSolution(*this); }

    // Pobiera rozwiazanie z lancucha znakow reprezentujacego je. Rzuca 
    // wyjatkiem w przypadku niepowodzenia.
    virtual void FromString(const char* sSol) 
         throw (Exception);

    // Zwraca rozwiazanie jako lancuch nazw ruchow rozdzielonych separatorem.
    virtual char* ToString() const;
    
    // Zwraca dlugosc rozwiazania znalezionego w pierwszej fazie.
    int GetPhase1Length() const { return iPhase1Length; }
     
    
  private:
          
    void CheckFormat(const char* sSol)
        throw (KociembaException);

    
};


#endif // KociembaSolution_h
    
