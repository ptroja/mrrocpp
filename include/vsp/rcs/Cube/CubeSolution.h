/////////////////////////////////////////////////////////
// Nazwa: CubeSolution.h
// Opis: Klasa CubeSolution opisuje rozwiazanie kostki.
/////////////////////////////////////////////////////////

#ifndef CubeSolution_h
#define CubeSolution_h

#include "Cube.h"
#include "CubeSolutionException.h"


// Klasa opisujaca rozwiazanie kostki.
class CubeSolution 
{

  public:

    const static char DELIM;
    
    enum eInfo
    {
        INFO_ALREADY_SOLVED,
        INFO_OPTIMAL
    };    



  private:

    // Dlugosc rozwiazania
    int iLength;

    // Maksymalna dlugosc rozwiazania przy aktualnych tabelach.
    int iMaxLength;
    
    // Lista ruchow stanowiacych rozwiazanie.
    CubeMove::eMove *iaMoves;
    
    // Lista liczby cwierc-obrotow ruchow stanowiacych rozwiazanie.
    CubeMove::eTurn *iaTurns;

    // Maska dodatkowych bledow, ostrzezen i komentarzy
    int iInfo;
    

  public:
            
    // Konstruktor. Tworzy puste rozwiazanie.
    CubeSolution();

    // Konstruktor kopiujacy.
    CubeSolution(const CubeSolution& sol);

    // Kopiuje stan kostki. Zwraca nowy stan, dla ktore alokuje pamiec.
    virtual CubeSolution* Clone() { return new CubeSolution(*this); }

    // Destruktor.
    virtual ~CubeSolution();

    // Operator przypisania.
    CubeSolution& operator=(const CubeSolution& sol);

    // Ustawia ruch na pozycji pos.
    void SetMove(int pos, CubeMove::eMove move, CubeMove::eTurn turn)
         throw (CubeSolutionException);

    // Pobiera ruch z pozycji pos.
    void GetMove(int pos, CubeMove::eMove& move, CubeMove::eTurn& turn) const
         throw (CubeSolutionException);
    
    // Zwraca dlugosc rozwiazania.
    int GetLength() const { return iLength; }
    
    bool IsAllowed();
    
    // Pobiera rozwiazanie z lancucha znakow reprezentujacego je. Rzuca 
    // wyjatkiem w przypadku niepowodzenia.
    virtual void FromString(const char* sSol) 
         throw (Exception);

    // Zwraca rozwiazanie jako lancuch nazw ruchow rozdzielonych separatorem.
    virtual char* ToString() const;
    
    void SetInfo(const eInfo info, bool on);
    bool GetInfo(const eInfo info) const;

  private:
          
    void Extend();
    void CheckFormat(const char* sSol)
         throw (CubeSolutionException);


};


#endif // CubeSolution_h

