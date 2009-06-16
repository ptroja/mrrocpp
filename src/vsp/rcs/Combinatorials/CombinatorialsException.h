/////////////////////////////////////////////////////////
// Nazwa: CombinatorialsException.h
// Opis: Klasa CombinatorialsException wyjatkow operacji 
//       kombinatorycznych.
/////////////////////////////////////////////////////////

#ifndef CombinatorialsException_h
#define CombinatorialsException_h

#include "../General/Exception.h"


class Collection;

// Klasa wyjatkow operacji kombinatorycznych.
class CombinatorialsException : public Exception
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }

      
  private:
          
    // Kolekcja, w ktorej wystapil blad.
    Collection* pColl;

    // Wartosc, ktora powoduje blad.
    int iValue;
    
    
  public:
         
    // Konstruktor. Tworzy obiekt dla okreslonego bledu.
    CombinatorialsException(Collection* coll, int value, int error);

    // Konstruktor kopiujacy.
    CombinatorialsException(const CombinatorialsException& exp);

    // Destruktor. Zwalnia zajmowana pamiec.
    virtual ~CombinatorialsException();
    
    // Operator przypisania.
    CombinatorialsException& operator=(const CombinatorialsException& exp);
    
    // Kopiuje blad. Zwraca nowy blad, dla ktorego alokuje pamiec.
    virtual Exception* Clone();

    
    // Zrzuca zawartosc bledu do lancucha znakow. Zajmuje pamiec dla zwracanego
    // lancucha, ktora nalezy pozniej zwolnic. Zwraca tenze lancuch.
    char* ToString();

    // Zwracane kody bledow
    enum eError
    {
        SUC_COMB_VALID,           //  0
        ERR_COMB_SIZENEG,         //  1
        ERR_COMB_SIZEZERO,        //  2
        ERR_COMB_SIZETOOBIG,      //  3
        ERR_COMB_COMPFORBIDDEN,   //  4
        ERR_COMB_COMPSIZEDIF,     //  5
        ERR_COMB_ORDNEG,          //  6
        ERR_COMB_ORDTOOBIG,       //  7
        ERR_COMB_CYCLENOTPOS,     //  8
        ERR_COMB_CYCLETOOBIG,     //  9
        ERR_COMB_CYCLEVALTOOBIG,  // 10
        ERR_COMB_CYCLEVALREP,     // 11
        ERR_COMB_NOTSUBSET,       // 12
        ERR_COMB_POSNEG,          // 13
        ERR_COMB_POSTOOBIG,       // 14
        ERR_COMB_TABNEG,          // 15
        ERR_COMB_TABTOOBIG,       // 16
        ERR_COMB_TABREP,          // 17
        ERR_COMB_NUMBER           // 18  - liczba zwracanych kodow
    };


  protected:

    // zwraca wiadomosc bledu
    const char* GetErrorText();


  private:

    // wiadomosci bledow
    static char *ErrorText[];

};


#endif // CombinatorialsException_h
