/////////////////////////////////////////////////////////
// Nazwa: CubeSolutionException.cpp
// Opis: Klasa CubeSolutionException bledu rozwiazania.
/////////////////////////////////////////////////////////

#ifndef CubeSolutionException_h
#define CubeSolutionException_h

#include "../General/Exception.h"


class CubeSolution;

// Klasa bledu opisu kostki Rubika.
class CubeSolutionException : public Exception
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }


  protected:

    // £ancuch reprezentujacy niepoprawne rozwiazanie.
    char *sSol;
    
    // Wartosc, ktora powoduje blad.
    int iValue;


  public:

    // Konstruktor. Tworzy obiekt dla okreslonego bledu.
    CubeSolutionException(const char *sSol, int val, int error);

    // Konstruktor kopiujacy.
    CubeSolutionException(const CubeSolutionException& exp);

    // Destruktor. Zwalnia zajmowana pamiec.
    virtual ~CubeSolutionException();
    
    // Operator przypisania.
    CubeSolutionException& operator=(const CubeSolutionException& exp);
    
    // Kopiuje blad. Zwraca nowy blad, dla ktorego alokuje pamiec.
    virtual Exception* Clone();

    // Zrzuca zawartosc bledu do lancucha znakow. Zajmuje pamiec dla zwracanego
    // lancucha, ktora nalezy pozniej zwolnic.
    char* ToString();
    
    // Zwracane kody bledow.
    enum eError
    {
        SUC_CSOL_VALID,                     //  0
        ERR_CSOL_INVALID_POS,               //  1
        ERR_CSOL_PARSE_MOVEINVALID,         //  2
        ERR_CSOL_PARSE_TURNINVALID,         //  3
        ERR_CSOL_PARSE_FORMATINVALID,       //  4
        ERR_CSOL_NUMBER                     //  5  - liczba zwracanych kodow
    };
    

  protected:
    
    virtual const char* GetErrorText();
  
  
  private:

    // Wiadomosci bledow.
    static char *ErrorText[];

};


#endif // CubeSolutionException_h        
