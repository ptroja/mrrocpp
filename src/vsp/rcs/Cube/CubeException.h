/////////////////////////////////////////////////////////
// Nazwa: CubeException.cpp
// Opis: Klasa CubeException bledu oznaczen kostki.
/////////////////////////////////////////////////////////

#ifndef CubeException_h
#define CubeException_h

#include "../General/Exception.h"


class Cube;

// Klasa bledu opisu kostki Rubika.
class CubeException : public Exception
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }


  protected:

    // Wyjatek, ktory spowodowal blad.
    Exception *pException;
    
    // Opis, w ktorym wystepuje blad.
    Cube *pCube;
    
    // £ancuch reprezentujacy niepoprawny stan kostki.
    char *sCube;
    
    // Wartosc, ktora powoduje blad.
    int iValue;


  public:

    // Konstruktor. Tworzy obiekt dla okreslonego bledu.
    CubeException(Cube *pCube, const char *sCube, int val, int error);

    // Konstruktor. Tworzy obiekt z wyjatku tworzacego go.
    CubeException(Exception *pExc, int error);

    // Konstruktor kopiujacy.
    CubeException(const CubeException& exp);

    // Destruktor. Zwalnia zajmowana pamiec.
    virtual ~CubeException();
    
    // Operator przypisania.
    CubeException& operator=(const CubeException& exp);
    
    // Kopiuje blad. Zwraca nowy blad, dla ktorego alokuje pamiec.
    virtual Exception* Clone();

    // Zrzuca zawartosc bledu do lancucha znakow. Zajmuje pamiec dla zwracanego
    // lancucha, ktora nalezy pozniej zwolnic.
    char* ToString();
    
    // Zwracane kody bledow.
    enum eError
    {
        SUC_CUBE_VALID,                     //  0
        ERR_FCUBE_INVALID_MARKER,           //  1
        ERR_FCUBE_INVALID_FACELETCOUNT,     //  2
        ERR_FCUBE_DUPLICATE_CENTER_MARKING, //  3
        ERR_FCUBE_INVALID_CORNER_MARKINGS,  //  4
        ERR_FCUBE_INVALID_CORNER_PARITY,    //  5
        ERR_FCUBE_INVALID_EDGE_MARKINGS,    //  6
        ERR_FCUBE_INVALID_EDGE_PARITY,      //  7
        ERR_FCUBE_INVALID_TOTAL_PARITY,     //  8
        ERR_FCUBE_INVALID_FACE,             //  9
        ERR_FCUBE_INVALID_FACELET,          // 10
        ERR_FCUBE_INVALID_FACENAME,         // 11
        ERR_FCUBE_PARSE_STRINGNULL,         // 12
        ERR_FCUBE_PARSE_STRINGTOOSHORT,     // 13
        ERR_FCUBE_PARSE_FACEINVALID,        // 14
        ERR_FCUBE_PARSE_FACELETINVALID,     // 15
        ERR_FCUBE_PARSE_FORMATINVALID,      // 16
        ERR_FCUBE_PARSE_FACENAMEREPEATED,   // 17
        ERR_FCUBE_TABLE_VALTOOSMALL,        // 18
        ERR_FCUBE_TABLE_VALTOOBIG,          // 19
        ERR_FCUBE_MOVE_NOTIMPLEMENTED,      // 20
        ERR_CCUBE_INVALID_TABLE,            // 21
        ERR_CCUBE_INVALID_CORNER_PARITY,    // 22
        ERR_CCUBE_INVALID_EDGE_PARITY,      // 23
        ERR_CCUBE_UNEQUAL_PERM_PARITY,      // 24
        ERR_CMOVE_INVALID_MOVE_NAME,        // 25
        ERR_CMOVE_INVALID_TURN_NAME,        // 26
        ERR_CUBE_NUMBER                     // 27  - liczba zwracanych kodow
    };


  protected:
    
    virtual const char* GetErrorText();
  
  
  private:

    // Wiadomosci bledow.
    static const char *ErrorText[];

};


#endif // CubeException_h        
