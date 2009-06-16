/////////////////////////////////////////////////////////
// Nazwa: KociembaException.cpp
// Opis: Klasa KociembaException bledu oznaczen kostki.
/////////////////////////////////////////////////////////

#ifndef KociembaException_h
#define KociembaException_h

#include "../General/Exception.h"


// Klasa bledu opisu kostki Rubika.
class KociembaException : public Exception
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() { return TYPE; }


  public:

    // Konstruktor. Tworzy obiekt dla okreslonego bledu.
    KociembaException(int error);

    // Konstruktor kopiujacy.
    KociembaException(const KociembaException& exp);

    // Destruktor. Zwalnia zajmowana pamiec.
    virtual ~KociembaException();
    
    // Kopiuje blad. Zwraca nowy blad, dla ktorego alokuje pamiec.
    virtual Exception* Clone();

    // Zwracane kody bledow.
    enum eError
    {
        SUCCESS,                            //  0
        ERR_PHASE2_CUBE_INVALID,            //  1
        ERR_PHASE2_MOVE_NOTALLOWED,         //  2
        ERR_MOVETABLE_COORD_INVALID,        //  3
        ERR_PRUNTABLE_COORD_INVALID,        //  4
        ERR_PHASE1_SOLVER_INPUT_INVALID,    //  5
        ERR_PHASE2_SOLVER_INPUT_INVALID,    //  6
        ERR_SOLVER_INPUT_INVALID,           //  7
        ERR_SOLVER_NEXT_IMPOSSIBLE,         //  8
        ERR_SOLUTION_PARSE_FORMATINVALID,   //  9
        ERR_SOLUTION_PARSE_P1MOVEINVALID,   // 10
        ERR_SOLUTION_PARSE_P1TURNINVALID,   // 11
        ERR_SOLUTION_PARSE_P1FORMATINVALID, // 12
        ERR_SOLUTION_PARSE_P2MOVEINVALID,   // 13
        ERR_SOLUTION_PARSE_P2TURNINVALID,   // 14
        ERR_SOLUTION_PARSE_P2FORMATINVALID, // 15
        ERR_NUMBER                          // 16  - liczba zwracanych kodow
    };

    
  protected:

    virtual const char* GetErrorText();

            
  private:

    // Wiadomosci bledow.
    static char *ErrorText[];

};


#endif // KociembaException_h        
