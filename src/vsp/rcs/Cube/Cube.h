/////////////////////////////////////////////////////////
// Nazwa: Cube.h
// Opis: Interfejs Cube definiujacy podstawowe metody na kostce.
/////////////////////////////////////////////////////////


#ifndef Cube_h
#define Cube_h

#include "CubeException.h"
#include "CubeMove.h"


// Interfejs definiujacy podstawowe metody na Kostce Rubika.
class Cube
{

  public:

    // Stala okreslajaca ciag znakow reprezentujacy typ klasy.
    const static char* TYPE;

    // Zwraca typ klasy.
    virtual const char* GetType() = 0;

    
  protected:
          
    // Znacznik trybu pracy - z walidacja stanu czy bez.
    static bool bValidation;
    
    
  public:
         
    virtual ~Cube() {}

    // Kopiuje stan kostki. Zwraca nowy stan, dla ktorego alokuje pamiec.
    virtual Cube* Clone() = 0;

    // Ustawia stan kostki jako ulozony. Nie zwraca zadnej wartosci.
    virtual void SetInitState() = 0;

    // Zamienia stan kostki na lancuch znakow reprezentujacy go. Zwraca tenze
    // lancuch znakow. Alokuje pamiec na lancuch, ktora trzeba pozniej zwolnic.
    virtual char* ToString() = 0;

    // Wykonuje ruch o numerze move na kostce. Nie zwraca zadnej wartosci.
    virtual void Move(CubeMove::eMove move, CubeMove::eTurn turn) = 0;

    // Sprawdza, czy mozna na kostce wykonac ruch move o obrocie turn.
    virtual bool IsAllowed(CubeMove::eMove move, CubeMove::eTurn turn) 
        { return true; };

    // Ustawia tryb pracy.
    static void SetValidation(bool on) { bValidation = on; }

};


#endif // Cube_h
