/*
**  CFGOPTS.H
plik naglowkowy do procedur odczytu i zapisu z pliku INI
*/

#if !defined(__CFGOPTS_H)
#define __CFGOPTS_H

#include <string>

#ifdef __cplusplus
extern "C" {
#endif


#ifndef Boolean_T_defined
#define Boolean_T_defined
// #undef ERROR // rem by Y
// #undef false // rem by Y
// #undef true // rem by Y

typedef enum {INCFG_ERROR = -1, INCFG_false, INCFG_true} Boolean_T;
#endif

#ifndef TAG_TYPE_defined
#define TAG_TYPE_defined
typedef enum {
      Error_Tag,
      Byte_Tag,
      Boolean_Tag,
      Int_Tag,
      Word_Tag,
      DWord_Tag,
      OctWord_Tag,
      DOctWord_Tag,
      HexWord_Tag,
      DHexWord_Tag,
      Float_Tag,
      Double_Tag,
      String_Tag,
      Function_Tag
      } TAG_TYPE;
#endif

struct Config_Tag {
      char        *code;                /* Option switch        */
      TAG_TYPE    type;                 /* Type of option       */
      void        *buf;                 /* Storage location     */
};

int input_config(std::string, struct Config_Tag *, std::string);
int update_config(std::string, struct Config_Tag *, std::string);

#ifdef __cplusplus
}
#endif


#endif
