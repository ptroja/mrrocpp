/*<<---------------[         cfgopts.c        ]------------------------/
/                                                                      /
/  Functional                                                          /
/     Description: Configuration file I/O                              /
/                                                                      /
/  Input         : Configuration file name                             /
/                  Configuration parameters in a structure             /
/                                                                      /
/  Process       : Interpret information by parameter and read or      /
/                  back to the configuration file.                     /
/                                                                      /
/  Ouput         : updated configuration file or updated structure.    /
/                                                                      /
/  Programmer    : Jeffry J. Brickley                                  /
/                                                                      /
/                                                                      /
/---------------------------------------------------------------------*/
#define       Assigned_Revision 950802
/*-------------------------[ Revision History ]------------------------/
/ Revision 1.0.0  :  Original Code by Jeffry J. Brickley               /
/          1.1.0  : added header capability, JJB, 950802
/------------------------------------------------------------------->>*/
/*  Please keep revision number current.                              */
#define       REVISION_NO "1.1.0"

/*---------------------------------------------------------------------/
/
/  Description:  CfgOpts is based on GETOPTS by Bob Stout.  It will
/                process a configuration file based one words and
/                store it in a structure pointing to physical data
/                area for each storage item.
/  i.e. ???.CFG:
/    Port=1
/    work_space=C:\temp
/    menus=INCFG_true
/    user=Jeffry Brickley
/  will write to the following structure:
/    struct Config_Tag configs[] = {
/    {"port",       Word_Tag,    &port_number},
/    {"work_space", String_Tag,  &work_space},
/    {"menus",      Boolean_Tag, &menu_flag},
/    {"user",       String_Tag,  &User_name},
/    {NULL,         Error_Tag,   NULL}
/    };
/  Note that the structure must always be terminated by a NULL row as
/     was the same with GETOPTS.  This however is slightly more
/     complicated than scaning the command line (but not by much) for
/     data as there can be more variety in words than letters and an
/     number of data items limited only by memory.  Currently CfgOpts
/     is not case sensitive, but this can be changed by replacing all
/     "strcasecmp" functions with "strcmp" functions.
/
/  Like the original code from which this was taken, this is released
/  to the Public Domain.  I cannot make any guarentees other than these
/  work for me and I find them usefull.  Feel free to pass these on to
/  a friend, but please do not charge him....
/
/---------------------------------------------------------------------*/

#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "lib/cfgopts.h"

static char line[256];
// static int far *Funct(char far *); // rem by Y

/*---------------------------------------------------------------------/
/   reads from an input configuration (INI) file.
/---------------------------------------------------------------------*/
/*>>------[   input_config()   ]-------------[ 08-02-95 14:02PM ]------/
/ return value:
/     int                     ; number of records read or -1 on error
/ parameters:
/     char *filename          ; filename of INI style file
/     struct Config_Tag configs[]; Configuration structure
/     char *header            ; INI header name (i.e. "[TEST]")
/-------------------------------------------------------------------<<*/
int input_config(const std::string & _filename, struct Config_Tag configs[], const std::string & _header)
{
   const char *filename = _filename.c_str();
   const char *header = _header.c_str();
   struct Config_Tag *ptr;
   int count=0,lineno=0,temp;
   FILE *file;
   char *fptr,*tok,*next;

   file=fopen(filename,"r");
   if ( file==NULL ) {
	   perror("fopen()");
	   fprintf(stderr, "Error opening config file %s\n", filename);
	   return INCFG_ERROR;   // return error designation.
   }

   if ( header!=NULL )
      do
   {
      fptr=fgets(line,255,file);  // get input line
   }
   while ( strncasecmp(line,header,strlen(header)) && !feof(file));
   if ( !feof(file) ) do {
      fptr=fgets(line,255,file);  // get input line
      if ( fptr==NULL ) continue;
      lineno++;
      if ( line[0]=='#' ) continue;    // skip comments
      if ( line[0]==';' ) continue;    // skip comments
      if ( line[0]=='[' ) continue;    // skip next header
      tok=strtok(line,"=\n\r");   // get first token
      if ( tok!=NULL )
      {
         next=strtok(NULL,"=\n\r"); // get actual config information
         for ( ptr=configs;ptr->buf;++ptr )   // scan for token
         {
            if ( !strcasecmp( tok , ptr->code ) )  // got a match?
            {
               switch ( ptr->type )     // check type
               {
               case Boolean_Tag:
                  if (!strcasecmp(next,"INCFG_false"))
                     *((Boolean_T *)(ptr->buf)) = INCFG_false;
                  else if (!strcasecmp(next,"INCFG_true"))
                     *((Boolean_T *)(ptr->buf)) = INCFG_true;
                  ++count;
                  break;

               case Byte_Tag:
                  sscanf(next, "%d", &temp);
                  *((char *)(ptr->buf))=(char)temp;
                  ++count;
                  break;

               case Int_Tag: // by Y
                  sscanf(next, "%d", (int *)(ptr->buf));
                  ++count;
                  break;


               case Word_Tag:
                  sscanf(next, "%hd", (short *)(ptr->buf));
                  ++count;
                  break;

               case DWord_Tag:
                  sscanf(next, "%ld", (long *)(ptr->buf));
                  ++count;
                  break;

               case OctWord_Tag:
                  sscanf(next, "%ho", (short *)(ptr->buf));
                  ++count;
                  break;

               case DOctWord_Tag:
                  sscanf(next, "%lo", (long *)(ptr->buf));
                  ++count;
                  break;

               case HexWord_Tag:
                  sscanf(next, "%hx", (short *)(ptr->buf));
                  ++count;
                  break;

               case DHexWord_Tag:
                  sscanf(next, "%lx", (long *)(ptr->buf));
                  ++count;
                  break;

               case Float_Tag:
                  sscanf(next, "%g", (float *)ptr->buf);
                  ++count;
                  break;

               case Double_Tag:
                  sscanf(next, "%lg", (double *)ptr->buf);
                  ++count;
                  break;

               case String_Tag:
                  strcpy((char *)ptr->buf, next);
                  ++count;
                  break;

               case Function_Tag:
               case Error_Tag:
               default:
                  printf("Error in Config file %s on line %d\n",
                  filename,lineno);
                  break;
               }
            }

         }
      }
   }
   while ( fptr!=NULL && line[0]!='[');
   fclose(file);
   return count;
}
