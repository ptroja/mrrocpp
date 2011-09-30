%module epos
%{
#include "epos.h"
%}

typedef unsigned int speed_t;
%include <std_string.i>
%include <std_except.i>
%include "epos.h"
