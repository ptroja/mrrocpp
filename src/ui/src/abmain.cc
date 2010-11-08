/* M a i n l i n e                                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.03  */

#ifdef __USAGE
%C - This is a QNX/Photon Application.
%C [options]

Options:
  -s server   Server node or device name
  -x x        Initial x position
  -y y        Initial y position
  -h h        Initial h dimension
  -w w        Initial w dimension

Examples:
%C -s4
  Run using Photon server on node 4

%C -s//4/dev/photon
  Same as above

%C -x10 -y10 -h200 -w300
  Run at initial position 10,10 with initial 
  dimension of 200x300.
#endif

/* Standard headers */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

/* Local headers */
#include "ablibs.h"
#include "../src/ui.h"
#include "abimport.h"
#include "proto.h"
#include "abwidgets.h"
#include "abdefine.h"
#include "abevents.h"
#include "ablinks.h"
#include "abvars.h"

int
main ( int argc, char *argv[] )

	{

	_Ap_.Ap_names = AB_ENABLE;
	_Ap_.Ap_winstate = 0;

	/* AppBuilder Initialization */
	ApInitialize( argc, argv, &AbContext );

	/* Display main window */
	ApLinkWindow( NULL, &AbApplLinks[0], NULL );

	/* Loop until user quits application */
	PtMainLoop( );
	PtExit( 0 );

	return 0;
	}

static const ApClassTab_t ClassTable[] = {
	{ "PtMenu", &PtMenu },
	{ "PtWindow", &PtWindow },
	{ "PtRect", &PtRect },
	{ "PtLabel", &PtLabel },
	{ "PtText", &PtText },
	{ "PtButton", &PtButton },
	{ "PtNumericFloat", &PtNumericFloat },
	{ "PtTimer", &PtTimer },
	{ "PtPane", &PtPane },
	{ "PtNumericInteger", &PtNumericInteger },
	{ "PtToolbar", &PtToolbar },
	{ "PtMenuButton", &PtMenuButton },
	{ "PtMultiText", &PtMultiText },
	{ "PtComboBox", &PtComboBox },
	{ "PtFileSel", &PtFileSel },
	{ "PtToggleButton", &PtToggleButton },
	{ "PtGroup", &PtGroup },
	{ NULL, NULL }
	};

ApContext_t AbContext = { ClassTable, 1, AbWidgets };


