/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/ui_class.h"

//
//
// KLASA ui
//
//


Ui::Ui() :
	config(NULL), all_ecp_msg(NULL), ui_msg(NULL), is_mp_and_ecps_active(false) {

	mp.state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	mp.last_state = UI_MP_NOT_PERMITED_TO_RUN;// mp wylaczone
	mp.pid = -1;

}

