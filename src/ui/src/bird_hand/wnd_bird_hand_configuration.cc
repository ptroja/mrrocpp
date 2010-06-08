/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/bird_hand/ui_ecp_r_bird_hand.h"
#include "ui/src/bird_hand/ui_r_bird_hand.h"
#include "ui/src/bird_hand/wnd_bird_hand_configuration.h"
#include "lib/robot_consts/bird_hand_const.h"
#include "ui/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

//
//
// KLASA UiRobotBirdHand
//
//


WndBirdHandConfiguration::WndBirdHandConfiguration(Ui& _ui,
		UiRobotBirdHand& _bird_hand) :
	ui(_ui), bird_hand(_bird_hand), is_open(false) {

}

int WndBirdHandConfiguration::get_configuration() {

	return 1;
}

int WndBirdHandConfiguration::set_configuration() {

	return 1;
}

int WndBirdHandConfiguration::copy_command() {

	return 1;
}
