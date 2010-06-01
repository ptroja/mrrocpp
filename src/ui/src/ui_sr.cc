/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/ui_sr.h"

ui_sr_buffer::ui_sr_buffer() :
	cb(UI_SR_BUFFER_LENGHT) {

}

void ui_sr_buffer::put_one_msg(const lib::sr_package_t& new_msg) {

	boost::mutex::scoped_lock lock(mtx);
	cb.push_back(new_msg);

	return;
}

void ui_sr_buffer::get_one_msg(lib::sr_package_t& new_msg) {
	boost::mutex::scoped_lock lock(mtx);
	new_msg = cb.front();
	cb.pop_front();

	return;
}

bool ui_sr_buffer::buffer_empty() // sprawdza czy bufor jest pusty
{
	boost::mutex::scoped_lock lock(mtx);
	return cb.empty();
}
